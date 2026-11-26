#!/usr/bin/env python3

"""Runtime keep-in supervisor and RViz wireframe for the FR3 TCP."""

import math
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Point
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class WorkspaceBoundarySupervisor(Node):
    """Cancel active trajectories when the measured TCP leaves its keep-in box."""

    def __init__(self):
        super().__init__("workspace_boundary_supervisor")

        self.declare_parameter("frame_id", "fr3_link0")
        self.declare_parameter("tcp_link", "fr3_hand_tcp")
        self.declare_parameter("hard_min", [0.25, -1.10, -0.1265])
        self.declare_parameter("hard_max", [1.45, 0.90, 0.85])
        self.declare_parameter("planning_min", [0.28, -1.05, -0.1165])
        self.declare_parameter("planning_max", [1.40, 0.85, 0.80])
        self.declare_parameter("monitor_rate_hz", 100.0)
        self.declare_parameter("marker_rate_hz", 10.0)
        self.declare_parameter("warning_distance", 0.01)
        self.declare_parameter("cancel_repeat_seconds", 0.20)

        self._frame_id = self.get_parameter("frame_id").value
        self._tcp_link = self.get_parameter("tcp_link").value
        self._hard_min = self._vector_parameter("hard_min")
        self._hard_max = self._vector_parameter("hard_max")
        self._planning_min = self._vector_parameter("planning_min")
        self._planning_max = self._vector_parameter("planning_max")
        monitor_rate = float(self.get_parameter("monitor_rate_hz").value)
        marker_rate = float(self.get_parameter("marker_rate_hz").value)
        self._warning_distance = float(
            self.get_parameter("warning_distance").value
        )
        self._cancel_repeat_seconds = float(
            self.get_parameter("cancel_repeat_seconds").value
        )

        self._validate_limits()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._current_position = None
        self._state = "WAITING_FOR_TF"
        self._armed = False
        self._recovery = False
        self._stop_latched = False
        self._last_cancel_time = 0.0
        self._last_reported_state = None

        self._moveit_cancel = self.create_client(
            CancelGoal,
            "/execute_trajectory/_action/cancel_goal",
        )
        self._controller_cancel = self.create_client(
            CancelGoal,
            "/fr3_arm_controller/follow_joint_trajectory/_action/cancel_goal",
        )

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._status_publisher = self.create_publisher(
            String,
            "/workspace_boundary/status",
            transient_qos,
        )
        self._stopped_publisher = self.create_publisher(
            Bool,
            "/workspace_boundary/stopped",
            transient_qos,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            "/rviz_visual_tools",
            10,
        )
        self._reset_service = self.create_service(
            Trigger,
            "/workspace_boundary/reset",
            self._reset_callback,
        )

        # The simulation does not bridge /clock. Steady timers keep the safety
        # monitor alive even while Gazebo is paused or simulation time is zero.
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._monitor_timer = self.create_timer(
            1.0 / monitor_rate,
            self._monitor,
            clock=steady_clock,
        )
        self._marker_timer = self.create_timer(
            1.0 / marker_rate,
            self._publish_markers,
            clock=steady_clock,
        )

        self._publish_state(force=True)
        self.get_logger().info(
            "Workspace supervisor ready: planning box "
            f"{self._planning_min} to {self._planning_max}; hard box "
            f"{self._hard_min} to {self._hard_max}."
        )

    def _vector_parameter(self, name):
        values = tuple(float(value) for value in self.get_parameter(name).value)
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError(f"Parameter {name} must contain three finite values")
        return values

    def _validate_limits(self):
        for axis, hard_minimum, planning_minimum, planning_maximum, hard_maximum in zip(
            "XYZ",
            self._hard_min,
            self._planning_min,
            self._planning_max,
            self._hard_max,
        ):
            if not hard_minimum < planning_minimum < planning_maximum < hard_maximum:
                raise ValueError(
                    f"Invalid {axis} limits: expected hard_min < planning_min < "
                    "planning_max < hard_max"
                )

    @staticmethod
    def _inside(position, lower, upper):
        return all(
            minimum <= value <= maximum
            for value, minimum, maximum in zip(position, lower, upper)
        )

    def _inside_planning(self, position):
        return self._inside(position, self._planning_min, self._planning_max)

    def _inside_hard(self, position):
        return self._inside(position, self._hard_min, self._hard_max)

    def _distance_to_planning_face(self, position):
        distances = []
        for value, minimum, maximum in zip(
            position,
            self._planning_min,
            self._planning_max,
        ):
            distances.extend((value - minimum, maximum - value))
        return min(distances)

    def _lookup_position(self):
        transform = self._tf_buffer.lookup_transform(
            self._frame_id,
            self._tcp_link,
            Time(),
        )
        translation = transform.transform.translation
        return (float(translation.x), float(translation.y), float(translation.z))

    def _monitor(self):
        try:
            self._current_position = self._lookup_position()
        except TransformException:
            if self._state != "WAITING_FOR_TF":
                self._state = "WAITING_FOR_TF"
                self._publish_state()
            return

        position = self._current_position

        if self._stop_latched:
            self._state = "STOPPED"
            self._cancel_active_trajectories()
            self._publish_state()
            return

        if self._recovery:
            if not self._inside_hard(position):
                self._trigger_stop("TCP left the hard envelope during recovery")
            elif self._inside_planning(position):
                self._recovery = False
                self._armed = True
                self._state = "SAFE"
                self.get_logger().info(
                    "TCP re-entered the planning boundary; full supervision re-armed."
                )
                self._publish_state()
            else:
                self._state = "RECOVERY"
                self._publish_state()
            return

        # Do not trap a robot that starts outside the normal box. Supervision
        # arms automatically after the first valid in-box TCP measurement.
        if not self._armed:
            if self._inside_planning(position):
                self._armed = True
                self._state = "SAFE"
                self.get_logger().info("Workspace supervision armed.")
            else:
                self._state = "WAITING_FOR_SAFE_ENTRY"
            self._publish_state()
            return

        if not self._inside_planning(position):
            self._trigger_stop("measured TCP crossed the planning boundary")
            return

        if self._distance_to_planning_face(position) <= self._warning_distance:
            self._state = "WARNING"
        else:
            self._state = "SAFE"
        self._publish_state()

    def _trigger_stop(self, reason):
        self._stop_latched = True
        self._armed = False
        self._state = "STOPPED"
        position = self._current_position
        self.get_logger().error(
            f"WORKSPACE STOP: {reason}; TCP={position}. "
            "Canceling MoveIt and arm-controller trajectories."
        )
        self._cancel_active_trajectories(force=True)
        self._publish_state(force=True)

    def _cancel_active_trajectories(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_cancel_time < self._cancel_repeat_seconds:
            return
        self._last_cancel_time = now

        # An all-zero GoalInfo requests cancellation of every active goal.
        for label, client in (
            ("MoveIt", self._moveit_cancel),
            ("arm controller", self._controller_cancel),
        ):
            if not client.service_is_ready():
                self.get_logger().warning(
                    f"{label} cancellation service is not ready."
                )
                continue
            client.call_async(CancelGoal.Request())

    def _reset_callback(self, _request, response):
        if self._current_position is None:
            response.success = False
            response.message = "Cannot reset before the first TCP measurement."
            return response
        if not self._inside_hard(self._current_position):
            response.success = False
            response.message = (
                "TCP is outside the hard envelope; manual recovery is required."
            )
            return response

        self._stop_latched = False
        if self._inside_planning(self._current_position):
            self._armed = True
            self._recovery = False
            self._state = "SAFE"
            response.message = "Workspace stop reset; supervision is armed."
        else:
            self._armed = False
            self._recovery = True
            self._state = "RECOVERY"
            response.message = (
                "Workspace stop reset in recovery mode. Move the TCP back into "
                "the green planning box; the hard envelope remains enforced."
            )
        response.success = True
        self.get_logger().warning(response.message)
        self._publish_state(force=True)
        return response

    def _publish_state(self, force=False):
        if not force and self._state == self._last_reported_state:
            return
        self._last_reported_state = self._state
        status = String()
        status.data = self._state
        self._status_publisher.publish(status)
        stopped = Bool()
        stopped.data = self._stop_latched
        self._stopped_publisher.publish(stopped)

    @staticmethod
    def _box_edges(lower, upper):
        corners = [
            (x, y, z)
            for x in (lower[0], upper[0])
            for y in (lower[1], upper[1])
            for z in (lower[2], upper[2])
        ]
        index = {(x, y, z): number for number, (x, y, z) in enumerate(corners)}
        pairs = []
        for corner in corners:
            for axis in range(3):
                other = list(corner)
                other[axis] = upper[axis] if corner[axis] == lower[axis] else lower[axis]
                other = tuple(other)
                if index[corner] < index[other]:
                    pairs.append((corner, other))
        points = []
        for start, end in pairs:
            for xyz in (start, end):
                point = Point()
                point.x, point.y, point.z = xyz
                points.append(point)
        return points

    def _line_marker(self, marker_id, namespace, lower, upper, color, width):
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = self._box_edges(lower, upper)
        return marker

    def _publish_markers(self):
        if self._state == "STOPPED":
            planning_color = (1.0, 0.05, 0.05, 1.0)
        elif self._state in ("WARNING", "RECOVERY"):
            planning_color = (1.0, 0.65, 0.0, 1.0)
        else:
            planning_color = (0.05, 0.95, 0.20, 0.95)

        hard = self._line_marker(
            0,
            "workspace_hard_boundary",
            self._hard_min,
            self._hard_max,
            (0.9, 0.1, 0.1, 0.35),
            0.006,
        )
        planning = self._line_marker(
            1,
            "workspace_planning_boundary",
            self._planning_min,
            self._planning_max,
            planning_color,
            0.012,
        )

        label = Marker()
        label.header.frame_id = self._frame_id
        label.header.stamp = self.get_clock().now().to_msg()
        label.ns = "workspace_boundary_status"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = (
            self._planning_min[0] + self._planning_max[0]
        ) / 2.0
        label.pose.position.y = (
            self._planning_min[1] + self._planning_max[1]
        ) / 2.0
        label.pose.position.z = self._hard_max[2] + 0.08
        label.pose.orientation.w = 1.0
        label.scale.z = 0.055
        label.color.r, label.color.g, label.color.b, label.color.a = planning_color
        label.text = f"TCP WORKSPACE: {self._state}"

        self._marker_publisher.publish(MarkerArray(markers=[hard, planning, label]))


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceBoundarySupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
