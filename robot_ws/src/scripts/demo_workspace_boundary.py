#!/usr/bin/env python3

"""Visual comparison demo for FR3 planning and runtime workspace protection."""

import argparse
import math
import sys
import time
from threading import Lock, Thread

import rclpy
from geometry_msgs.msg import Point
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from basic_function.fr3_robot import get_robot
from basic_function.fr3_gripper import get_gripper
from voxposer_fr3_adapter import (
    PICK_POSITION,
    VoxPoserFR3Adapter,
    run_pick_and_place,
)


WORLD_NAME = "fr3_pick_place"
TARGET_MODEL = "pick_object"
STOP_MODEL = "workspace_demo_stop_marker"
PICK_OBJECT_TABLE_Z = -0.1065


def position_error(first, second):
    return math.sqrt(
        sum((float(a) - float(b)) ** 2 for a, b in zip(first[:3], second[:3]))
    )


class DemoVisualizer(Node):
    """Keep the Gazebo target and RViz comparison graphics synchronized."""

    def __init__(self, frame_id="fr3_link0", tcp_link="fr3_hand_tcp"):
        super().__init__("workspace_boundary_demo")
        self.frame_id = frame_id
        self.tcp_link = tcp_link

        self._lock = Lock()
        self._target = (0.45, -0.25, 0.10)
        self._target_color = (0.1, 0.8, 0.2, 0.95)
        self._label = "Workspace boundary demo ready"
        self._tcp = None
        self._trail = []
        self._stop_point = None
        self._status = None
        self._stopped = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._set_pose = self.create_client(
            SetEntityPose,
            f"/world/{WORLD_NAME}/set_pose",
        )
        self._reset = self.create_client(
            Trigger,
            "/workspace_boundary/reset",
        )

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            "/workspace_boundary/status",
            self._status_callback,
            transient_qos,
        )
        self.create_subscription(
            Bool,
            "/workspace_boundary/stopped",
            self._stopped_callback,
            transient_qos,
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray,
            "/rviz_visual_tools",
            10,
        )

        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            0.05,
            self._update_and_publish,
            clock=steady_clock,
        )

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._executor_thread = Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

    def _status_callback(self, message):
        with self._lock:
            self._status = message.data

    def _stopped_callback(self, message):
        with self._lock:
            self._stopped = bool(message.data)

    @property
    def boundary_state(self):
        with self._lock:
            return self._status, self._stopped

    @staticmethod
    def _wait_future(future, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.01)
        raise TimeoutError("ROS operation timed out")

    def _move_gazebo_model(self, model_name, position):
        request = SetEntityPose.Request()
        request.entity.name = model_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(position[0])
        request.pose.position.y = float(position[1])
        request.pose.position.z = float(position[2])
        request.pose.orientation.w = 1.0
        response = self._wait_future(self._set_pose.call_async(request))
        return response is not None and bool(response.success)

    def set_target(
        self,
        position,
        color,
        label,
        clear_trail=True,
    ):
        position = tuple(float(value) for value in position)
        with self._lock:
            self._target = position
            self._target_color = color
            self._label = label
            self._stop_point = None
            if clear_trail:
                self._trail = []

        if not self._set_pose.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                "Gazebo set-pose service unavailable; RViz target is still active."
            )
            return False

        try:
            object_position = (position[0], position[1], PICK_OBJECT_TABLE_Z)
            moved = self._move_gazebo_model(TARGET_MODEL, object_position)
            self._move_gazebo_model(STOP_MODEL, (0.0, 0.0, -3.6))
        except Exception as error:
            self.get_logger().warning(f"Could not move Gazebo target: {error}")
            return False
        if moved:
            print(
                f"[Gazebo] {TARGET_MODEL} moved on the table to "
                f"({object_position[0]:.3f}, {object_position[1]:.3f}, "
                f"{object_position[2]:.4f}); TCP target Z={position[2]:.3f}"
            )
        return moved

    def set_result(self, label, color, stop_point=None):
        with self._lock:
            self._label = label
            self._target_color = color
            self._stop_point = (
                None if stop_point is None else tuple(stop_point[:3])
            )
        if self._set_pose.service_is_ready():
            try:
                marker_position = (0.0, 0.0, -3.6) if stop_point is None else stop_point
                self._move_gazebo_model(STOP_MODEL, marker_position)
            except Exception as error:
                self.get_logger().warning(f"Could not move Gazebo stop marker: {error}")

    def reset_supervisor(self):
        if not self._reset.wait_for_service(timeout_sec=5.0):
            return False, "Workspace reset service is unavailable"
        try:
            response = self._wait_future(
                self._reset.call_async(Trigger.Request()),
                timeout=5.0,
            )
        except Exception as error:
            return False, str(error)
        if response is None:
            return False, "Workspace reset returned no response"
        return bool(response.success), response.message

    def wait_for_state(self, accepted, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status, stopped = self.boundary_state
            if status in accepted:
                return status, stopped
            time.sleep(0.05)
        return self.boundary_state

    def _update_tcp(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.frame_id,
                self.tcp_link,
                Time(),
            )
        except TransformException:
            return

        translation = transform.transform.translation
        point = (float(translation.x), float(translation.y), float(translation.z))
        with self._lock:
            self._tcp = point
            if not self._trail or position_error(point, self._trail[-1]) >= 0.002:
                self._trail.append(point)
                if len(self._trail) > 500:
                    self._trail = self._trail[-500:]

    @staticmethod
    def _point(xyz):
        point = Point()
        point.x, point.y, point.z = (float(value) for value in xyz)
        return point

    def _base_marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace_demo"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _publish_markers(self):
        with self._lock:
            target = self._target
            target_color = self._target_color
            label_text = self._label
            tcp = self._tcp
            trail = list(self._trail)
            stop_point = self._stop_point

        target_marker = self._base_marker(100, Marker.CUBE)
        target_marker.pose.position.x = target[0]
        target_marker.pose.position.y = target[1]
        target_marker.pose.position.z = target[2]
        target_marker.scale.x = 0.065
        target_marker.scale.y = 0.065
        target_marker.scale.z = 0.065
        (
            target_marker.color.r,
            target_marker.color.g,
            target_marker.color.b,
            target_marker.color.a,
        ) = target_color

        direction = self._base_marker(101, Marker.LINE_LIST)
        direction.scale.x = 0.008
        direction.color.r = target_color[0]
        direction.color.g = target_color[1]
        direction.color.b = target_color[2]
        direction.color.a = 0.75
        if tcp is not None:
            direction.points = [self._point(tcp), self._point(target)]

        trail_marker = self._base_marker(102, Marker.LINE_STRIP)
        trail_marker.scale.x = 0.009
        trail_marker.color.r = 0.65
        trail_marker.color.g = 0.15
        trail_marker.color.b = 0.95
        trail_marker.color.a = 0.9
        trail_marker.points = [self._point(point) for point in trail]

        label = self._base_marker(103, Marker.TEXT_VIEW_FACING)
        label.pose.position.x = target[0]
        label.pose.position.y = target[1]
        label.pose.position.z = target[2] + 0.10
        label.scale.z = 0.045
        label.color.r = target_color[0]
        label.color.g = target_color[1]
        label.color.b = target_color[2]
        label.color.a = 1.0
        label.text = label_text

        markers = [target_marker, direction, trail_marker, label]
        stop_marker = self._base_marker(104, Marker.SPHERE)
        if stop_point is None:
            stop_marker.action = Marker.DELETE
        else:
            stop_marker.pose.position.x = stop_point[0]
            stop_marker.pose.position.y = stop_point[1]
            stop_marker.pose.position.z = stop_point[2]
            stop_marker.scale.x = 0.055
            stop_marker.scale.y = 0.055
            stop_marker.scale.z = 0.055
            stop_marker.color.r = 1.0
            stop_marker.color.g = 0.85
            stop_marker.color.b = 0.0
            stop_marker.color.a = 1.0
        markers.append(stop_marker)

        self._marker_publisher.publish(MarkerArray(markers=markers))

    def _update_and_publish(self):
        self._update_tcp()
        self._publish_markers()

    def shutdown(self):
        self._executor.remove_node(self)
        self._executor.shutdown(timeout_sec=2.0)
        self._executor_thread.join(timeout=2.0)
        self.destroy_node()


class BoundaryDemo:
    def __init__(
        self,
        robot,
        gripper,
        visualizer,
        stage_delay=3.0,
        interactive=False,
    ):
        self.robot = robot
        self.gripper = gripper
        self.visualizer = visualizer
        self.stage_delay = float(stage_delay)
        self.interactive = bool(interactive)
        self.results = []

    def stage_pause(self, message):
        print(f"\n{message}")
        if self.interactive:
            input("Press Enter to continue...")
        else:
            time.sleep(self.stage_delay)

    def record(self, name, passed, detail=""):
        passed = bool(passed)
        self.results.append((name, passed, detail))
        label = "PASS" if passed else "FAIL"
        suffix = f" — {detail}" if detail else ""
        print(f"[{label}] {name}{suffix}")
        return passed

    def prepare_home(self):
        reset_ok, reset_message = self.visualizer.reset_supervisor()
        if not reset_ok:
            self.record("Reset workspace supervisor", False, reset_message)
            return False
        return self.record("Move to Work Home", self.robot.go_home())

    def inside(self, target=PICK_POSITION):
        print("\n====== EXPERIMENT A: FULL PICK AND PLACE / PLANNING ON ======")
        reset_ok, reset_message = self.visualizer.reset_supervisor()
        if not self.record("Reset workspace supervisor", reset_ok, reset_message):
            return

        def prepare_object():
            moved = self.visualizer.set_target(
                target,
                (0.05, 0.95, 0.20, 0.95),
                "A: FULL PICK AND PLACE | PLANNING ON",
            )
            return moved

        passed, failed_stage = run_pick_and_place(
            self.robot,
            self.gripper,
            prepare_pick_object=prepare_object,
        )
        self.visualizer.set_result(
            f"A: {'FULL PICK AND PLACE COMPLETE' if passed else 'FAILED'}",
            (0.05, 0.95, 0.20, 0.95) if passed else (1.0, 0.05, 0.05, 1.0),
        )
        detail = "all stages completed" if passed else f"failed_stage={failed_stage}"
        self.record("Full in-boundary pick-and-place", passed, detail)

    def planning_reject(self, target=None):
        print("\n=========== EXPERIMENT B: TARGET / PLANNING ON ===========")
        if not self.prepare_home():
            return
        start = self.robot.get_actual_pose()
        if target is None:
            target = (
                self.robot.workspace_boundary.planning_max[0] + 0.2, # self.robot.workspace_boundary.planning_min[0] - 0.011,
                start[1],
                start[2],
            )
        inside = self.robot.workspace_boundary.contains(target, region="planning")
        target_color = (
            (0.05, 0.95, 0.20, 0.95)
            if inside
            else (1.0, 0.08, 0.08, 1.0)
        )
        self.visualizer.set_target(
            target,
            target_color,
            f"B: {'VALID' if inside else 'OUTSIDE'} TARGET | PLANNING ON",
        )
        self.stage_pause(
            "B: target is inside the boundary; the robot should move to it."
            if inside
            else "B: target is outside the boundary; planning should reject it."
        )
        moved = self.robot.move_to_xyz(*target)
        actual = self.robot.get_actual_pose()
        drift = position_error(actual, start)
        target_error = position_error(actual, target)

        if inside:
            passed = moved and target_error <= 0.010
            result = "VALID TARGET REACHED" if passed else "VALID TARGET FAILED"
            detail = f"target_error={target_error*1000:.2f} mm"
            test_name = "Planning protection permits valid target"
        else:
            passed = not moved and drift <= 0.010
            result = "REJECTED — NO MOTION" if passed else "OUTSIDE TARGET FAILED"
            detail = f"drift={drift*1000:.2f} mm"
            test_name = "Planning protection rejects outside target"

        self.visualizer.set_result(
            f"B: {result} | {detail}",
            target_color if passed else (1.0, 0.05, 0.05, 1.0),
        )
        self.record(test_name, passed, detail)
        self.stage_pause("Experiment B result is displayed in Gazebo and RViz.")

    def runtime_stop(self, target=None):
        print("\n========== EXPERIMENT C: TARGET / PLANNING OFF ==========")
        if not self.prepare_home():
            return
        start = self.robot.get_actual_pose()
        if target is None:
            target = (
                self.robot.workspace_boundary.planning_min[0] - 0.011,
                start[1],
                start[2],
            )
        inside = self.robot.workspace_boundary.contains(target, region="planning")
        target_color = (
            (0.05, 0.95, 0.20, 0.95)
            if inside
            else (1.0, 0.55, 0.0, 1.0)
        )
        self.visualizer.set_target(
            target,
            target_color,
            f"C: {'VALID' if inside else 'OUTSIDE'} TARGET | PLANNING OFF",
        )
        self.stage_pause(
            "C: target is inside the boundary; the runtime supervisor should allow the motion."
            if inside
            else "C: target is outside and planning protection is disabled. "
            "The runtime supervisor should stop the moving robot."
        )
        moved = self.robot.move_to_xyz(
            *target,
            enforce_workspace_boundary=False,
        )

        if inside:
            actual = self.robot.get_actual_pose()
            status, stopped = self.visualizer.boundary_state
            target_error = position_error(actual, target)
            passed = (
                moved
                and stopped is not True
                and target_error <= 0.010
            )
            detail = (
                f"status={status}, target_error={target_error*1000:.2f} mm"
            )
            self.visualizer.set_result(
                f"C: {'VALID TARGET REACHED' if passed else 'VALID TARGET FAILED'}"
                f" | error={target_error*1000:.1f} mm",
                target_color if passed else (1.0, 0.05, 0.05, 1.0),
            )
            self.record("Runtime supervisor permits valid target", passed, detail)
            self.stage_pause("Experiment C result is displayed in Gazebo and RViz.")
            return

        status, stopped = self.visualizer.wait_for_state({"STOPPED"}, timeout=5.0)
        # Let the canceled controller settle before marking the physical stop
        # point in Gazebo.
        time.sleep(0.25)
        stop_pose = self.robot.get_actual_pose()
        inside_hard = self.robot.workspace_boundary.contains(
            stop_pose[:3],
            region="hard",
        )
        passed = not moved and status == "STOPPED" and stopped is True and inside_hard
        self.visualizer.set_result(
            f"C: {'RUNTIME STOP' if passed else 'FAIL'} | "
            f"TCP={tuple(round(value, 3) for value in stop_pose[:3])}",
            (1.0, 0.05, 0.05, 1.0),
            stop_point=stop_pose,
        )
        self.record(
            "Runtime supervisor stops unconstrained motion",
            passed,
            f"status={status}, TCP={stop_pose[:3]}",
        )
        self.stage_pause(
            "The yellow sphere marks the actual runtime stop point in Gazebo and RViz."
        )

        reset_ok, reset_message = self.visualizer.reset_supervisor()
        self.record("Reset runtime stop", reset_ok, reset_message)
        if reset_ok:
            recovery_pose = self.robot.get_actual_pose()
            inside_planning = self.robot.workspace_boundary.contains(
                recovery_pose[:3],
                region="planning",
            )
            recovered = True
            if not inside_planning:
                recovery_margin = 0.03
                recovery_target = tuple(
                    min(max(value, lower + recovery_margin), upper - recovery_margin)
                    for value, lower, upper in zip(
                        recovery_pose[:3],
                        self.robot.workspace_boundary.planning_min,
                        self.robot.workspace_boundary.planning_max,
                    )
                )
                recovered = self.robot.move_to_xyz(
                    *recovery_target,
                    enforce_workspace_boundary=False,
                )
            status, _ = self.visualizer.wait_for_state({"SAFE", "WARNING"})
            recovered = recovered and status in ("SAFE", "WARNING")
            if recovered:
                recovered = self.robot.go_home()
            self.visualizer.set_result(
                f"C: {'RECOVERED AND RE-ARMED' if recovered else 'RECOVERY FAILED'}",
                (0.05, 0.95, 0.20, 0.95) if recovered else (1.0, 0.05, 0.05, 1.0),
            )
            self.record("Recover to Home and re-arm", recovered, f"status={status}")

    def custom(self, target, planning_protection=True):
        mode = "ON" if planning_protection else "OFF"
        print(f"\n========== CUSTOM TARGET / PLANNING {mode} ==========")
        if not self.prepare_home():
            return
        inside = self.robot.workspace_boundary.contains(target, region="planning")
        color = (0.05, 0.95, 0.20, 0.95) if inside else (1.0, 0.20, 0.05, 1.0)
        self.visualizer.set_target(
            target,
            color,
            f"CUSTOM {tuple(round(v, 3) for v in target)} | PLANNING {mode}",
        )
        self.stage_pause("Custom target is ready.")
        moved = self.robot.move_to_xyz(
            *target,
            enforce_workspace_boundary=planning_protection,
        )
        status, stopped = self.visualizer.boundary_state
        actual = self.robot.get_actual_pose()
        if stopped:
            self.visualizer.set_result(
                f"CUSTOM: RUNTIME STOP | TCP={tuple(round(v, 3) for v in actual[:3])}",
                (1.0, 0.05, 0.05, 1.0),
                stop_point=actual,
            )
        else:
            self.visualizer.set_result(
                f"CUSTOM: {'MOVED' if moved else 'REJECTED'} | status={status}",
                color,
            )
        self.record(
            "Custom target completed",
            True,
            f"moved={moved}, status={status}, stopped={stopped}, TCP={actual[:3]}",
        )
        self.stage_pause("Custom experiment result is displayed in Gazebo and RViz.")

    def summary(self):
        failures = sum(not passed for _, passed, _ in self.results)
        print("\n========== DEMO SUMMARY ==========")
        print(f"Passed: {len(self.results) - failures}")
        print(f"Failed: {failures}")
        return failures


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        choices=("all", "inside", "planning-reject", "runtime-stop", "custom"),
        default="all",
    )
    parser.add_argument(
        "--target",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help=(
            "Target XYZ in fr3_link0 coordinates. Applies to planning-reject, "
            "runtime-stop, and custom; with --scenario all it is shared by B and C."
        ),
    )
    parser.add_argument(
        "--disable-planning-protection",
        action="store_true",
        help="Custom scenario only: deliberately omit target checks and MoveIt path constraints.",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Wait for Enter between visual stages instead of using a timer.",
    )
    parser.add_argument(
        "--stage-delay",
        type=float,
        default=3.0,
        help="Seconds to show each visual stage (default: 3).",
    )
    arguments = parser.parse_args()
    if arguments.scenario == "custom" and arguments.target is None:
        parser.error("--scenario custom requires --target X Y Z")
    if arguments.target is not None and not all(
        math.isfinite(value) for value in arguments.target
    ):
        parser.error("--target values must be finite")
    if arguments.stage_delay < 0.0:
        parser.error("--stage-delay cannot be negative")
    return arguments


def main():
    arguments = parse_arguments()
    robot = VoxPoserFR3Adapter(robot=get_robot(), gripper=None)
    gripper = get_gripper()
    visualizer = DemoVisualizer(
        frame_id=robot.workspace_boundary.frame_id,
        tcp_link=robot.workspace_boundary.tcp_link,
    )
    demo = BoundaryDemo(
        robot,
        gripper,
        visualizer,
        stage_delay=arguments.stage_delay,
        interactive=arguments.interactive,
    )

    try:
        selected_target = (
            None if arguments.target is None else tuple(arguments.target)
        )
        if arguments.scenario in ("inside", "all"):
            demo.inside()
        if arguments.scenario in ("planning-reject", "all"):
            demo.planning_reject(selected_target)
        if arguments.scenario in ("runtime-stop", "all"):
            demo.runtime_stop(selected_target)
        if arguments.scenario == "custom":
            demo.custom(
                tuple(arguments.target),
                planning_protection=not arguments.disable_planning_protection,
            )
        failures = demo.summary()
    finally:
        try:
            visualizer.reset_supervisor()
            robot.go_home()
        except Exception as error:
            print(f"Cleanup warning: {error}")
        visualizer.shutdown()
        gripper.shutdown_gripper(False)
        robot.shutdown()

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
