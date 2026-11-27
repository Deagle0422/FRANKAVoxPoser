#!/usr/bin/env python3

import time
from threading import Event, Lock, Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionFK

from scipy.spatial.transform import Rotation

from tf2_ros import Buffer, TransformException, TransformListener


# =============================================================
# FR3 joint names
# =============================================================

JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


# =============================================================
# Work Home joint configuration
#
# Order:
# [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
#
# These values are in radians.
# go_home() uses these joint values directly and does not solve
# the Home position from a Cartesian pose.
# =============================================================

WORK_HOME_JOINTS = [
    -0.035564810037612915,
    -0.5172552466392517,
    0.04230542108416557,
    -3.015706777572632,
    0.03355789557099342,
    2.4991559982299805,
    0.8,
]

class FR3Robot(Node):
    """
    Simple FR3 Cartesian robot controller.

    Cartesian pose format:

        [x, y, z, rx, ry, rz]

    The main functions are:

        robot.get_actual_pose()
        robot.get_joint_pose()
        robot.get_home_pose()
        robot.go_home()

        robot.create_pose_from_home(x, y, z)
        robot.move_to_xyz(x, y, z)

        robot.execute_cartesian_trajectory(pose_list)
    """

    def __init__(
        self,
        base_frame="fr3_link0",
        ee_link="fr3_hand_tcp",
        planning_group="fr3_arm",
        joint_state_topic="/franka/joint_states",
    ):
        # Initialize ROS only once.
        if not rclpy.ok():
            rclpy.init()

        super().__init__("fr3_robot_client")

        self.base_frame = base_frame
        self.ee_link = ee_link
        self.planning_group = planning_group
        self.joint_state_topic = joint_state_topic

        # Current joint state.
        self._joint_pose = [0.0] * 7
        self._joint_ready = Event()
        self._joint_lock = Lock()

        # Last commanded Cartesian pose.
        self._desired_pose = None

        # Fixed Work Home joint configuration.
        self.home_joints = WORK_HOME_JOINTS.copy()

        # Cartesian TCP pose corresponding to the Home joints.
        # It is captured after go_home() reaches the joint Home.
        self.home_pos = None

        # ---------------------------------------------------------
        # Joint-state subscriber
        # ---------------------------------------------------------

        self._joint_subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

        # ---------------------------------------------------------
        # TF listener for current end-effector pose
        # ---------------------------------------------------------

        self._tf_buffer = Buffer()

        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
        )

        # ---------------------------------------------------------
        # MoveIt service and action clients
        # ---------------------------------------------------------

        self._cartesian_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )

        self._fk_client = self.create_client(
            GetPositionFK,
            "/compute_fk",
        )

        self._execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory",
        )

        # Used only for moving to the fixed joint-space Home.
        self._joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/fr3_arm_controller/follow_joint_trajectory",
        )

        # ---------------------------------------------------------
        # Background ROS executor
        #
        # Do not use self.executor because Node already uses
        # that name internally.
        # ---------------------------------------------------------

        self._executor = MultiThreadedExecutor(
            num_threads=4
        )

        self._executor.add_node(self)

        self._executor_thread = Thread(
            target=self._executor.spin,
            daemon=True,
        )

        self._executor_thread.start()

        # Wait briefly for joint states.
        if not self._joint_ready.wait(timeout=3.0):
            self.get_logger().warning(
                f"No joint states received from "
                f"{self.joint_state_topic} within 3 seconds."
            )

        # Wait briefly for TF.
        try:
            self._lookup_ee_transform(timeout=3.0)

        except RuntimeError as error:
            self.get_logger().warning(str(error))

    # =============================================================
    # Internal utilities
    # =============================================================

    @staticmethod
    def _wait_for_future(future, timeout):
        """
        Wait for a ROS future while the background executor
        processes callbacks.
        """

        deadline = time.time() + timeout

        while rclpy.ok():
            if future.done():
                return future.result()

            if time.time() >= deadline:
                raise TimeoutError(
                    "ROS operation timed out."
                )

            time.sleep(0.01)

        raise RuntimeError(
            "ROS was shut down while waiting."
        )

    @staticmethod
    def _duration_from_seconds(seconds):
        """
        Convert floating-point seconds to builtin_interfaces/Duration.
        """

        seconds = float(seconds)

        if seconds <= 0.0:
            raise ValueError(
                "Duration must be greater than zero."
            )

        whole_seconds = int(seconds)
        nanoseconds = int(
            round(
                (seconds - whole_seconds)
                * 1_000_000_000
            )
        )

        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000

        return Duration(
            sec=whole_seconds,
            nanosec=nanoseconds,
        )

    def _joint_state_callback(self, msg):
        """
        Save the seven FR3 joint positions in JOINT_NAMES order.
        """

        joint_dictionary = dict(
            zip(msg.name, msg.position)
        )

        missing_joints = [
            name
            for name in JOINT_NAMES
            if name not in joint_dictionary
        ]

        if missing_joints:
            return

        with self._joint_lock:
            self._joint_pose = [
                float(joint_dictionary[name])
                for name in JOINT_NAMES
            ]

        self._joint_ready.set()

    def _lookup_ee_transform(self, timeout=2.0):
        """
        Get the current transform from the base frame to the TCP.
        """

        deadline = time.time() + timeout
        last_error = None

        while rclpy.ok() and time.time() < deadline:
            try:
                return self._tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_link,
                    Time(),
                )

            except TransformException as error:
                last_error = error
                time.sleep(0.05)

        raise RuntimeError(
            f"Cannot find TF transform from "
            f"'{self.base_frame}' to '{self.ee_link}'. "
            f"Last error: {last_error}"
        )

    @staticmethod
    def _pose_list_to_message(pose_list):
        """
        Convert:

            [x, y, z, rx, ry, rz]

        to geometry_msgs/Pose.
        """

        if len(pose_list) != 6:
            raise ValueError(
                "Pose must contain exactly six values: "
                "[x, y, z, rx, ry, rz]"
            )

        pose_message = Pose()

        pose_message.position.x = float(
            pose_list[0]
        )

        pose_message.position.y = float(
            pose_list[1]
        )

        pose_message.position.z = float(
            pose_list[2]
        )

        quaternion = Rotation.from_rotvec(
            [
                float(pose_list[3]),
                float(pose_list[4]),
                float(pose_list[5]),
            ]
        ).as_quat()

        pose_message.orientation.x = float(
            quaternion[0]
        )

        pose_message.orientation.y = float(
            quaternion[1]
        )

        pose_message.orientation.z = float(
            quaternion[2]
        )

        pose_message.orientation.w = float(
            quaternion[3]
        )

        return pose_message

    @staticmethod
    def _pose_message_to_list(pose_message):
        """
        Convert geometry_msgs/Pose to:

            [x, y, z, rx, ry, rz]
        """

        quaternion = [
            pose_message.orientation.x,
            pose_message.orientation.y,
            pose_message.orientation.z,
            pose_message.orientation.w,
        ]

        rotation_vector = Rotation.from_quat(
            quaternion
        ).as_rotvec()

        return [
            float(pose_message.position.x),
            float(pose_message.position.y),
            float(pose_message.position.z),
            float(rotation_vector[0]),
            float(rotation_vector[1]),
            float(rotation_vector[2]),
        ]

    # =============================================================
    # Read robot state
    # =============================================================

    def get_joint_pose(self):
        """
        Return the current seven joint positions in radians.
        """

        with self._joint_lock:
            return self._joint_pose.copy()

    def get_actual_pose(self):
        """
        Return the current TCP pose:

            [x, y, z, rx, ry, rz]
        """

        transform = self._lookup_ee_transform()

        translation = (
            transform.transform.translation
        )

        orientation = (
            transform.transform.rotation
        )

        rotation_vector = Rotation.from_quat(
            [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        ).as_rotvec()

        return [
            float(translation.x),
            float(translation.y),
            float(translation.z),
            float(rotation_vector[0]),
            float(rotation_vector[1]),
            float(rotation_vector[2]),
        ]

    def get_desired_pose(self):
        """
        Return the last commanded Cartesian target pose.
        """

        if self._desired_pose is None:
            return None

        return self._desired_pose.copy()

    def get_pos_error(self):
        """
        Return Cartesian position error:

            actual position - desired position
        """

        if self._desired_pose is None:
            return [0.0, 0.0, 0.0]

        actual_pose = self.get_actual_pose()

        return [
            actual_pose[0] - self._desired_pose[0],
            actual_pose[1] - self._desired_pose[1],
            actual_pose[2] - self._desired_pose[2],
        ]

    # =============================================================
    # Home utilities
    # =============================================================

    def get_home_joint_pose(self):
        """
        Return a copy of the fixed seven-joint Work Home.
        """

        return self.home_joints.copy()

    def get_home_pose(self):
        """
        Return the Cartesian TCP pose corresponding to Work Home.

        After go_home(), this is the measured actual TCP pose.
        Before go_home(), MoveIt forward kinematics is used.
        """

        if self.home_pos is None:
            return self.joint_to_cart(
                self.home_joints
            )

        return self.home_pos[0].copy()

    def set_home_from_current(self):
        """
        Save the current seven arm joints as the new Work Home.
        """

        self.home_joints = self.get_joint_pose()

        current_pose = self.get_actual_pose()

        self.home_pos = [
            current_pose.copy()
        ]

        print("FR3 Work Home joints set to:")
        print(self.home_joints)

        print("FR3 Work Home TCP pose:")
        print(current_pose)

    def set_home(self, joint_positions):
        """
        Set the Work Home using seven joint values in radians.
        """

        if len(joint_positions) != 7:
            raise ValueError(
                "Home joint position must contain exactly "
                "seven values."
            )

        self.home_joints = [
            float(value)
            for value in joint_positions
        ]

        # Recalculate the Cartesian Home pose when requested.
        self.home_pos = None

    def create_pose_from_home(self, x, y, z):
        """
        Create a Cartesian target using the requested XYZ and the
        TCP orientation corresponding to the joint-space Home.

        Only XYZ changes.
        """

        home_pose = self.get_home_pose()

        return [
            float(x),
            float(y),
            float(z),
            home_pose[3],
            home_pose[4],
            home_pose[5],
        ]

    def execute_joint_trajectory(
        self,
        joint_positions,
        duration=5.0,
        start_delay=0.1,
    ):
        """
        Move the seven FR3 joints using a smooth quintic trajectory.

        Both trajectory points contain position, velocity, and
        acceleration. The controller therefore uses quintic spline
        interpolation instead of position-only linear interpolation.
        """

        if len(joint_positions) != 7:
            raise ValueError(
                "Joint target must contain exactly seven values."
            )

        if duration <= 0.0:
            raise ValueError(
                "Joint trajectory duration must be greater than zero."
            )

        if start_delay <= 0.0:
            raise ValueError(
                "Joint trajectory start_delay must be greater than zero."
            )

        action_available = (
            self._joint_trajectory_client.wait_for_server(
                timeout_sec=5.0
            )
        )

        if not action_available:
            self.get_logger().error(
                "/fr3_arm_controller/follow_joint_trajectory "
                "is unavailable."
            )
            return False

        current_joints = self.get_joint_pose()
        zero_values = [0.0] * 7

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES.copy()

        start_point = JointTrajectoryPoint()
        start_point.positions = current_joints
        start_point.velocities = zero_values.copy()
        start_point.accelerations = zero_values.copy()
        start_point.time_from_start = self._duration_from_seconds(
            start_delay
        )

        target_point = JointTrajectoryPoint()
        target_point.positions = [
            float(value)
            for value in joint_positions
        ]
        target_point.velocities = zero_values.copy()
        target_point.accelerations = zero_values.copy()
        target_point.time_from_start = self._duration_from_seconds(
            start_delay + duration
        )

        goal.trajectory.points = [
            start_point,
            target_point,
        ]

        self.get_logger().info(
            f"Sending smooth joint-space Home trajectory. "
            f"Movement duration: {duration:.2f} seconds."
        )

        send_goal_future = (
            self._joint_trajectory_client.send_goal_async(
                goal
            )
        )

        try:
            goal_handle = self._wait_for_future(
                send_goal_future,
                timeout=10.0,
            )

        except Exception as error:
            self.get_logger().error(
                f"Could not send Home joint trajectory: "
                f"{error}"
            )
            return False

        if goal_handle is None:
            self.get_logger().error(
                "No Home trajectory goal handle was returned."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                "Home joint trajectory was rejected."
            )
            return False

        self.get_logger().info(
            "Home joint trajectory accepted. "
            "Robot is moving..."
        )

        result_future = goal_handle.get_result_async()
        total_duration = start_delay + duration

        try:
            wrapped_result = self._wait_for_future(
                result_future,
                timeout=max(30.0, total_duration + 15.0),
            )

        except Exception as error:
            self.get_logger().error(
                f"Home joint trajectory failed: {error}"
            )
            return False

        if wrapped_result is None:
            self.get_logger().error(
                "No Home trajectory result was returned."
            )
            return False

        result = wrapped_result.result

        if result.error_code != 0:
            self.get_logger().error(
                f"Home trajectory failed with code "
                f"{result.error_code}: "
                f"{result.error_string}"
            )
            return False

        self.get_logger().info(
            "Joint-space Home movement completed."
        )

        return True

    def _wait_until_joints_settle(
        self,
        timeout=2.0,
        sample_period=0.05,
        movement_tolerance=0.001,
        required_stable_samples=6,
    ):
        """Wait until consecutive joint-state samples stop changing."""

        deadline = time.time() + timeout
        previous_joints = self.get_joint_pose()
        stable_samples = 0

        while rclpy.ok() and time.time() < deadline:
            time.sleep(sample_period)
            current_joints = self.get_joint_pose()

            largest_sample_change = max(
                abs(current - previous)
                for current, previous in zip(
                    current_joints,
                    previous_joints,
                )
            )

            if largest_sample_change <= movement_tolerance:
                stable_samples += 1

                if stable_samples >= required_stable_samples:
                    return True
            else:
                stable_samples = 0

            previous_joints = current_joints

        self.get_logger().warning(
            "Robot did not satisfy the joint settling condition "
            f"within {timeout:.1f} seconds."
        )
        return False

    def go_home(
        self,
        max_joint_speed=0.80,
        max_joint_acceleration=0.80,
        minimum_duration=2.0,
        home_tolerance=0.1,
        settling_timeout=2.0,
    ):
        """
        Move smoothly to the fixed Work Home joint configuration.

        The movement time is calculated from the largest joint
        displacement and conservative speed/acceleration limits.
        A quintic trajectory gives zero velocity and acceleration at
        both the beginning and the end of the movement.
        """

        if max_joint_speed <= 0.0:
            raise ValueError(
                "max_joint_speed must be greater than zero."
            )

        if max_joint_acceleration <= 0.0:
            raise ValueError(
                "max_joint_acceleration must be greater than zero."
            )

        if minimum_duration <= 0.0:
            raise ValueError(
                "minimum_duration must be greater than zero."
            )

        current_joints = self.get_joint_pose()

        joint_differences = [
            abs(home - current)
            for current, home in zip(
                current_joints,
                self.home_joints,
            )
        ]

        largest_joint_difference = max(joint_differences)

        if largest_joint_difference <= home_tolerance:
            self.get_logger().info(
                "Robot is already within the Work Home tolerance."
            )
        else:
            # Quintic smoothstep peak factors:
            # max velocity     = 1.875 * distance / duration
            # max acceleration = 5.774 * distance / duration^2
            duration_from_speed = (
                1.875
                * largest_joint_difference
                / max_joint_speed
            )

            duration_from_acceleration = (
                5.774
                * largest_joint_difference
                / max_joint_acceleration
            ) ** 0.5

            duration = max(
                minimum_duration,
                duration_from_speed,
                duration_from_acceleration,
            )

            self.get_logger().info(
                f"Largest Home joint difference: "
                f"{largest_joint_difference:.4f} rad. "
                f"Using smooth duration: {duration:.2f} seconds."
            )

            success = self.execute_joint_trajectory(
                self.home_joints,
                duration=duration,
            )

            if not success:
                return False

        self._wait_until_joints_settle(
            timeout=settling_timeout
        )

        # Extra pause before force-baseline collection or handover logic.
        # time.sleep(0.3)

        actual_home_pose = self.get_actual_pose()
        self.home_pos = [actual_home_pose.copy()]

        final_joints = self.get_joint_pose()
        final_joint_error = max(
            abs(home - current)
            for current, home in zip(
                final_joints,
                self.home_joints,
            )
        )

        self.get_logger().info(
            f"Work Home reached. Maximum joint error: "
            f"{final_joint_error:.6f} rad."
        )

        self.get_logger().info(
            f"Measured Work Home TCP pose: "
            f"{actual_home_pose}"
        )

        return final_joint_error <= home_tolerance

    def move_to_xyz(
        self,
        x,
        y,
        z,
        max_step=0.02,
        max_cartesian_speed=0.02,
        avoid_collisions=True,
        minimum_fraction=0.99,
    ):
        """
        Move the TCP to XYZ while preserving the orientation
        corresponding to the joint-space Work Home.
        """

        target_pose = self.create_pose_from_home(
            x=x,
            y=y,
            z=z,
        )

        self.get_logger().info(
            f"Moving to XYZ: "
            f"x={target_pose[0]:.4f}, "
            f"y={target_pose[1]:.4f}, "
            f"z={target_pose[2]:.4f}"
        )

        return self.execute_cartesian_trajectory(
            [target_pose],
            max_step=max_step,
            max_cartesian_speed=max_cartesian_speed,
            avoid_collisions=avoid_collisions,
            minimum_fraction=minimum_fraction,
        )

    # =============================================================
    # Cartesian movement
    # =============================================================

    def execute_cartesian_trajectory(
        self,
        pose_list,
        max_step=0.02,
        max_cartesian_speed=0.03,
        avoid_collisions=True,
        minimum_fraction=0.99,
    ):
        """
        Plan and execute a Cartesian trajectory.

        pose_list example:

            [
                [x1, y1, z1, rx1, ry1, rz1],
                [x2, y2, z2, rx2, ry2, rz2],
            ]
        """

        if not pose_list:
            self.get_logger().error(
                "Cartesian pose list is empty."
            )
            return False

        try:
            waypoints = [
                self._pose_list_to_message(pose)
                for pose in pose_list
            ]

        except (ValueError, TypeError) as error:
            self.get_logger().error(
                f"Invalid Cartesian pose: {error}"
            )
            return False

        service_available = (
            self._cartesian_client.wait_for_service(
                timeout_sec=5.0
            )
        )

        if not service_available:
            self.get_logger().error(
                "/compute_cartesian_path is unavailable."
            )
            return False

        request = GetCartesianPath.Request()

        request.header.frame_id = self.base_frame

        request.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Use MoveIt's currently monitored robot state.
        request.start_state.is_diff = True

        request.group_name = self.planning_group
        request.link_name = self.ee_link

        request.waypoints = waypoints
        request.max_step = float(max_step)
        request.jump_threshold = 0.0

        request.avoid_collisions = bool(
            avoid_collisions
        )

        # These fields depend on the installed MoveIt version.
        if hasattr(
            request,
            "prismatic_jump_threshold",
        ):
            request.prismatic_jump_threshold = 0.0

        if hasattr(
            request,
            "revolute_jump_threshold",
        ):
            request.revolute_jump_threshold = 0.0

        if (
            hasattr(
                request,
                "cartesian_speed_limited_link",
            )
            and hasattr(
                request,
                "max_cartesian_speed",
            )
        ):
            request.cartesian_speed_limited_link = (
                self.ee_link
            )

            request.max_cartesian_speed = float(
                max_cartesian_speed
            )

        self.get_logger().info(
            f"Computing Cartesian path with "
            f"{len(waypoints)} waypoint(s)..."
        )

        future = self._cartesian_client.call_async(
            request
        )

        try:
            response = self._wait_for_future(
                future,
                timeout=30.0,
            )

        except TimeoutError:
            self.get_logger().error(
                "Cartesian planning timed out."
            )
            return False

        except Exception as error:
            self.get_logger().error(
                f"Cartesian planning service failed: "
                f"{error}"
            )
            return False

        if response is None:
            self.get_logger().error(
                "No response from "
                "/compute_cartesian_path."
            )
            return False

        percentage = response.fraction * 100.0

        self.get_logger().info(
            f"Cartesian path fraction: "
            f"{percentage:.1f}%"
        )

        if (
            response.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):
            self.get_logger().error(
                f"Cartesian planning failed with "
                f"MoveIt error code "
                f"{response.error_code.val}."
            )
            return False

        if response.fraction < minimum_fraction:
            self.get_logger().error(
                f"Only {percentage:.1f}% of the "
                f"Cartesian path was planned."
            )
            return False

        self._desired_pose = [
            float(value)
            for value in pose_list[-1]
        ]

        return self._execute_trajectory(
            response.solution
        )

    def _execute_trajectory(self, trajectory):
        """
        Send a planned MoveIt trajectory for execution.
        """

        action_available = (
            self._execute_client.wait_for_server(
                timeout_sec=5.0
            )
        )

        if not action_available:
            self.get_logger().error(
                "/execute_trajectory is unavailable."
            )
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            "Sending trajectory for execution..."
        )

        send_goal_future = (
            self._execute_client.send_goal_async(
                goal
            )
        )

        try:
            goal_handle = self._wait_for_future(
                send_goal_future,
                timeout=10.0,
            )

        except TimeoutError:
            self.get_logger().error(
                "Sending trajectory timed out."
            )
            return False

        except Exception as error:
            self.get_logger().error(
                f"Could not send trajectory: "
                f"{error}"
            )
            return False

        if goal_handle is None:
            self.get_logger().error(
                "No goal handle was returned."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                "Trajectory was rejected."
            )
            return False

        self.get_logger().info(
            "Trajectory accepted. Robot is moving..."
        )

        result_future = (
            goal_handle.get_result_async()
        )

        try:
            wrapped_result = self._wait_for_future(
                result_future,
                timeout=120.0,
            )

        except TimeoutError:
            self.get_logger().error(
                "Trajectory execution timed out."
            )
            return False

        except Exception as error:
            self.get_logger().error(
                f"Trajectory execution failed: "
                f"{error}"
            )
            return False

        if wrapped_result is None:
            self.get_logger().error(
                "No trajectory result was returned."
            )
            return False

        result = wrapped_result.result

        if (
            result.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):
            self.get_logger().error(
                f"Trajectory execution failed with "
                f"MoveIt error code "
                f"{result.error_code.val}."
            )
            return False

        self.get_logger().info(
            "Cartesian movement completed."
        )

        return True

    # =============================================================
    # Forward kinematics
    # =============================================================

    def joint_to_cart(self, joint_pose):
        """
        Convert seven FR3 joint positions to:

            [x, y, z, rx, ry, rz]

        using MoveIt's /compute_fk service.
        """

        if len(joint_pose) != 7:
            raise ValueError(
                "FR3 joint pose must contain "
                "exactly seven values."
            )

        service_available = (
            self._fk_client.wait_for_service(
                timeout_sec=5.0
            )
        )

        if not service_available:
            raise RuntimeError(
                "/compute_fk is unavailable."
            )

        request = GetPositionFK.Request()

        request.header.frame_id = self.base_frame

        request.header.stamp = (
            self.get_clock().now().to_msg()
        )

        request.fk_link_names = [
            self.ee_link
        ]

        request.robot_state.joint_state.name = (
            JOINT_NAMES.copy()
        )

        request.robot_state.joint_state.position = [
            float(value)
            for value in joint_pose
        ]

        future = self._fk_client.call_async(
            request
        )

        response = self._wait_for_future(
            future,
            timeout=10.0,
        )

        if response is None:
            raise RuntimeError(
                "No response from /compute_fk."
            )

        if (
            response.error_code.val
            != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f"Forward kinematics failed with "
                f"MoveIt error code "
                f"{response.error_code.val}."
            )

        if not response.pose_stamped:
            raise RuntimeError(
                "Forward kinematics returned no pose."
            )

        return self._pose_message_to_list(
            response.pose_stamped[0].pose
        )

    # =============================================================
    # Shutdown
    # =============================================================

    def shutdown(self):
        """
        Stop callbacks before destroying ROS resources.

        When using the gripper and arm together, stop the gripper
        first:

            gripper.shutdown_gripper(shutdown_ros=False)
            robot.shutdown()
        """

        # Allow final action callbacks to complete.
        time.sleep(0.1)

        # Stop the executor before destroying the node.
        try:
            executor_stopped = self._executor.shutdown(
                timeout_sec=5.0
            )

            if executor_stopped is False:
                self.get_logger().warning(
                    "Executor did not stop within 5 seconds."
                )

        except Exception as error:
            print(
                f"Executor shutdown warning: {error}"
            )

        # Wait for the executor thread.
        try:
            if self._executor_thread.is_alive():
                self._executor_thread.join(
                    timeout=5.0
                )

        except Exception as error:
            print(
                f"Executor thread warning: {error}"
            )

        # Destroy the node after the executor has stopped.
        try:
            self.destroy_node()

        except Exception as error:
            print(
                f"Node destruction warning: {error}"
            )

        # Shut down the shared ROS context last.
        try:
            if rclpy.ok():
                rclpy.shutdown()

        except Exception as error:
            print(
                f"ROS shutdown warning: {error}"
            )


# =============================================================
# Process-wide singleton
# =============================================================

_robot_singleton = None


def get_robot():
    """
    Return one shared FR3Robot object.
    """

    global _robot_singleton

    if _robot_singleton is None:
        _robot_singleton = FR3Robot()

    return _robot_singleton
