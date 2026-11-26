#!/usr/bin/env python3

import time
from threading import Event, Lock, Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState

from franka_msgs.action import Grasp
from franka_msgs.action import Homing
from franka_msgs.action import Move


class FrankaGripper(Node):
    """
    Simple controller for the Franka Hand.

    Main functions:

        gripper.home()
        gripper.open()
        gripper.close()
        gripper.close_with_force(20.0)
        gripper.grasp(width=0.03, force=20.0)

        gripper.get_width()
        gripper.gripper_is_open()
        gripper.gripper_is_closed()
    """

    def __init__(self, action_prefix=None):
        if not rclpy.ok():
            rclpy.init()

        super().__init__("fr3_gripper_client")

        self._current_width = None
        self._last_success = None
        self._last_error = ""

        self._width_lock = Lock()
        self._width_ready = Event()

        # Different franka_ros2 versions may use either name.
        if action_prefix is None:
            self._candidate_prefixes = [
                "/fr3_gripper",
                "/franka_gripper",
            ]
        else:
            self._candidate_prefixes = [
                action_prefix.rstrip("/")
            ]

        self._action_prefix = None
        self._action_clients = {}

        for prefix in self._candidate_prefixes:
            self._action_clients[prefix] = {
                "homing": ActionClient(
                    self,
                    Homing,
                    f"{prefix}/homing",
                ),
                "move": ActionClient(
                    self,
                    Move,
                    f"{prefix}/move",
                ),
                "grasp": ActionClient(
                    self,
                    Grasp,
                    f"{prefix}/grasp",
                ),
            }

        # Listen to both possible joint-state topic names.
        self._joint_subscriptions = []

        for topic_name in [
            "/fr3_gripper/joint_states",
            "/franka_gripper/joint_states",
        ]:
            subscription = self.create_subscription(
                JointState,
                topic_name,
                self._joint_state_callback,
                qos_profile_sensor_data,
            )

            self._joint_subscriptions.append(
                subscription
            )

        # Use _executor, not executor, because Node already uses that name.
        self._executor = MultiThreadedExecutor(
            num_threads=3
        )

        self._executor.add_node(self)

        self._executor_thread = Thread(
            target=self._executor.spin,
            daemon=True,
        )

        self._executor_thread.start()

    # ============================================================
    # Internal functions
    # ============================================================

    @staticmethod
    def _wait_for_future(future, timeout):
        deadline = time.time() + timeout

        while rclpy.ok():
            if future.done():
                return future.result()

            if time.time() >= deadline:
                raise TimeoutError(
                    "ROS action timed out."
                )

            # time.sleep(0.01)

        raise RuntimeError(
            "ROS was shut down."
        )

    def _joint_state_callback(self, msg):
        finger_positions = []

        for name, position in zip(
            msg.name,
            msg.position,
        ):
            if "finger_joint" in name:
                finger_positions.append(
                    abs(float(position))
                )

        if len(finger_positions) >= 2:
            width = (
                finger_positions[0]
                + finger_positions[1]
            )

        elif len(finger_positions) == 1:
            width = 2.0 * finger_positions[0]

        else:
            return

        with self._width_lock:
            self._current_width = width

        self._width_ready.set()

    def _feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback

        if hasattr(feedback, "current_width"):
            with self._width_lock:
                self._current_width = float(
                    feedback.current_width
                )

            self._width_ready.set()

    def _find_action_prefix(self, timeout=5.0):
        if self._action_prefix is not None:
            clients = self._action_clients[
                self._action_prefix
            ]

            if clients["move"].wait_for_server(
                timeout_sec=0.1
            ):
                return self._action_prefix

            self._action_prefix = None

        deadline = time.time() + timeout

        while rclpy.ok() and time.time() < deadline:
            for prefix in self._candidate_prefixes:
                clients = self._action_clients[prefix]

                if clients["move"].wait_for_server(
                    timeout_sec=0.2
                ):
                    self._action_prefix = prefix

                    self.get_logger().info(
                        f"Using gripper actions at "
                        f"{prefix}"
                    )

                    return prefix

        names = ", ".join(
            self._candidate_prefixes
        )

        raise RuntimeError(
            "Cannot find a Franka gripper action "
            f"server. Checked: {names}"
        )

    def _run_action(
        self,
        action_name,
        goal,
        timeout=60.0,
    ):
        prefix = self._find_action_prefix()

        client = self._action_clients[
            prefix
        ][action_name]

        if not client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                f"{prefix}/{action_name} is unavailable."
            )
            return False

        self.get_logger().info(
            f"Sending gripper {action_name} command..."
        )

        send_future = client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )

        try:
            goal_handle = self._wait_for_future(
                send_future,
                timeout=10.0,
            )

        except TimeoutError:
            self.get_logger().error(
                "Sending gripper goal timed out."
            )
            return False

        if goal_handle is None:
            self.get_logger().error(
                "No goal handle was returned."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                "Gripper command was rejected."
            )
            return False

        result_future = (
            goal_handle.get_result_async()
        )

        try:
            wrapped_result = self._wait_for_future(
                result_future,
                timeout=timeout,
            )

        except TimeoutError:
            self.get_logger().error(
                "Gripper operation timed out."
            )
            return False

        if wrapped_result is None:
            self.get_logger().error(
                "No gripper result was returned."
            )
            return False

        result = wrapped_result.result

        self._last_success = bool(
            result.success
        )

        self._last_error = str(
            result.error
        )

        if not result.success:
            self.get_logger().error(
                f"Gripper command failed: "
                f"{result.error}"
            )
            return False

        self.get_logger().info(
            f"Gripper {action_name} completed."
        )

        return True

    # ============================================================
    # Gripper commands
    # ============================================================

    def home(self):
        """
        Home/calibrate the gripper.

        This normally opens the fingers and determines the
        maximum available opening width.
        """

        goal = Homing.Goal()

        return self._run_action(
            "homing",
            goal,
            timeout=30.0,
        )

    def open(
        self,
        width=0.075,
        speed=0.05,
    ):
        """
        Open or move the gripper to a specified width.

        width:
            Total distance between the two fingers in metres.

        speed:
            Finger movement speed in metres per second.
        """

        if width < 0.0 or width > 0.075:
            raise ValueError(
                "Width must be between 0.0 and "
                "0.075 metres."
            )

        if speed <= 0.0:
            raise ValueError(
                "Speed must be greater than zero."
            )

        goal = Move.Goal()

        goal.width = float(width)
        goal.speed = float(speed)

        return self._run_action(
            "move",
            goal,
        )

    def move(
        self,
        width,
        speed=0.05,
    ):
        """
        Move to any requested finger width.
        """

        return self.open(
            width=width,
            speed=speed,
        )

    def grasp(
        self,
        width,
        force=20.0,
        speed=0.05,
        epsilon_inner=0.005,
        epsilon_outer=0.005,
    ):
        """
        Grasp an object with an expected object width.

        Example for an approximately 30 mm object:

            gripper.grasp(
                width=0.03,
                force=20.0,
            )
        """

        if width < 0.0 or width > 0.075:
            raise ValueError(
                "Width must be between 0.0 and "
                "0.075 metres."
            )

        if speed <= 0.0:
            raise ValueError(
                "Speed must be greater than zero."
            )

        if force <= 0.0:
            raise ValueError(
                "Force must be greater than zero."
            )

        goal = Grasp.Goal()

        goal.width = float(width)
        goal.speed = float(speed)
        goal.force = float(force)

        goal.epsilon.inner = float(
            epsilon_inner
        )

        goal.epsilon.outer = float(
            epsilon_outer
        )

        return self._run_action(
            "grasp",
            goal,
        )

    def close(
        self,
        force=60.0,
        speed=0.05,
    ):
        """
        Fully close the gripper.

        A large outer tolerance is used so this command can also
        report success when an object prevents complete closure.
        """

        return self.grasp(
            width=0.0,
            force=force,
            speed=speed,
            epsilon_inner=0.005,
            epsilon_outer=0.075,
        )

    def close_with_force(
        self,
        force,
        speed=0.03,
    ):
        """
        Close the gripper using a specified force.
        """

        return self.close(
            force=float(force),
            speed=speed,
        )

    # ============================================================
    # Gripper state
    # ============================================================

    def get_width(self, timeout=1.0):
        """
        Return the current total finger opening in metres.
        """

        self._width_ready.wait(timeout=timeout)

        with self._width_lock:
            return self._current_width

    def gripper_is_open(
        self,
        tolerance=0.005,
    ):
        width = self.get_width()

        if width is None:
            return False

        return width >= (0.075 - tolerance)

    def gripper_is_closed(
        self,
        tolerance=0.01,
    ):
        width = self.get_width()

        if width is None:
            return False

        return width <= tolerance

    def get_stat(self):
        """
        Return basic gripper status information.
        """

        width = self.get_width()

        return {
            "width": width,
            "is_open": (
                width is not None
                and width >= 0.075
            ),
            "is_closed": (
                width is not None
                and width <= 0.01
            ),
            "last_success": self._last_success,
            "last_error": self._last_error,
            "action_prefix": self._action_prefix,
        }

    # ============================================================
    # Shutdown
    # ============================================================

    def shutdown_gripper(
        self,
        shutdown_ros=False,
    ):
        """
        Shut down this gripper client.

        When using FR3Robot and FrankaGripper together:

            gripper.shutdown_gripper(False)
            robot.shutdown()

        For a gripper-only script:

            gripper.shutdown_gripper(True)
        """

        # time.sleep(0.01)

        try:
            self._executor.shutdown(
                timeout_sec=5.0
            )
        except Exception:
            pass

        try:
            if self._executor_thread.is_alive():
                self._executor_thread.join(
                    timeout=5.0
                )
        except Exception:
            pass

        try:
            self.destroy_node()
        except Exception:
            pass

        if shutdown_ros and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

        print("Franka gripper shutdown.")


_gripper_singleton = None


def get_gripper(action_prefix=None):
    """
    Return one process-wide FrankaGripper object.
    """

    global _gripper_singleton

    if _gripper_singleton is None:
        _gripper_singleton = FrankaGripper(
            action_prefix=action_prefix
        )

    return _gripper_singleton
