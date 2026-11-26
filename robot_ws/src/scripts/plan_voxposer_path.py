#!/usr/bin/env python3
"""Validate a saved VoxPoser TCP path and optionally execute it in isolated Gazebo."""

import argparse
import copy
import math
import os
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node


def rotation_matrix_to_quaternion(matrix):
    """Convert a 3x3 rotation matrix to ROS quaternion order (x, y, z, w)."""

    matrix = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


def wait_for_future(node, future, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() >= deadline:
            raise TimeoutError("ROS service call timed out")
    return future.result()


class MoveItPlanOnlyValidator(Node):
    def __init__(self):
        super().__init__("voxposer_moveit_plan_only")
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")

    def validate(self, path_world, ee_transform, max_step):
        if not self.cartesian_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/compute_cartesian_path is unavailable")
        if not self.validity_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/check_state_validity is unavailable")
        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/compute_fk is unavailable")

        quaternion = rotation_matrix_to_quaternion(ee_transform[:3, :3])
        waypoints = []
        for xyz in path_world[1:]:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) = map(float, quaternion)
            waypoints.append(pose)

        request = GetCartesianPath.Request()
        request.header.frame_id = "fr3_link0"
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = "fr3_arm"
        request.link_name = "fr3_hand_tcp"
        request.waypoints = waypoints
        request.max_step = float(max_step)
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        if hasattr(request, "prismatic_jump_threshold"):
            request.prismatic_jump_threshold = 0.0
        if hasattr(request, "revolute_jump_threshold"):
            request.revolute_jump_threshold = 0.0

        response = wait_for_future(
            self, self.cartesian_client.call_async(request), timeout_sec=45.0
        )
        if response is None:
            raise RuntimeError("MoveIt returned no Cartesian-path response")

        trajectory = response.solution.joint_trajectory
        joint_names = list(trajectory.joint_names)
        invalid_states = 0
        invalid_contacts = []
        for index, point in enumerate(trajectory.points):
            validity_request = GetStateValidity.Request()
            validity_request.group_name = "fr3_arm"
            validity_request.robot_state = RobotState()
            validity_request.robot_state.joint_state.name = joint_names
            validity_request.robot_state.joint_state.position = list(point.positions)
            validity_response = wait_for_future(
                self,
                self.validity_client.call_async(validity_request),
                timeout_sec=5.0,
            )
            if validity_response is None or not validity_response.valid:
                invalid_states += 1
                if validity_response is not None:
                    invalid_contacts.extend(
                        f"{contact.contact_body_1}<->{contact.contact_body_2}"
                        for contact in validity_response.contacts
                    )

        positions = np.asarray(
            [point.positions for point in trajectory.points], dtype=np.float64
        )
        velocities = np.asarray(
            [point.velocities for point in trajectory.points], dtype=np.float64
        ) if trajectory.points and trajectory.points[0].velocities else np.empty((0, 0))
        times = np.asarray([
            point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            for point in trajectory.points
        ])
        max_joint_step = (
            float(np.max(np.abs(np.diff(positions, axis=0))))
            if len(positions) > 1 else 0.0
        )
        finite = bool(
            np.all(np.isfinite(positions))
            and np.all(np.isfinite(velocities))
            and np.all(np.isfinite(times))
        )
        strictly_timed = bool(len(times) < 2 or np.all(np.diff(times) > 0.0))
        endpoint_position_error_m = float("inf")
        endpoint_orientation_error_rad = float("inf")
        if trajectory.points:
            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = "fr3_link0"
            fk_request.header.stamp = self.get_clock().now().to_msg()
            fk_request.fk_link_names = ["fr3_hand_tcp"]
            fk_request.robot_state = RobotState()
            fk_request.robot_state.joint_state.name = joint_names
            fk_request.robot_state.joint_state.position = list(
                trajectory.points[-1].positions
            )
            fk_response = wait_for_future(
                self, self.fk_client.call_async(fk_request), timeout_sec=5.0
            )
            if (
                fk_response is not None
                and fk_response.error_code.val == MoveItErrorCodes.SUCCESS
                and fk_response.pose_stamped
            ):
                final_pose = fk_response.pose_stamped[0].pose
                fk_position = np.asarray([
                    final_pose.position.x,
                    final_pose.position.y,
                    final_pose.position.z,
                ])
                endpoint_position_error_m = float(
                    np.linalg.norm(fk_position - path_world[-1])
                )
                fk_quaternion = np.asarray([
                    final_pose.orientation.x,
                    final_pose.orientation.y,
                    final_pose.orientation.z,
                    final_pose.orientation.w,
                ])
                dot = float(np.clip(abs(np.dot(fk_quaternion, quaternion)), 0.0, 1.0))
                endpoint_orientation_error_rad = float(2.0 * math.acos(dot))
        return {
            "error_code": int(response.error_code.val),
            "fraction": float(response.fraction),
            "joint_names": joint_names,
            "trajectory_points": len(trajectory.points),
            "invalid_states": invalid_states,
            "invalid_contacts": sorted(set(invalid_contacts)),
            "finite": finite,
            "strictly_timed": strictly_timed,
            "duration_sec": float(times[-1]) if len(times) else 0.0,
            "max_joint_step_rad": max_joint_step,
            "endpoint_position_error_m": endpoint_position_error_m,
            "endpoint_orientation_error_rad": endpoint_orientation_error_rad,
            "trajectory": response.solution,
        }

    def execute_in_isolated_simulation(
        self, trajectory, expected_domain_id, time_scale
    ):
        self.assert_isolated_simulation(expected_domain_id)
        if time_scale < 1.0:
            raise ValueError("time_scale must be at least 1.0")

        slowed = copy.deepcopy(trajectory)
        for point in slowed.joint_trajectory.points:
            seconds = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            scaled_nanoseconds = int(round(seconds * time_scale * 1e9))
            point.time_from_start = Duration(
                sec=scaled_nanoseconds // 1_000_000_000,
                nanosec=scaled_nanoseconds % 1_000_000_000,
            )
            if point.velocities:
                point.velocities = [value / time_scale for value in point.velocities]
            if point.accelerations:
                point.accelerations = [
                    value / (time_scale * time_scale)
                    for value in point.accelerations
                ]

        execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        if not execute_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/execute_trajectory is unavailable")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = slowed
        goal_handle = wait_for_future(
            self, execute_client.send_goal_async(goal), timeout_sec=10.0
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Simulation trajectory goal was rejected")
        wrapped_result = wait_for_future(
            self, goal_handle.get_result_async(), timeout_sec=120.0
        )
        if wrapped_result is None:
            raise RuntimeError("Simulation execution returned no result")
        return {
            "status": int(wrapped_result.status),
            "error_code": int(wrapped_result.result.error_code.val),
            "time_scale": float(time_scale),
            "success": (
                wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                and wrapped_result.result.error_code.val == MoveItErrorCodes.SUCCESS
            ),
        }

    def assert_isolated_simulation(self, expected_domain_id):
        """Reject all actuation unless this node can prove it is in Gazebo."""

        actual_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        if actual_domain_id != expected_domain_id:
            raise RuntimeError(
                f"Simulation execution requires ROS_DOMAIN_ID={expected_domain_id}; "
                f"current value is {actual_domain_id}"
            )
        services = {name for name, _ in self.get_service_names_and_types()}
        gazebo_markers = {
            "/world/fr3_pick_place/control",
            "/world/fr3_pick_place/set_pose",
        }
        if not services.intersection(gazebo_markers):
            raise RuntimeError("Gazebo world service marker was not found")
        topics = {name for name, _ in self.get_topic_names_and_types()}
        if "/clock" not in topics:
            raise RuntimeError("Simulation /clock topic was not found")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--max-step", type=float, default=0.01)
    parser.add_argument("--minimum-fraction", type=float, default=0.99)
    parser.add_argument("--execute-simulation", action="store_true")
    parser.add_argument("--simulation-domain-id", type=int, default=42)
    parser.add_argument("--time-scale", type=float, default=3.0)
    args = parser.parse_args()

    data = np.load(args.input)
    path_world = np.asarray(data["path_world"], dtype=np.float64)
    ee_transform = np.asarray(data["ee_transform"], dtype=np.float64)
    if path_world.ndim != 2 or path_world.shape[1] != 3 or len(path_world) < 2:
        raise ValueError("path_world must have shape (N, 3) with N >= 2")
    if ee_transform.shape != (4, 4):
        raise ValueError("ee_transform must have shape (4, 4)")

    rclpy.init()
    node = MoveItPlanOnlyValidator()
    execution_result = None
    try:
        result = node.validate(path_world, ee_transform, args.max_step)
        success = (
            result["error_code"] == MoveItErrorCodes.SUCCESS
            and result["fraction"] >= args.minimum_fraction
            and result["trajectory_points"] > 1
            and result["invalid_states"] == 0
            and result["finite"]
            and result["strictly_timed"]
            and result["endpoint_position_error_m"] <= 0.002
            and result["endpoint_orientation_error_rad"] <= 0.01
        )
        print(
            "mode: SIMULATION EXECUTION (guarded)"
            if args.execute_simulation
            else "mode: PLAN ONLY (no execution action client created)"
        )
        print(f"MoveIt error code: {result['error_code']}")
        print(f"Cartesian fraction: {result['fraction']:.6f}")
        print(f"trajectory joints: {', '.join(result['joint_names'])}")
        print(f"trajectory points: {result['trajectory_points']}")
        print(f"duration_sec: {result['duration_sec']:.6f}")
        print(f"max_joint_step_rad: {result['max_joint_step_rad']:.6f}")
        print(
            f"endpoint_position_error_m: "
            f"{result['endpoint_position_error_m']:.9f}"
        )
        print(
            f"endpoint_orientation_error_rad: "
            f"{result['endpoint_orientation_error_rad']:.9f}"
        )
        print(f"finite trajectory: {result['finite']}")
        print(f"strictly increasing timestamps: {result['strictly_timed']}")
        print(f"invalid states: {result['invalid_states']}")
        if result["invalid_contacts"]:
            print(f"collision contacts: {', '.join(result['invalid_contacts'])}")
        print(f"plan-only validation: {'PASS' if success else 'FAIL'}")
        if args.execute_simulation:
            if not success:
                raise RuntimeError("Refusing execution because plan-only validation failed")
            execution_result = node.execute_in_isolated_simulation(
                result["trajectory"],
                expected_domain_id=args.simulation_domain_id,
                time_scale=args.time_scale,
            )
            print(f"simulation domain: {args.simulation_domain_id}")
            print(f"execution time scale: {execution_result['time_scale']:.1f}x")
            print(f"execution status: {execution_result['status']}")
            print(f"execution error code: {execution_result['error_code']}")
            print(
                f"simulation execution: "
                f"{'PASS' if execution_result['success'] else 'FAIL'}"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not success:
        return 2
    if args.execute_simulation and not execution_result["success"]:
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
