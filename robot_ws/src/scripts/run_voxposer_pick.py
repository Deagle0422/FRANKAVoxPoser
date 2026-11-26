#!/usr/bin/env python3
"""Run VoxPoser pick stages through the public FR3Robot/FrankaGripper API."""

import argparse
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node

from basic_function.fr3_gripper import get_gripper
from basic_function.fr3_robot import get_robot


def assert_isolated_simulation(expected_domain_id):
    """Prove that this process is in the requested Gazebo ROS domain."""

    actual_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if actual_domain_id != expected_domain_id:
        raise RuntimeError(
            f"Simulation execution requires ROS_DOMAIN_ID={expected_domain_id}; "
            f"current value is {actual_domain_id}"
        )
    if not rclpy.ok():
        rclpy.init()
    guard = Node("voxposer_simulation_guard")
    try:
        deadline = time.monotonic() + 3.0
        services = set()
        topics = set()
        while time.monotonic() < deadline:
            rclpy.spin_once(guard, timeout_sec=0.1)
            services = {
                name for name, _ in guard.get_service_names_and_types()
            }
            topics = {name for name, _ in guard.get_topic_names_and_types()}
            if "/world/fr3_pick_place/set_pose" in services and "/clock" in topics:
                return
        raise RuntimeError(
            "Gazebo world service or simulation /clock was not found"
        )
    finally:
        guard.destroy_node()


class PickStateMachine:
    """Stage runner using the same high-level API as test_cartesian.py."""

    def __init__(
        self,
        robot,
        gripper,
        max_step,
        max_cartesian_speed,
        minimum_fraction,
    ):
        self.robot = robot
        self.gripper = gripper
        self.max_step = max_step
        self.max_cartesian_speed = max_cartesian_speed
        self.minimum_fraction = minimum_fraction

    def move_to(self, name, target):
        xyz = tuple(float(value) for value in target)
        print(f"\n[state] {name}: robot.move_to_xyz{xyz}")
        success = self.robot.move_to_xyz(
            *xyz,
            max_step=self.max_step,
            max_cartesian_speed=self.max_cartesian_speed,
            avoid_collisions=True,
            minimum_fraction=self.minimum_fraction,
            enforce_workspace_boundary=True,
        )
        if not success:
            raise RuntimeError(f"{name}: robot.move_to_xyz() failed")
        print(f"[state] {name}: PASS")

    def open_gripper(self, state_name="OPEN_GRIPPER"):
        print(f"\n[state] {state_name}: gripper.open()")
        if not self.gripper.open():
            raise RuntimeError(f"{state_name}: gripper.open() failed")
        print(f"[state] {state_name}: PASS")

    def close_gripper(self):
        print("\n[state] CLOSE_GRIPPER: gripper.close()")
        if not self.gripper.close():
            raise RuntimeError("CLOSE_GRIPPER: gripper.close() failed")
        print("[state] CLOSE_GRIPPER: PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--simulation-domain-id", type=int, default=42)
    parser.add_argument("--max-step", type=float, default=0.01)
    parser.add_argument("--max-cartesian-speed", type=float, default=0.02)
    parser.add_argument("--minimum-fraction", type=float, default=0.99)
    parser.add_argument("--lift-distance", type=float, default=0.10)
    # Backward-compatible only; FR3Robot now owns trajectory timing.
    parser.add_argument("--time-scale", type=float, help=argparse.SUPPRESS)
    args = parser.parse_args()

    data = np.load(args.input)
    path_world = np.asarray(data["path_world"], dtype=np.float64)
    target_aabb = np.asarray(data["target_aabb_world"], dtype=np.float64)
    target_name = str(data["expected_target_name"])
    if path_world.ndim != 2 or path_world.shape[1] != 3 or len(path_world) < 2:
        raise ValueError("path_world must have shape (N, 3) with N >= 2")
    if target_aabb.shape != (2, 3):
        raise ValueError("target_aabb_world must have shape (2, 3)")
    if target_name != "pick_object":
        raise ValueError(
            f"Refusing grasp for {target_name!r}; expected 'pick_object'"
        )
    if not 0.03 <= args.lift_distance <= 0.30:
        raise ValueError("lift-distance must be between 0.03 and 0.30 m")

    pregrasp = path_world[-1].copy()
    grasp = pregrasp.copy()
    grasp[2] = float(np.mean(target_aabb[:, 2]))
    descent = float(pregrasp[2] - grasp[2])
    if not 0.03 <= descent <= 0.12:
        raise ValueError(f"Unsafe inferred descent distance: {descent:.4f} m")
    lift = grasp + np.asarray([0.0, 0.0, args.lift_distance])

    print(f"target: {target_name}")
    print(f"pregrasp_m: {np.round(pregrasp, 6)}")
    print(f"grasp_m: {np.round(grasp, 6)}")
    print(f"lift_m: {np.round(lift, 6)}")

    robot = None
    gripper = None
    try:
        assert_isolated_simulation(args.simulation_domain_id)
        robot = get_robot()
        gripper = get_gripper()
        machine = PickStateMachine(
            robot,
            gripper,
            args.max_step,
            args.max_cartesian_speed,
            args.minimum_fraction,
        )
        machine.open_gripper()
        machine.move_to("PREGRASP", pregrasp)
        machine.move_to("DESCEND", grasp)
        machine.close_gripper()
        time.sleep(0.5)
        machine.move_to("LIFT", lift)
        print("\npick state machine: PASS")
    finally:
        if gripper is not None:
            gripper.shutdown_gripper(shutdown_ros=False)
        if robot is not None:
            robot.shutdown()
        elif rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
