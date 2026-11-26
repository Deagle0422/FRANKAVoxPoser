#!/usr/bin/env python3
"""Run VoxPoser place stages through the public FR3Robot/FrankaGripper API."""

import argparse
import sys
import time

import numpy as np
import rclpy

from basic_function.fr3_gripper import get_gripper
from basic_function.fr3_robot import get_robot
from run_voxposer_pick import PickStateMachine, assert_isolated_simulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--simulation-domain-id", type=int, default=42)
    parser.add_argument("--max-step", type=float, default=0.01)
    parser.add_argument("--max-cartesian-speed", type=float, default=0.02)
    parser.add_argument("--minimum-fraction", type=float, default=0.99)
    parser.add_argument("--object-height", type=float, default=0.04)
    parser.add_argument("--retreat-distance", type=float, default=0.10)
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
    if target_name != "place_target":
        raise ValueError(
            f"Refusing placement on {target_name!r}; expected 'place_target'"
        )
    if not 0.01 <= args.object_height <= 0.10:
        raise ValueError("object-height must be between 0.01 and 0.10 m")
    if not 0.03 <= args.retreat_distance <= 0.30:
        raise ValueError("retreat-distance must be between 0.03 and 0.30 m")

    target_center = np.mean(target_aabb, axis=0)
    preplace = path_world[-1].copy()
    preplace[:2] = target_center[:2]
    place = preplace.copy()
    place[2] = float(target_aabb[1, 2] + args.object_height / 2.0)
    descent = float(preplace[2] - place[2])
    if not 0.02 <= descent <= 0.10:
        raise ValueError(f"Unsafe inferred placement descent: {descent:.4f} m")
    retreat = place + np.asarray([0.0, 0.0, args.retreat_distance])

    print(f"target: {target_name}")
    print(f"preplace_m: {np.round(preplace, 6)}")
    print(f"place_m: {np.round(place, 6)}")
    print(f"retreat_m: {np.round(retreat, 6)}")

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
        machine.move_to("TRANSFER", preplace)
        machine.move_to("PLACE_DESCEND", place)
        machine.open_gripper("RELEASE")
        time.sleep(0.5)
        machine.move_to("RETREAT", retreat)
        print("\nplace state machine: PASS")
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
