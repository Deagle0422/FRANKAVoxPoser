#!/usr/bin/env python3
"""Validate FR3 object masks, object clouds, and a VoxPoser target value map."""

import argparse
import os
import sys

import numpy as np
import yaml

SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from interfaces import LMP_interface
from perception import FR3RGBDPerception, build_segmenter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--rgbd-config",
        default=os.path.join(SRC_DIR, "configs", "fr3_rgbd_sim.yaml"),
    )
    parser.add_argument(
        "--voxposer-config",
        default=os.path.join(SRC_DIR, "configs", "rlbench_config.yaml"),
    )
    parser.add_argument("--movable", default="red cube")
    parser.add_argument("--target", default="green target")
    parser.add_argument("--save", help="Optional output .npz with masks, clouds, and value map")
    args = parser.parse_args()

    with open(args.rgbd_config, "r", encoding="utf-8") as config_file:
        rgbd = yaml.safe_load(config_file)["rgbd"]
    with open(args.voxposer_config, "r", encoding="utf-8") as config_file:
        voxposer = yaml.safe_load(config_file)

    segmenter = build_segmenter(rgbd["segmentation"])
    perception = FR3RGBDPerception(rgbd, segmenter=segmenter)

    frame = perception.get_frame()
    movable_mask = perception.get_mask(args.movable, frame)
    target_mask = perception.get_mask(args.target, frame)
    movable_points, movable_colors = perception.get_object_cloud(args.movable, frame)
    target_points, target_colors = perception.get_object_cloud(args.target, frame)

    interface = LMP_interface(
        perception,
        voxposer["lmp_config"]["env"],
        voxposer["controller"],
        voxposer["planner"],
        env_name=voxposer["env_name"],
    )
    movable_obs = interface.detect(args.movable)
    target_obs = interface.detect(args.target)
    target_value_map = np.asarray(target_obs.occupancy_map)

    print(f"objects: {', '.join(perception.get_object_names())}")
    print(
        f"{args.movable}: mask_pixels={int(movable_mask.sum())}, "
        f"points={len(movable_points)}, center_m={np.mean(movable_points, axis=0).round(4)}, "
        f"voxel={np.asarray(movable_obs.position)}"
    )
    print(
        f"{args.target}: mask_pixels={int(target_mask.sum())}, "
        f"points={len(target_points)}, center_m={np.mean(target_points, axis=0).round(4)}, "
        f"voxel={np.asarray(target_obs.position)}"
    )
    print(
        f"target value map: shape={target_value_map.shape}, "
        f"active_voxels={int(np.count_nonzero(target_value_map))}"
    )

    if args.save:
        np.savez_compressed(
            args.save,
            color=frame.color,
            movable_mask=movable_mask,
            target_mask=target_mask,
            movable_points=movable_points,
            movable_colors=movable_colors,
            target_points=target_points,
            target_colors=target_colors,
            target_value_map=target_value_map,
        )
        print(f"saved: {os.path.abspath(args.save)}")


if __name__ == "__main__":
    main()
