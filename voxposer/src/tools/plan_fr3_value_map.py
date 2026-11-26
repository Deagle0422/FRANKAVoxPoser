#!/usr/bin/env python3
"""Generate an FR3 value map and plan a collision-aware path without motion."""

import argparse
import os
import sys

import numpy as np
import yaml

SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_DIR = os.path.dirname(SRC_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from arguments import get_config
from interfaces import setup_LMP
from perception import ColorRuleSegmenter, FR3RGBDPerception
from utils import set_lmp_objects


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "query", nargs="?", default="a point 5cm above the green target"
    )
    parser.add_argument("--expected-target", default="green target")
    parser.add_argument(
        "--api-key-file", default=os.path.join(PROJECT_DIR, "api.txt")
    )
    parser.add_argument(
        "--rgbd-config",
        default=os.path.join(SRC_DIR, "configs", "fr3_rgbd_sim.yaml"),
    )
    parser.add_argument(
        "--voxposer-config",
        default=os.path.join(SRC_DIR, "configs", "rlbench_config.yaml"),
    )
    parser.add_argument("--save", default="/tmp/fr3_planned_path.npz")
    args = parser.parse_args()

    os.environ["DEEPSEEK_API_KEY_FILE"] = os.path.abspath(args.api_key_file)
    with open(args.rgbd_config, "r", encoding="utf-8") as config_file:
        rgbd = yaml.safe_load(config_file)["rgbd"]
    voxposer = get_config(config_path=args.voxposer_config)

    segmenter = ColorRuleSegmenter(rgbd["segmentation"]["objects"])
    perception = FR3RGBDPerception(rgbd, segmenter=segmenter)
    frame = perception.get_frame()
    if frame.end_effector_to_target is None:
        raise RuntimeError(
            "RGB-D snapshot has no end-effector pose. Start the complete "
            "fr3_moveit_gazebo.launch.py instead of the camera-only launch."
        )

    lmps, interface = setup_LMP(perception, voxposer, debug=True)
    set_lmp_objects(lmps, perception.get_object_names())
    lmps["get_affordance_map"]._variable_vars["parse_query_obj"] = interface.detect
    value_map_fn = lmps["get_affordance_map"](args.query)
    value_map = np.asarray(value_map_fn(), dtype=np.float32)

    map_size = int(voxposer["lmp_config"]["env"]["map_size"])
    expected_shape = (map_size, map_size, map_size)
    if value_map.shape != expected_shape or not np.all(np.isfinite(value_map)):
        raise ValueError(f"Invalid value map: shape={value_map.shape}")
    active = np.argwhere(value_map > 0)
    if not len(active):
        raise ValueError("Value map contains no positive target voxels")

    ee_position_world = np.asarray(
        frame.end_effector_to_target[:3, 3], dtype=np.float32
    )
    start_voxel = interface._world_to_voxel(ee_position_world)
    movable_obs = {"name": "ee", "position": start_voxel}
    avoidance_map = interface._preprocess_avoidance_map(
        np.zeros(expected_shape, dtype=np.float32), value_map, movable_obs
    )
    path_voxel, planner_info = interface._planner.optimize(
        start_voxel,
        value_map,
        avoidance_map,
        object_centric=False,
    )
    path_world = np.asarray([
        interface._voxel_to_world(point) for point in path_voxel
    ])

    target_obs = interface.detect(args.expected_target)
    target_cloud_world = np.asarray(
        target_obs._point_cloud_world, dtype=np.float64
    )
    target_aabb_world = np.asarray([
        np.min(target_cloud_world, axis=0),
        np.max(target_cloud_world, axis=0),
    ])
    expected_above_voxel = np.asarray(target_obs.position, dtype=np.int32)
    expected_above_voxel[2] = np.clip(
        int(target_obs.aabb[1, 2]) + interface.cm2index(5, "z"),
        0,
        map_size - 1,
    )
    language_target_error = float(
        np.min(np.linalg.norm(active - expected_above_voxel, axis=1))
    )
    path_target_error = float(
        np.min(np.linalg.norm(active - path_voxel[-1], axis=1))
    )
    path_indices = np.rint(path_voxel).astype(np.int32).clip(0, map_size - 1)
    occupied_waypoints = int(np.count_nonzero(
        avoidance_map[
            path_indices[:, 0], path_indices[:, 1], path_indices[:, 2]
        ] > 0.5
    ))
    path_length_m = float(np.linalg.norm(np.diff(path_world, axis=0), axis=1).sum())

    np.savez_compressed(
        args.save,
        query=np.asarray(args.query),
        value_map=value_map,
        avoidance_map=avoidance_map,
        start_voxel=start_voxel,
        ee_position_world=ee_position_world,
        ee_transform=np.asarray(frame.end_effector_to_target, dtype=np.float64),
        path_voxel=path_voxel,
        path_world=path_world,
        expected_above_voxel=expected_above_voxel,
        expected_target_name=np.asarray(args.expected_target),
        target_aabb_world=target_aabb_world,
        planner_costmap=planner_info["costmap"],
    )
    print(f"query: {args.query}")
    print(f"current EE: world_m={ee_position_world.round(4)}, voxel={start_voxel}")
    print(
        f"value map: active_voxels={len(active)}, "
        f"expected_above_voxel={expected_above_voxel}, "
        f"language_target_error={language_target_error:.3f} voxels"
    )
    print(
        f"planned path: waypoints={len(path_voxel)}, "
        f"length_m={path_length_m:.3f}, "
        f"final_voxel={np.asarray(path_voxel[-1]).round(3)}, "
        f"target_error={path_target_error:.3f} voxels, "
        f"occupied_waypoints={occupied_waypoints}"
    )
    print(f"saved: {os.path.abspath(args.save)}")
    print("motion execution: disabled")


if __name__ == "__main__":
    main()
