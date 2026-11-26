#!/usr/bin/env python3
"""Ask the configured LLM for one FR3 affordance map without executing motion."""

import argparse
import os
import sys

import numpy as np
import yaml

SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_DIR = os.path.dirname(SRC_DIR)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from interfaces import setup_LMP
from perception import ColorRuleSegmenter, FR3RGBDPerception
from utils import set_lmp_objects


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("query", nargs="?", default="the green target")
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
    parser.add_argument("--save", default="/tmp/fr3_llm_value_map.npz")
    args = parser.parse_args()

    os.environ["DEEPSEEK_API_KEY_FILE"] = os.path.abspath(args.api_key_file)
    with open(args.rgbd_config, "r", encoding="utf-8") as config_file:
        rgbd = yaml.safe_load(config_file)["rgbd"]
    with open(args.voxposer_config, "r", encoding="utf-8") as config_file:
        voxposer = yaml.safe_load(config_file)

    segmenter = ColorRuleSegmenter(rgbd["segmentation"]["objects"])
    perception = FR3RGBDPerception(rgbd, segmenter=segmenter)
    lmps, interface = setup_LMP(perception, voxposer, debug=True)
    set_lmp_objects(lmps, perception.get_object_names())
    # FR3 object aliases already resolve deterministically from the configured
    # segmenter. Keep the LLM responsible for spatial/value-map code, while
    # grounding object names in the latest camera observation locally.
    lmps["get_affordance_map"]._variable_vars["parse_query_obj"] = interface.detect

    value_map_fn = lmps["get_affordance_map"](args.query)
    value_map = np.asarray(value_map_fn(), dtype=np.float32)
    map_size = int(voxposer["lmp_config"]["env"]["map_size"])
    expected_shape = (map_size, map_size, map_size)
    if value_map.shape != expected_shape:
        raise ValueError(
            f"LLM value map has shape {value_map.shape}, expected {expected_shape}"
        )
    if not np.all(np.isfinite(value_map)):
        raise ValueError("LLM value map contains non-finite values")
    active = np.argwhere(value_map > 0)
    if not len(active):
        raise ValueError("LLM value map contains no positive target voxels")

    target_voxel = np.asarray(interface.detect(args.expected_target).position)
    nearest_distance = float(np.min(np.linalg.norm(active - target_voxel, axis=1)))
    np.savez_compressed(
        args.save,
        value_map=value_map,
        query=np.asarray(args.query),
        expected_target_voxel=target_voxel,
    )
    print(f"query: {args.query}")
    print(f"value map: shape={value_map.shape}, active_voxels={len(active)}")
    print(f"expected target voxel: {target_voxel}")
    print(f"nearest active voxel distance: {nearest_distance:.3f}")
    print(f"saved: {os.path.abspath(args.save)}")
    print("motion execution: disabled")


if __name__ == "__main__":
    main()
