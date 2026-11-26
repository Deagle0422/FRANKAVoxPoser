#!/usr/bin/env python3
"""Fetch one FR3 RGB-D snapshot and report the generated point cloud."""

import argparse
import os
import sys

import numpy as np
import yaml

SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from perception.rgbd import depth_to_point_cloud
from perception.rgbd_stream import RGBDStreamClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=os.path.join(SRC_DIR, "configs", "fr3_rgbd_sim.yaml"),
    )
    parser.add_argument("--save", help="Optional output .npz containing points and colors")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)["rgbd"]

    stream = config["stream"]
    host = stream.get("client_host", "127.0.0.1")
    frame = RGBDStreamClient(host=host, port=stream.get("port", 8765)).get_frame()
    cloud_config = config.get("point_cloud", {})
    points, colors = depth_to_point_cloud(
        frame,
        min_depth_m=cloud_config.get("min_depth_m", 0.10),
        max_depth_m=cloud_config.get("max_depth_m", 2.50),
        stride=cloud_config.get("stride", 2),
    )
    print(f"frame: {frame.camera_frame} -> {frame.target_frame}")
    print(f"image: {frame.color.shape[1]}x{frame.color.shape[0]}")
    print(f"timestamp_ns: {frame.timestamp_ns}")
    if frame.end_effector_to_target is not None:
        print(
            f"end effector: {frame.end_effector_frame} at "
            f"{frame.end_effector_to_target[:3, 3].round(4)}"
        )
    print(f"valid points: {len(points)}")
    if len(points):
        print(f"point min: {np.min(points, axis=0).round(4)}")
        print(f"point max: {np.max(points, axis=0).round(4)}")
    if args.save:
        np.savez_compressed(args.save, points=points, colors=colors)
        print(f"saved: {os.path.abspath(args.save)}")


if __name__ == "__main__":
    main()
