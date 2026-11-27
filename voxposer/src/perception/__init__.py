"""Perception adapters shared by the FR3 simulation and real robot."""

from .color_segmenter import ColorRule, ColorRuleSegmenter
from .remote_segmenter import RemoteSegmenter
from .segmenter_factory import build_segmenter
from .rgbd import PinholeIntrinsics, RGBDFrame, depth_to_point_cloud
from .rgbd_stream import RGBDStreamClient
from .fr3_rgbd import FR3RGBDPerception

__all__ = [
    "ColorRule",
    "ColorRuleSegmenter",
    "RemoteSegmenter",
    "build_segmenter",
    "PinholeIntrinsics",
    "RGBDFrame",
    "RGBDStreamClient",
    "FR3RGBDPerception",
    "depth_to_point_cloud",
]
