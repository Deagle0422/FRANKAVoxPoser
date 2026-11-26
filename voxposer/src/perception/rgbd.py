"""ROS-independent RGB-D geometry used by the VoxPoser FR3 adapter."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class PinholeIntrinsics:
    """Pinhole camera parameters in pixel units."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_camera_matrix(cls, width, height, matrix):
        matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        return cls(
            width=int(width),
            height=int(height),
            fx=float(matrix[0, 0]),
            fy=float(matrix[1, 1]),
            cx=float(matrix[0, 2]),
            cy=float(matrix[1, 2]),
        )

    def validate(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Camera width and height must be positive")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("Camera focal lengths must be positive")


@dataclass
class RGBDFrame:
    """One registered RGB-D frame and its camera-to-target transform."""

    color: np.ndarray
    depth_m: np.ndarray
    intrinsics: PinholeIntrinsics
    camera_to_target: np.ndarray
    timestamp_ns: int = 0
    camera_frame: str = ""
    target_frame: str = ""
    end_effector_to_target: Optional[np.ndarray] = None
    end_effector_frame: str = ""

    def validate(self):
        self.intrinsics.validate()
        if self.color.ndim != 3 or self.color.shape[2] != 3:
            raise ValueError("Color image must have shape (height, width, 3)")
        if self.depth_m.ndim != 2:
            raise ValueError("Depth image must have shape (height, width)")
        if self.color.shape[:2] != self.depth_m.shape:
            raise ValueError("Color and depth images must be registered and equal-sized")
        if self.depth_m.shape != (self.intrinsics.height, self.intrinsics.width):
            raise ValueError("Image dimensions do not match CameraInfo")
        transform = np.asarray(self.camera_to_target)
        if transform.shape != (4, 4):
            raise ValueError("camera_to_target must be a 4x4 matrix")
        if not np.all(np.isfinite(transform)):
            raise ValueError("camera_to_target contains non-finite values")
        if self.end_effector_to_target is not None:
            end_effector_transform = np.asarray(self.end_effector_to_target)
            if end_effector_transform.shape != (4, 4):
                raise ValueError("end_effector_to_target must be a 4x4 matrix")
            if not np.all(np.isfinite(end_effector_transform)):
                raise ValueError("end_effector_to_target contains non-finite values")


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a homogeneous transform to an ``(N, 3)`` point array."""

    points = np.asarray(points, dtype=np.float32)
    transform = np.asarray(transform, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if transform.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    return (points @ transform[:3, :3].T + transform[:3, 3]).astype(np.float32)


def depth_to_point_cloud(
    frame: RGBDFrame,
    min_depth_m: float = 0.10,
    max_depth_m: float = 2.50,
    stride: int = 1,
    mask: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Project a registered RGB-D frame into the configured target frame.

    Returns ``(points, colors)`` where points are metres in ``target_frame`` and
    colors are uint8 RGB. Invalid, out-of-range and unselected pixels are removed.
    """

    frame.validate()
    if stride < 1:
        raise ValueError("stride must be at least 1")
    if min_depth_m < 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("Invalid depth range")

    depth = np.asarray(frame.depth_m, dtype=np.float32)
    color = np.asarray(frame.color, dtype=np.uint8)
    rows, cols = np.indices(depth.shape, dtype=np.float32)

    selection = np.isfinite(depth)
    selection &= depth >= float(min_depth_m)
    selection &= depth <= float(max_depth_m)
    if stride > 1:
        sampled = np.zeros(depth.shape, dtype=bool)
        sampled[::stride, ::stride] = True
        selection &= sampled
    if mask is not None:
        mask = np.asarray(mask, dtype=bool)
        if mask.shape != depth.shape:
            raise ValueError("mask must have the same height and width as depth")
        selection &= mask

    z = depth[selection]
    x = (cols[selection] - frame.intrinsics.cx) * z / frame.intrinsics.fx
    y = (rows[selection] - frame.intrinsics.cy) * z / frame.intrinsics.fy
    camera_points = np.stack((x, y, z), axis=1)
    points = transform_points(camera_points, frame.camera_to_target)
    colors = color[selection].copy()
    return points, colors
