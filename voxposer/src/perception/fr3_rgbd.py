"""VoxPoser-facing scene observation API backed by the FR3 RGB-D stream."""

from typing import Callable, Optional

import numpy as np

from .rgbd import depth_to_point_cloud
from .rgbd_stream import RGBDStreamClient


class FR3RGBDPerception:
    """Generate scene and named-object point clouds from registered RGB-D.

    ``segmenter`` is injected so simulation can use ground truth while the real
    robot can later use an open-vocabulary detector and SAM 2. It receives
    ``(object_name, rgb_image)`` and returns a boolean image mask.
    """

    def __init__(
        self,
        config,
        segmenter: Optional[Callable[[str, np.ndarray], np.ndarray]] = None,
    ):
        stream = config.get("stream", {})
        self.client = RGBDStreamClient(
            host=stream.get("client_host", "127.0.0.1"),
            port=stream.get("port", 8765),
            timeout=stream.get("timeout_sec", 5.0),
        )
        self.config = config
        self.segmenter = segmenter
        workspace = config.get("workspace", {})
        self.workspace_bounds_min = np.asarray(
            workspace.get("min", [-np.inf, -np.inf, -np.inf]), dtype=np.float32
        )
        self.workspace_bounds_max = np.asarray(
            workspace.get("max", [np.inf, np.inf, np.inf]), dtype=np.float32
        )
        if self.workspace_bounds_min.shape != (3,) or self.workspace_bounds_max.shape != (3,):
            raise ValueError("Workspace min and max must contain three coordinates")
        if np.any(self.workspace_bounds_max <= self.workspace_bounds_min):
            raise ValueError("Workspace max must be greater than workspace min")

    def get_frame(self):
        return self.client.get_frame()

    def _cloud(self, frame, mask=None):
        cloud = self.config.get("point_cloud", {})
        return depth_to_point_cloud(
            frame,
            min_depth_m=cloud.get("min_depth_m", 0.10),
            max_depth_m=cloud.get("max_depth_m", 2.50),
            stride=cloud.get("stride", 2),
            mask=mask,
        )

    def get_object_names(self):
        """Return canonical names exposed by the configured segmenter."""

        return list(getattr(self.segmenter, "object_names", []))

    def _inside_workspace(self, points):
        return np.all(
            (points >= self.workspace_bounds_min) & (points <= self.workspace_bounds_max),
            axis=1,
        )

    def get_scene_3d_obs(self, ignore_robot=False, ignore_grasped_obj=False):
        """Return ``(points, RGB colors)`` in the FR3 base frame."""

        del ignore_robot, ignore_grasped_obj
        points, colors = self._cloud(self.get_frame())
        inside = self._inside_workspace(points)
        return points[inside], colors[inside]

    def get_mask(self, object_name, frame=None):
        """Return a named object's boolean image mask for inspection/debugging."""

        if self.segmenter is None:
            raise RuntimeError(
                "Named-object observation requires a segmentation callback"
            )
        frame = self.get_frame() if frame is None else frame
        mask = np.asarray(self.segmenter(object_name, frame.color), dtype=bool)
        if mask.shape != frame.depth_m.shape:
            raise ValueError("Segmenter returned a mask with the wrong dimensions")
        return mask

    def get_object_cloud(self, object_name, frame=None):
        """Return named-object points and RGB colors in the FR3 base frame."""

        frame = self.get_frame() if frame is None else frame
        points, colors = self._cloud(frame, mask=self.get_mask(object_name, frame))
        inside = self._inside_workspace(points)
        points, colors = points[inside], colors[inside]
        if not len(points):
            raise ValueError(f"Object {object_name!r} has no valid depth points")
        return points, colors

    def get_3d_obs_by_name(self, object_name):
        """Return ``(points, normals)`` for a named object."""

        frame = self.get_frame()
        points, _ = self.get_object_cloud(object_name, frame)

        import open3d as o3d

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.estimate_normals()
        normals = np.asarray(cloud.normals)
        camera_origin = np.asarray(frame.camera_to_target[:3, 3])
        away_from_camera = np.sum(normals * (points - camera_origin), axis=1) > 0
        normals[away_from_camera] *= -1
        return points, normals
