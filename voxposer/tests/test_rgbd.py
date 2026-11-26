import os
import socket
import sys
import time
import unittest

import numpy as np

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from perception.rgbd import PinholeIntrinsics, RGBDFrame, depth_to_point_cloud
from perception.rgbd_stream import RGBDStreamClient, RGBDStreamServer


def make_frame():
    return RGBDFrame(
        color=np.array(
            [
                [[255, 0, 0], [0, 255, 0]],
                [[0, 0, 255], [255, 255, 255]],
            ],
            dtype=np.uint8,
        ),
        depth_m=np.array([[1.0, 2.0], [0.0, np.nan]], dtype=np.float32),
        intrinsics=PinholeIntrinsics(width=2, height=2, fx=1.0, fy=1.0, cx=0.0, cy=0.0),
        camera_to_target=np.array(
            [[1.0, 0.0, 0.0, 0.5], [0.0, 1.0, 0.0, -0.5], [0.0, 0.0, 1.0, 0.25], [0.0, 0.0, 0.0, 1.0]]
        ),
        timestamp_ns=123,
        camera_frame="camera",
        target_frame="fr3_link0",
        end_effector_to_target=np.array(
            [[1.0, 0.0, 0.0, 0.4], [0.0, 1.0, 0.0, 0.1], [0.0, 0.0, 1.0, 0.5], [0.0, 0.0, 0.0, 1.0]]
        ),
        end_effector_frame="fr3_hand_tcp",
    )


class RGBDGeometryTest(unittest.TestCase):
    def test_projection_filters_invalid_depth_and_transforms(self):
        points, colors = depth_to_point_cloud(make_frame(), min_depth_m=0.1, max_depth_m=3.0)
        np.testing.assert_allclose(
            points,
            np.array([[0.5, -0.5, 1.25], [2.5, -0.5, 2.25]], dtype=np.float32),
        )
        np.testing.assert_array_equal(colors, np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8))

    def test_mask_selects_object_pixels(self):
        mask = np.array([[False, True], [False, False]])
        points, colors = depth_to_point_cloud(make_frame(), min_depth_m=0.1, max_depth_m=3.0, mask=mask)
        self.assertEqual(points.shape, (1, 3))
        np.testing.assert_array_equal(colors[0], [0, 255, 0])


class RGBDTransportTest(unittest.TestCase):
    def test_snapshot_round_trip(self):
        probe = socket.socket()
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        server = RGBDStreamServer(make_frame, host="127.0.0.1", port=port)
        server.start()
        try:
            time.sleep(0.05)
            result = RGBDStreamClient(port=port).get_frame()
            np.testing.assert_array_equal(result.color, make_frame().color)
            np.testing.assert_allclose(result.depth_m, make_frame().depth_m, equal_nan=True)
            np.testing.assert_allclose(result.camera_to_target, make_frame().camera_to_target)
            np.testing.assert_allclose(
                result.end_effector_to_target,
                make_frame().end_effector_to_target,
            )
            self.assertEqual(result.end_effector_frame, "fr3_hand_tcp")
            self.assertEqual(result.timestamp_ns, 123)
            self.assertEqual(result.target_frame, "fr3_link0")
        finally:
            server.close()


if __name__ == "__main__":
    unittest.main()
