"""ROS 2 RGB-D topic subscriber and snapshot gateway.

Run this module with the ROS 2 Humble Python interpreter. VoxPoser itself can
remain in its original Python 3.9 Conda environment and connect over localhost.
"""

import argparse
import copy
import threading

import numpy as np
import yaml

from .rgbd import PinholeIntrinsics, RGBDFrame
from .rgbd_stream import RGBDStreamServer


def _stamp_ns(message):
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _quaternion_matrix(x, y, z, w):
    quaternion = np.asarray([w, x, y, z], dtype=np.float64)
    norm = float(quaternion @ quaternion)
    if norm < np.finfo(float).eps:
        return np.eye(3, dtype=np.float64)
    quaternion *= np.sqrt(2.0 / norm)
    outer = np.outer(quaternion, quaternion)
    return np.array(
        [
            [1.0 - outer[2, 2] - outer[3, 3], outer[1, 2] - outer[3, 0], outer[1, 3] + outer[2, 0]],
            [outer[1, 2] + outer[3, 0], 1.0 - outer[1, 1] - outer[3, 3], outer[2, 3] - outer[1, 0]],
            [outer[1, 3] - outer[2, 0], outer[2, 3] + outer[1, 0], 1.0 - outer[1, 1] - outer[2, 2]],
        ],
        dtype=np.float64,
    )


def _transform_matrix(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _quaternion_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def _decode_image(message):
    """Decode common raw sensor_msgs/Image encodings without cv_bridge."""

    encoding = message.encoding.lower()
    definitions = {
        "rgb8": (np.uint8, 3),
        "bgr8": (np.uint8, 3),
        "rgba8": (np.uint8, 4),
        "bgra8": (np.uint8, 4),
        "mono8": (np.uint8, 1),
        "16uc1": (np.dtype("<u2"), 1),
        "mono16": (np.dtype("<u2"), 1),
        "32fc1": (np.dtype("<f4"), 1),
    }
    if encoding not in definitions:
        raise ValueError(f"Unsupported ROS image encoding: {message.encoding}")
    dtype, channels = definitions[encoding]
    dtype = np.dtype(dtype).newbyteorder(">" if message.is_bigendian else "<")
    row_values = int(message.step) // dtype.itemsize
    raw = np.frombuffer(message.data, dtype=dtype).reshape(int(message.height), row_values)
    needed = int(message.width) * channels
    image = raw[:, :needed].reshape(int(message.height), int(message.width), channels)
    if channels == 1:
        return image[:, :, 0].copy()
    if encoding in ("bgr8", "bgra8"):
        image = image[:, :, [2, 1, 0] + ([3] if channels == 4 else [])]
    return image[:, :, :3].copy()


class RosRGBDGateway:
    def __init__(self, config):
        try:
            import rclpy
            from rclpy.duration import Duration
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.time import Time
            from sensor_msgs.msg import CameraInfo, Image
            from tf2_ros import Buffer, TransformListener
        except ImportError as error:
            raise RuntimeError(
                "ros_rgbd_gateway must run in a sourced ROS 2 Humble environment"
            ) from error

        class GatewayNode(Node):
            pass

        self._rclpy = rclpy
        self._duration_type = Duration
        self._time_type = Time
        self.node = GatewayNode("voxposer_rgbd_gateway")
        self.config = config
        self._lock = threading.Lock()
        self._latest_color = None
        self._latest_depth = None
        self._camera_info = None
        self._frame = None
        self._last_error = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)
        topics = config["topics"]
        self._subscriptions = [
            self.node.create_subscription(Image, topics["color"], self._color_callback, qos_profile_sensor_data),
            self.node.create_subscription(Image, topics["depth"], self._depth_callback, qos_profile_sensor_data),
            self.node.create_subscription(CameraInfo, topics["camera_info"], self._info_callback, qos_profile_sensor_data),
        ]
        stream = config.get("stream", {})
        self.server = RGBDStreamServer(
            self.get_latest_frame,
            host=stream.get("host", "127.0.0.1"),
            port=stream.get("port", 8765),
        )

    def _color_callback(self, message):
        with self._lock:
            self._latest_color = message
        self._try_build_frame()

    def _depth_callback(self, message):
        with self._lock:
            self._latest_depth = message
        self._try_build_frame()

    def _info_callback(self, message):
        with self._lock:
            self._camera_info = message
        self._try_build_frame()

    def _try_build_frame(self):
        with self._lock:
            color_message = self._latest_color
            depth_message = self._latest_depth
            info_message = self._camera_info
        if color_message is None or depth_message is None or info_message is None:
            return

        color_stamp = _stamp_ns(color_message)
        depth_stamp = _stamp_ns(depth_message)
        tolerance_ns = int(float(self.config.get("sync_tolerance_sec", 0.05)) * 1e9)
        if abs(color_stamp - depth_stamp) > tolerance_ns:
            return

        try:
            color = _decode_image(color_message)
            depth_raw = _decode_image(depth_message)
            if depth_raw.dtype.kind == "f":
                depth_m = depth_raw.astype(np.float32, copy=False)
            else:
                scale = float(self.config.get("integer_depth_scale", 0.001))
                depth_m = depth_raw.astype(np.float32) * scale
            if color.shape[:2] != depth_m.shape:
                raise ValueError(
                    "Color and depth topic dimensions differ; publish registered/aligned depth"
                )

            camera_frame = self.config.get("camera_frame_override") or depth_message.header.frame_id
            target_frame = self.config.get("target_frame", "fr3_link0")
            transform = self._lookup_transform(target_frame, camera_frame)
            intrinsics = PinholeIntrinsics.from_camera_matrix(
                info_message.width, info_message.height, info_message.k
            )
            frame = RGBDFrame(
                color=color,
                depth_m=depth_m,
                intrinsics=intrinsics,
                camera_to_target=transform,
                timestamp_ns=max(color_stamp, depth_stamp),
                camera_frame=camera_frame,
                target_frame=target_frame,
            )
            frame.validate()
        except Exception as error:
            error_text = str(error)
            if error_text != self._last_error:
                self.node.get_logger().warning(f"RGB-D frame rejected: {error_text}")
                self._last_error = error_text
            return

        with self._lock:
            self._frame = frame
        if self._last_error is not None:
            self.node.get_logger().info("RGB-D stream recovered")
        self._last_error = None

    def _lookup_transform(self, target_frame, camera_frame):
        static = self.config.get("static_camera_to_target")
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                camera_frame,
                self._time_type(),
                timeout=self._duration_type(seconds=float(self.config.get("tf_timeout_sec", 0.1))),
            )
            return _transform_matrix(transform)
        except Exception:
            if static is None:
                raise
            matrix = np.asarray(static, dtype=np.float64)
            if matrix.shape != (4, 4):
                raise ValueError("static_camera_to_target must be a 4x4 matrix")
            return matrix

    def get_latest_frame(self):
        with self._lock:
            return copy.deepcopy(self._frame)

    def run(self):
        self.server.start()
        stream = self.config.get("stream", {})
        self.node.get_logger().info(
            "RGB-D gateway listening on "
            f"{stream.get('host', '127.0.0.1')}:{stream.get('port', 8765)}"
        )
        try:
            self._rclpy.spin(self.node)
        finally:
            self.server.close()
            self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="RGB-D gateway YAML file")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)["rgbd"]

    import rclpy

    rclpy.init()
    try:
        RosRGBDGateway(config).run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
