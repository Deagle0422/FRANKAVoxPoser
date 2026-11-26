#!/usr/bin/env python3

"""Expose synchronized ROS RGB-D topics to the VoxPoser Python environment."""

import json
import socket
import struct
import threading

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


LENGTH = struct.Struct("!Q")


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def quaternion_matrix(x, y, z, w):
    quaternion = np.asarray([w, x, y, z], dtype=np.float64)
    norm = float(quaternion @ quaternion)
    if norm < np.finfo(float).eps:
        return np.eye(3, dtype=np.float64)
    quaternion *= np.sqrt(2.0 / norm)
    outer = np.outer(quaternion, quaternion)
    return np.array([
        [1.0 - outer[2, 2] - outer[3, 3], outer[1, 2] - outer[3, 0], outer[1, 3] + outer[2, 0]],
        [outer[1, 2] + outer[3, 0], 1.0 - outer[1, 1] - outer[3, 3], outer[2, 3] - outer[1, 0]],
        [outer[1, 3] - outer[2, 0], outer[2, 3] + outer[1, 0], 1.0 - outer[1, 1] - outer[2, 2]],
    ])


def decode_image(message):
    definitions = {
        "rgb8": (np.uint8, 3), "bgr8": (np.uint8, 3),
        "rgba8": (np.uint8, 4), "bgra8": (np.uint8, 4),
        "mono8": (np.uint8, 1), "16uc1": (np.dtype("<u2"), 1),
        "mono16": (np.dtype("<u2"), 1), "32fc1": (np.dtype("<f4"), 1),
    }
    encoding = message.encoding.lower()
    if encoding not in definitions:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    dtype, channels = definitions[encoding]
    dtype = np.dtype(dtype).newbyteorder(">" if message.is_bigendian else "<")
    row_values = int(message.step) // dtype.itemsize
    raw = np.frombuffer(message.data, dtype=dtype).reshape(int(message.height), row_values)
    image = raw[:, :int(message.width) * channels].reshape(
        int(message.height), int(message.width), channels
    )
    if channels == 1:
        return image[:, :, 0].copy()
    if encoding in ("bgr8", "bgra8"):
        image = image[:, :, [2, 1, 0] + ([3] if channels == 4 else [])]
    return image[:, :, :3].copy()


class VoxPoserRGBDGateway(Node):
    def __init__(self):
        super().__init__("voxposer_rgbd_gateway")
        defaults = {
            "color_topic": "/voxposer/camera/color/image_raw",
            "depth_topic": "/voxposer/camera/depth/image_raw",
            "camera_info_topic": "/voxposer/camera/color/camera_info",
            "camera_frame": "voxposer_camera_optical_frame",
            "target_frame": "fr3_link0",
            "end_effector_frame": "fr3_hand_tcp",
            "sync_tolerance_sec": 0.05,
            "integer_depth_scale": 0.001,
            "stream_host": "127.0.0.1",
            "stream_port": 8765,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._camera_frame = self.get_parameter("camera_frame").value
        self._target_frame = self.get_parameter("target_frame").value
        self._end_effector_frame = self.get_parameter("end_effector_frame").value
        self._sync_tolerance_ns = int(
            float(self.get_parameter("sync_tolerance_sec").value) * 1e9
        )
        self._integer_depth_scale = float(
            self.get_parameter("integer_depth_scale").value
        )
        self._lock = threading.Lock()
        self._color = None
        self._depth = None
        self._info = None
        self._frame = None
        self._last_error = None
        self._stop = threading.Event()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Image, self.get_parameter("color_topic").value,
            self._color_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value,
            self._depth_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._info_callback, qos_profile_sensor_data,
        )

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        host = self.get_parameter("stream_host").value
        port = int(self.get_parameter("stream_port").value)
        self._server.bind((host, port))
        self._server.listen(4)
        self._server.settimeout(0.25)
        self._server_thread = threading.Thread(target=self._serve, daemon=True)
        self._server_thread.start()
        self.get_logger().info(f"RGB-D snapshot gateway listening on {host}:{port}")

    def _color_callback(self, message):
        with self._lock:
            self._color = message
        self._update_frame()

    def _depth_callback(self, message):
        with self._lock:
            self._depth = message
        self._update_frame()

    def _info_callback(self, message):
        with self._lock:
            self._info = message
        self._update_frame()

    def _update_frame(self):
        with self._lock:
            color_message, depth_message, info_message = self._color, self._depth, self._info
        if color_message is None or depth_message is None or info_message is None:
            return
        color_stamp, depth_stamp = stamp_ns(color_message), stamp_ns(depth_message)
        if abs(color_stamp - depth_stamp) > self._sync_tolerance_ns:
            return
        try:
            color = decode_image(color_message)
            depth = decode_image(depth_message)
            if depth.dtype.kind == "f":
                depth_m = depth.astype(np.float32, copy=False)
            else:
                depth_m = depth.astype(np.float32) * self._integer_depth_scale
            if color.shape[:2] != depth_m.shape:
                raise ValueError("color and depth must be registered and equal-sized")
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, self._camera_frame, Time(),
                timeout=Duration(seconds=0.1),
            )
            camera_to_target = np.eye(4, dtype=np.float64)
            rotation = transform.transform.rotation
            camera_to_target[:3, :3] = quaternion_matrix(
                rotation.x, rotation.y, rotation.z, rotation.w
            )
            translation = transform.transform.translation
            camera_to_target[:3, 3] = [translation.x, translation.y, translation.z]
            end_effector_to_target = None
            if self._end_effector_frame:
                try:
                    end_effector_transform = self._tf_buffer.lookup_transform(
                        self._target_frame, self._end_effector_frame, Time(),
                        timeout=Duration(seconds=0.02),
                    )
                    end_effector_to_target = np.eye(4, dtype=np.float64)
                    ee_rotation = end_effector_transform.transform.rotation
                    end_effector_to_target[:3, :3] = quaternion_matrix(
                        ee_rotation.x, ee_rotation.y, ee_rotation.z, ee_rotation.w
                    )
                    ee_translation = end_effector_transform.transform.translation
                    end_effector_to_target[:3, 3] = [
                        ee_translation.x, ee_translation.y, ee_translation.z
                    ]
                except Exception:
                    # Camera-only launches intentionally have no robot TF tree.
                    pass
            k = info_message.k
            intrinsics = {
                "width": int(info_message.width), "height": int(info_message.height),
                "fx": float(k[0]), "fy": float(k[4]),
                "cx": float(k[2]), "cy": float(k[5]),
            }
            if intrinsics["fx"] <= 0.0 or intrinsics["fy"] <= 0.0:
                raise ValueError("CameraInfo contains invalid focal lengths")
            frame = (
                np.ascontiguousarray(color, dtype=np.uint8),
                np.ascontiguousarray(depth_m, dtype="<f4"),
                np.ascontiguousarray(camera_to_target, dtype="<f8"),
                intrinsics,
                max(color_stamp, depth_stamp),
                None if end_effector_to_target is None else np.ascontiguousarray(
                    end_effector_to_target, dtype="<f8"
                ),
            )
        except Exception as error:
            error_text = str(error)
            if error_text != self._last_error:
                self.get_logger().warning(f"RGB-D frame rejected: {error_text}")
                self._last_error = error_text
            return
        with self._lock:
            self._frame = frame
        if self._last_error is not None:
            self.get_logger().info("RGB-D stream recovered")
        self._last_error = None

    def _serve(self):
        while not self._stop.is_set():
            try:
                connection, _ = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with connection:
                connection.settimeout(2.0)
                try:
                    if connection.recv(1) != b"F":
                        continue
                    with self._lock:
                        frame = self._frame
                    if frame is None:
                        header = {"ok": False, "error": "No synchronized RGB-D frame yet"}
                        payload = b""
                    else:
                        (
                            color, depth, transform, intrinsics, timestamp_ns,
                            end_effector_transform,
                        ) = frame
                        payload = color.tobytes() + depth.tobytes() + transform.tobytes()
                        if end_effector_transform is not None:
                            payload += end_effector_transform.tobytes()
                        header = {
                            "ok": True,
                            "color_shape": list(color.shape),
                            "depth_shape": list(depth.shape),
                            "color_bytes": color.nbytes,
                            "depth_bytes": depth.nbytes,
                            "transform_bytes": transform.nbytes,
                            "end_effector_transform_bytes": (
                                0 if end_effector_transform is None
                                else end_effector_transform.nbytes
                            ),
                            "intrinsics": intrinsics,
                            "timestamp_ns": timestamp_ns,
                            "camera_frame": self._camera_frame,
                            "target_frame": self._target_frame,
                            "end_effector_frame": (
                                self._end_effector_frame
                                if end_effector_transform is not None else ""
                            ),
                        }
                    encoded = json.dumps(header, separators=(",", ":")).encode("utf-8")
                    connection.sendall(LENGTH.pack(len(encoded)) + encoded)
                    connection.sendall(LENGTH.pack(len(payload)) + payload)
                except (ConnectionError, OSError, socket.timeout):
                    continue

    def close(self):
        self._stop.set()
        self._server.close()
        self._server_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = VoxPoserRGBDGateway()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
