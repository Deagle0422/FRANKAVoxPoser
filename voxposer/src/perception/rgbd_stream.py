"""Small dependency-free TCP transport between ROS 2 Humble and VoxPoser.

ROS 2 Humble and the original VoxPoser environment use different Python ABIs.
The ROS process subscribes to camera topics and serves the latest frame; the
VoxPoser process requests a snapshot only when it needs to replan.
"""

import json
import socket
import struct
import threading
from dataclasses import asdict
from typing import Callable, Optional

import numpy as np

from .rgbd import PinholeIntrinsics, RGBDFrame


_LENGTH = struct.Struct("!Q")
_REQUEST = b"F"


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("RGB-D stream closed before the frame completed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _encode_frame(frame: RGBDFrame):
    frame.validate()
    color = np.ascontiguousarray(frame.color, dtype=np.uint8)
    depth = np.ascontiguousarray(frame.depth_m, dtype="<f4")
    transform = np.ascontiguousarray(frame.camera_to_target, dtype="<f8")
    end_effector_transform = None
    if frame.end_effector_to_target is not None:
        end_effector_transform = np.ascontiguousarray(
            frame.end_effector_to_target, dtype="<f8"
        )
    header = {
        "ok": True,
        "color_shape": list(color.shape),
        "depth_shape": list(depth.shape),
        "color_bytes": color.nbytes,
        "depth_bytes": depth.nbytes,
        "transform_bytes": transform.nbytes,
        "end_effector_transform_bytes": (
            0 if end_effector_transform is None else end_effector_transform.nbytes
        ),
        "intrinsics": asdict(frame.intrinsics),
        "timestamp_ns": int(frame.timestamp_ns),
        "camera_frame": frame.camera_frame,
        "target_frame": frame.target_frame,
        "end_effector_frame": frame.end_effector_frame,
    }
    payload = color.tobytes() + depth.tobytes() + transform.tobytes()
    if end_effector_transform is not None:
        payload += end_effector_transform.tobytes()
    return json.dumps(header, separators=(",", ":")).encode("utf-8"), payload


class RGBDStreamServer:
    """Serve the latest frame returned by ``frame_provider`` over TCP."""

    def __init__(
        self,
        frame_provider: Callable[[], Optional[RGBDFrame]],
        host: str = "127.0.0.1",
        port: int = 8765,
    ):
        self._frame_provider = frame_provider
        self._host = host
        self._port = int(port)
        self._socket = None
        self._thread = None
        self._stop = threading.Event()

    def start(self):
        if self._thread is not None:
            return
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self._host, self._port))
        self._socket.listen(4)
        self._socket.settimeout(0.25)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._socket = None

    def _run(self):
        while not self._stop.is_set():
            try:
                connection, _ = self._socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with connection:
                connection.settimeout(2.0)
                try:
                    if _recv_exact(connection, 1) != _REQUEST:
                        continue
                    frame = self._frame_provider()
                    if frame is None:
                        header = json.dumps({"ok": False, "error": "No synchronized RGB-D frame yet"}).encode("utf-8")
                        payload = b""
                    else:
                        header, payload = _encode_frame(frame)
                    connection.sendall(_LENGTH.pack(len(header)))
                    connection.sendall(header)
                    connection.sendall(_LENGTH.pack(len(payload)))
                    if payload:
                        connection.sendall(payload)
                except (ConnectionError, OSError, socket.timeout):
                    continue


class RGBDStreamClient:
    """Fetch registered RGB-D snapshots from a ROS-side stream server."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8765, timeout: float = 5.0):
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)

    def get_frame(self) -> RGBDFrame:
        with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
            sock.settimeout(self.timeout)
            sock.sendall(_REQUEST)
            header_size = _LENGTH.unpack(_recv_exact(sock, _LENGTH.size))[0]
            header = json.loads(_recv_exact(sock, header_size).decode("utf-8"))
            payload_size = _LENGTH.unpack(_recv_exact(sock, _LENGTH.size))[0]
            payload = _recv_exact(sock, payload_size) if payload_size else b""

        if not header.get("ok"):
            raise RuntimeError(header.get("error", "RGB-D stream returned an error"))
        if len(payload) != payload_size:
            raise ValueError("RGB-D payload size mismatch")

        color_bytes = int(header["color_bytes"])
        depth_bytes = int(header["depth_bytes"])
        transform_bytes = int(header["transform_bytes"])
        end_effector_transform_bytes = int(
            header.get("end_effector_transform_bytes", 0)
        )
        expected = (
            color_bytes + depth_bytes + transform_bytes
            + end_effector_transform_bytes
        )
        if expected != payload_size:
            raise ValueError("RGB-D header and payload lengths disagree")

        color = np.frombuffer(payload[:color_bytes], dtype=np.uint8).reshape(header["color_shape"]).copy()
        depth_end = color_bytes + depth_bytes
        depth = np.frombuffer(payload[color_bytes:depth_end], dtype="<f4").reshape(header["depth_shape"]).copy()
        transform_end = depth_end + transform_bytes
        transform = np.frombuffer(
            payload[depth_end:transform_end], dtype="<f8"
        ).reshape(4, 4).copy()
        end_effector_transform = None
        if end_effector_transform_bytes:
            end_effector_transform = np.frombuffer(
                payload[transform_end:], dtype="<f8"
            ).reshape(4, 4).copy()
        intrinsics = PinholeIntrinsics(**header["intrinsics"])
        return RGBDFrame(
            color=color,
            depth_m=depth,
            intrinsics=intrinsics,
            camera_to_target=transform,
            timestamp_ns=int(header.get("timestamp_ns", 0)),
            camera_frame=header.get("camera_frame", ""),
            target_frame=header.get("target_frame", ""),
            end_effector_to_target=end_effector_transform,
            end_effector_frame=header.get("end_effector_frame", ""),
        )
