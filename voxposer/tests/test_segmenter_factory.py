import io
import os
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
from unittest import mock

import numpy as np

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from perception import ColorRuleSegmenter, RemoteSegmenter, build_segmenter
from perception.vision_service import make_handler


class _FakeResponse:
    def __init__(self, array):
        payload = io.BytesIO()
        np.save(payload, array, allow_pickle=False)
        self.payload = payload.getvalue()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return self.payload


class SegmenterFactoryTest(unittest.TestCase):
    def test_builds_color_rules(self):
        segmenter = build_segmenter({
            "type": "color_rules",
            "objects": {"cube": {"dominant_channel": "red"}},
        })
        self.assertIsInstance(segmenter, ColorRuleSegmenter)

    def test_builds_remote_and_sends_configured_prompt(self):
        segmenter = build_segmenter({
            "type": "remote_sam2",
            "service": {"endpoint": "http://127.0.0.1:8770", "timeout_sec": 7},
            "objects": {
                "pick_object": {
                    "aliases": ["red cube"],
                    "prompt": "the red cube on the table",
                }
            },
        })
        self.assertIsInstance(segmenter, RemoteSegmenter)
        image = np.zeros((2, 3, 3), dtype=np.uint8)
        expected = np.array([[True, False, False], [False, True, False]])
        with mock.patch(
            "perception.remote_segmenter.request.urlopen",
            return_value=_FakeResponse(expected),
        ) as urlopen:
            actual = segmenter("red cube", image)
        np.testing.assert_array_equal(actual, expected)
        sent_request = urlopen.call_args.args[0]
        self.assertIn("query=the+red+cube+on+the+table", sent_request.full_url)
        self.assertEqual(urlopen.call_args.kwargs["timeout"], 7.0)

    def test_rejects_unknown_type(self):
        with self.assertRaisesRegex(ValueError, "Unsupported segmentation"):
            build_segmenter({"type": "magic", "objects": {"cube": {}}})

    def test_remote_client_and_http_service_round_trip(self):
        received = {}

        def backend(query, image):
            received["query"] = query
            received["image"] = image.copy()
            return image[:, :, 0] > 100

        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(backend))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            segmenter = RemoteSegmenter(
                {"cube": {"aliases": ["red block"], "prompt": "a red cube"}},
                endpoint=f"http://127.0.0.1:{server.server_port}",
                timeout=2,
            )
            image = np.array([[[200, 0, 0], [10, 0, 0]]], dtype=np.uint8)
            mask = segmenter("red block", image)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

        self.assertEqual(received["query"], "a red cube")
        np.testing.assert_array_equal(received["image"], image)
        np.testing.assert_array_equal(mask, [[True, False]])


if __name__ == "__main__":
    unittest.main()
