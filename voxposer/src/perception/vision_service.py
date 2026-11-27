"""Small loopback HTTP service for the isolated OWLv2 + SAM 2 backend."""

import argparse
import io
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib import parse

import numpy as np

from .open_vocabulary_sam2 import OpenVocabularySAM2


MAX_IMAGE_BYTES = 32 * 1024 * 1024


def make_handler(backend):
    class SegmentationHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/healthz":
                self._send_bytes(200, b"ok", "text/plain")
            else:
                self._send_bytes(404, b"not found", "text/plain")

        def do_POST(self):
            url = parse.urlparse(self.path)
            query = parse.parse_qs(url.query).get("query", [""])[0].strip()
            if url.path != "/segment" or not query:
                self._send_bytes(400, b"missing /segment?query=...", "text/plain")
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > MAX_IMAGE_BYTES:
                    raise ValueError("invalid image payload size")
                image = np.load(io.BytesIO(self.rfile.read(length)), allow_pickle=False)
                mask = np.asarray(backend(query, image), dtype=bool)
                if mask.shape != image.shape[:2]:
                    raise ValueError("backend returned a mask with the wrong shape")
                payload = io.BytesIO()
                np.save(payload, mask, allow_pickle=False)
                self._send_bytes(200, payload.getvalue(), "application/x-npy")
            except Exception as exc:
                self._send_bytes(422, str(exc).encode("utf-8"), "text/plain")

        def _send_bytes(self, status, payload, content_type):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, format_string, *args):
            print(f"vision-service: {format_string % args}", flush=True)

    return SegmentationHandler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8770)
    parser.add_argument(
        "--detector-model", default="google/owlv2-base-patch16-ensemble"
    )
    parser.add_argument("--sam2-model", default="facebook/sam2.1-hiera-tiny")
    parser.add_argument("--detection-threshold", type=float, default=0.15)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()

    backend = OpenVocabularySAM2(
        detector_model=args.detector_model,
        sam2_model=args.sam2_model,
        detection_threshold=args.detection_threshold,
        device=args.device,
    )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(backend))
    print(
        f"OWLv2 + SAM 2 vision service listening on {args.host}:{args.port}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
