"""Client for an isolated open-vocabulary segmentation service."""

import io
from typing import Dict, Iterable, Mapping
from urllib import error, parse, request

import numpy as np


class RemoteSegmenter:
    """Resolve object aliases and request a mask from the vision container."""

    def __init__(
        self,
        object_configs: Mapping[str, Mapping],
        endpoint: str = "http://127.0.0.1:8770",
        timeout: float = 60.0,
    ):
        self.endpoint = endpoint.rstrip("/")
        self.timeout = float(timeout)
        self._canonical_names: Dict[str, str] = {}
        self._prompts: Dict[str, str] = {}
        for canonical_name, config in object_configs.items():
            prompt = str(config.get("prompt", canonical_name.replace("_", " ")))
            names: Iterable[str] = [canonical_name, *config.get("aliases", [])]
            for name in names:
                key = self._normalize(name)
                previous = self._canonical_names.get(key)
                if previous is not None and previous != canonical_name:
                    raise ValueError(f"Duplicate segmentation alias: {name}")
                self._canonical_names[key] = canonical_name
            self._prompts[canonical_name] = prompt

    @staticmethod
    def _normalize(name: str) -> str:
        return " ".join(str(name).lower().replace("_", " ").split())

    @property
    def object_names(self):
        return sorted(set(self._canonical_names.values()))

    def canonical_name(self, object_name: str) -> str:
        key = self._normalize(object_name)
        if key not in self._canonical_names:
            raise ValueError(
                f"Unknown object {object_name!r}; available objects: "
                f"{', '.join(self.object_names)}"
            )
        return self._canonical_names[key]

    def __call__(self, object_name: str, rgb_image: np.ndarray) -> np.ndarray:
        image = np.asarray(rgb_image)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("RGB image must have shape (height, width, 3)")
        if image.dtype != np.uint8:
            image = np.clip(image, 0, 255).astype(np.uint8)

        canonical_name = self.canonical_name(object_name)
        query = parse.urlencode({"query": self._prompts[canonical_name]})
        payload = io.BytesIO()
        np.save(payload, image, allow_pickle=False)
        http_request = request.Request(
            f"{self.endpoint}/segment?{query}",
            data=payload.getvalue(),
            method="POST",
            headers={"Content-Type": "application/x-npy"},
        )
        try:
            with request.urlopen(http_request, timeout=self.timeout) as response:
                mask = np.load(io.BytesIO(response.read()), allow_pickle=False)
        except error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                f"Vision service rejected {canonical_name!r}: {detail or exc.reason}"
            ) from exc
        except error.URLError as exc:
            raise RuntimeError(
                f"Cannot reach vision service at {self.endpoint}: {exc.reason}"
            ) from exc

        mask = np.asarray(mask, dtype=bool)
        if mask.shape != image.shape[:2]:
            raise RuntimeError(
                f"Vision service returned mask shape {mask.shape}; "
                f"expected {image.shape[:2]}"
            )
        return mask
