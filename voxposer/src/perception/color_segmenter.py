"""Configurable RGB color segmentation for deterministic simulation objects."""

from dataclasses import dataclass
from typing import Dict, Iterable, Mapping

import numpy as np


@dataclass(frozen=True)
class ColorRule:
    """Select pixels whose named channel dominates the other RGB channels."""

    channel: int
    min_value: float
    dominance_ratio: float

    def apply(self, image: np.ndarray) -> np.ndarray:
        rgb = np.asarray(image)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("RGB image must have shape (height, width, 3)")
        rgb = rgb.astype(np.float32, copy=False)
        selected = rgb[:, :, self.channel]
        others = np.delete(rgb, self.channel, axis=2)
        return (selected >= self.min_value) & (
            selected >= self.dominance_ratio * np.max(others, axis=2)
        )


class ColorRuleSegmenter:
    """Resolve object aliases and return a boolean mask from configured rules."""

    CHANNELS = {"red": 0, "green": 1, "blue": 2}

    def __init__(self, object_configs: Mapping[str, Mapping]):
        self._rules: Dict[str, ColorRule] = {}
        self._canonical_names: Dict[str, str] = {}
        for canonical_name, config in object_configs.items():
            channel_name = str(config["dominant_channel"]).lower()
            if channel_name not in self.CHANNELS:
                raise ValueError(f"Unsupported dominant channel: {channel_name}")
            rule = ColorRule(
                channel=self.CHANNELS[channel_name],
                min_value=float(config.get("min_value", 100)),
                dominance_ratio=float(config.get("dominance_ratio", 1.4)),
            )
            names: Iterable[str] = [canonical_name, *config.get("aliases", [])]
            for name in names:
                key = self._normalize(name)
                if key in self._rules:
                    if self._canonical_names[key] != canonical_name:
                        raise ValueError(f"Duplicate segmentation alias: {name}")
                    continue
                self._rules[key] = rule
                self._canonical_names[key] = canonical_name

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
        key = self._normalize(object_name)
        if key not in self._rules:
            self.canonical_name(object_name)
        return self._rules[key].apply(rgb_image)
