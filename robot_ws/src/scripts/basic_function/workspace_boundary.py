#!/usr/bin/env python3

"""Shared loader and geometry helpers for the FR3 TCP workspace boundary."""

import math
import os
from dataclasses import dataclass
from pathlib import Path

import yaml


CONFIG_RELATIVE_PATH = Path("config/workspace_boundary.yaml")


@dataclass(frozen=True)
class WorkspaceBoundary:
    frame_id: str
    tcp_link: str
    hard_min: tuple
    hard_max: tuple
    planning_min: tuple
    planning_max: tuple
    monitor_rate_hz: float
    marker_rate_hz: float
    warning_distance: float
    cancel_repeat_seconds: float

    def contains(self, point, region="planning"):
        """Return whether XYZ is finite and inside the selected box."""

        values = tuple(float(value) for value in point[:3])
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            return False

        if region == "planning":
            lower, upper = self.planning_min, self.planning_max
        elif region == "hard":
            lower, upper = self.hard_min, self.hard_max
        else:
            raise ValueError(f"Unknown workspace region: {region}")

        return all(
            minimum <= value <= maximum
            for value, minimum, maximum in zip(values, lower, upper)
        )

    def violation(self, point, region="planning"):
        """Describe the first violated face, or return an empty string."""

        values = tuple(float(value) for value in point[:3])
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            return "XYZ contains a non-finite coordinate"

        if region == "planning":
            lower, upper = self.planning_min, self.planning_max
        elif region == "hard":
            lower, upper = self.hard_min, self.hard_max
        else:
            raise ValueError(f"Unknown workspace region: {region}")

        for axis, value, minimum, maximum in zip("XYZ", values, lower, upper):
            if value < minimum:
                return f"{axis}={value:.4f} is below {minimum:.4f} m"
            if value > maximum:
                return f"{axis}={value:.4f} is above {maximum:.4f} m"
        return ""


def _config_candidates():
    override = os.environ.get("FR3_WORKSPACE_BOUNDARY_CONFIG")
    if override:
        yield Path(override)

    try:
        from ament_index_python.packages import get_package_share_directory

        yield (
            Path(get_package_share_directory("franka_fr3_moveit_config"))
            / CONFIG_RELATIVE_PATH
        )
    except (ImportError, LookupError):
        pass

    # Source-tree fallback used by scripts launched directly from /ros2_ws/src.
    yield (
        Path(__file__).resolve().parents[2]
        / "franka_fr3_moveit_config"
        / CONFIG_RELATIVE_PATH
    )


def find_workspace_boundary_config():
    for candidate in _config_candidates():
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "Could not find config/workspace_boundary.yaml. Build and source "
        "franka_fr3_moveit_config or set FR3_WORKSPACE_BOUNDARY_CONFIG."
    )


def load_workspace_boundary():
    config_path = find_workspace_boundary_config()
    with config_path.open("r", encoding="utf-8") as config_file:
        document = yaml.safe_load(config_file)

    values = document["workspace_boundary"]

    def vector(name):
        result = tuple(float(value) for value in values[name])
        if len(result) != 3 or not all(math.isfinite(value) for value in result):
            raise ValueError(f"workspace_boundary.{name} must contain 3 finite values")
        return result

    boundary = WorkspaceBoundary(
        frame_id=str(values["frame_id"]),
        tcp_link=str(values["tcp_link"]),
        hard_min=vector("hard_min"),
        hard_max=vector("hard_max"),
        planning_min=vector("planning_min"),
        planning_max=vector("planning_max"),
        monitor_rate_hz=float(values["monitor_rate_hz"]),
        marker_rate_hz=float(values["marker_rate_hz"]),
        warning_distance=float(values["warning_distance"]),
        cancel_repeat_seconds=float(values["cancel_repeat_seconds"]),
    )

    for axis, hard_minimum, planning_minimum, planning_maximum, hard_maximum in zip(
        "XYZ",
        boundary.hard_min,
        boundary.planning_min,
        boundary.planning_max,
        boundary.hard_max,
    ):
        if not hard_minimum < planning_minimum < planning_maximum < hard_maximum:
            raise ValueError(
                f"Invalid {axis} limits: expected hard_min < planning_min < "
                "planning_max < hard_max"
            )

    if boundary.monitor_rate_hz <= 0.0 or boundary.marker_rate_hz <= 0.0:
        raise ValueError("Workspace monitor and marker rates must be positive")
    if boundary.warning_distance < 0.0:
        raise ValueError("Workspace warning distance cannot be negative")
    if boundary.cancel_repeat_seconds <= 0.0:
        raise ValueError("Workspace cancel repeat period must be positive")

    return boundary
