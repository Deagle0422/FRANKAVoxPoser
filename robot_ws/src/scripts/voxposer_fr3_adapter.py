#!/usr/bin/env python3

"""VoxPoser-facing adapter around the unmodified FR3 public control API."""

import math
import os
import time
from dataclasses import dataclass
from pathlib import Path

from basic_function.fr3_gripper import get_gripper
from basic_function.fr3_robot import get_robot


PICK_APPROACH = (0.4, -0.4, 0.0)
PICK_POSITION = (0.4, -0.4, -0.1065)
PLACE_POSITION = (0.6, 0.2, -0.1065)
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
        return not self.violation(point, region=region)

    def violation(self, point, region="planning"):
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
    yield Path(__file__).resolve().parents[1] / "franka_fr3_moveit_config" / CONFIG_RELATIVE_PATH


def load_workspace_boundary():
    import yaml

    config_path = next(
        (candidate for candidate in _config_candidates() if candidate.is_file()),
        None,
    )
    if config_path is None:
        raise FileNotFoundError(
            "Could not find config/workspace_boundary.yaml; set "
            "FR3_WORKSPACE_BOUNDARY_CONFIG to an explicit file."
        )
    with config_path.open("r", encoding="utf-8") as config_file:
        values = yaml.safe_load(config_file)["workspace_boundary"]

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
    for axis, hard_min, plan_min, plan_max, hard_max in zip(
        "XYZ",
        boundary.hard_min,
        boundary.planning_min,
        boundary.planning_max,
        boundary.hard_max,
    ):
        if not hard_min < plan_min < plan_max < hard_max:
            raise ValueError(f"Invalid {axis} workspace limits")
    return boundary


class VoxPoserFR3Adapter:
    """Validate VoxPoser targets, then call only the native public methods."""

    def __init__(
        self,
        robot=None,
        gripper=None,
        max_step=0.01,
        max_cartesian_speed=0.02,
        minimum_fraction=0.99,
        workspace_boundary=None,
    ):
        self.robot = get_robot() if robot is None else robot
        self.gripper = gripper
        self.max_step = float(max_step)
        self.max_cartesian_speed = float(max_cartesian_speed)
        self.minimum_fraction = float(minimum_fraction)
        self.workspace_boundary = (
            load_workspace_boundary()
            if workspace_boundary is None else workspace_boundary
        )
        if self.robot.base_frame != self.workspace_boundary.frame_id:
            raise ValueError("Workspace frame does not match the native robot base frame")
        if self.robot.ee_link != self.workspace_boundary.tcp_link:
            raise ValueError("Workspace TCP link does not match the native robot EE link")

    @classmethod
    def create(cls, with_gripper=True, **kwargs):
        return cls(
            robot=get_robot(),
            gripper=get_gripper() if with_gripper else None,
            **kwargs,
        )

    def __getattr__(self, name):
        """Delegate unchanged read/query APIs such as go_home/get_actual_pose."""

        return getattr(self.robot, name)

    def move_to_xyz(
        self,
        x,
        y,
        z,
        max_step=None,
        max_cartesian_speed=None,
        avoid_collisions=True,
        minimum_fraction=None,
        enforce_workspace_boundary=True,
    ):
        target = (float(x), float(y), float(z))
        if enforce_workspace_boundary:
            violation = self.workspace_boundary.violation(target, region="planning")
            if violation:
                self.robot.get_logger().error(
                    f"VoxPoser adapter rejected target: {violation}"
                )
                return False
        return self.robot.move_to_xyz(
            *target,
            max_step=self.max_step if max_step is None else float(max_step),
            max_cartesian_speed=(
                self.max_cartesian_speed
                if max_cartesian_speed is None else float(max_cartesian_speed)
            ),
            avoid_collisions=bool(avoid_collisions),
            minimum_fraction=(
                self.minimum_fraction
                if minimum_fraction is None else float(minimum_fraction)
            ),
        )

    def move_to(self, state_name, target):
        xyz = tuple(float(value) for value in target)
        print(f"\n[state] {state_name}: adapter -> robot.move_to_xyz{xyz}")
        if not self.move_to_xyz(*xyz):
            raise RuntimeError(f"{state_name}: robot.move_to_xyz() failed")
        print(f"[state] {state_name}: PASS")

    def open_gripper(self, state_name="OPEN_GRIPPER"):
        if self.gripper is None:
            raise RuntimeError("This adapter has no gripper")
        print(f"\n[state] {state_name}: adapter -> gripper.open()")
        if not self.gripper.open():
            raise RuntimeError(f"{state_name}: gripper.open() failed")
        print(f"[state] {state_name}: PASS")

    def close_gripper(self):
        if self.gripper is None:
            raise RuntimeError("This adapter has no gripper")
        print("\n[state] CLOSE_GRIPPER: adapter -> gripper.close()")
        if not self.gripper.close():
            raise RuntimeError("CLOSE_GRIPPER: gripper.close() failed")
        print("[state] CLOSE_GRIPPER: PASS")

    def shutdown(self):
        if self.gripper is not None:
            self.gripper.shutdown_gripper(shutdown_ros=False)
            self.gripper = None
        if self.robot is not None:
            self.robot.shutdown()
            self.robot = None


def run_pick_and_place(
    robot,
    gripper,
    stage_callback=None,
    prepare_pick_object=None,
):
    """Simulation demo helper using only adapter/native public calls."""

    def completed(stage):
        if stage_callback is not None:
            stage_callback(stage)

    for name, operation in (("home", robot.go_home), ("open", gripper.open)):
        if not operation():
            return False, name
        completed(name)
    if prepare_pick_object is not None:
        if not prepare_pick_object():
            return False, "prepare_pick_object"
        completed("target_ready")
    stages = (
        ("approach", lambda: robot.move_to_xyz(*PICK_APPROACH)),
        ("descend", lambda: robot.move_to_xyz(*PICK_POSITION)),
        ("grasp", gripper.close),
        ("lift", robot.go_home),
        ("place", lambda: robot.move_to_xyz(*PLACE_POSITION)),
        ("release", gripper.open),
    )
    for name, operation in stages:
        if not operation():
            return False, name
        completed(name)
        if name in ("descend", "release"):
            time.sleep(0.1)
    if not robot.go_home():
        return False, "return_home"
    completed("return_home")
    return True, None
