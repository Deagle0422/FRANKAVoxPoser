#!/usr/bin/env python3

"""Automated FR3 Gazebo tests for safety behavior."""

import argparse
import math
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from basic_function.fr3_gripper import get_gripper
from basic_function.fr3_robot import get_robot


WORLD_NAME = 'fr3_pick_place'
PICK_OBJECT = 'pick_object'
PICK_POSITION = (0.4, -0.4, -0.1065)
PLACE_POSITION = (0.6, 0.2, -0.1065)
POSITION_TOLERANCE = 0.005
TABLE_X_BOUNDS = (0.25, 1.45)
TABLE_Y_BOUNDS = (-1.10, 0.90)
MINIMUM_TCP_Z = -0.1065
CONSERVATIVE_REACH_RADIUS = 0.85


class TestReport:
    def __init__(self):
        self.results = []

    def check(self, name, condition, detail=''):
        passed = bool(condition)
        self.results.append((name, passed, detail))
        label = 'PASS' if passed else 'FAIL'
        suffix = f' — {detail}' if detail else ''
        print(f'[{label}] {name}{suffix}')
        return passed

    @property
    def failure_count(self):
        return sum(not passed for _, passed, _ in self.results)

    def print_summary(self):
        passed = len(self.results) - self.failure_count
        print('\n========== TEST SUMMARY ==========')
        print(f'Passed: {passed}')
        print(f'Failed: {self.failure_count}')
        print(f'Total:  {len(self.results)}')


class SimulationProbe(Node):
    """Small test-only helper for resetting the Gazebo pick object."""

    def __init__(self):
        super().__init__('fr3_simulation_test_probe')
        self._set_pose = self.create_client(
            SetEntityPose,
            f'/world/{WORLD_NAME}/set_pose',
        )
        self._detach = self.create_publisher(
            Empty,
            '/fr3_gripper/sim_detach',
            10,
        )
        self.boundary_status = None
        self.boundary_stopped = None
        self.create_subscription(
            String,
            '/workspace_boundary/status',
            self._boundary_status_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/workspace_boundary/stopped',
            self._boundary_stopped_callback,
            10,
        )
        self._boundary_reset = self.create_client(
            Trigger,
            '/workspace_boundary/reset',
        )

    def _boundary_status_callback(self, message):
        self.boundary_status = message.data

    def _boundary_stopped_callback(self, message):
        self.boundary_stopped = bool(message.data)

    def wait_for_boundary_state(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate(self.boundary_status, self.boundary_stopped):
                return True
        return False

    def reset_workspace_stop(self):
        if not self._boundary_reset.wait_for_service(timeout_sec=5.0):
            return False, 'Workspace reset service is unavailable'
        future = self._boundary_reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return False, 'Workspace reset request timed out'
        return bool(future.result().success), future.result().message

    def reset_pick_object(self, position=PICK_POSITION):
        if not self._set_pose.wait_for_service(timeout_sec=5.0):
            return False, 'Gazebo set-pose service is unavailable'

        # Detach first so the fixed joint cannot pull the reset object back.
        for _ in range(5):
            self._detach.publish(Empty())
            time.sleep(0.05)

        request = SetEntityPose.Request()
        request.entity.name = PICK_OBJECT
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(position[0])
        request.pose.position.y = float(position[1])
        request.pose.position.z = float(position[2])
        request.pose.orientation.w = 1.0

        future = self._set_pose.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            return False, 'Gazebo set-pose request timed out'
        if not future.result().success:
            return False, 'Gazebo rejected the object reset'

        time.sleep(0.25)
        return True, ''


def read_pick_object_pose():
    """Read the physical Gazebo pose using the installed Ignition CLI."""

    completed = subprocess.run(
        ['ign', 'model', '-m', PICK_OBJECT, '-p'],
        check=False,
        capture_output=True,
        text=True,
        timeout=10.0,
    )
    output = completed.stdout + completed.stderr
    match = re.search(
        r'Pose \[ XYZ \(m\) \].*?\n\s*'
        r'\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]',
        output,
        flags=re.DOTALL,
    )
    if completed.returncode != 0 or match is None:
        raise RuntimeError(f'Could not read {PICK_OBJECT} pose: {output.strip()}')
    return tuple(float(match.group(index)) for index in range(1, 4))


def position_error(actual, target):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(actual[:3], target[:3])))


def validate_safe_target(x, y, z):
    """Reject clearly unsafe targets before they reach MoveIt or a controller."""

    values = (float(x), float(y), float(z))
    if not all(math.isfinite(value) for value in values):
        return False, 'target contains a non-finite coordinate'
    if z < MINIMUM_TCP_Z:
        return False, 'target is below the permitted tabletop TCP height'
    if math.hypot(x, y) > CONSERVATIVE_REACH_RADIUS:
        return False, 'target is outside the conservative radial reach limit'
    if z <= 0.10 and not (
        TABLE_X_BOUNDS[0] <= x <= TABLE_X_BOUNDS[1]
        and TABLE_Y_BOUNDS[0] <= y <= TABLE_Y_BOUNDS[1]
    ):
        return False, 'low target is outside the tabletop footprint'
    return True, ''


def guarded_move_to_xyz(robot, x, y, z):
    safe, reason = validate_safe_target(x, y, z)
    if not safe:
        return False, reason
    return bool(robot.move_to_xyz(x, y, z)), ''


def call_step(report, name, callback):
    try:
        result = bool(callback())
        return report.check(name, result)
    except Exception as error:
        report.check(name, False, str(error))
        return False


def run_smoke_suite(robot, gripper, probe, report):
    """Level 2: one complete pick, transfer, release, and Home cycle."""

    print('\n========== LEVEL 2: SMOKE =========')
    reset_ok, reset_error = probe.reset_pick_object()
    report.check('Reset cube to the pick position', reset_ok, reset_error)
    if not reset_ok:
        return

    if not call_step(report, 'Open gripper', gripper.open):
        return
    if not call_step(report, 'Move to Work Home', robot.go_home):
        return
    if not call_step(
        report,
        'Move above pick position',
        lambda: robot.move_to_xyz(PICK_POSITION[0], PICK_POSITION[1], 0.0),
    ):
        return
    if not call_step(
        report,
        'Descend to pick position',
        lambda: robot.move_to_xyz(*PICK_POSITION),
    ):
        return
    if not call_step(report, 'Grasp cube', gripper.close):
        return

    grasp_width = gripper.get_width()
    report.check(
        'Simulated gripper reaches the commanded closed state',
        grasp_width is not None and grasp_width <= 0.010,
        f'width={grasp_width}',
    )

    if not call_step(report, 'Carry cube through Work Home', robot.go_home):
        return
    if not call_step(
        report,
        'Move cube to place position',
        lambda: robot.move_to_xyz(*PLACE_POSITION),
    ):
        return
    if not call_step(report, 'Release cube', gripper.open):
        return

    time.sleep(0.3)
    try:
        cube_pose = read_pick_object_pose()
        error = position_error(cube_pose, PLACE_POSITION)
        report.check(
            'Cube placement error is within 5 mm',
            error <= POSITION_TOLERANCE,
            f'pose={cube_pose}, error={error * 1000.0:.2f} mm',
        )
    except Exception as error:
        report.check('Read final physical cube pose', False, str(error))

    report.check(
        'Gripper is open after release',
        gripper.gripper_is_open(),
        f'width={gripper.get_width()}',
    )
    call_step(report, 'Return to Work Home', robot.go_home)


def run_workspace_suite(robot, gripper, report):
    """Level 3: cover representative reachable points along the Y-long table."""

    print('\n======= LEVEL 3: WORKSPACE ========')
    call_step(report, 'Workspace setup: open gripper', gripper.open)
    if not call_step(report, 'Workspace setup: Work Home', robot.go_home):
        return

    # These points stay inside the 1.2 x 2.0 m tabletop and inside the
    # conservative FR3 reach envelope inferred from the nominal task.
    points = [
        ('near-negative-Y', (0.35, -0.55, 0.05)),
        ('centre-negative-Y', (0.55, -0.25, 0.05)),
        ('centre-positive-Y', (0.65, 0.15, 0.05)),
        ('near-positive-Y', (0.45, 0.40, 0.05)),
    ]

    for label, target in points:
        moved = call_step(
            report,
            f'Workspace motion: {label}',
            lambda target=target: robot.move_to_xyz(*target),
        )
        if not moved:
            call_step(report, 'Recover to Work Home', robot.go_home)
            continue

        try:
            actual = robot.get_actual_pose()
            error = position_error(actual, target)
            report.check(
                f'Workspace accuracy: {label}',
                error <= POSITION_TOLERANCE,
                f'error={error * 1000.0:.2f} mm',
            )
        except Exception as error:
            report.check(f'Workspace accuracy: {label}', False, str(error))

    call_step(report, 'Workspace finish: Work Home', robot.go_home)


def expect_value_error(report, name, callback):
    try:
        callback()
    except ValueError as error:
        report.check(name, True, str(error))
        return
    except Exception as error:
        report.check(name, False, f'wrong exception: {error}')
        return
    report.check(name, False, 'request was unexpectedly accepted')


def run_safety_suite(robot, gripper, report):
    """Level 4: verify local validation and MoveIt safety rejection paths."""

    print('\n========= LEVEL 4: SAFETY =========')
    if not call_step(report, 'Safety setup: Work Home', robot.go_home):
        return

    expect_value_error(
        report,
        'Reject gripper width above 75 mm',
        lambda: gripper.open(width=0.09),
    )
    expect_value_error(
        report,
        'Reject zero gripper speed',
        lambda: gripper.open(speed=0.0),
    )
    report.check(
        'Reject empty Cartesian trajectory',
        not robot.execute_cartesian_trajectory([]),
    )

    before = robot.get_actual_pose()
    below_table_accepted, below_table_reason = guarded_move_to_xyz(
        robot, 0.4, -0.4, -0.20
    )
    after = robot.get_actual_pose()
    report.check(
        'Preflight rejects Cartesian target below the tabletop',
        not below_table_accepted,
        below_table_reason,
    )
    report.check(
        'Rejected table-collision request does not move the TCP',
        position_error(after, before) <= 0.010,
        f'drift={position_error(after, before) * 1000.0:.2f} mm',
    )

    before = robot.get_actual_pose()
    unreachable_accepted, unreachable_reason = guarded_move_to_xyz(
        robot, 1.6, 0.0, 0.20
    )
    after = robot.get_actual_pose()
    report.check(
        'Preflight rejects target outside conservative reach',
        not unreachable_accepted,
        unreachable_reason,
    )
    report.check(
        'Rejected unreachable request does not move the TCP',
        position_error(after, before) <= 0.010,
        f'drift={position_error(after, before) * 1000.0:.2f} mm',
    )

    call_step(report, 'Safety finish: Work Home', robot.go_home)


def run_boundary_suite(robot, probe, report):
    """Verify planning rejection and independent runtime trajectory stopping."""

    print('\n====== LEVEL 5: MOVE BOUNDARY =====')
    reset_ok, reset_message = probe.reset_workspace_stop()
    report.check(
        'Boundary supervisor reset before test',
        reset_ok,
        reset_message,
    )
    if not reset_ok:
        return
    if not call_step(report, 'Boundary setup: Work Home', robot.go_home):
        return

    boundary = robot.workspace_boundary
    start = robot.get_actual_pose()
    outside_target = (
        boundary.hard_min[0] - 0.03,
        start[1],
        start[2],
    )

    # Layer 1: the normal API rejects the target before asking MoveIt to plan.
    normally_accepted = robot.move_to_xyz(*outside_target)
    after_rejection = robot.get_actual_pose()
    report.check(
        'Planning layer rejects a target outside the keep-in box',
        not normally_accepted,
    )
    report.check(
        'Planning-layer rejection does not move the TCP',
        position_error(after_rejection, start) <= 0.010,
        f'drift={position_error(after_rejection, start) * 1000.0:.2f} mm',
    )

    # Layer 2 demonstration: deliberately bypass only the planning guard. The
    # independent supervisor must cancel execution before the hard envelope.
    bypass_result = robot.move_to_xyz(
        *outside_target,
        enforce_workspace_boundary=False,
    )
    stopped = probe.wait_for_boundary_state(
        lambda status, is_stopped: status == 'STOPPED' and is_stopped is True,
        timeout=5.0,
    )
    stopped_pose = robot.get_actual_pose()
    report.check(
        'Runtime supervisor cancels a plan that ignored the boundary',
        stopped and not bypass_result,
        f'status={probe.boundary_status}, action_result={bypass_result}',
    )
    report.check(
        'Runtime stop keeps the measured TCP inside the hard envelope',
        boundary.contains(stopped_pose[:3], region='hard'),
        f'TCP={stopped_pose[:3]}, hard={boundary.hard_min}..{boundary.hard_max}',
    )

    reset_ok, reset_message = probe.reset_workspace_stop()
    report.check('Reset latched runtime stop', reset_ok, reset_message)
    if reset_ok:
        call_step(report, 'Boundary recovery: Work Home', robot.go_home)
        rearmed = probe.wait_for_boundary_state(
            lambda status, is_stopped: (
                status in ('SAFE', 'WARNING') and is_stopped is False
            ),
            timeout=5.0,
        )
        report.check(
            'Supervisor re-arms after TCP returns to the planning box',
            rearmed,
            f'status={probe.boundary_status}',
        )


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--suite',
        choices=('smoke', 'workspace', 'safety', 'boundary', 'all'),
        default='all',
        help='Test suite to execute (default: all)',
    )
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    report = TestReport()
    robot = get_robot()
    gripper = get_gripper()
    probe = SimulationProbe()

    try:
        if arguments.suite in ('smoke', 'all'):
            run_smoke_suite(robot, gripper, probe, report)
        if arguments.suite in ('workspace', 'all'):
            run_workspace_suite(robot, gripper, report)
        if arguments.suite in ('safety', 'all'):
            run_safety_suite(robot, gripper, report)
        if arguments.suite in ('boundary', 'all'):
            run_boundary_suite(robot, probe, report)
    finally:
        try:
            probe.reset_workspace_stop()
            gripper.open()
            robot.go_home()
        except Exception as error:
            print(f'Cleanup warning: {error}')
        try:
            probe.destroy_node()
        except Exception:
            pass
        gripper.shutdown_gripper(shutdown_ros=False)
        robot.shutdown()

    report.print_summary()
    return 1 if report.failure_count else 0


if __name__ == '__main__':
    raise SystemExit(main())
