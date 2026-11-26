#!/usr/bin/env python3

from basic_function.fr3_robot import get_robot
from basic_function.fr3_gripper import get_gripper
import time


PICK_APPROACH = (0.4, -0.4, 0.0)
PICK_POSITION = (0.4, -0.4, -0.1065)
PLACE_POSITION = (0.6, 0.2, -0.1065)


def run_pick_and_place(
    robot,
    gripper,
    stage_callback=None,
    prepare_pick_object=None,
):
    """Run the canonical Cartesian pick-and-place sequence.

    Returns ``(success, failed_stage)``. ``stage_callback`` is called after
    every successful stage so a visual demo can pause at useful moments.
    ``prepare_pick_object`` runs after opening the gripper, which guarantees
    that an object left attached by an interrupted run is released before its
    Gazebo pose is reset.
    """

    def completed(stage):
        if stage_callback is not None:
            stage_callback(stage)

    stages = (
        ("home", robot.go_home),
        ("open", gripper.open),
    )
    for name, operation in stages:
        if not operation():
            return False, name
        completed(name)

    if prepare_pick_object is not None:
        if not prepare_pick_object():
            return False, "prepare_pick_object"
        completed("target_ready")

    motion_stages = (
        ("approach", lambda: robot.move_to_xyz(*PICK_APPROACH)),
        ("descend", lambda: robot.move_to_xyz(*PICK_POSITION)),
        ("grasp", gripper.close),
        ("lift", robot.go_home),
        ("place", lambda: robot.move_to_xyz(*PLACE_POSITION)),
        ("release", gripper.open),
    )
    for name, operation in motion_stages:
        if not operation():
            return False, name
        completed(name)
        if name in ("descend", "release"):
            time.sleep(0.1)

    if not robot.go_home():
        return False, "return_home"
    completed("return_home")
    return True, None


def main():
    robot = get_robot()
    gripper = get_gripper()

    try:
        success, failed_stage = run_pick_and_place(robot, gripper)
        if not success:
            print(f"Pick-and-place failed at stage: {failed_stage}")

        # robot.move_to_xyz(
        #     x=0.70,
        #     y=0.2,
        #     z=0.2
        # )

        # time.sleep(0.5)

        # robot.go_home()

        # gripper.home()

        # TCP 保持当前姿态，直线向下移动 5 cm。
        # success = robot.move_by_xyz(
        #     dz=-0.05,
        #     max_step=0.002,
        #     max_cartesian_speed=0.015,
        # )
        
        # z should fix at 0.3.

        # robot.move_to_xyz(
        #     x=0.30,
        #     y=0.0,
        #     z=-0.03,
        # )

        # robot.move_to_xyz(
        #     x=0.3,
        #     y=0.0,
        #     z=0.41,
        # )





        # robot.move_to_xyz(
        #     x=0.4,
        #     y=0.3,
        #     z=0.1,
        # )

        

        # robot.move_to_xyz(
        #     x=0.4,
        #     y=0.0,
        #     z=0.10,
        # )



        # gripper.close()

        # time.sleep(0.5)

        # robot.move_to_xyz(
        #     x=0.3,
        #     y=0.3,
        #     z=0.10,
        # )

        # robot.move_to_xyz(
        #     x=0.3,
        #     y=0.3,
        #     z=-0.03,
        # )

        # gripper.open()

        # robot.go_home()

        # robot.move_to_xyz(
        #     x=0.4,
        #     y=0.2,
        #     z=0.05,
        # )

        # if not success:
        #     print("Downward movement failed.")
        #     return

        # print("The gripper moved downward successfully.")

    finally:
        gripper.shutdown_gripper(False)

        robot.shutdown()


if __name__ == "__main__":
    main()
