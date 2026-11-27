#!/usr/bin/env python3

from basic_function.fr3_robot import get_robot
from basic_function.fr3_gripper import get_gripper
import time


def main():
    robot = get_robot()
    gripper = get_gripper()

    try:
        # if not robot.go_home():
        #     print("Failed to reach Home.")
        #     return

        # gripper.open()

        robot.go_home()

        robot.move_to_xyz(
                x=0.4,
                y=-0.4,
                z=0.0
            )

        robot.move_to_xyz(
            x=0.4,
            y=-0.4,
            z=-0.1065
        )

        time.sleep(0.1)

        # robot.go_home()

        gripper.close()

        robot.go_home()

        robot.move_to_xyz(
            x=0.6,
            y=0.2,
            z=-0.1065
        )

        gripper.open()

        time.sleep(0.1)

        robot.go_home()

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
        if hasattr(gripper, "shutdown"):
            gripper.shutdown()

        robot.shutdown()


if __name__ == "__main__":
    main()
