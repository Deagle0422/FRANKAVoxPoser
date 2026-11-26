#!/usr/bin/env python3

"""Add the inferred work table to the MoveIt planning scene."""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class PickPlaceScenePublisher(Node):
    def __init__(self):
        super().__init__('pick_place_scene_publisher')
        self._client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
        )

    def apply(self):
        if not self._client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(
                'MoveIt apply_planning_scene service is unavailable.'
            )
            return False

        table = CollisionObject()
        table.header.frame_id = 'fr3_link0'
        table.id = 'work_table'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.2, 2.0, 0.05]

        pose = Pose()
        pose.position.x = 0.85
        pose.position.y = -0.1
        pose.position.z = -0.1515
        pose.orientation.w = 1.0

        table.primitives = [box]
        table.primitive_poses = [pose]
        table.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [table]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done() or future.result() is None:
            self.get_logger().error('Applying the work table timed out.')
            return False
        if not future.result().success:
            self.get_logger().error('MoveIt rejected the work table scene.')
            return False

        self.get_logger().info(
            'Added 1.2 x 2.0 m work table to the MoveIt planning scene.'
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceScenePublisher()
    try:
        node.apply()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
