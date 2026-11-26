#!/usr/bin/env python3

"""Franka Hand action compatibility server backed by a Gazebo controller."""

import copy

import rclpy
from control_msgs.action import FollowJointTrajectory
from franka_msgs.action import Grasp, Homing, Move
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.clock import Clock, ClockType
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class SimulatedFrankaGripper(Node):
    """Expose the real Franka gripper API while commanding Gazebo joints."""

    MAX_WIDTH = 0.08
    PICK_OBJECT_SIZE = 0.04
    # A 1 mm clearance per face prevents an exact table-contact boundary from
    # making Cartesian collision checking numerically ambiguous. Gazebo keeps
    # the physical 40 mm cube.
    PLANNING_OBJECT_SIZE = 0.038
    PICK_OBJECT_ID = 'pick_object'
    CONTROLLED_JOINT = 'fr3_finger_joint1'
    MIMIC_JOINT = 'fr3_finger_joint2'

    def __init__(self):
        super().__init__('fr3_gripper')

        self._callback_group = ReentrantCallbackGroup()
        self._current_width = 0.0
        self._have_gripper_state = False
        self._holding_pick_object = False
        self._attachment_state = 'unknown'
        self._startup_detach_attempts = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._attach_publisher = self.create_publisher(
            Empty,
            '/fr3_gripper/sim_attach',
            10,
        )
        self._detach_publisher = self.create_publisher(
            Empty,
            '/fr3_gripper/sim_detach',
            10,
        )
        self.create_subscription(
            String,
            '/fr3_gripper/sim_attachment_state',
            self._attachment_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._planning_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
            callback_group=self._callback_group,
        )
        self._set_pose_client = self.create_client(
            SetEntityPose,
            '/world/fr3_pick_place/set_pose',
            callback_group=self._callback_group,
        )
        self._startup_detach_timer = self.create_timer(
            0.2,
            self._ensure_initially_detached,
            callback_group=self._callback_group,
            # The Gazebo world does not bridge /clock, so startup handshaking
            # uses a steady wall clock.
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/fr3_gripper_controller/follow_joint_trajectory',
            callback_group=self._callback_group,
        )

        self._arm_joint_state_publisher = self.create_publisher(
            JointState,
            '/franka/joint_states',
            qos_profile_sensor_data,
        )
        self._gripper_joint_state_publisher = self.create_publisher(
            JointState,
            '~/joint_states',
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )

        self._homing_server = ActionServer(
            self,
            Homing,
            '~/homing',
            execute_callback=self._execute_homing,
            callback_group=self._callback_group,
        )
        self._move_server = ActionServer(
            self,
            Move,
            '~/move',
            execute_callback=self._execute_move,
            goal_callback=self._validate_move_goal,
            callback_group=self._callback_group,
        )
        self._grasp_server = ActionServer(
            self,
            Grasp,
            '~/grasp',
            execute_callback=self._execute_grasp,
            goal_callback=self._validate_grasp_goal,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            'Simulated Franka gripper compatibility server is ready.'
        )

    def _attachment_state_callback(self, message):
        state = message.data.strip().lower()
        if state and state != self._attachment_state:
            self._attachment_state = state
            self.get_logger().info(f'Gazebo detachable joint state: {state}.')

    def _ensure_initially_detached(self):
        """Detach the plugin's default joint after ROS/Gazebo discovery."""

        self._detach_publisher.publish(Empty())
        self._startup_detach_attempts += 1
        if self._startup_detach_attempts >= 10:
            self._startup_detach_timer.cancel()

    def _attach_sim_object(self):
        self._attach_publisher.publish(Empty())
        self.get_logger().info(
            'Requested Gazebo fixed-joint attachment for the pick object.'
        )

    def _detach_sim_object(self):
        self._detach_publisher.publish(Empty())
        self.get_logger().info(
            'Requested Gazebo fixed-joint release for the pick object.'
        )

    async def _snap_sim_object_to_tcp(self):
        """Synchronize the cube once before creating the native fixed joint."""

        if not self._set_pose_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                'Gazebo set-pose service is unavailable; attaching at the '
                'current physical pose.'
            )
            return False

        try:
            transform = self._tf_buffer.lookup_transform(
                'fr3_link0',
                'fr3_hand_tcp',
                Time(),
            )
        except TransformException as error:
            self.get_logger().error(
                f'Could not synchronize the pick object to the TCP: {error}'
            )
            return False

        request = SetEntityPose.Request()
        request.entity.name = self.PICK_OBJECT_ID
        request.entity.type = Entity.MODEL
        request.pose.position.x = transform.transform.translation.x
        request.pose.position.y = transform.transform.translation.y
        request.pose.position.z = transform.transform.translation.z
        # The cube begins axis-aligned and the pick/place TCP orientations are
        # equivalent. Keeping this orientation also matches the MoveIt object.
        request.pose.orientation.w = 1.0

        try:
            response = await self._set_pose_client.call_async(request)
        except Exception as error:
            self.get_logger().error(
                f'Gazebo rejected pick-object pose synchronization: {error}'
            )
            return False

        if response is None or not response.success:
            self.get_logger().error(
                'Gazebo could not synchronize the pick object to the TCP.'
            )
            return False

        self.get_logger().info(
            'Synchronized pick object to the TCP before fixed-joint attachment.'
        )
        return True

    def _cube_collision_object(self, frame_id, pose):
        """Build the MoveIt representation of the physical Gazebo cube."""

        cube = CollisionObject()
        cube.header.frame_id = frame_id
        cube.id = self.PICK_OBJECT_ID

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self.PLANNING_OBJECT_SIZE,
            self.PLANNING_OBJECT_SIZE,
            self.PLANNING_OBJECT_SIZE,
        ]
        cube.primitives = [primitive]
        cube.primitive_poses = [pose]
        cube.operation = CollisionObject.ADD
        return cube

    def _apply_planning_scene(self, scene, description):
        """Apply a small planning-scene diff without blocking an action."""

        if not self._planning_scene_client.service_is_ready():
            self.get_logger().warning(
                f'MoveIt is unavailable; could not {description}.'
            )
            return

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._planning_scene_client.call_async(request)

        def report_result(completed):
            try:
                response = completed.result()
            except Exception as error:
                self.get_logger().error(
                    f'Could not {description}: {error}'
                )
                return
            if response is None or not response.success:
                self.get_logger().error(f'MoveIt rejected request to {description}.')
                return
            self.get_logger().info(f'MoveIt planning scene: {description}.')

        future.add_done_callback(report_result)

    def _attach_planning_object(self):
        """Attach the cube to the hand so carried-object collisions are checked."""

        pose = Pose()
        pose.orientation.w = 1.0

        attached = AttachedCollisionObject()
        attached.link_name = 'fr3_hand_tcp'
        attached.object = self._cube_collision_object('fr3_hand_tcp', pose)
        attached.touch_links = [
            'fr3_link7',
            'fr3_link8',
            'fr3_hand',
            'fr3_hand_tcp',
            'fr3_leftfinger',
            'fr3_rightfinger',
        ]

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        self._apply_planning_scene(scene, 'attached pick object to the hand')

    def _detach_planning_object(self):
        """Detach the cube and leave it in the world at the current TCP pose."""

        try:
            transform = self._tf_buffer.lookup_transform(
                'fr3_link0',
                'fr3_hand_tcp',
                Time(),
            )
        except TransformException as error:
            self.get_logger().error(
                f'Could not place released object in MoveIt: {error}'
            )
            return

        remove = AttachedCollisionObject()
        remove.link_name = 'fr3_hand_tcp'
        remove.object.id = self.PICK_OBJECT_ID
        remove.object.operation = CollisionObject.REMOVE

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation.w = 1.0

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [remove]
        scene.world.collision_objects = [
            self._cube_collision_object('fr3_link0', pose)
        ]
        self._apply_planning_scene(scene, 'released pick object into the world')

    def _joint_state_callback(self, message):
        # The arm client intentionally uses the same topic as on hardware.
        self._arm_joint_state_publisher.publish(message)

        finger_position = None
        for name, position in zip(message.name, message.position):
            if name == self.CONTROLLED_JOINT:
                finger_position = max(0.0, float(position))
                break

        if finger_position is None:
            return

        self._current_width = min(self.MAX_WIDTH, 2.0 * finger_position)
        self._have_gripper_state = True

        gripper_state = JointState()
        gripper_state.header = copy.deepcopy(message.header)
        gripper_state.name = [self.CONTROLLED_JOINT, self.MIMIC_JOINT]
        gripper_state.position = [finger_position, finger_position]
        gripper_state.velocity = [0.0, 0.0]
        gripper_state.effort = [0.0, 0.0]
        self._gripper_joint_state_publisher.publish(gripper_state)

    def _validate_move_goal(self, goal):
        return self._validate_width_and_speed(goal.width, goal.speed)

    def _validate_grasp_goal(self, goal):
        if goal.force <= 0.0:
            self.get_logger().error('Grasp force must be greater than zero.')
            return GoalResponse.REJECT
        return self._validate_width_and_speed(goal.width, goal.speed)

    def _validate_width_and_speed(self, width, speed):
        if width < 0.0 or width > self.MAX_WIDTH:
            self.get_logger().error(
                f'Gripper width {width:.4f} is outside [0, {self.MAX_WIDTH:.2f}] m.'
            )
            return GoalResponse.REJECT
        if speed <= 0.0:
            self.get_logger().error('Gripper speed must be greater than zero.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _command_width(
        self,
        width,
        speed,
        feedback_callback,
        accepted_width_range=None,
    ):
        if not self._trajectory_client.wait_for_server(timeout_sec=10.0):
            return False, 'Gazebo gripper trajectory controller is unavailable.'

        target_width = min(self.MAX_WIDTH, max(0.0, float(width)))
        start_width = self._current_width if self._have_gripper_state else 0.0
        duration = max(0.25, abs(target_width - start_width) / float(speed))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.CONTROLLED_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [target_width / 2.0]
        seconds = int(duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int((duration - seconds) * 1e9)
        goal.trajectory.points = [point]

        feedback_callback(self._current_width)
        inner_goal = await self._trajectory_client.send_goal_async(goal)
        if inner_goal is None or not inner_goal.accepted:
            return False, 'Gazebo gripper trajectory was rejected.'

        wrapped_result = await inner_goal.get_result_async()
        feedback_callback(self._current_width)
        if wrapped_result is None or wrapped_result.result.error_code != 0:
            if accepted_width_range is not None:
                minimum_width, maximum_width = accepted_width_range
                if minimum_width <= self._current_width <= maximum_width:
                    self.get_logger().info(
                        'The simulated fingers are blocked within the requested '
                        'grasp tolerance; treating this as a successful grasp.'
                    )
                    return True, ''
            error_code = (
                'unknown'
                if wrapped_result is None
                else str(wrapped_result.result.error_code)
            )
            return False, f'Gazebo gripper controller failed with code {error_code}.'

        return True, ''

    async def _execute_homing(self, goal_handle):
        if self._holding_pick_object:
            self._detach_sim_object()
            self._detach_planning_object()
        self._holding_pick_object = False

        def publish_feedback(width):
            feedback = Homing.Feedback()
            feedback.current_width = width
            goal_handle.publish_feedback(feedback)

        success, error = await self._command_width(
            self.MAX_WIDTH, 0.05, publish_feedback
        )
        result = Homing.Result()
        result.success = success
        result.error = error
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def _execute_move(self, goal_handle):
        goal = goal_handle.request
        releasing_pick_object = (
            goal.width >= 0.06 and self._holding_pick_object
        )

        if releasing_pick_object:
            # A cube fixed to the hand can physically lock the simulated
            # fingers at their grasp width.  Put it at the requested place
            # pose and remove both attachments before commanding the fingers.
            # The real Franka action API remains unchanged.
            await self._snap_sim_object_to_tcp()
            self._detach_sim_object()
            self._detach_planning_object()
            self._holding_pick_object = False

        def publish_feedback(width):
            feedback = Move.Feedback()
            feedback.current_width = width
            goal_handle.publish_feedback(feedback)

        success, error = await self._command_width(
            goal.width,
            goal.speed,
            publish_feedback,
        )
        if success and releasing_pick_object:
            # Opening contacts can nudge the now-free cube.  Restore the
            # measured TCP placement pose once after the fingers clear it.
            await self._snap_sim_object_to_tcp()
        result = Move.Result()
        result.success = success
        result.error = error
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def _execute_grasp(self, goal_handle):
        goal = goal_handle.request

        def publish_feedback(width):
            feedback = Grasp.Feedback()
            feedback.current_width = width
            goal_handle.publish_feedback(feedback)

        # Lock the object at its measured pre-grasp pose before the simulated
        # fingers close. Otherwise contact with position-controlled fingers can
        # push the free cube sideways before the grasp is declared successful.
        await self._snap_sim_object_to_tcp()
        self._attach_sim_object()

        # Gazebo's position controller cannot reproduce the real hand's force
        # loop. The requested force is validated but the target width drives
        # the simulated fingers.
        success, error = await self._command_width(
            goal.width,
            goal.speed,
            publish_feedback,
            accepted_width_range=(
                max(0.0, goal.width - goal.epsilon.inner),
                min(self.MAX_WIDTH, goal.width + goal.epsilon.outer),
            ),
        )
        result = Grasp.Result()
        result.success = success
        result.error = error
        if success:
            self._holding_pick_object = True
            self._attach_planning_object()
            goal_handle.succeed()
        else:
            self._detach_sim_object()
            goal_handle.abort()
        return result

    def destroy_node(self):
        self._homing_server.destroy()
        self._move_server.destroy()
        self._grasp_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedFrankaGripper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
