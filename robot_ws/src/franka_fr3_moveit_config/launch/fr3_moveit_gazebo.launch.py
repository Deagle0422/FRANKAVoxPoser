#!/usr/bin/env python3

"""Launch an FR3 Gazebo simulation with the hardware-compatible ROS API."""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, 'r', encoding='utf-8') as yaml_file:
        return yaml.safe_load(yaml_file)


def generate_launch_description():
    gazebo_bringup_share = get_package_share_directory(
        'franka_gazebo_bringup'
    )
    description_share = get_package_share_directory('franka_description')
    moveit_share = get_package_share_directory('franka_fr3_moveit_config')

    gazebo_xacro = os.path.join(
        gazebo_bringup_share, 'urdf', 'franka_arm.gazebo.xacro'
    )
    robot_description_xml = xacro.process_file(
        gazebo_xacro,
        mappings={
            'robot_type': 'fr3',
            'hand': 'true',
            'ee_id': 'franka_hand',
            'gazebo': 'true',
            # MoveIt simulation uses Gazebo's native position interface.
            # The effort-only gravity-compensation decorator is intended for
            # torque controllers and does not forward position commands
            # reliably on the Humble/Fortress combination used here.
            'gazebo_effort': 'false',
            'gazebo_pick_object_attachment': 'true',
        },
    ).toxml()
    robot_description = {'robot_description': robot_description_xml}
    simulation_time = {'use_sim_time': True}

    semantic_xacro = os.path.join(
        description_share, 'robots', 'fr3', 'fr3.srdf.xacro'
    )
    robot_description_semantic = {
        'robot_description_semantic': xacro.process_file(
            semantic_xacro,
            mappings={
                'robot_type': 'fr3',
                'hand': 'true',
                'ee_id': 'franka_hand',
            },
        ).toxml()
    }

    kinematics = load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml'
    )
    ompl = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    moveit_controller_config = load_yaml(
        'franka_fr3_moveit_config',
        'config/fr3_gazebo_moveit_controllers.yaml',
    )
    workspace_boundary_config = load_yaml(
        'franka_fr3_moveit_config',
        'config/workspace_boundary.yaml',
    )['workspace_boundary']

    planning_pipeline = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    planning_pipeline['move_group'].update(ompl)

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controller_config,
        'moveit_controller_manager': (
            'moveit_simple_controller_manager/'
            'MoveItSimpleControllerManager'
        ),
    }
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.02,
    }
    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    gz_args = LaunchConfiguration('gz_args')
    rviz_enabled = LaunchConfiguration('rviz')
    pick_place_world = os.path.join(
        gazebo_bringup_share, 'worlds', 'fr3_pick_place.sdf'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, simulation_time],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'fr3_arm_controller',
            'fr3_gripper_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
        output='screen',
    )

    simulated_gripper = Node(
        package='franka_gripper',
        executable='simulated_franka_gripper.py',
        output='screen',
        parameters=[simulation_time],
    )

    set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/fr3_pick_place/set_pose'
            '@ros_gz_interfaces/srv/SetEntityPose',
            '/fr3_gripper/sim_attach'
            '@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/fr3_gripper/sim_detach'
            '@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/fr3_gripper/sim_attachment_state'
            '@std_msgs/msg/String[ignition.msgs.StringMsg',
            '/voxposer_rgbd/image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',
            '/voxposer_rgbd/depth_image'
            '@sensor_msgs/msg/Image[gz.msgs.Image',
            '/voxposer_rgbd/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/voxposer_rgbd/image', '/voxposer/camera/color/image_raw'),
            ('/voxposer_rgbd/depth_image', '/voxposer/camera/depth/image_raw'),
            ('/voxposer_rgbd/camera_info', '/voxposer/camera/color/camera_info'),
        ],
        output='screen',
    )

    # The SDF camera uses Gazebo's +X-forward convention. This optical frame
    # uses ROS REP-103 (+Z forward, +X right, +Y down) and is the calibrated
    # frame consumed by the simulation and real-robot RGB-D gateway.
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='voxposer_camera_static_tf',
        arguments=[
            '--x', '0.85', '--y', '-0.10', '--z', '1.20',
            '--qx', '0.7071067812', '--qy', '-0.7071067812',
            '--qz', '0.0', '--qw', '0.0',
            '--frame-id', 'fr3_link0',
            '--child-frame-id', 'voxposer_camera_optical_frame',
        ],
        parameters=[simulation_time],
        output='screen',
    )

    rgbd_gateway = Node(
        package='franka_gazebo_bringup',
        executable='voxposer_rgbd_gateway.py',
        name='voxposer_rgbd_gateway',
        parameters=[simulation_time, {
            'color_topic': '/voxposer/camera/color/image_raw',
            'depth_topic': '/voxposer/camera/depth/image_raw',
            'camera_info_topic': '/voxposer/camera/color/camera_info',
            'camera_frame': 'voxposer_camera_optical_frame',
            'target_frame': 'fr3_link0',
            'end_effector_frame': 'fr3_hand_tcp',
            'stream_host': '127.0.0.1',
            'stream_port': 8765,
        }],
        output='screen',
    )

    planning_scene = Node(
        package='franka_gazebo_bringup',
        executable='publish_pick_place_scene.py',
        output='screen',
        parameters=[simulation_time],
    )

    workspace_boundary_supervisor = Node(
        package='franka_gazebo_bringup',
        executable='workspace_boundary_supervisor.py',
        output='screen',
        parameters=[workspace_boundary_config, simulation_time],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            planning_pipeline,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor,
            simulation_time,
        ],
    )

    rviz_config = os.path.join(moveit_share, 'rviz', 'moveit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            planning_pipeline,
            simulation_time,
        ],
        condition=IfCondition(rviz_enabled),
    )

    resource_path = os.pathsep.join(
        [os.path.dirname(description_share), gazebo_bringup_share]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gz_args',
                default_value=f'{pick_place_world} -r',
                description='Arguments forwarded to Gazebo Sim.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Start RViz with the MoveIt configuration.',
            ),
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
            gazebo,
            set_pose_bridge,
            camera_static_tf,
            rgbd_gateway,
            robot_state_publisher,
            move_group,
            planning_scene,
            workspace_boundary_supervisor,
            rviz,
            spawn_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controller_spawner,
                    on_exit=[simulated_gripper],
                )
            ),
        ]
    )


if __name__ == '__main__':
    generate_launch_description()
