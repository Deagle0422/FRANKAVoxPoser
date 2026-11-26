#!/usr/bin/env python3

"""Launch the FR3 pick-place scene and VoxPoser RGB-D pipeline only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory("franka_gazebo_bringup")
    world = os.path.join(gazebo_share, "worlds", "fr3_pick_place.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/voxposer_rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/voxposer_rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/voxposer_rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/voxposer_rgbd/image", "/voxposer/camera/color/image_raw"),
            ("/voxposer_rgbd/depth_image", "/voxposer/camera/depth/image_raw"),
            ("/voxposer_rgbd/camera_info", "/voxposer/camera/color/camera_info"),
        ],
        output="screen",
    )

    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="voxposer_camera_static_tf",
        arguments=[
            "--x", "0.85", "--y", "-0.10", "--z", "1.20",
            "--qx", "0.7071067812", "--qy", "-0.7071067812",
            "--qz", "0.0", "--qw", "0.0",
            "--frame-id", "fr3_link0",
            "--child-frame-id", "voxposer_camera_optical_frame",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    gateway = Node(
        package="franka_gazebo_bringup",
        executable="voxposer_rgbd_gateway.py",
        parameters=[{
            "use_sim_time": True,
            "camera_frame": "voxposer_camera_optical_frame",
            "target_frame": "fr3_link0",
            "stream_host": "127.0.0.1",
            "stream_port": 8765,
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gz_args",
            default_value=f"{world} -r -s",
            description="Arguments forwarded to Gazebo Sim",
        ),
        gazebo,
        bridge,
        camera_static_tf,
        gateway,
    ])


if __name__ == "__main__":
    generate_launch_description()
