#!/usr/bin/env python3

"""Launch only the local VoxPoser RGB-D gateway for an existing camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("target_frame", default_value="fr3_link0"),
        DeclareLaunchArgument("end_effector_frame", default_value="fr3_hand_tcp"),
        DeclareLaunchArgument("stream_port", default_value="8765"),
    ]
    gateway = Node(
        package="franka_gazebo_bringup",
        executable="voxposer_rgbd_gateway.py",
        parameters=[{
            "color_topic": LaunchConfiguration("color_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "camera_frame": LaunchConfiguration("camera_frame"),
            "target_frame": LaunchConfiguration("target_frame"),
            "end_effector_frame": LaunchConfiguration("end_effector_frame"),
            "stream_host": "127.0.0.1",
            "stream_port": ParameterValue(
                LaunchConfiguration("stream_port"), value_type=int
            ),
        }],
        output="screen",
    )
    return LaunchDescription(arguments + [gateway])


if __name__ == "__main__":
    generate_launch_description()
