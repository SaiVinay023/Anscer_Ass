import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="trajectory_manager",
            executable="trajectory_publisher_saver",
            name="trajectory_publisher_saver",
            output="screen",
            parameters=[{"trajectory_topic": "/robot_path"}]
        ),
        Node(
            package="trajectory_manager",
            executable="trajectory_reader_publisher",
            name="trajectory_reader_publisher",
            output="screen"
        )
    ])
