import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_path = os.path.join(
        os.path.expanduser("~"), "ros2_ws", "src", "Anscer_Ass", "trajectory_manager", "rviz", "trajectory_manager.rviz"
    )

    return LaunchDescription([
        Node(
            package="trajectory_manager",
            executable="trajectory_publisher_saver",
            name="trajectory_publisher_saver",
            output="screen"
        ),
        Node(
            package="trajectory_manager",
            executable="trajectory_reader_publisher",
            name="trajectory_reader_publisher",
            output="screen"
        ),
        Node(
            package="trajectory_manager",
            executable="path_publisher",
            name="path_publisher",
            output="screen"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path]  # Use the saved config
        )
    ])
