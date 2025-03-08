import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    rviz_config_path = os.path.join(
        os.path.expanduser("~"), "ros2_ws", "src", "Anscer_Ass", "trajectory_manager", "rviz", "trajectory_manager.rviz"
    )

    spawn_robot = TimerAction(
        period=5.0,  # Wait for Gazebo to fully start
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "burger",
                    "-file", "/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
                ],
                output="screen"
            )
        ]
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
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path]
        ),
        Node(
    package="gazebo_ros",
    executable="gzserver",
    arguments=["-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", "~/ros2_ws/src/Anscer_Ass/trajectory_manager/worlds/turtlebot3_world.world"],
    output="screen"
)  # Ensure this is inside the list
    ])
