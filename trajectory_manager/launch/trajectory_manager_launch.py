import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rviz_config_path = os.path.join(
        os.path.expanduser("~"), "ros2_ws", "src", "Anscer_Ass", "trajectory_manager", "rviz", "trajectory_manager.rviz"
    )

   # gazebo = Node(
   # executable="/usr/bin/gzserver",
   # arguments=["--verbose", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"],
   # output="screen"
#)

   # spawn_robot = TimerAction(
    #    period=5.0,  # Wait for Gazebo to fully start before spawning the robot
    #    actions=[
     #       Node(
      #          package="gazebo_ros",
      #          executable="spawn_entity.py",
      #          arguments=[
      #              "-entity", "burger",
       #             "-file", "/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"
       #         ],
       #         output="screen"
        #    )
       # ]
   # )

    return LaunchDescription([
        #gazebo,  # Start Gazebo first
  # Start the Gazebo GUI
        #spawn_robot,  # Wait and then spawn the robot
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
        )
    ])
