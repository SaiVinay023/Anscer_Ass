# Trajectory Manager - README

## Overview
The **Trajectory Manager** is a ROS 2 package designed to record, save, and replay robot trajectories using Odometry data. The package includes multiple ROS nodes responsible for trajectory publishing, saving, and replaying. This README covers setup, execution, debugging, and troubleshooting steps.

## Features
- **Trajectory Recording**: Subscribes to `/odom` and records the robot's movement.
- **Trajectory Saving**: Saves recorded trajectory data to a CSV file.
- **Trajectory Replay**: Reads the saved trajectory and publishes it for visualization.
- **Visualization**: Publishes trajectory markers in RViz for easy debugging.
- **Fake Data Handling**: Includes functionality to generate and save fake trajectory data for testing.

## Package Structure
```
trajectory_manager/
├── src/
│   ├── path_publisher.cpp
│   ├── trajectory_publisher_saver.cpp
│   ├── trajectory_reader_publisher.cpp
│   ├── trajectory_reader.cpp
│   ├── trajectory_replay_publisher.cpp
├── srv/
│   ├── SaveTrajectory.srv
├── CMakeLists.txt
├── package.xml
├── rviz/
│   ├── trajectory_manager.rviz
├── launch/
│   ├── trajectory_manager_launch.py
```

## Nodes and Their Functionalities

### 1. `trajectory_publisher_saver`
- **Subscribes to**: `/odom`
- **Publishes**: `/trajectory_markers`
- **Provides Service**: `/save_trajectory`
- **Function**: Collects and stores trajectory points in memory, allows saving the trajectory to a file.
- **Fake Data Usage**: If real odometry data is unavailable, fake trajectory points are generated and saved.

### 2. `trajectory_reader_publisher`
- **Subscribes to**: None
- **Publishes**: `/trajectory_markers`
- **Function**: Reads saved trajectory data from a CSV file and publishes it for visualization.

### 3. `trajectory_replay_publisher`
- **Subscribes to**: None
- **Publishes**: `/cmd_vel`
- **Function**: Reads saved trajectory and publishes velocity commands to make the robot follow it.

## Installation and Setup
### 1. Clone the Repository
```bash
cd ~/Desktop/Ros_vis/ros2_ws/src
mkdir -p Anscer_Ass && cd Anscer_Ass
```
Place the `trajectory_manager` package inside `Anscer_Ass`.

### 2. Build the Package
```bash
cd ~/Desktop/Ros_vis/ros2_ws
colcon build --packages-select trajectory_manager
source install/setup.bash
```

### 3. Run Gazebo with TurtleBot3
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 4. Start Teleoperation (to generate movement data)
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

### 5. Start the Trajectory Manager
```bash
ros2 launch trajectory_manager trajectory_manager_launch.py
```

### 6. Save the Trajectory
After moving the robot using teleop, run:
```bash
ros2 service call /save_trajectory trajectory_manager/srv/SaveTrajectory "{filename: 'trajectory.csv', duration: 300.0}"
```
This will save the last 5 minutes (300 seconds) of trajectory data.

### 7. Replay the Trajectory
```bash
ros2 run trajectory_manager trajectory_replay_publisher
```

## Debugging and Troubleshooting
### 1. Check Node Status
```bash
ros2 node list
ros2 node info /trajectory_publisher_saver
```

### 2. Check Topic Data
```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /trajectory_markers
```

### 3. Ensure Data is Being Saved
If data is not being saved, verify:
- The robot has moved before calling the save service.
- The `/odom` topic is publishing valid messages.
- The `trajectory_publisher_saver` node is receiving odometry data.
- If necessary, fake data can be injected to verify functionality.

### 4. Gazebo Not Starting?
Try manually starting Gazebo first:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Then launch the trajectory manager separately.

### 5. Teleoperation Issues
Ensure the correct model is set before running teleop:
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

## Further Enhancements
- **Improve Data Storage**: Optimize trajectory data storage for large-scale operations.
- **Dynamic Parameter Tuning**: Allow runtime adjustment of parameters such as trajectory duration and visualization settings.
- **Multi-Robot Support**: Extend functionality to support multiple robots operating simultaneously.
- **ROS2 Lifecycle Nodes**: Implement lifecycle management to enhance robustness and flexibility.

## Conclusion
This package successfully records, saves, and replays a TurtleBot3's trajectory. Ensure that Gazebo, teleop, and trajectory nodes are correctly launched for proper operation.

For further troubleshooting, check logs using:
```bash
ros2 launch trajectory_manager trajectory_manager_launch.py --debug
```
Happy Robotics Development! 🚀



## Authors
- Saivinay Manda  

🎯 For any issues, feel free to open an issue on GitHub! 🚀

