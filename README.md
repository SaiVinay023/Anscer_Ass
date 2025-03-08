# Anscer Assignment
# Trajectory Manager

## Project Overview
This ROS 2 package provides a trajectory recording, saving, and replaying system for a mobile robot. The system consists of:

- Trajectory Publisher Saver: Subscribes to odometry (`/odom`), stores the trajectory, visualizes it in RViz, and saves it to a CSV file.
- Trajectory Reader Publisher: Reads a saved trajectory from a CSV file and publishes it, allowing the robot to follow the saved path.
- Gazebo Simulation: Simulated robot in Gazebo for trajectory collection and replay.
- RViz Visualization: Displays trajectory markers in real-time.

## Implemented Features
✅ Odometry Subscription (`/odom`): Reads robot position in real-time.  
✅ Trajectory Storage: Saves robot's trajectory in a vector.  
✅ Visualization in RViz (`/trajectory_markers`): Shows trajectory using markers.  
✅ Save Trajectory Service (`/save_trajectory`): Stores trajectory data in a CSV file.  
✅ Replay from CSV: Loads and replays saved trajectory.  
✅ Gazebo Integration: Simulates robot movement.  

---

## Package Installation & Build
### Install Dependencies
Ensure you have ROS 2 (Humble) and required dependencies installed:
```
sudo apt update && sudo apt install -y ros-humble-gazebo-ros ros-humble-nav-msgs ros-humble-visualization-msgs ros-humble-rviz2 ros-humble-tf2-ros
```

### Clone the Repository
```bash
git clone https://github.com/SaiVinay023/Anscer_Ass.git ~/ros2_ws/src/Anscer_Ass
cd ~/ros2_ws
```

### Build the Package
```bash
colcon build --packages-select trajectory_manager
source install/setup.bash
```

---

## Execution Steps
### Launch Gazebo & RViz
Start the Gazebo simulation and RViz visualization:
```bash
ros2 launch trajectory_manager trajectory_manager_launch.py
```
💡 Expected Output:Gazebo should open with the robot, and RViz should display the trajectory markers.

### Move the Robot (Using Teleop)
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
Move the robot using WASD keys.

### Save Trajectory to CSV
After moving the robot, call the service to save its trajectory:
```bash
ros2 service call /save_trajectory trajectory_manager/srv/SaveTrajectory "{filename: 'trajectory.csv', duration: 30.0}"
```
💡 Expected Output:
```
[INFO] [trajectory_publisher_saver]: Saving trajectory with 150 points.
[INFO] [trajectory_publisher_saver]: Trajectory saved to /home/vinay/Desktop/trajectory.csv
```
Check if the file is saved:
```bash
cat ~/Desktop/trajectory.csv
```

### Replay the Saved Trajectory
To replay the saved trajectory:
```bash
ros2 run trajectory_manager trajectory_reader_publisher
```
💡 Expected Behavior: The robot should follow the recorded path in Gazebo.

---

## Package Structure
```
trajectory_manager/
├── launch/
│   ├── trajectory_manager_launch.py  # Launches Gazebo, RViz, and nodes
│
├── rviz/
│   ├── trajectory_manager.rviz  # RViz configuration file
│
├── src/
│   ├── trajectory_publisher_saver.cpp  # Publishes and saves trajectory
│   ├── trajectory_reader_publisher.cpp  # Reads and replays saved trajectory
│
├── srv/
│   ├── SaveTrajectory.srv  # ROS 2 service definition for saving trajectory
│
├── CMakeLists.txt  # ROS 2 package build file
├── package.xml  # Package dependencies
```

---

## Troubleshooting
### Service Call Not Working
```bash
ros2 service list | grep save_trajectory
```
If missing, restart the node:
```bash
ros2 run trajectory_manager trajectory_publisher_saver
```

### No Data Saved to CSV
Check timestamps:
```bash
ros2 service call /save_trajectory trajectory_manager/srv/SaveTrajectory "{filename: 'test.csv', duration: 30.0}"
```
If Saved 0 points, adjust filtering in `save_trajectory_callback()`.

### Gazebo Not Starting
Manually start it:
```bash
gazebo --verbose
```

### No Markers in RViz
Ensure correct QoS settings for `/trajectory_markers`.
```bash
ros2 topic info /trajectory_markers --verbose
```

---

## Future Improvements
🚀 Add robot autonomy to follow dynamic waypoints.
🚀 Improve timestamp handling for more accurate replay. 
🚀 Support multiple trajectory save & load.

---

## Authors
- Saivinay Manda  

🎯 For any issues, feel free to open an issue on GitHub! 🚀

