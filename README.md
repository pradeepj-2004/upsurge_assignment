# 🐾 Upsurge Assignment – Quadruped Robot Simulation & Control

This repository contains a simulation and controller for a quadruped robot using **ROS 2** and **Gazebo**. It is structured into two main packages:

- `go2_description`: Contains the robot model and Gazebo launch configuration.
- `quadrupled_control`: Contains control logic such as inverse kinematics script, robot_controller , and square path following.

**Note:**
The controller script can be found in the robot_controller.cpp file within the src directory. It utilizes the ik_service script to compute inverse kinematics. The entire quadrupled_control package was developed by me for the assignment.

The video demo of square path following is available in video folder.

---



## Install ROS-based dependencies:
```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
```

## ⚙️ Setup Instructions

1) Clone and build the workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/pradeepj-2004/upsurge_assignment.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## Run commands
 
### 1) First Launch Gazebo simulation

```bash
ros2 launch go2_description gazebo.launch.py
```

### 2) Next, Start the inverse kinematics service
```bash
ros2 run quadrupled_control ik_service
```

### 3) Run the robot controller which subscribes to cmd_vel topic and generate joint trajectory accordingly.
```bash
ros2 run quadrupled_control robot_controller
```

### To control robot using teleoperation 
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### This script used to make the quadruple robot in square path
```bash
ros2 run quadrupled_control path_follower.py
```
