# KUKA Robotics Challenge
This repo contains ROS packages to move a payload through a 2D maze using a KUKA industrial robotic arm as shown below.
![KUKA robotic arm moving a payload through a maze](videos/maze_solving.gif)

This project aims to solve the [KUKA Robotics challenge](https://www.udacity.com/kuka-robotics-challenge) organized by Udacity, KUKA, and KIT. This repo builds upon ROS packages provided by KIT, available on Github, [here](https://github.com/KITrobotics/rll_sdk) and [here](https://github.com/KITrobotics/rll_path_planning_project). Really good documentation about these packages is available on KIT's KUKA Robotics Learning Lab website [here](https://rll-doc.ipr.kit.edu/planning_project.html)

The available code provides an easy interface using ROS services to communicate with the KUKA Robotic Arm. The path planning and control algorithm I implemented, used the following services:
- `check_path`: verify if a linear path between two 2D poses is valid (collision-free and within workspace)
- `move`: command the robot by sending a 2D pose, the robot will move on a linear path to this pose

# Setup
This project was tested with ROS Kinetic.

A catkin workspace can be setup as follows:
```
mkdir ~/catkin_ws
cd ~/catkin_ws
catkin_init_workspace
mkdir src
cd src
git clone https://github.com/ssharma1991/kuka-robotics-challenge
```

To run the project, use the created shell script as follows:
```
cd rll_planning_project/scripts
./start_project.sh
```
The script starts all required ROS launch files to start the simulation in Gazebo and visualization in Rviz. The payload dimensions, start pose and final pose can be tweaked within `planning_iface.launch` file. 

Video of a complete simulation is available here: [https://github.com/ssharma1991/kuka-robotics-challenge/tree/master/videos](https://github.com/ssharma1991/kuka-robotics-challenge/tree/master/videos)

# Algorithm
A-star algorithm has been used for Path Planning. The state space is 3D and includes (x, y, theta) coordinates. A grid resolution of 2cm was used to discretize the maze and gave satisfactory results. The path planning code exists in `path_planning.py` file.