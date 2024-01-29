# ROS Footballer

The project's theme refers to the remote control of a robot. Essentially, it involves the remote transmission through computer-mediated means of signals captured by a robot and the commands addressed to it, decided by an operator in a control room or a control system. In a generic sense, the transmission of commands to a remotely located robot is called teleoperation, regardless of whether the decision-maker is a human operator or an automatic control system.

The goal was to programatically make a Turtlebot3 Waffle Pi robot model in such a way that it simulates a soccer player.
To achieve this simulation, I used the Gazebo simulation environment, where I created a soccer field map for our robot to navigate.
Placed on the field, the robot starts searching for a red ball on the soccer field. Once it detects the ball, it coordinates itself in front of the ball and begins searching for a yellow goal. With both the ball and the goal in front of it, the robot starts pushing the ball towards the goal.

# Development

The project was developed and tested using:
- Ubuntu 20.04
- ROS1 Noetic (Install guide: http://wiki.ros.org/noetic/Installation/Ubuntu)
- Turtlebot3 Waffle Pi
- Gazebo
- catkin build system (Setup guide: https://wiki.ros.org/catkin)

# Testing

After configuring and installing the necessary tools, we will set up the Catkin workspace:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Download the code and place the folder ros-footballer inside /catkin_ws/src and run catkin_make to build the newly added package.

To start the simulation, we'll launch the ros-footballer package which will start gazebo and the simulation world:

```
$ roslaunch footballer start.launch
```

After placing a red ball on the map from gazebo, we can run the program and test the robot's performance in various positions relative to the ball. To execute the program successfully, it needs to be configured as an executable.

```
$ cd catkin_ws/src/footballer/scripts
$ python3 footballer.py
```

# Screenshots


