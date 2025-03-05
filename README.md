# Autonomous Navigation with ROS2

This milestone is a part of the Autonomous Vehicle project. BB-8, affectionately known as "Beebee-Ate," is a lovable and intelligent droid character from the __Star Wars__ franchise. Inspired by this iconic character, our robot BB-8 is designed to navigate and interact with its environment, showcasing advanced capabilities in object tracking and autonomous navigation.

## Overview

This ROS2 package allows a TurtleBot3 to autonomously navigate between predefined waypoints while avoiding obstacles, leveraging mapping, localization, and path planning techniques. The navigation process is implemented using ROS2 nodes that interact with the __Nav2__ stack to achieve accurate and efficient movement.

## Dependencies

- ROS2 Humble
- Gazebo
- Navigation2 (Nav2)
- geometry_msgs
- sensor_msgs

## Features

- Mapping using SLAM
- Localization and autonomous waypoint navigation
- Integration with Nav2 for path planning and obstacle avoidance

## Nodes
**1. test**

This node allows us to send goals one-by-one manually.

**2. navigate**

This node handles a sequential list of waypoints.

## Usage
1. Clone this package into your ROS2 workspace.
2. Ensure all dependencies are installed.
3. Build the package using `colcon build`.
4. Source your workspace.
5. Running the simulation.
6. Run the nodes using:
```
ros2 run bb8_navigation test.py
ros2 run bb8_navigation navigate.py
```
7. Navigate through waypoints using RViz or command line.

## Tips
- Adjust Nav2 parameters in turtlebot3_navigation/param/ for better performance.
- Visualize navigation and mapping in RViz2.
- Tune costmap parameters to improve obstacle avoidance behavior.
