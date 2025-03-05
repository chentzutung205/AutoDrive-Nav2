# Autonomous Navigation with ROS2

This project is a part of the larger Autonomous Vehicle initiative. BB-8, affectionately known as "Beebee-Ate," is a lovable and intelligent droid character from the __Star Wars__ franchise. Inspired by this iconic character, our robot BB-8 is designed to navigate and interact with its environment, showcasing advanced capabilities in object tracking and autonomous navigation.

## Overview

This ROS2 package allows a TurtleBot3 to autonomously navigate between predefined waypoints while avoiding obstacles, leveraging mapping, localization, and path planning techniques. The navigation process is implemented using ROS2 nodes that interact with the __Nav2__ stack to achieve accurate and efficient movement.

## Dependencies

- ROS2 Humble
- Gazebo
- Navigation2 (Nav2)
- geometry_msgs

## Features

- Mapping using SLAM
- Localization and autonomous waypoint navigation
- Integration with Nav2 for path planning and obstacle avoidance

## Nodes
**1. **

This node processes LIDAR data to detect the range and orientation of obstacles in the robot's local coordinate frame.

**2. **

This node drives the robot through the sequence of waypoints. It incorporates obstacle data from getObjectRange and uses onboard odometry for accurate global positioning.

## Usage
1. Ensure all dependencies are installed.
2. Clone this package into your ROS2 workspace.
3. Build the package using `colcon build`.
4. Source your workspace.
5. Run the nodes using:
```
ros2 run bb8_navigate_to_goal getObjectRange
ros2 run bb8_navigate_to_goal goToGoal
```

## Tips
- Adjust Nav2 parameters in turtlebot3_navigation/param/ for better performance.
- Visualize navigation and mapping in RViz2.
- Tune costmap parameters to improve obstacle avoidance behavior.
