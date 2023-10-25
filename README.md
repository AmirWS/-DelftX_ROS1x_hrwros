# DelftX_ROS1x_hrwros
# Hello (Real) World with ROS – Robot Operating System

Welcome to the GitHub repository for the **Hello (Real) World with ROS – Robot Operating System** course provided by DelftX on edX. This README will provide you with an overview of the course content, the software environment components, and the purpose of the course.

## Course Overview

This repository contains resources related to the ROS1x course offered by DelftX on edX. The course introduces you to various aspects of Robot Operating System (ROS) and equips you with the skills necessary to work with ROS in real-world applications.

### Course Content

The course covers a wide range of topics, including:

- **ROS:** Learn the fundamentals of Robot Operating System.
- **RViz:** Explore the capabilities of the ROS Visualization tool.
- **Gazebo:** Gain proficiency in the Gazebo simulator, an essential tool for robotic simulation.
- **Flexbe Behavior:** Understand and implement Flexbe behaviors for robot control.
- **TF2 (Transforms):** Master the use of TF2 for managing coordinate transforms.
- **ROS Action:** Explore ROS action servers and clients for complex robotic actions.
- **ROS Publisher and Subscriber:** Learn how to publish and subscribe to ROS topics.
- **ROS Servers and Clients:** Develop ROS services for communication between nodes.
- **RQT (ROS Qt):** Utilize the ROS Qt framework for GUI development.
- **MoveIt:** Work with MoveIt for motion planning and manipulation.

### Software Environment (Gazebo)

The course provides a simulated software environment within Gazebo, featuring the following components:

- **Two Serial Manipulator Robots:** Used for various tasks within the course.
- **Two Cameras:** Essential for visual sensing and object detection.
- **Conveyor:** Simulates a conveyor belt for moving objects.
- **TurtleBot3:** A mobile robot capable of autonomous navigation.

## Course Purpose

The primary purpose of this course is to provide a practical understanding of ROS by simulating a real-world industrial operation. The course scenario involves a complex operation in which robots, cameras, and other components work together to achieve a specific task. The scenario consists of the following steps:

1. A box moves along the conveyor.
2. Camera 1 detects the box and stops the conveyor.
3. Robot 1 goes to picking location, uses its suction cup to pick up the box.
4. TurtleBot3 autonomously navigates to Robot 1's placing location.
5. Robot 1 moves to the placing location.
6. The box is released onto TurtleBot3.
7. TurtleBot3 takes the box to Robot 2's holding location.
8. Camera 2 detects TurtleBot3 with the box and signals Robot 2.
9. Robot 2 retrieves the box from TurtleBot3's holding location and places it in the release location.

This course aims to provide hands-on experience in building, configuring, and controlling the robots and components involved in this industrial operation. It demonstrates the power and flexibility of ROS in orchestrating complex robotics tasks.
## Course Introduction Video

[![Course Introduction](https://img.youtube.com/vi/n485FJn0iek/0.jpg)](https://www.youtube.com/watch?v=n485FJn0iek)

Watch the course introduction video to get a sneak peek of what you'll be learning in this ROS course.

## Getting Started

To get started with the course materials, please refer to the course content in this repository. You will find code, resources, and instructions to follow along with the course modules.

We hope you enjoy your journey into the exciting world of Robot Operating System!

For any questions, issues, or assistance, feel free to create GitHub issues or reach out to the course instructors through the edX platform.

Happy learning and happy ROS programming!
