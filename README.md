# ROS2_Tutorial
A simple teaching to understand the tools needed to begin projects with the Robot Operating System

## Introduction
**ROS2** stands for the Robot Operating System which is defined as an open source development kit and middleware used for robotic development. It consists of four main elements:
1. Framework - Is a re-usable software foundation with pre-written code, libraries and tools that provide a structured template to simplify and accelerate development of software applications. ROS is therefore made up of packages that contain nodes(contains your programmes e.g. motor driver code) which communicate with each other.
2. Set of tools - you can find command-line tools to build the application, introspection tools to monitor the flow of communication, logging functionalities, plots, and more.
3. Plug-and-Play plugins - such as the nav2 or moveit2 stacks that prevent you from re-inventing the wheel or starting from scratch e.g. in the design of a mobile navigating or object sorting robot.
4. Online Community - Platforms such as the Robotics Discorce Forum and the Robotics Stack Exchange allow for collaboration and knowledge sharing among users.

- When is it necessary to consider ROS for your application?
  1. A robot that can perform mapping using a laser scan.
  2. A sorting Robotic arm
  3. A robotic system with many sensors and actuators
  4. You want to create a hardware driver and want to share it with other developers

- ROS 1 was released in 2007 but in 2014, ROS 2 was created to cater for more industrial applications. ROS 2 distributions are released every year and every even year, a Long Term Support (LTS) distribution is released that will receive maintenance for 5 years instead of 1.5 years. Every ROS (ROS 2) distribution released is compatible to an Ubuntu distribution such as Ubuntu 22.04 and ROS 2 Humble.
- Some of the prerequisites for learning ROS are:
  1. Linux command line.
  2. Python programming.
  3. C++ programming.

- Once you get an idea of what ROS2 is, proceed to installing the correct Ubuntu and ROS2 distribution to a Virtual Machine or on a separate partition in your disk.
- Just as a bonus, you can install the 'terminator' which allows you to split your terminals and work on different commads on one terminal. Press Ctrl+alt+T on a normal terminal open a terminal that enables terminator.
  - Ctr + Shift + O : Horizontal split
  - Ctr + Shift + E : Vertical split
  - Ctr + Shift + X : Make the current terminal fill the entire window, use again to revert
  - Ctr + Shift + W : Close a terminal
  ``` bash
  sudo apt install terminator
  ```
## Nodes
- Now let's begin learning about the building blocks of ROS known as nodes into which you write your programs. Nodes contain topics, services, actions, parameters and launch files.
- You can start your first nodes in two differen terminals, a talker and a listener.
  ``` bash
  ros2 run demo_nodes_cpp talker
  ```
  ``` bash
  ros2 run demo_nodes_cpp listener
  ```
- On the third terminal you can run the 'rqt_graph' to view the active nodes and the channel through which they are communicating e.g. topics, services e.t.c.
  ``` bash
  rqt_graph
  ```
- You can run a 2D simulation and send velocity commands from your keyboard
  ``` bash
  ros2 run turtlesim turtlesim_node
  ```
  ``` bash
  ros2 run turtlesim turtle_teleop_key
  ```
## Topics
- Suitable for data streams flowing in one direction between nodes. (Publisher and Subscriber) The talker and listener that was being run before used a topic called /Chatter. Each topic has a **Name** and **Interface**
- Run the talker and listener on separate terminals and on third terminal, list the available topics. THis will give you the topic **name**
    ``` bash
    ros2 topic list
    ```
- Now time to obtain the interface (what kind of data is being sent).
    ``` bash
    ros2 topic info <topic_name>
    ```
- To see what's inside an interface, run:
    ``` bash
    ros2 interface show <interface_name>
    ```
- **Challenge** find the topic used to send velocity commands to the turtle in turtle_sim and send a velocity command from the terminal
- **Solution**
    ``` bash
    ros2 topic pub <topic_name> <interface_name> "<data>"
    ```
## Services













