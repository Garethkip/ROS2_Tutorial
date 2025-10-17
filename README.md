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
## Topics intro.
- Suitable for data streams flowing in one direction between nodes. (Publisher and Subscriber) The talker and listener that was being run before used a topic called /Chatter. Each topic has a **Name** and **Interface**. Each topic can have multiple subscribers and multiple publishers.
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
## Services intro.
- Allows for two nodes (client and server) to communicate by sending request and receiving a response. It is therefore not suitable for data streams. It allows for multiple clients but only one server.
- We can run a demo server and demo client.
  ``` bash
  ros2 run demo_nodes_cpp add_two_ints_server
  ```
  ``` bash
  ros2 run demo_nodes_cpp add_two_ints_client
  ```
- Let's find the name and interface of the above service. The command below will allow you to find the service name.
  ``` bash
  ros2 service list
  ```
- Find the imterface name
  ``` bash
  ros2 service type <service_name>
  ```
- See the structure of the interface with: (the varibales above the 3 dashes represent the request while that below represent the result)
  ``` bash
  ros2 interface show <interface_name>
  ```
- Once you know the service_name and interface_name and structure, you can make a custom command from the terminal:
  ``` bash
  ros2 service call <service_name> <interface_name> "<request in json>"
  ```
- **Challenge** : Use a custom service to spawn a new turtle on the turtlesim window.

## Actions intro.
- Is basically a service that has feedback so that it can be used on tasks that take sometime to complete. They also allow you to cancel a task midway.
- You can run the turtlesim node again and list the active actions: (this will give you the action name)
    ``` bash
    ros2 action list
    ```
- Find the name(previous command) and interface of the action. This time the interface has a goal, result and feedback.
  ``` bash
  ros2 action info <action_name> -t
  ```
- The above command will give you the interface name (the one in square brackets)
- Let's see what's in the interface:(goal, result, feedback)
  ``` bash
  ros2 interface show <interface_name>
  ```
- Let's send a goal from the terminal:
  ``` bash
  ros2 action send_goal <action_name> <interface_name> "<goal_in_json>"
  ```

## Parameters intro.
- They are the settings you give to a node when you start it.
- Run the turtlesim node again and list the active parameters.
  ``` bash
  ros2 param list
  ```
- What's inside a parameter?
  ``` bash
  ros2 param get <node_name> <param_name>
  ```
- Run a node by setting values to its parameters at the terminal. (add -p <param_name>:=value for each parameter)
  ``` bash
  ros2 run <package_name> <executable_name> --ros-args -p <param_name>:=value
  ```

## Launch files intro.

- Allows you to start several nodes and parameters from just one file
  ``` bash
  ros2 launch <package_name> <launch_file>
  ```
- Start a demo launch file.
  ``` bash
  ros2 launch demo_nodes_cpp talker_listener_launch.py
  ```
- If you list the nodes you'll observe both talker and listener are active.
- You can also start multiple instances of a node each with a different name using a launch file. An example is the turtlesim node:
  ``` bash
  ros2 launch turtlesim multisim.launch.py
  ```

## Building a ROS2 node
- Create a workspace (you can name it anything insted of ros2_ws)
  ``` bash
  mkdir ros2_ws
  ```
- Create the 'src' which is where all your ros2 code will go:
  ``` bash
  cd ros2_ws
  mkdir src
  ```
- Build the workspace (We haven't yet created any packages)
  ``` bash
  cd ~/ros2_ws
  colcon build
  ```
- This will create 3 new directories on the same level as the src: build, log and install. The build contains all the intermediate files needed for the overall build, the log contains the logs for each build and install is where all your nodes will be installed after you build the workspace.
- Source your workspace (Should be done every after every build and also on new terminals after building
  ``` bash
  source ~/ros2_ws/install/setup.bash
  ```
- Add it to your bashrc file so that it can be sourced on each new terminal automatically
  ``` bash
  gedit ~/.bashrc
  ```
- Create a package. A package is a sub-part of your application and is responsible for a particular function is a system such as motion planning in a mobile robot system. They can be of Python or C++ type.
- Create a python package
  ``` bash
  ros2 pkg create <pkg_name> --build-type ament_python --dependencies rclpy
  ```
- Important directories to note include:
    - <pkg_name>(has the same name os your pkg): This is where we create our python nodes
    - package.xml: provide the dependencies of the pkg
    - setup.py: write the inctructions to build your python nodes.
- Create a C++ package
  ``` bash
  ros2 pkg create <pkg_name> --build-type ament_cmake --dependencies rclcpp
  ```
- Here's a quick explanation of the directories and files:
  - CMakeLists.txt: provides instructions on how to compile your C++ nodes, create libraries and so on.
  - include: If you split your project into .cpp and .h (header) files, place the .h files in the include directory.
  - package.xml: conains more info about the pkg and dependencies on other pkgs
  - src: where you write your nodes.

- Now go back to the workspace, build and source (or open a new terminal).

- Now let's make a node within the python package (the chmod +x <node_name> is used to make the node executable)
  ``` bash
  cd ~/ros2_ws/src/<pkg_name>/<pkg_name>
  touch my_first_node.py
  chmod +x my_first_node.py
  ```
### Writing a basic python node

- You can open the my_py_pkg for more elaboration.
- The node prints a string on the terminal, utilizes a timer to increment a counter variable every second and print the result. (The code as enough comments, hehe)
- Also after writing the node, do not forget to edit the setup.py file:
  ``` python
      entry_points={
        'console_scripts': [
            "my_node_execute = my_py_pkg.my_first_node:main" #<executable_name> = <package_name>.<file_name>:<function_name>
        ],
    },
  ```
- The **executable name** is what you will use when running (ros2 run <package_name> <executable_name>)
- The **package name** is the name of your package i.e "my_py_pkg"
- The **file name** is the name of the file in which you created the node
- The **function name** is the "main" function in the file

## Writing a basic C++ node

- Open the my_cpp_pkg and find the my_cpp_node.cpp (Study the structure of the node).
- Add the following to your **CMakeLists.txt** in the dependencies section so that the node's executable is created(ros2 run <pkg_name> <executable_name>) and it is installed so that we can find it when we run "ros2 run".
  ``` txt
  add_executable(my_cpp_execute src/my_cpp_node.cpp)
  ament_target_dependencies(my_cpp_execute rclcpp)
  install (TARGETS
    my_cpp_execute
    DESTINATION lib/${PROJECT_NAME}/
  )
  ```

### Here's a template for the pythone and C++ nodes
  






































