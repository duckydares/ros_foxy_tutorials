# ROS2 Comprehension Overview

#### Author: Ducky

#### Last Update: 2021.11.01

## Overview

This document goal is to give a overview of the ROS2 core components and how to interact with them. ROS2 builds upon the concept of ROS1 with the usage of a network graph where-in **nodes** communicate data through _actions_, _topics_, _services_, or _parameters_. Several additions have been made to the environment and for a comparison will point you to the following [article](https://roboticsbackend.com/ros1-vs-ros2-practical-overview/).

## Tutorial Runthrough

_Is ROS2 package installed?_: `ros2 pkg executables <package-name>`

### Nodes

_Responsible for a single modular purpose_

Unlike ROS1, all ROS2 commands are under a single global command `ros2` (e.g. `rosrun` -> `ros2 run`, `rostopic` -> `ros2 topic`). The sub-commands all maintain a similar fomat as in ROS1 having their own commands of `list`, `info`, etc.. An addition that is made to nodes is that there is another form of interacting with them; namely through an _action_ which can be considered a superset of a topic and service. Generally a node is run individually as `ros2 run <package-name> <node>` and later down the line we will see how multiple nodes are run in a launch file.

- `ros2 node list`: lists all the active nodes
- `ros2 node info <node-name>`: provides a lists of subscribers, publishers, services, and actions that interact with the node

### Topics

_A communication method that acts as a bus for nodes to exchange messages under a publisher-subscriber model_

A node can simultaneously publish to many topics while being subscribed to many topics. We can use `rqt_graph` to visualize the communication graph to verify that the nodes are generating the assumed directed graph with topics, services, parameters, and actions. This can be used to visually inspect the graph for a more verbose description of the message types, publishers, and subscribers we can use the command line tools.

- `ros2 topic list [-t]`: lists all the active topics (-t appends the message type in brackets)
- `ros2 topic echo <topic-name>`: echos the datastream to the terminal output
- `ros2 topic info <topic-name>`: returns message type and publisher/subscriber count
- `ros2 topic pub [--once] [--rate f] <topic-name> <message-type> "<data>"`: publish to a topic in message type with the data in YAML syntax with an optional one time occurence or rate in Hz
- `ros2 topic hz <topic-name>`: View the rate at which the data is being published

> To view more information regarding the message type seen with the -t flag in the list command or with the info command use `ros2 interface show <message-type>`

### Services

_Another method of communication based on a call-and-response model providing data when specifically called by a client_

A service is comprised of a server and client. There can only be one server per service in a graph but there may be many clients using the same service.

- `ros2 service list [-t]`: lists all the active services (-t appends the message type in brackets)
- `ros2 service type <service-name>`: lists the request/response message type of the service
- `ros2 service find <service-type>`: lists all services that use the request/response message type
- `ros2 service call <service-name> <service-type> "<arguments>"`: calls a service with the arguments in YAML syntax

> For more information about the service type we can use `ros2 interface show`

### Parameters

_A node's settings_

Parameters can be either intergers, floats, booleans, strings, and lists and are dependent to a node. These are dynamically reconfigurable and built off of services.

- `ros2 param list`: lists the parameters belonging to each active node
- `ros2 param get <node-name> <param-name>`: get the type and current value of a parameter
- `ros2 param set <node-name> <param-name> <value>`: set the current value of a parameter
- `ros2 param dump <node-name>`: saves all of your current parameter values to a YAML file
- `ros2 param load <node-name> <param-file>`: load parameter values from a YAML file to a node

> To load a parameter file on node start up from the command line: `ros2 run <package-name> <node-name> --ros-args --params-file <file-name>`

### Actions

_A communication type intended for long running tasks consisting of a goal, feedback, and a result_

As previously mention these are built upon topics and services. The action functions as a call-and-response model with feedback. The action client will make a goal service request followed by a results service requests when receiving the goal service response. This will trigger the action server to start publishing on the feedback topic till it has accomplished the task gives a results response to the aciton client. _Actions are preemptable_. The addition of the action means a new base message structure where `---` act as delineators for the goal request, goal response, and feedback stuctures. 

- `ros2 action list [-t]`: list all the active actions (-t appends the message type in brackets)
- `ros2 action info <action-name>`: lists the servers and clients
- `ros2 action send_goal <action-name> <action-type> "<values>" [--feedback]`: send an action goal to an action server where values is in YAML syntax with optionally streaming the feedback>

> For more information about the action type we can use `ros2 interface show`

### Logging

`rqt_console` is a GUI tool for visualizing log messages in ROS2. There are 5 different levels of loggers with them ordered in severity: `Fatal, Error, Warn, Info, Debug` with the default being `Info`. The logger level can be set at node setup: `ros2 run <package-name> <node-name> --ros-args --log-level <log-level>`.