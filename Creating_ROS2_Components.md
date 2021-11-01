# Creating ROS2 components

#### Author: Ducky

#### Last Update: 2021.11.01

## Overview

This document serves to record the process of creating various fundamental components for ROS2. This creation process for each of the components will serve as a base for developing future more complex components.

## Launch files

> Launch files can be written in Python, XML, or YAML. More later...

#### Setup

- `mkdir launch`
- `touch launch/<launch-file-name>.py`

#### Simple Sample Structure

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package-name',
            namespace='namespace',
            executable='node-name',
            name='name',
        ),
        Node(
            package='package-name',
            namespace='namespace',
            executable='node-name',
            name='name',
            remappings=[
                ('/input1', '/output1'),
                ('/input2', 'output2'),
            ]
        ),
    ])
```

#### Launching

We can launch the file either directly from the `launch` directory or if it is provided by a package we can find it through the package name.

- `ros2 launch <package-name> <launch-file-name>` or `ros2 launch <launch-file-name>` if in `launch` directory