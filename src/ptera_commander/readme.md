# ptera_commander

The **ptera_commander** is a command line tool for sending commands to various subsystems

## Dependencies

The list of dependencies are in deps.sh. This file will be used by ptera_msgs to install the required dependencies.

## Install

This module depends on custom ROS2 msgs defined in the ptera_msgs repo. Clone ptera_msgs into a workspace and run the setup script for the vehicle as shown below.

```bash
$ mkdir -p ~/robot_ws/src
$ git clone https://github.com/samlovelace/ptera_msgs.git
$ cd ptera_msgs/scripts
$ chmod +x setup.sh
$ sudo ./setup commander
```

## Run

To run the ptera_commander, from the root of the workspace

```bash
source install/setup.bash
ros2 run ptera_commander ptera_commander
```
