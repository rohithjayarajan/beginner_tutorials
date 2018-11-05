# Beginner Tutorials to Create ROS Publisher and Subscriber
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

---

## Overview

A ROS package consisting of:
- publisher node which publishes a data stream
- subscriber node which subscribes to the messages published by the publisher

## License
```
BSD 3-Clause License

Copyright (c) 2018, Rohith Jayarajan
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Dependencies

### ROS

[ROS][reference-id-for-ROS] The Robot Operating System (ROS) is a flexible 
framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify 
the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
This package is developed and tested in ROS Kinetic on Ubuntu 16.04 LTS and needs ROS Kinetic Kame installed for use. 
The entire installation instructions for ROS Kinetic Kame and its dependencies can be found [here][reference-id-for-ROS Kinetic].

[reference-id-for-ROS Kinetic]: http://wiki.ros.org/kinetic
[reference-id-for-ROS]: http://www.ros.org/install/

### catkin

Catkin is included by default when ROS is installed. Catkin can also be installed from source or prebuilt packages. 
Most users will want to use the prebuilt packages, but installing it from source is also quite simple. Installation 
instructions can be found [here][reference-id-for-catkin]

[reference-id-for-catkin]: http://wiki.ros.org/catkin

### Package Dependencies
- roscpp
- rospy
- std_msgs

## Build Instructions

### Creating catkin workspace:
Follow the below comamnds to create a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
The above commands will create a workspace for your packages with CMakeLists.txt link in the src folder of catkin_ws. 
Source the setup.*sh file: 
```
source devel/setup.bash
```
### Building package inside catkin workspace: 
Follow the below comamnds and clone this package in the src folder of the catkin workspace 
```
cd ~/catkin_ws/src/
git clone https://github.com/rohithjayarajan/beginner_tutorials.git
```
Follow the below comamnds to build the package
```
cd ~/catkin_ws/src/
catkin_make
```

## Run Instructions

Follow the below comamnds in the terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roscore
```

In a new terminal, follow the below command to run the publisher:
```
rosrun beginner_tutorials talker
```

This will produce an output similar to the below:
```
[ INFO] [1540870090.945308411]: Small step for a man 38
[ INFO] [1540870091.045340713]: Small step for a man 39
[ INFO] [1540870091.145322321]: Small step for a man 40
[ INFO] [1540870091.245318646]: Small step for a man 41
[ INFO] [1540870091.345162339]: Small step for a man 42
[ INFO] [1540870091.445337574]: Small step for a man 43
[ INFO] [1540870091.545341943]: Small step for a man 44
[ INFO] [1540870091.645176469]: Small step for a man 45
[ INFO] [1540870091.745346018]: Small step for a man 46
[ INFO] [1540870091.845338566]: Small step for a man 47
[ INFO] [1540870091.945339660]: Small step for a man 48
[ INFO] [1540870092.045339571]: Small step for a man 49
[ INFO] [1540870092.145312919]: Small step for a man 50
```

In a new terminal, follow the below command to run the subscriber:
```
rosrun beginner_tutorials listener
```
This will produce an output similar to the below:
```
[ INFO] [1540870090.946012164]: I heard: [Small step for a man 38]
[ INFO] [1540870091.045869269]: I heard: [Small step for a man 39]
[ INFO] [1540870091.145848990]: I heard: [Small step for a man 40]
[ INFO] [1540870091.245844892]: I heard: [Small step for a man 41]
[ INFO] [1540870091.345417764]: I heard: [Small step for a man 42]
[ INFO] [1540870091.445901801]: I heard: [Small step for a man 43]
[ INFO] [1540870091.545900712]: I heard: [Small step for a man 44]
[ INFO] [1540870091.645711634]: I heard: [Small step for a man 45]
[ INFO] [1540870091.745869826]: I heard: [Small step for a man 46]
[ INFO] [1540870091.845894422]: I heard: [Small step for a man 47]
[ INFO] [1540870091.945898786]: I heard: [Small step for a man 48]
[ INFO] [1540870092.045894560]: I heard: [Small step for a man 49]
[ INFO] [1540870092.145866303]: I heard: [Small step for a man 50]
```

Kill the above three processes by pressing CTRL+C in the aforementioned terminals where roscore and rosrun have been run.
Another way to kill the nodes is by running the below command in a new terminal

```
rosnode kill [node_name]
```