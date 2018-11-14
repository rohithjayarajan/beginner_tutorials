# Beginner Tutorials to Create ROS Publisher and Subscriber
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

---

## Overview

A ROS package consisting of:
- publisher node which publishes a data stream and broadcasts a tf frame called /talk with parent /world
- subscriber node which subscribes to the messages published by the publisher
- level 2 integration tests, that tests publisher (talker) node service

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
- message_generation
- genmsg
- tf

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
git clone --single-branch -b Week11_HW https://github.com/rohithjayarajan/beginner_tutorials.git
```
Follow the below comamnds to build the package
```
cd ~/catkin_ws/
catkin_make
```

## Run Instructions

Follow the below commands in the terminal to run both talker and listener using the launch file created (frequency of publishing can be modified here. Default=20):
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials talk_and_listen.launch frequency:=<value_of_publishing_rate_type_double>
```

Follow the below comamnds in the terminal to run talker and listener separately:
```
cd ~/catkin_ws/
source devel/setup.bash
roscore
```

In a new terminal, follow the below command to run only the publisher (frequency of publishing can be modified here. Default=20):
```
source devel/setup.bash
rosrun beginner_tutorials talker <value_of_publishing_rate_type_double>
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

In a new terminal, follow the below command to run only the subscriber:
```
source devel/setup.bash
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

## Service

Once both nodes are running in the background using either the roslaunch or rosrun method, in a new terminal, follow the below command to call the service (enter the custom string in double quotes):
```
source devel/setup.bash
rosservice call /change_string "<custom_string>"
```

When given custom string "giant leap for mankind" using a service call, an output similar to the below is produced in talker:
```
[ INFO] [1541491519.430439864]: Small step for a man 16
[ INFO] [1541491520.430588102]: Small step for a man 17
[ INFO] [1541491521.430578899]: Small step for a man 18
[ INFO] [1541491522.430512879]: Small step for a man 19
[ INFO] [1541491523.430586864]: Small step for a man 20
[ INFO] [1541491524.430506032]: Small step for a man 21
[ WARN] [1541491524.430654923]: Message published by the talker node will be changed
[ INFO] [1541491524.430697795]: Changed message published by the talker
[ INFO] [1541491525.430489336]: giant leap for mankind 22
[ INFO] [1541491526.430562885]: giant leap for mankind 23
[ INFO] [1541491527.430524923]: giant leap for mankind 24
[ INFO] [1541491528.430403725]: giant leap for mankind 25
[ INFO] [1541491529.430578619]: giant leap for mankind 26
[ INFO] [1541491530.430454509]: giant leap for mankind 27
[ INFO] [1541491531.430607672]: giant leap for mankind 28
```

When given custom string "giant leap for mankind" using a service call, an output similar to the below is produced in listener:
```
[ INFO] [1541491519.430653829]: I heard: [Small step for a man 16]
[ INFO] [1541491520.431087346]: I heard: [Small step for a man 17]
[ INFO] [1541491521.431052072]: I heard: [Small step for a man 18]
[ INFO] [1541491522.430990694]: I heard: [Small step for a man 19]
[ INFO] [1541491523.431052525]: I heard: [Small step for a man 20]
[ INFO] [1541491524.430970950]: I heard: [Small step for a man 21]
[ INFO] [1541491525.430975187]: I heard: [giant leap for mankind 22]
[ INFO] [1541491526.431013869]: I heard: [giant leap for mankind 23]
[ INFO] [1541491527.431011497]: I heard: [giant leap for mankind 24]
[ INFO] [1541491528.430582249]: I heard: [giant leap for mankind 25]
[ INFO] [1541491529.431042541]: I heard: [giant leap for mankind 26]
[ INFO] [1541491530.430942442]: I heard: [giant leap for mankind 27]
[ INFO] [1541491531.431103188]: I heard: [giant leap for mankind 28]
```
## Logging

Once both nodes are running in the background using either the roslaunch or rosrun method, to visualize the logger messages in a GUI, in a new terminal follow the below commands

```
source devel/setup.bash
rosrun rqt_console rqt_console
```

Once both nodes are running in the background using either the roslaunch or rosrun method, to visualize logger_level GUI, in a new terminal follow the below commands

```
source devel/setup.bash
rosrun rqt_logger_level rqt_logger_level
```
## Inspecting TF 

The talker node broadcasts a tf frame called `/talk` with parent `/world`. The transform has non-zero translation and rotation which is time-variant.

### Using view_frames

With roscore and the talker node running in the backgorund, use `rosrun tf view_frames` in a new terminal to view diagram of the frames being broadcast by tf over ROS. An output similar to the below is produced where a tf listener is listening to the frames that are being broadcast over ROS and drawing a tree of how the frames are connected, which is also saved in a .pdf file.

```
Listening to /tf for 5.000000 seconds
Done Listening
dot - graphviz version 2.38.0 (20140413.2041)

Detected dot version 2.38
frames.pdf generated
```

### Using rqt_tf_tree

With roscore and the talker node running in the backgorund, use `rosrun rqt_tf_tree rqt_tf_tree` in a new terminal to use a runtime tool for visualizing the tree of frames being broadcast over ROS. You can refresh the tree simply by the refresh bottom in the top-left corner of the diagram. 

### Using tf_echo

With roscore and the talker node running in the backgorund, use `rosrun tf tf_echo [reference_frame] [target_frame]` in a new terminal to get the information of the transform between any two frames broadcast over ROS. In our case, `[reference_frame]` will be `/world` and `[target_frame]` will be `/talk`. Hence `rosrun tf tf_echo /world /talk` will report the transform of `/talk` with respect to `/world`. An output similar to the below is produced on using the aforementioned command

```
At time 1542153100.809
- Translation: [-1.538, 1.057, 0.014]
- Rotation: in Quaternion [0.600, 0.624, -0.075, -0.494]
            in RPY (radian) [-2.200, -0.556, 1.322]
            in RPY (degree) [-126.061, -31.852, 75.756]
At time 1542153101.509
- Translation: [-1.473, 0.318, 0.896]
- Rotation: in Quaternion [0.185, 0.846, 0.195, -0.461]
            in RPY (radian) [2.835, -1.019, 2.882]
            in RPY (degree) [162.423, -58.366, 165.144]
```

### rviz and tf

With roscore and the talker node running in the backgorund, use the command `rosrun rviz rviz` in a new terminal. This should open rviz which is a visualization tool that is useful for examining tf frames. Once rviz has been opened, set the `Fixed Frame` to `world` in `Global Options`, and then add a tf frame with `talk` and `world` frame enabled. This will help visualize the motion of `talk` frame in a precomputed trajectory as specified in the talker node.

## Running Rostest

Rostest and gtest have been used to write the unit tests for the talker node service. Run the following commands in the terminal to build the tests

```
cd ~/catkin_ws/
source devel/setup.bash
catkin_make run_tests_beginner_tutorials
```

After building it, run the tests using the following commands in a the terminal

```
cd ~/catkin_ws/
source devel/setup.bash
rostest beginner_tutorials talker_service_test.launch
```

An output similar to the below is produced on following the above command

```
[ROSUNIT] Outputting test results to /home/rohith/.ros/test_results/beginner_tutorials/rostest-test_talker_service_test.xml
[Testcase: testtalker_service_test] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talker_service_test/srvTestExists][passed]
[beginner_tutorials.rosunit-talker_service_test/srvTestChangeStr][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/rohith/.ros/log/rostest-Skynet-1528.log
```

## Recording bag Files with the Launch File

To record a rosbag containing all topics being published, follow the steps below in the terminal:

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials talk_and_listen.launch record_all:=true
```

Toggling the `record_all` parameter will enable or disable recording of a rosbag using the `roslaunch` command. Setting `record_all` to `true` will enable rosbag recording and setting `record_all` to `false` will disable rosbag recording. Default value of `record_all` parameter is `false` (rosbag is not recorded in default case)

## Inspecting the bag File

`rosbag info <your bagfile>` helps to inspect a recorded rosbag. In our case, the recorded bag file is record_all.bag. Open the terminal in the folder where the .bag file of interest is present and use the following command to inspect it:

```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag info record_all.bag
```

An output similar to the below is produced on following the above command

```
path:        record_all.bag
version:     2.0
duration:    16.1s
start:       Nov 13 2018 17:34:08.42 (1542148448.42)
end:         Nov 13 2018 17:34:24.50 (1542148464.50)
size:        370.3 KB
messages:    1884
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      318 msgs    : std_msgs/String   
             /rosout       624 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   624 msgs    : rosgraph_msgs/Log 
             /tf           318 msgs    : tf2_msgs/TFMessage
```


## Playing Back the bag File with the Listener Node

Once the bag file of publisher topics has been generated and saved, use the below commands to play back the bag file with the listener node

In a new terminal, follow the below command

```
roscore
```

In a new terminal, follow the below commands

```
cd ~/catkin_ws/
source devel/setup.bash
rosrun beginner_tutorials listener
```

By doing the above, we have started the listener node

In a new terminal, follow the below commands

```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag play record_all.bag
```

By doing this, the listener node is able to subscribe from the messages that is being played by the rosbag

## Killing Processes

Kill the above three processes by pressing CTRL+C in the aforementioned terminals where roscore and rosrun have been run.
Another way to kill the nodes is by running the below command in a new terminal

```
rosnode kill [node_name]
```
