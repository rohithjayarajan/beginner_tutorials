/******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (C) 2018, Rohith Jayarajan
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 * names of its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 *  @file    talker.cpp
 *  @author  rohithjayarajan
 *  @date 11/13/2018
 *  @version 1.3
 *
 *  @brief Code to create a publisher node
 *
 *  @section DESCRIPTION
 *
 *  Code to create and implement a publisher ("talker") node which will
 * continually broadcast a message
 *
 */

// inlcude talker header file
#include "talker.hpp"
#include <tf/transform_broadcaster.h>

dataStreamMessage publishStr;
/**
 *   @brief This function provides the service changing the base output string
 * in the published message
 *
 *   @param request and response type defined in srv file
 *   @return boolean value indicating success
 */
bool editString(beginner_tutorials::change_string::Request &req,
                beginner_tutorials::change_string::Response &res) {
  ROS_WARN_STREAM("Message published by the talker node will be changed");
  publishStr.messageString = req.newString;
  ROS_INFO_STREAM("Changed message published by the talker");
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
/**
 *   @brief main function for ROS publisher node
 *
 *   @param nothing
 *   @return int value of 0 on successful execution of function
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command
   * line. For programmatic remappings you can use a different version of init()
   * which takes remappings directly, but for most command-line programs,
   * passing argc and argv is the easiest way to do it.  The third argument to
   * init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // create a TransformBroadcaster object that will be used to send the
  // transformations
  static tf::TransformBroadcaster br;

  // create a Transform object
  tf::Transform transform;

  // create a Quaternion object
  tf::Quaternion q;

  // define radius along x axis of ellipsoid
  double a = 3;

  // define radius along y axis of ellipsoid
  double b = 1;

  // define radius along z axis of ellipsoid
  double c = 2;

  // define angular velocity;
  double w = 0.5 * M_PI;

  // rate of publishing messages by node
  double frequency = 10;

  // check if custom frequency is requested in argument
  if (argc == 2) {
    ROS_DEBUG_STREAM("Frequency of publishing will be changed to " << argv[1]
                                                                   << "Hz");
    frequency = atoi(argv[1]);
    if (frequency < 0) {
      ROS_FATAL_STREAM("Rate of publishing can't be negative" << std::endl);
      return -1;
    }
  }

  // create service and advertise over ROS
  ros::ServiceServer service = n.advertiseService("change_string", editString);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // specifying rate at which to loop at
  ros::Rate loop_rate(frequency);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    // Get the time value from ROS
    double t = ros::Time::now().toSec();

    // Find the coordinate of frame at given time with fixed angular velocity
    double theta = w * t;
    double x = a * cos(theta / 4) * sin(theta / 2);
    double y = b * sin(theta / 4) * cos(theta / 2);
    double z = c * cos(theta / 2);

    // set origin of frame
    transform.setOrigin(tf::Vector3(x, y, z));

    // set orientation of frame
    q.setRPY(0.75 * M_PI, 0.25 * M_PI, theta);
    transform.setRotation(q);

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << publishStr.messageString << " " << count;
    msg.data = ss.str();

    // check if custom message is empty and print error to notify user
    if (publishStr.messageString == "") {
      ROS_ERROR_STREAM(
          "No message received as input. Are you sure message was entered "
          "properly? "
          << std::endl);
    } else {
      // echo the message heard from listener
      ROS_INFO("%s", msg.data.c_str());
    }

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    // broadcast transform
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    // call the callbacks
    ros::spinOnce();
    // sleep for remaining time to hit 10Hz publish rate
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
