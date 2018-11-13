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
 *  @file    talker_service_test.cpp
 *  @author  rohithjayarajan
 *  @date 11/13/2018
 *  @version 1.3
 *
 *  @brief Test the talker node service
 *
 *  @section DESCRIPTION
 *
 *  Code to run ROS tests on the talker node in beginner_tutorials
 *
 */

#include <gtest/gtest.h>
// ROS header
#include <ros/ros.h>
// Service header
#include <ros/service_client.h>
#include "beginner_tutorials/change_string.h"

/**
 *   @brief test function to check if service exists
 *
 *   @param TESTSuite, the gtest framework
 *   @param srvTestExists, the name of test
 */
TEST(TESTSuite, srvTestExists) {
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  // create client to the service change_string
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::change_string>("change_string");
  // check whether the service exists
  bool exists(client.waitForExistence(ros::Duration(5)));
  EXPECT_TRUE(exists);
}

/**
 *   @brief test function to check if client is reporting correct message
 *
 *   @param TESTSuite, the gtest framework
 *   @param srvTestChangeStr, the name of test
 */
TEST(TESTSuite, srvTestChangeStr) {
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  // create client to the service change_string
  ros::ServiceClient client =
      nh.serviceClient<beginner_tutorials::change_string>("change_string");
  beginner_tutorials::change_string srv;
  // set the newString
  srv.request.newString = "test input";
  // call the client to the service
  client.call(srv);
  // check if the string entered is the same as what the client reports
  EXPECT_STREQ("test input", srv.response.newStringResp.c_str());
}
