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
 *  @file    main.cpp
 *  @author  rohithjayarajan
 *  @date 11/13/2018
 *  @version 1.3
 *
 *  @brief Test the talker node service
 *
 *  @section DESCRIPTION
 *
 *  Code to run ROS service tests on the talker node in beginner_tutorials
 *
 */

#include <gtest/gtest.h>
// ROS header
#include <ros/ros.h>

/**
 *   @brief main function for ROS tests
 *
 *   @param argc  The argc
 *   @param argv  The argv
 *   @return int value of 0 on successful execution of function, -1 otherwise
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker_service_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
