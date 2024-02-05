/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/ros_filter_types.h"
#include <std_msgs/Bool.h> 
#include <ros/ros.h>

#include <cstdlib>
#include <vector>

bool init_flag = false;

void boolCallback(const std_msgs::Bool::ConstPtr& msg) {
    init_flag = true;
    ROS_INFO("get cb");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ukf_navigation_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  std::vector<double> args(3, 0);
  nhLocal.param("alpha", args[0], 0.001);
  nhLocal.param("kappa", args[1], 0.0);
  nhLocal.param("beta",  args[2], 2.0);

  ros::Subscriber bool_subscriber = nh.subscribe("init_topic", 10, boolCallback);
  ros::Publisher start_record_pub = nh.advertise<std_msgs::Bool>("start_record_topic", 1);
  
  RobotLocalization::RosUkf ukf(nh, nhLocal, args);

  // // 不能改用spinOnce，因為他有排程問題
  // // ros::Rate loop_rate(10);
  // // while(init_flag!= true) {
  // //   ukf.initialize();
  // //   ros::spinOnce();
  // //   loop_rate.sleep();
  // // }


  // // 不能用try catch, 因為不是spin()本身出錯
  // try{
  //   ukf.initialize();
  //   ros::spin();
  // }
  // catch (...) {
  //   std::cout << "!!!!!!!!!!!!!!!!!!!" << std::endl;
  //   ukf.initialize();
  //   ros::spin();
  // }
  std_msgs::Bool bool_msg;
  bool_msg.data = true;

  std::cin.get();
  ukf.initialize();
  start_record_pub.publish(bool_msg);
  ros::spin();



  return EXIT_SUCCESS;
}
