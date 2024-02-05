#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ostream>
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("start_move", 1000);

  ros::Rate loop_rate(10);
  std_msgs::Int32 count;
  while (ros::ok())
  {
    std::cin>>count.data;
    chatter_pub.publish(count);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}