#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
using namespace std;

tf::Quaternion q_robot2tag0(0.0030784, -0.0068594, 0.71196, 0.70218);
// tf::Vector3 v_robot2tag0(0.51722, 0.16441, 0.003284 );
tf::Vector3 v_robot2tag0(0.5178511108398437, 0.16416395263671874, 0.001284 );// (0.51722, 0.16441, 0.003284 )
// tf::Vector3 v_robot2tag0(0.5114511108398437, 0.16886395263671874, 0.001284 );
tf::Transform T_robot2tag0(q_robot2tag0,v_robot2tag0);

tf::Transform T_cam2tag0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_robot2cam");

  ros::NodeHandle n;
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  ros::Rate r(30);
  while(ros::ok()){
 //  	tf::StampedTransform transform;
 //  	try{
 //  		listener.lookupTransform("/camera_color_optical_frame", "/tag0", ros::Time(0), transform);
 //  	}
 //  	catch(tf::TransformException ex){
 //  		ROS_ERROR("%s",ex.what());
 //  		ros::Duration(1.0).sleep();
 //  	}
 //  	tf::Transform temp(transform.getRotation(),transform.getOrigin());
 //  	T_cam2tag0=temp;
 //  	// cout << T_cam2tag0.getOrigin().x() << " "<< T_cam2tag0.getOrigin().y() << " "<< T_cam2tag0.getOrigin().z() << "\n";

	// tf::Transform T_robot2cam=T_robot2tag0*T_cam2tag0.inverse();

	// cout << T_robot2cam.getOrigin().x() << " "<< T_robot2cam.getOrigin().y() << " "<< T_robot2cam.getOrigin().z() << "\n";
	// broadcaster.sendTransform(tf::StampedTransform(T_robot2cam,ros::Time::now(),"base", "camera_color_optical_frame"));
    broadcaster.sendTransform(tf::StampedTransform(T_robot2tag0.inverse(),ros::Time::now(), "tag0", "base"));
    r.sleep();
  }

  return 0;
}
