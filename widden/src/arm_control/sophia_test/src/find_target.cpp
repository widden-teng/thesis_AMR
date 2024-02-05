#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <geometry_msgs/Transform.h>
using namespace std;

class Tag_Pose{
public:
	ros::NodeHandle nh;
	ros::Subscriber sub_tag;
	ros::Publisher pub_hole;
	ros::Publisher pub_target;

	tf::Transform T_robot_to_tag0;
	tf::Transform T_tag9_to_hole;

	tf::Transform T_cam_to_tag0;
	tf::Transform T_cam_to_tag1;
	tf::Transform T_cam_to_tag9;

	tf::Transform T_robot_to_hole;
	tf::Transform T_target;

	Tag_Pose(){
		tf::Quaternion q_robot2tag0(0.0030784, -0.0068594, 0.71196, 0.70218);
		// tf::Vector3 v_robot2tag0(0.51682, 0.16841, 0.003284 );// (0.51722, 0.16441, 0.003284 )
		tf::Vector3 v_robot2tag0(0.5178511108398437, 0.16416395263671874, 0.001284);//0.513
		// tf::Vector3 v_robot2tag0(0.5114511108398437, 0.16886395263671874, 0.001284);
		T_robot_to_tag0.setOrigin(v_robot2tag0);
		T_robot_to_tag0.setRotation(q_robot2tag0);

		tf::Quaternion q_tag9tohole;
		q_tag9tohole.setRPY(-M_PI/2,0,0);
		tf::Vector3 v_tag9tohole(0,0.075,-0.044);
		// q_tag9tohole.setRPY(0.0,M_PI/2,0);
		// tf::Vector3 v_tag9tohole(0.078,0.0,-0.043);
		T_tag9_to_hole.setOrigin(v_tag9tohole);
		T_tag9_to_hole.setRotation(q_tag9tohole);

		pub_hole=nh.advertise<geometry_msgs::Transform> ("/robot_to_hole", 1);
		pub_target=nh.advertise<geometry_msgs::Transform> ("/robot_target", 1);
		sub_tag=nh.subscribe("/tag_detections", 10, &Tag_Pose::tagCallback,this);
	}
	void tagCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr &msg){
		// ROS_ERROR("in callback\n");
		for(int i=0;i<msg->detections.size();i++){
			tf::Quaternion q(msg->detections[i].pose.pose.orientation.x,msg->detections[i].pose.pose.orientation.y,msg->detections[i].pose.pose.orientation.z,msg->detections[i].pose.pose.orientation.w);
			tf::Vector3 vec(msg->detections[i].pose.pose.position.x,msg->detections[i].pose.pose.position.y,msg->detections[i].pose.pose.position.z);
			if(msg->detections[i].id==0){
				T_cam_to_tag0.setOrigin(vec);
				T_cam_to_tag0.setRotation(q);
			}
			else if(msg->detections[i].id==1){
				T_cam_to_tag1.setOrigin(vec);
				T_cam_to_tag1.setRotation(q);
			}
			else if(msg->detections[i].id==9){
				T_cam_to_tag9.setOrigin(vec);
				T_cam_to_tag9.setRotation(q);

				T_robot_to_hole=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_tag9*T_tag9_to_hole;
				geometry_msgs::Transform msg_robot2hole;
				msg_robot2hole.translation.x=T_robot_to_hole.getOrigin().x();
				msg_robot2hole.translation.y=T_robot_to_hole.getOrigin().y();
				msg_robot2hole.translation.z=T_robot_to_hole.getOrigin().z();
				msg_robot2hole.rotation.x=T_robot_to_hole.getRotation().x();
				msg_robot2hole.rotation.y=T_robot_to_hole.getRotation().y();
				msg_robot2hole.rotation.z=T_robot_to_hole.getRotation().z();
				msg_robot2hole.rotation.w=T_robot_to_hole.getRotation().w();
				pub_hole.publish(msg_robot2hole);
				target_set();

			}
		}
	}
	void target_set(){
		//test rosrun tf static_transform_publisher 0 0 0.3 1 0 0 0 /hole /tt 100
		tf::Quaternion q_hole2target(1.0, 0.0 ,0.0 ,0.0);
		tf::Vector3 v_hole2target(0.0, 0.0, 0.273);
		tf::Transform T_hole2target(q_hole2target,v_hole2target);
		T_target=T_robot_to_hole*T_hole2target;
		// tf::Quaternion qq(1.0/sqrt(2.0), 1.0/sqrt(2.0) ,0.0 ,0.0);
		// T_target.setRotation(qq);

		geometry_msgs::Transform msg_robottarget;
		msg_robottarget.translation.x=T_target.getOrigin().x();
		msg_robottarget.translation.y=T_target.getOrigin().y();
		msg_robottarget.translation.z=T_target.getOrigin().z();
		msg_robottarget.rotation.x=T_target.getRotation().x();
		msg_robottarget.rotation.y=T_target.getRotation().y();
		msg_robottarget.rotation.z=T_target.getRotation().z();
		msg_robottarget.rotation.w=T_target.getRotation().w();


		// tf::Quaternion q(msg_robottarget.rotation.x,msg_robottarget.rotation.y,msg_robottarget.rotation.z,msg_robottarget.rotation.w);
  //   	tf::Matrix3x3 m(q);
  //   	double deg[3];
  //   	m.getRPY(deg[0], deg[1], deg[2]);
  //   	deg[0]=-M_PI;
  //   	deg[1]=0.0;

  //   	Eigen::AngleAxisf yawAngle (deg[2], Eigen::Vector3f::UnitZ());
	 //    Eigen::AngleAxisf pitchAngle (deg[1], Eigen::Vector3f::UnitY());
	 //    Eigen::AngleAxisf rollAngle(deg[0], Eigen::Vector3f::UnitX());
	 //    Eigen::Quaternion<float> eq = yawAngle * pitchAngle *rollAngle;

		
		// msg_robottarget.rotation.x=eq.x();
		// msg_robottarget.rotation.y=eq.y();
		// msg_robottarget.rotation.z=eq.z();
		// msg_robottarget.rotation.w=eq.w();
		pub_target.publish(msg_robottarget);
	}
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_robot2target");

  Tag_Pose tag_pose;
  
  ros::spin();
  return 0;
}
