#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "target_pub");
    ros::NodeHandle nh;

    ros::Publisher pub_target=nh.advertise<geometry_msgs::PoseStamped> ("/calibrate", 1);
    double value[7];

    while(ros::ok()){
    	geometry_msgs::PoseStamped targetA;
    	targetA.header.frame_id="test";

    	for(int i=0;i<7;i++)
    		cin >> value[i];

    	for(int i=0;i<7;i++){
    		cout<<value[i]<<" ";
    	}
    	cout<<endl;

    	targetA.pose.position.x=value[0];
    	targetA.pose.position.y=value[1];
    	targetA.pose.position.z=value[2];
    	targetA.pose.orientation.x=value[3];
    	targetA.pose.orientation.y=value[4];
    	targetA.pose.orientation.z=value[5];
    	targetA.pose.orientation.w=value[6];

    	pub_target.publish(targetA);




    }

	




	return 0;
}