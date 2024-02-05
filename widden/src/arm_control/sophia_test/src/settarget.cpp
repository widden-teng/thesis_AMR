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
	ros::init(argc, argv, "settarget_pub");
    ros::NodeHandle nh;

    ros::Publisher pub_target=nh.advertise<geometry_msgs::Transform> ("/robot_target", 1);

    tf::Quaternion q0;
    q0.setRPY(3.134912270940612, 0.03675302120266578, 1.529686307469834);
    tf::Vector3 v0(0.4229355163574219, 0.42133221435546875, 0.31526019287109375);
    tf::Transform T0(q0,v0);

    tf::Quaternion q1;
    q1.setRPY(3.0562710975341614, 0.07808548938015572, 1.5659933157689387);
    tf::Vector3 v1(0.3909290954589844, 0.42178546142578127, 0.25426927185058595);
    tf::Transform T1(q1,v1);

    tf::Quaternion q2;
    q2.setRPY(-2.9490767331296004, 0.003962182150123712, 1.5268347277326315);
    tf::Vector3 v2(0.49142669677734374, 0.39846527099609375, 0.25338877868652343);
    tf::Transform T2(q2,v2);



    tf::Quaternion qup(0.0, 0.0 ,0.0, 1.0);
    tf::Vector3 vup(0.0, 0.0, -0.03);
    tf::Transform Tup(qup,vup);
    tf::Quaternion qdown(0.0, 0.0 ,0.0, 1.0);
    tf::Vector3 vdown(0.0, 0.0, 0.03);
    tf::Transform Tdown(qdown,vdown);

    tf::Quaternion qq(0.0, 0.0 ,0.0, 1.0);
    tf::Vector3 vv(0.0, 0.0, 0.00);
    tf::Transform TT(qq,vv);

    tf::Transform T_target;
    tf::Transform T_target2;


    double valid;

    while(ros::ok()){
    	geometry_msgs::Transform msg_robottarget;

        cout<<"enter point 0/1/2\n";
        cin>>valid;
        if(valid==1){
            T_target=T1;
        }
        else if(valid==2){
            T_target=T2;
        }
        else{
            T_target=T0;
        }
        cout<<"enter point 0/ up 1/ down 2\n";
        cin>>valid;
        if(valid==2){
            T_target2=Tdown;
        }
        else if(valid==1){
            T_target2=Tup;
        }
        else{
            T_target2=TT;
        }

        T_target=T_target*T_target2;


        msg_robottarget.translation.x=T_target.getOrigin().x();
        msg_robottarget.translation.y=T_target.getOrigin().y();
        msg_robottarget.translation.z=T_target.getOrigin().z();
        msg_robottarget.rotation.x=T_target.getRotation().x();
        msg_robottarget.rotation.y=T_target.getRotation().y();
        msg_robottarget.rotation.z=T_target.getRotation().z();
        msg_robottarget.rotation.w=T_target.getRotation().w();

    	pub_target.publish(msg_robottarget);




    }

	




	return 0;
}