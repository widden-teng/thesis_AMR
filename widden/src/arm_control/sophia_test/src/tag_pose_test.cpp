#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "tf/tf.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

using namespace std;

ofstream tag0_pose;
ofstream tag1_pose;
ofstream tag1_pose_in_world;
tf::Quaternion q_cam2tag;
tf::Vector3 v_cam2tag;
tf::Quaternion q_cam2world;
tf::Vector3 v_cam2world;

int cnt=100;

void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
  //cout<<*msg<<endl;

  for(int i=0;i<msg->detections.size();i++){
    if(msg->detections[i].id[0]==0 && cnt<=100){
      tf::Quaternion q(msg->detections[i].pose.pose.pose.orientation.x,msg->detections[i].pose.pose.pose.orientation.y,msg->detections[i].pose.pose.pose.orientation.z,msg->detections[i].pose.pose.pose.orientation.w);
      // q_cam2world=q;
      tf::Vector3 vec(msg->detections[i].pose.pose.pose.position.x,msg->detections[i].pose.pose.pose.position.y,msg->detections[i].pose.pose.pose.position.z);
      // v_cam2world=vec;

      if(cnt==0){
        q_cam2world=q;
        v_cam2world=vec;
        cnt++;
      }
      else{
        q_cam2world=q/2+q_cam2world/2;
        v_cam2world=vec/2+v_cam2world/2;
        cnt++;
      }
      cout<<cnt<<endl;

      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      // m.getRPY(roll, pitch, yaw);
      m.getEulerYPR( yaw, pitch, roll);

      char outmsg[200];
      // sprintf(outmsg,"%.15f %.15f %.15f %.15f %.15f %.15f",msg->detections[i].pose.pose.pose.position.x,msg->detections[i].pose.pose.pose.position.y,msg->detections[i].pose.pose.pose.position.z,roll,pitch,yaw);
      sprintf(outmsg,"%.15f %.15f %.15f %.15f %.15f %.15f %.15f",msg->detections[i].pose.pose.pose.position.x,msg->detections[i].pose.pose.pose.position.y,msg->detections[i].pose.pose.pose.position.z,msg->detections[i].pose.pose.pose.orientation.w,msg->detections[i].pose.pose.pose.orientation.x,msg->detections[i].pose.pose.pose.orientation.y,msg->detections[i].pose.pose.pose.orientation.z);
      // tag0_pose << msg->detections[i].pose.header.stamp << " " << outmsg << endl;
    }

    else if(msg->detections[i].id[0]==8){
      tf::Quaternion q(msg->detections[i].pose.pose.pose.orientation.x,msg->detections[i].pose.pose.pose.orientation.y,msg->detections[i].pose.pose.pose.orientation.z,msg->detections[i].pose.pose.pose.orientation.w);
      q_cam2tag=q;
      tf::Vector3 vec(msg->detections[i].pose.pose.pose.position.x,msg->detections[i].pose.pose.pose.position.y,msg->detections[i].pose.pose.pose.position.z);
      v_cam2tag=vec;
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      // m.getRPY(roll, pitch, yaw);
      m.getEulerYPR( yaw, pitch, roll);

      char outmsg[200];
      sprintf(outmsg,"%.15f %.15f %.15f %.15f %.15f %.15f",msg->detections[i].pose.pose.pose.position.x,msg->detections[i].pose.pose.pose.position.y,msg->detections[i].pose.pose.pose.position.z,roll/M_PI*180,pitch/M_PI*180,yaw/M_PI*180);
      // sprintf(outmsg,"%.15f %.15f %.15f %.15f %.15f %.15f %.15f",msg->detections[i].pose.pose.pose.position.x,msg->detections[i].pose.pose.pose.position.y,msg->detections[i].pose.pose.pose.position.z,msg->detections[i].pose.pose.pose.orientation.w,msg->detections[i].pose.pose.pose.orientation.x,msg->detections[i].pose.pose.pose.orientation.y,msg->detections[i].pose.pose.pose.orientation.z);
      // tag1_pose << msg->detections[i].pose.header.stamp << " " << outmsg << endl;
      //cout << roll/M_PI*180 << " " << pitch/M_PI*180 << " " << yaw/M_PI*180 << endl;
      cout<<outmsg<<endl;

      // tf::Transform T_cam2tag(q_cam2tag,v_cam2tag);
      // tf::Transform T_cam2world(q_cam2world,v_cam2world);
      // tf::Transform T_world2tag;
      // T_world2tag=T_cam2world.inverse()*T_cam2tag;
      // // T_world2tag=T_cam2world.inverseTimes(T_cam2tag);

      // // double roll, pitch, yaw;
      // // T_world2tag.m_basis.getRPY(roll, pitch, yaw);
      // T_world2tag.getBasis().getEulerYPR( yaw, pitch, roll);


      // cout << T_world2tag.getOrigin().x() << " "<< T_world2tag.getOrigin().y() << " "<< T_world2tag.getOrigin().z() << " ";
      // cout << roll/M_PI*180 << " " << pitch/M_PI*180 << " " << yaw/M_PI*180 << endl;
      // // cout<< T_world2tag.getRotation().w() << " " << T_world2tag.getRotation().x() << " " << T_world2tag.getRotation().y() << " " << T_world2tag.getRotation().z() <<endl;

      // char outmsg2[200];

      // // sprintf(outmsg2,"%.15f %.15f %.15f %.15f %.15f %.15f %.15f",T_world2tag.getOrigin().x(),T_world2tag.getOrigin().y(),T_world2tag.getOrigin().z(),T_world2tag.getRotation().w(),T_world2tag.getRotation().x(),T_world2tag.getRotation().y(),T_world2tag.getRotation().z());
      // sprintf(outmsg2,"%.15f %.15f %.15f %.15f %.15f %.15f",T_world2tag.getOrigin().x(),T_world2tag.getOrigin().y(),T_world2tag.getOrigin().z(),roll/M_PI*180,pitch/M_PI*180,yaw/M_PI*180);
      // if(cnt>100)tag1_pose_in_world << msg->detections[i].pose.header.stamp << " " << outmsg2 << endl;

    }
  }

  // tf::Transform T_cam2tag(q_cam2tag,v_cam2tag);
  // tf::Transform T_cam2world(q_cam2world,v_cam2world);
  // tf::Transform T_world2tag;
  // T_world2tag=T_cam2world.inverse()*T_cam2tag;
  // // T_world2tag=T_cam2world.inverseTimes(T_cam2tag);

  // double roll, pitch, yaw;
  // // T_world2tag.m_basis.getRPY(roll, pitch, yaw);
  // T_world2tag.getBasis().getEulerYPR( yaw, pitch, roll);


  // cout << T_world2tag.getOrigin().x() << " "<< T_world2tag.getOrigin().y() << " "<< T_world2tag.getOrigin().z() << " ";
  // cout << roll/M_PI*180 << " " << pitch/M_PI*180 << " " << yaw/M_PI*180;// << endl;
  // // cout<< T_world2tag.getRotation().w() << " " << T_world2tag.getRotation().x() << " " << T_world2tag.getRotation().y() << " " << T_world2tag.getRotation().z() <<endl;

  // char outmsg[200];
  // sprintf(outmsg,"%.15f %.15f %.15f %.15f %.15f %.15f %.15f",T_world2tag.getOrigin().x(),T_world2tag.getOrigin().y(),T_world2tag.getOrigin().z(),T_world2tag.getRotation().w(),T_world2tag.getRotation().x(),T_world2tag.getRotation().y(),T_world2tag.getRotation().z());
  // tag1_pose_in_world << msg->detections[i].pose.header.stamp << " " << outmsg << endl;
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  // tag0_pose.open ("world_pose.txt", std::ofstream::out | std::ofstream::trunc);
  // tag1_pose.open ("tag_pose.txt", std::ofstream::out | std::ofstream::trunc);
  // tag1_pose_in_world.open ("tag_pose_in_world.txt", std::ofstream::out | std::ofstream::trunc);

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/tag_detections", 10, tagCallback);

  ros::spin();
  return 0;
}