// picking/src/tm_DL_binpicking/src/pose_estimation.cpp
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include <time.h>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

//TM5 Driver
#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_robot_state.h"
#include "tm_driver/tm_ros_node.h"
#include "tm_kinematics/tm_kin.h"

#include  <tm_msgs/FeedbackState.h>

#define D2R 0.01745329252
#define R2D 57.29577951
#define NUMBER_OF_DOFS    6
using namespace std;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pubpub");
    ros::NodeHandle node_handle;
    vector< vector<double> > matrix;


    ROS_INFO("pubpub /robot_target");
    ros::Publisher pub = node_handle.advertise <geometry_msgs::Transform>("/robot_target",1000);

    vector<double>  goal;
    int ccount;
    ccount = 0;
    fstream file;
    file.open("output.csv");
    string line;
    vector <double> aline;
    while (getline( file, line,'\n')){  //讀檔讀到跳行字元
        istringstream templine(line); // string 轉換成 stream
        string data;
        aline.clear();
        while (getline( templine, data,',')){ //讀檔讀到逗號
            aline.push_back(atof(data.c_str()));  //string 轉換成數字

        }
        //cout << aline.at(0)<<endl;
        matrix.push_back(aline);
    }
    ros::Rate loop_rate(1);
    cout << "all_size : "<< matrix.size()<<endl;
    int csv_length = matrix.size();
    file.close();
    // ros::Publisher pub_estimation;
    tf::Quaternion q;
    q.setRPY(M_PI, 0, M_PI/2);
    while(ros::ok() && ccount < csv_length){
        geometry_msgs::Transform trans;
        printf("nowx: %f\n",matrix.at(ccount).at(0));
        trans.translation.x = 0.51 - matrix.at(ccount).at(1);
        trans.translation.y = 0.164 + matrix.at(ccount).at(0);
        trans.translation.z = 0.28-0.095 + matrix.at(ccount).at(2);
        trans.rotation.x = q.getX();
        trans.rotation.y = q.getY();
        trans.rotation.z = q.getZ();
        trans.rotation.w = q.getW();

        pub.publish(trans);
        ccount++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    
    return 0;
}
