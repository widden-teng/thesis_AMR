#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#define X_UPLIMIT 0.8
#define X_DOWNLIMIT 0.0
#define Y_UPLIMIT 0.2
#define Y_DOWNLIMIT -0.6 //-0.6
#define Z_UPLIMIT 1.6 // 1.6
#define Z_DOWNLIMIT 0.95 //1.0


using namespace std;

class Pose_adjust_tf{

    public:
        Pose_adjust_tf();
        ~Pose_adjust_tf();
        // tf2_ros::Buffer tfBuffer;
        // tf2_ros::TransformListener tfListener(tfBuffer);
        void send_final_goal_CB(const nav_msgs::Odometry msg);
        void outrange_judgment();
        // void run();
        ros::Publisher pub_sendgoal;

    private:
        ros::NodeHandle n;
        ros::Subscriber handpose_sub;
        // ros::Publisher pub_sendgoal;


        geometry_msgs::PointStamped hand_pt, armbase_pt;
        tf2_ros::Buffer tfBuffer_;

        geometry_msgs::PoseArray vel_static_path;


        bool inrange_flag = false; 
        bool MoveArm_flag = true;// 用來擋pub點到手臂
        int step = 0;

        geometry_msgs::PoseStamped hand_posestamped;
        geometry_msgs::PoseStamped goal_posestamped;
        geometry_msgs::Pose goal_pose;

        int pub_time = 0;

        tf::TransformListener* tf_listener_= new tf::TransformListener();
        // tf_listener_ = new tf::TransformListener();

};

Pose_adjust_tf::Pose_adjust_tf()
{
    handpose_sub = n.subscribe("/odometry/filtered", 1, &Pose_adjust_tf::send_final_goal_CB, this);
    pub_sendgoal = n.advertise<geometry_msgs::Pose>("/sendGoal", 1);
}

Pose_adjust_tf::~Pose_adjust_tf(){

}
// 用來限制 handover 的範圍(避免機器手臂超出範圍)
// 超過的話回傳 true(不會pub到手臂)
void Pose_adjust_tf::outrange_judgment(){
    if (MoveArm_flag)
        if( goal_pose.position.x >  X_DOWNLIMIT && goal_pose.position.x < X_UPLIMIT \
            && goal_pose.position.y > Y_DOWNLIMIT && goal_pose.position.y < Y_UPLIMIT \
            && goal_pose.position.z > Z_DOWNLIMIT && goal_pose.position.z < Z_UPLIMIT){
            // if( goal_pose.position.y > Y_DOWNLIMIT && goal_pose.position.y < Y_UPLIMIT \
            // && goal_pose.position.z > Z_DOWNLIMIT && goal_pose.position.z < Z_UPLIMIT){

            inrange_flag = true;
            cout<<"in range"<< endl;

        }       
        else
            cout<<"Out of range"<< endl;
}


void Pose_adjust_tf::send_final_goal_CB(const nav_msgs::Odometry msg){
    

    tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
    // 長度可能不等於1, 你的旋轉部分可能會不正確(不確定)
    q.normalize();
    
    
    hand_posestamped.header.frame_id = "top_camera_link";
    hand_posestamped.pose.position.x = msg.pose.pose.position.x;
    hand_posestamped.pose.position.y = msg.pose.pose.position.y;
    hand_posestamped.pose.position.z = msg.pose.pose.position.z;
    hand_posestamped.pose.orientation.x = q.x();
    hand_posestamped.pose.orientation.y = q.y();
    hand_posestamped.pose.orientation.z = q.z();
    hand_posestamped.pose.orientation.w = q.w();
    // cout<<hand_posestamped<<endl; 

    try{
        tf_listener_->waitForTransform("odom","top_camera_link", ros::Time(0), ros::Duration(1.0));
        tf_listener_->transformPose("odom", hand_posestamped, goal_posestamped);
        //載具+gripper(夾緊) 約為 24.5cm
        goal_pose.position.x = goal_posestamped.pose.position.x-0.295; 
        goal_pose.position.y = goal_posestamped.pose.position.y;
        goal_pose.position.z = goal_posestamped.pose.position.z+0.15; //15
        // goal_pose.orientation.x = goal_posestamped.pose.orientation.x;
        // goal_pose.orientation.y = goal_posestamped.pose.orientation.y;
        // goal_pose.orientation.z = goal_posestamped.pose.orientation.z;
        // goal_pose.orientation.w = goal_posestamped.pose.orientation.w;
        // 旋轉卡死
        goal_pose.orientation.x =  0.509;
        goal_pose.orientation.y =  0.515;
        goal_pose.orientation.z =  0.492;
        goal_pose.orientation.w =   0.485;
        
        outrange_judgment();
        if(inrange_flag){
            pub_sendgoal.publish(goal_pose);
            inrange_flag = false;
            // cout<<goal_pose<< endl;
        }

    }
    catch (tf::TransformException ex){ 
      ROS_ERROR("%s",ex.what());
    }
    catch (...) {
        cout<<"something wrong (not tf)"<< endl;
    }


    // cout<<goal_posestamped<<endl;
    // cout<<step++<<endl;
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_handpose_release_gripper_ukf");
    ros::NodeHandle nh;
    // ros::Subscriber handpose_sub = n.subscribe<geometry_msgs::Point>("camera_xyz", 100, send_final_goal_CB);

    // 若此 class 並沒有要輸入參數, 只能下面這種寫法, 後面不行有括號！！！！！！！！！！！！
    Pose_adjust_tf pose_adjust_tf;


    // pose也可以
    // geometry_msgs::PoseStamped target_pose, transformed_pose;
    ros::Rate loop_rate(30);

    // ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    // while (ros::ok()){
    //     ros::spinOnce();
	// 	loop_rate.sleep();
    // }
    // while (ros::ok() && pose_adjust_tf.pub_sendgoal.getNumSubscribers() == 0) {
    //     ros::Rate(100).sleep();
    //     cout<<"wait for sendgoal"<<endl;
    //     loop_rate.sleep();
    // }
    cout<<"before while"<< endl;
    ros::spin();
    return 0;
}