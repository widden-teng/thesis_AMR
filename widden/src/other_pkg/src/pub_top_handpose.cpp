#include <ros/ros.h>
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

#define X_UPLIMIT 0.8
#define X_DOWNLIMIT 0.0
#define Y_UPLIMIT 0.2
#define Y_DOWNLIMIT -0.6
#define Z_UPLIMIT 1.6
#define Z_DOWNLIMIT 1.0


using namespace std;

class Pose_adjust_tf{

    public:
        Pose_adjust_tf();
        ~Pose_adjust_tf();
        // tf2_ros::Buffer tfBuffer;
        // tf2_ros::TransformListener tfListener(tfBuffer);
        void tf_CamtoArmbase(const geometry_msgs::PoseArray::ConstPtr& hand_pta);
        void outrange_judgment();
        // void run();
        ros::Publisher pub_sendgoal;

    private:
        ros::NodeHandle n;
        ros::Subscriber handpose_sub;
        // ros::Publisher pub_sendgoal;


        geometry_msgs::PointStamped hand_pt, armbase_pt;
  

        geometry_msgs::PoseArray vel_static_path;


        bool inrange_flag = false; 
        bool MoveArm_flag = true;// 用來擋pub點到手臂
        int step = 0;

        geometry_msgs::PoseStamped hand_posestamped;
        geometry_msgs::PoseStamped goal_posestamped;
        geometry_msgs::Pose goal_pose;

        int pub_time = 0;
        int handpose_flag = 0;

        tf::TransformListener* tf_listener_= new tf::TransformListener();
        // tf_listener_ = new tf::TransformListener();
        tf::TransformBroadcaster handpose_broadcaster, handpose_broadcaster_realtime;
        tf::Vector3  hand_trans, hand_trans_realtime;    
        tf::Transform T_handpose, T_handpose_realtime;

};

Pose_adjust_tf::Pose_adjust_tf()
{
    handpose_sub = n.subscribe("handpose_cameralink", 1, &Pose_adjust_tf::tf_CamtoArmbase, this);
    pub_sendgoal = n.advertise<geometry_msgs::Pose>("/sendGoal", 1);
}

Pose_adjust_tf::~Pose_adjust_tf(){

}

void Pose_adjust_tf::outrange_judgment(){
    // 用來限制 handover 的範圍(避免機器手臂超出範圍)
    // 超過的話回傳 true(不會pub到手臂)
    if (MoveArm_flag)
        if( goal_pose.position.x >  X_DOWNLIMIT && goal_pose.position.x < X_UPLIMIT \
            && goal_pose.position.y > Y_DOWNLIMIT && goal_pose.position.y < Y_UPLIMIT \
            && goal_pose.position.z > Z_DOWNLIMIT && goal_pose.position.z < Z_UPLIMIT){

            inrange_flag = true;
            cout<<"in range"<< endl;

        }       
        else
            cout<<"Out of range"<< endl;
}


void Pose_adjust_tf::tf_CamtoArmbase(const geometry_msgs::PoseArray::ConstPtr& hand_psa){
    


    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

    for(int i = 0; i < 3; i++)
        rotation.col(i) << hand_psa->poses[i].position.x, hand_psa->poses[i].position.y, hand_psa->poses[i].position.z;

    Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0);
    tf::Quaternion q;
    q.setRPY(euler_angles(2), euler_angles(1), euler_angles(0));
    

    if (handpose_flag ==0){
        // // ros::Rate rate(10);
        // static tf::TransformBroadcaster handpose_broadcaster;
        // tf::Vector3  hand_trans(hand_psa->poses[3].position.x,hand_psa->poses[3].position.y,hand_psa->poses[3].position.z);    
        // tf::Transform T_handpose;
        // T_handpose.setOrigin(hand_trans);
        // T_handpose.setRotation(q);
        // handpose_broadcaster.sendTransform(tf::StampedTransform(T_handpose, ros::Time::now(), "camera_link", "handpose"));
        // cout<<"pub_tf"<< endl;
        // handpose_flag++;
        hand_trans.setX(hand_psa->poses[3].position.x);
        hand_trans.setY(hand_psa->poses[3].position.y);
        hand_trans.setZ(hand_psa->poses[3].position.z);
        
        T_handpose.setOrigin(hand_trans);
        T_handpose.setRotation(q);
        handpose_flag++;
    }
    hand_trans_realtime.setX(hand_psa->poses[3].position.x);
    hand_trans_realtime.setY(hand_psa->poses[3].position.y);
    hand_trans_realtime.setZ(hand_psa->poses[3].position.z);
    
    T_handpose_realtime.setOrigin(hand_trans_realtime);
        T_handpose_realtime.setRotation(q);
    handpose_broadcaster.sendTransform(tf::StampedTransform(T_handpose, ros::Time::now(), "camera_link", "handpose_init"));
    handpose_broadcaster_realtime.sendTransform(tf::StampedTransform(T_handpose_realtime, ros::Time::now(), "camera_link", "handpose_realtime"));
    cout<<"pub_tf"<< endl;

    
    
    // hand_posestamped.header.frame_id = "camera_link";
    // hand_posestamped.pose.position.x = hand_psa->poses[3].position.x;
    // hand_posestamped.pose.position.y = hand_psa->poses[3].position.y;
    // hand_posestamped.pose.position.z = hand_psa->poses[3].position.z;
    // hand_posestamped.pose.orientation.x = q.x();
    // hand_posestamped.pose.orientation.y = q.y();
    // hand_posestamped.pose.orientation.z = q.z();
    // hand_posestamped.pose.orientation.w = q.w();
    // cout<<hand_posestamped<<endl;
    


    // try{
        // tfBuffer.canTransform("odom", "camera_link", ros::Time(0), ros::Duration(0.3)); //0.5
        // transformStamped = tfBuffer.lookupTransform("odom", "camera_link",ros::Time(0), ros::Duration(0.3));    //0.5
                    
    //     tf2::doTransform(hand_posestamped, goal_posestamped, transformStamped);


    //     goal_pose.position.x = goal_posestamped.pose.position.x;
    //     goal_pose.position.y = goal_posestamped.pose.position.y;
    //     goal_pose.position.z = goal_posestamped.pose.position.z;
    //     goal_pose.orientation.x = goal_posestamped.pose.orientation.x;
    //     goal_pose.orientation.y = goal_posestamped.pose.orientation.y;
    //     goal_pose.orientation.z = goal_posestamped.pose.orientation.z;
    //     goal_pose.orientation.w = goal_posestamped.pose.orientation.w;

    //     // cout<<"------------hand_psa----------------"<<endl;
    //     // cout<<*hand_psa<<endl;
    //     // cout<<"------------goal_pose----------------"<<endl;
        
    //     // cout<<goal_pose<<endl;

    //     outrange_judgment();
    //     if(inrange_flag){
    //         pub_sendgoal.publish(goal_pose);
    //         inrange_flag = false;
    //     }
    // }
    // catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s",ex.what());
    // }
    // //吃到剩餘的錯(因為放在catch tf2的後面)
    // catch (...) {
    //     cout<<"something wrong (not tf2)"<< endl;
    // }

    

    // try{
    //     tf_listener_->waitForTransform("odom","camera_link", ros::Time(0), ros::Duration(1.0));
    //     tf_listener_->transformPose("odom", hand_posestamped, goal_posestamped);
    //     goal_pose.position.x = goal_posestamped.pose.position.x;
    //     goal_pose.position.y = goal_posestamped.pose.position.y;
    //     goal_pose.position.z = goal_posestamped.pose.position.z;
    //     goal_pose.orientation.x = goal_posestamped.pose.orientation.x;
    //     goal_pose.orientation.y = goal_posestamped.pose.orientation.y;
    //     goal_pose.orientation.z = goal_posestamped.pose.orientation.z;
    //     goal_pose.orientation.w = goal_posestamped.pose.orientation.w;
        
    //     outrange_judgment();
    //     if(inrange_flag){
    //         pub_sendgoal.publish(goal_pose);
    //         inrange_flag = false;
    //     }

    // }
    // catch (tf::TransformException ex){ 
    //   ROS_ERROR("%s",ex.what());
    // }
    // catch (...) {
    //     cout<<"something wrong (not tf)"<< endl;
    // }



    // cout<<goal_posestamped<<endl;
    // cout<<step++<<endl;
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_handpose");
    ros::NodeHandle nh;
    // ros::Subscriber handpose_sub = n.subscribe<geometry_msgs::Point>("camera_xyz", 100, tf_CamtoArmbase);

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