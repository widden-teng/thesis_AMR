#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <unistd.h>

#define X_UPLIMIT 100
#define X_DOWNLIMIT -100
#define Y_UPLIMIT 100
#define Y_DOWNLIMIT -100
#define Z_UPLIMIT 100
#define Z_DOWNLIMIT -100


// using namespace std;

// class pose_adjust_tf{

//     public:
//         pose_adjust_tf();
//         ~pose_adjust_tf();
//         // tf2_ros::Buffer tfBuffer;
//         // tf2_ros::TransformListener tfListener(tfBuffer);
//         void tf_CamtoArmbase(const geometry_msgs::Point::ConstPtr& hand_pt);
//         void outrange_judgment();
        

//     private:
//         ros::NodeHandle n;
//         ros::Subscriber handpose_sub;
//         ros::Publisher pub_final_handpose;
//         geometry_msgs::PointStamped hand_pt, armbase_pt;
//         tf2_ros::Buffer tfBuffer_;

//         bool inrange_flag = false; 
//         bool MoveArm_flag = false;// 用來擋pub點到手臂

// };

// pose_adjust_tf::pose_adjust_tf()
// {
//     handpose_sub = n.subscribe("camera_xyz", 10, &pose_adjust_tf::tf_CamtoArmbase, this);
//     pub_final_handpose = n.advertise<geometry_msgs::PoseArray>("/MovingPath", 10);
// }

// pose_adjust_tf::~pose_adjust_tf(){

// }

// void pose_adjust_tf::outrange_judgment(){
//     // 用來限制 handover 的範圍(避免機器手臂超出範圍)
//     // 超過的話回傳 true(不會pub到手臂)
//     if (MoveArm_flag)
//         if( armbase_pt.point.x >  X_DOWNLIMIT && armbase_pt.point.x < X_UPLIMIT \
//             && armbase_pt.point.y > Y_DOWNLIMIT && armbase_pt.point.y < Y_UPLIMIT \
//             && armbase_pt.point.z > Z_DOWNLIMIT && armbase_pt.point.z < Z_UPLIMIT)
//             inrange_flag = true;
// }

// void pose_adjust_tf::tf_CamtoArmbase(const geometry_msgs::Point::ConstPtr& cam_point){
    
//     tf2_ros::Buffer tfBuffer;
//     tf2_ros::TransformListener tfListener(tfBuffer);
//     geometry_msgs::TransformStamped transformStamped;

  
//     hand_pt.point.x = cam_point->x;
//     hand_pt.point.y = cam_point->y;
//     hand_pt.point.z = cam_point->z;

//     try{
//         transformStamped = tfBuffer.lookupTransform("base_link", "camera_link",ros::Time(0), ros::Duration(3.0));
//         tf2::doTransform(hand_pt, armbase_pt, transformStamped);
//         pose_adjust_tf::outrange_judgment();
//         if (inrange_flag){
//             pub_final_handpose.publish(armbase_pt);
//             inrange_flag = false;
//         }
        
//     }
//     catch (tf2::TransformException &ex) {
//         ROS_WARN("%s",ex.what());
//     }
//     //吃到剩餘的錯(因為放在catch tf2的後面)
//     catch (...) {
//         cout<<"something wrong (not tf2)"<< endl;
//     }
    

// }


// // void tf_CamtoArmbase(const geometry_msgs::Point hand_pt){
    
// //     tf2_ros::Buffer tfBuffer;
// //     tf2_ros::TransformListener tfListener(tfBuffer);
// //     geometry_msgs::TransformStamped transformStamped;

// //     geometry_msgs::PointStamped target_pt, armbase_pt;
// //     target_pt.point.x = 0;
// //     target_pt.point.y = 3;
// //     target_pt.point.z = 5;

// //     try{
// //         transformStamped = tfBuffer.lookupTransform("base_link", "camera_link",ros::Time(0), ros::Duration(3.0));
// //         tf2::doTransform(target_pt, armbase_pt, transformStamped);
// //         // pub_final_handpose.publish(armbase_pt);

// //     }
// //     catch (tf2::TransformException &ex) {
// //         ROS_WARN("%s",ex.what());
// //         ros::Duration(1.0).sleep();
// //     }
    

// // }

//x+20, z+15
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_adjust");
    ros::NodeHandle n;
    ros::Publisher pub_test = n.advertise<geometry_msgs::PoseArray>("/MovingPath", 1000);

    geometry_msgs::PoseArray static_path;
    geometry_msgs::Pose test_point;
    test_point.position.x = 0.567394;  //0.6035    //0.6035   //0.5392    //0.7392
    test_point.position.y = -0.137148; //-0.0244  //-0.0244    //-0.0348518    //-0.0348518
    test_point.position.z = 0.266213;  //0.2335   //0.4335  //0.361063  //0.361063
    static_path.poses.push_back(test_point);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()){
        pub_test.publish(static_path);     
        if(count == 5)
            break;
        count++;   
        loop_rate.sleep();
    }

//     // // 要先宣告,且待轉換的點須先定義, 才能dotransform
//     // geometry_msgs::PointStamped target_pt, transformed_pt;
//     // // target_pt.header.frame_id = "test2";
//     // target_pt.point.x = 0;
//     // target_pt.point.y = 3;
//     // target_pt.point.z = 5;

//     // pose也可以
//     // geometry_msgs::PoseStamped target_pose, transformed_pose;

//     // while (ros::ok()){
//     //     geometry_msgs::TransformStamped transformStamped;
//     //     try{
//     //         //吃特定frame的tf(此為test1 到 test2)
//     //         transformStamped = tfBuffer.lookupTransform("test1", "test2", ros::Time(0), ros::Duration(3.0));
//     //         //直接轉換, 將target_pt 經tf轉成transformed_pt
//     //         tf2::doTransform(target_pt, transformed_pt, transformStamped);
//     //         std::cout <<  transformed_pt << std::endl;
//     //     }
//     //     catch (tf2::TransformException &ex) {
//     //         ROS_WARN("%s",ex.what());
//     //         ros::Duration(1.0).sleep();
//     //         continue;
//     //     }
//     //     // rate.sleep();
//     // }

    return 0;
}