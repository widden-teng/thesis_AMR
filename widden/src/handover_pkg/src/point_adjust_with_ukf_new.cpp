#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Bool.h"
#include <fstream>

#define X_UPLIMIT 150 //100
#define X_DOWNLIMIT -100
#define Y_UPLIMIT 100
#define Y_DOWNLIMIT -100
#define Z_UPLIMIT 100
#define Z_DOWNLIMIT -100


using namespace std;

class Pose_adjust_tf{

    public:
        Pose_adjust_tf();
        ~Pose_adjust_tf();
        // tf2_ros::Buffer tfBuffer;
        // tf2_ros::TransformListener tfListener(tfBuffer);
        void top_camera_cb(const geometry_msgs::PoseArray::ConstPtr& top_hand_psa);
        void side_camera_cb(const geometry_msgs::PoseArray::ConstPtr& side_hand_psa);
        void fuseCallback(const nav_msgs::Odometry msg);
        void record_cb(const std_msgs::Bool::ConstPtr& msg);
        
        void outrange_judgment();
        void top_noupdate_cb(const std_msgs::Bool::ConstPtr& msg);
        void side_noupdate_cb(const std_msgs::Bool::ConstPtr& msg);
        
        // void run();

    private:
        ros::NodeHandle n;
        ros::Subscriber top_handpose_sub, side_handpose_sub,fuse_sub, start_record_sub, top_noupdate_sub, side_noupdate_sub;
        tf::Transform tf_fuse;
        bool start_record = false;
        char outmsg_fuse[200], outmsg_top_camera[200], outmsg_side_camera[200];
        ofstream fuse_pose, top_camera_pose, side_camera_pose;
        double begin_time;
        nav_msgs::Odometry odo_msg_top_camera, odo_msg_side_camera;
        tf::TransformListener* tf_listener_= new tf::TransformListener();
        geometry_msgs::PoseStamped side_posestamped;
        geometry_msgs::PoseStamped side2top_posestamped;

        ros::Publisher pub_final_handpose;
        ros::Publisher pub_ukf_top_camera, pub_ukf_side_camera;

        geometry_msgs::PointStamped hand_pt, armbase_pt;
        tf2_ros::Buffer tfBuffer_;

        bool inrange_flag = false; 
        bool MoveArm_flag = false;// 用來擋pub點到手臂
        Eigen::Matrix3f rotation;
        tf::TransformBroadcaster odom_broadcaster, base_link_broadcaster, side_broadcaster, top_broadcaster;

        // 用來更換發handpose_odom 與 handpose_baselink tf的flag
        bool top_tf_board_flag = false;
        bool side_tf_board_flag = false;
        

};

Pose_adjust_tf::Pose_adjust_tf()
{
    pub_ukf_top_camera =  n.advertise<nav_msgs::Odometry>("handpose_top_camera", 10);
    pub_ukf_side_camera =  n.advertise<nav_msgs::Odometry>("handpose_side_camera", 10);
    fuse_sub = n.subscribe("/odometry/filtered", 1, &Pose_adjust_tf::fuseCallback, this);
    start_record_sub = n.subscribe("start_record_topic", 1, &Pose_adjust_tf::record_cb, this);
    begin_time = ros::Time::now().toSec();
    fuse_pose.open ("/home/tm5/Documents/widden_workspace/ocs_ws/output/ukf/fuse_pose.txt", std::ofstream::out | std::ofstream::trunc);
    top_camera_pose.open  ("/home/tm5/Documents/widden_workspace/ocs_ws/output/ukf/top_camera_pose.txt",  std::ofstream::out | std::ofstream::trunc);
    side_camera_pose.open ("/home/tm5/Documents/widden_workspace/ocs_ws/output/ukf/side_camera_pose.txt", std::ofstream::out | std::ofstream::trunc);

    pub_final_handpose = n.advertise<geometry_msgs::PoseArray>("/MovingPath", 10);
    top_handpose_sub = n.subscribe("/top_camera/handpose_cameralink", 1, &Pose_adjust_tf::top_camera_cb, this);
    side_handpose_sub = n.subscribe("/side_camera/handpose_cameralink", 1, &Pose_adjust_tf::side_camera_cb, this);


    // 用來判斷是否還要根據top 或 side 來發布tf( handpose_base_link&handpose_odom), 可關掉(把side的部份註解掉)
    top_noupdate_sub = n.subscribe("/top_camera/stop_update_target_flag", 1, &Pose_adjust_tf::top_noupdate_cb, this);
    side_noupdate_sub = n.subscribe("/side_camera/stop_update_target_flag", 1, &Pose_adjust_tf::side_noupdate_cb, this);
    
}

Pose_adjust_tf::~Pose_adjust_tf(){

}

void Pose_adjust_tf::top_noupdate_cb(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        top_tf_board_flag = false;
    }
    else{
        top_tf_board_flag = true;
    }
}

void Pose_adjust_tf::side_noupdate_cb(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        side_tf_board_flag = false;
    }
    else{
        side_tf_board_flag = true;
    }
}


void Pose_adjust_tf::outrange_judgment(){
    // 用來限制 handover 的範圍(避免機器手臂超出範圍)
    // 超過的話回傳 true(不會pub到手臂)
    if (MoveArm_flag)
        if( armbase_pt.point.x >  X_DOWNLIMIT && armbase_pt.point.x < X_UPLIMIT \
            && armbase_pt.point.y > Y_DOWNLIMIT && armbase_pt.point.y < Y_UPLIMIT \
            && armbase_pt.point.z > Z_DOWNLIMIT && armbase_pt.point.z < Z_UPLIMIT)
            inrange_flag = true;
    cout<<"In outrange_judgment"<< endl;
}

void Pose_adjust_tf::record_cb(const std_msgs::Bool::ConstPtr& msg){
    start_record = true;
    std::cout << "recording"<<std::endl;
}

void Pose_adjust_tf::fuseCallback(const nav_msgs::Odometry msg){
  tf_fuse.setOrigin(tf::Vector3(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z));
  tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
  q.normalize();
  tf_fuse.setRotation(q);
  static tf::TransformBroadcaster Transform_broadcaster;

  Transform_broadcaster.sendTransform(tf::StampedTransform(tf_fuse, msg.header.stamp, "top_camera_link", "fuse"));
      
  if(start_record) {
    double roll, pitch, yaw;
    
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double fuse_now_time = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9;
    sprintf(outmsg_fuse,"%.7f %.7f %.7f %.7f %.7f %.7f",msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,roll/M_PI*180,pitch/M_PI*180,yaw/M_PI*180);
    fuse_pose << fuse_now_time - begin_time << " " << outmsg_fuse << std::endl;
  }
}

void Pose_adjust_tf::top_camera_cb(const geometry_msgs::PoseArray::ConstPtr& top_hand_psa){
    
    static tf::TransformBroadcaster handpose_broadcaster, handpose_broadcaster_realtime;
    rotation = Eigen::Matrix3f::Identity();
    for(int i = 0; i < 3; i++)
        rotation.col(i) << top_hand_psa->poses[i].position.x, top_hand_psa->poses[i].position.y, top_hand_psa->poses[i].position.z;

    Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0);
    tf::Quaternion q;
    q.setRPY(euler_angles(2), euler_angles(1), euler_angles(0));

    odo_msg_top_camera.header.stamp = ros::Time::now();
    odo_msg_top_camera.header.frame_id = "top_camera_link";
    odo_msg_top_camera.child_frame_id = "handpose_base_link";
    // top_hand_psa->poses[4] 是手掌位置
    odo_msg_top_camera.pose.pose.position.x = top_hand_psa->poses[4].position.x;
    odo_msg_top_camera.pose.pose.position.y = top_hand_psa->poses[4].position.y;
    odo_msg_top_camera.pose.pose.position.z = top_hand_psa->poses[4].position.z;
    odo_msg_top_camera.pose.pose.orientation.w = q.w();
    odo_msg_top_camera.pose.pose.orientation.x = q.x();
    odo_msg_top_camera.pose.pose.orientation.y = q.y();
    odo_msg_top_camera.pose.pose.orientation.z = q.z();
    odo_msg_top_camera.pose.covariance = {1E-3,0, 0, 0, 0, 0,
                            0, 1E-3, 0, 0, 0, 0,
                            0, 0, 1E-3, 0, 0, 0,
                            0, 0, 0, 1E-1, 0, 0,
                            0, 0, 0, 0, 1E-1, 0,
                            0, 0, 0, 0, 0, 1E-1};
    // 在下面pub

    
    if(top_tf_board_flag){

        pub_ukf_top_camera.publish(odo_msg_top_camera);
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(top_hand_psa->poses[4].position.x, top_hand_psa->poses[4].position.y, top_hand_psa->poses[4].position.z) );
        transform.setRotation( tf::Quaternion(q.x(), q.y(), q.z(), q.w()) );
        odom_broadcaster.sendTransform(tf::StampedTransform(transform, odo_msg_top_camera.header.stamp, "top_camera_link", "handpose_odom"));
        base_link_broadcaster.sendTransform(tf::StampedTransform(transform, odo_msg_top_camera.header.stamp, "top_camera_link", "handpose_base_link"));
        top_broadcaster.sendTransform(tf::StampedTransform(transform, odo_msg_top_camera.header.stamp, "top_camera_link", "top_handpose"));
        // std::cout << "pubing"<<std::endl;
    }


    if(start_record) {
        double now_time = top_hand_psa->header.stamp.sec + top_hand_psa->header.stamp.nsec*1e-9;
        sprintf(outmsg_top_camera,"%.7f %.7f %.7f %.7f %.7f %.7f",top_hand_psa->poses[4].position.x,top_hand_psa->poses[4].position.y,top_hand_psa->poses[4].position.z,euler_angles(2)/M_PI*180,euler_angles(1)/M_PI*180,euler_angles(0)/M_PI*180);
        top_camera_pose << now_time - begin_time << " " << outmsg_top_camera << std::endl;
    }
    
}

void Pose_adjust_tf::side_camera_cb(const geometry_msgs::PoseArray::ConstPtr& side_hand_psa){
    

    static tf::TransformBroadcaster handpose_broadcaster, handpose_broadcaster_realtime;
    rotation = Eigen::Matrix3f::Identity();
    for(int i = 0; i < 3; i++)
        rotation.col(i) << side_hand_psa->poses[i].position.x, side_hand_psa->poses[i].position.y, side_hand_psa->poses[i].position.z;

    Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0);
    tf::Quaternion q;
    q.setRPY(euler_angles(2), euler_angles(1), euler_angles(0));

    tf::Transform side_tf;
    side_tf.setOrigin(tf::Vector3(side_hand_psa->poses[4].position.x, side_hand_psa->poses[4].position.y, side_hand_psa->poses[4].position.z) );
    side_tf.setRotation( tf::Quaternion(q.x(), q.y(), q.z(), q.w()) );
    side_broadcaster.sendTransform(tf::StampedTransform(side_tf, ros::Time::now(), "side_camera_link", "side_handpose"));


    side_posestamped.header.frame_id = "side_camera_link";
    // side_hand_psa->poses[4] 是手掌位置
    side_posestamped.pose.position.x = side_hand_psa->poses[4].position.x;
    side_posestamped.pose.position.y = side_hand_psa->poses[4].position.y;
    side_posestamped.pose.position.z = side_hand_psa->poses[4].position.z;
    side_posestamped.pose.orientation.x = q.x();
    side_posestamped.pose.orientation.y = q.y();
    side_posestamped.pose.orientation.z = q.z();
    side_posestamped.pose.orientation.w = q.w();



    try{
        tf_listener_->waitForTransform("top_camera_link","side_camera_link", ros::Time(0), ros::Duration(1.0));
        tf_listener_->transformPose("top_camera_link", side_posestamped, side2top_posestamped);
        odo_msg_side_camera.header.stamp = ros::Time::now();
        odo_msg_side_camera.header.frame_id = "top_camera_link";
        odo_msg_side_camera.child_frame_id = "handpose_base_link";
        odo_msg_side_camera.pose.pose.position.x = side2top_posestamped.pose.position.x;
        odo_msg_side_camera.pose.pose.position.y = side2top_posestamped.pose.position.y;
        odo_msg_side_camera.pose.pose.position.z = side2top_posestamped.pose.position.z;
        odo_msg_side_camera.pose.pose.orientation.w = side2top_posestamped.pose.orientation.w;
        odo_msg_side_camera.pose.pose.orientation.x = side2top_posestamped.pose.orientation.x;
        odo_msg_side_camera.pose.pose.orientation.y = side2top_posestamped.pose.orientation.y;
        odo_msg_side_camera.pose.pose.orientation.z = side2top_posestamped.pose.orientation.z;
        odo_msg_side_camera.pose.covariance = {1E-3,0, 0, 0, 0, 0,
                                0, 1E-3, 0, 0, 0, 0,
                                0, 0, 1E-3, 0, 0, 0,
                                0, 0, 0, 1E-1, 0, 0,
                                0, 0, 0, 0, 1E-1, 0,
                                0, 0, 0, 0, 0, 1E-1};

        // 在下面pub

        // // pub side_camera tf base on top_camera_link
        // tf::Transform side2top_tf;
        // side2top_tf.setOrigin(tf::Vector3(odo_msg_side_camera.pose.pose.position.x, odo_msg_side_camera.pose.pose.position.y, odo_msg_side_camera.pose.pose.position.z) );
        // side2top_tf.setRotation( tf::Quaternion(odo_msg_side_camera.pose.pose.orientation.x, odo_msg_side_camera.pose.pose.orientation.y, odo_msg_side_camera.pose.pose.orientation.z, odo_msg_side_camera.pose.pose.orientation.w) );
        // side_broadcaster.sendTransform(tf::StampedTransform(side2top_tf, odo_msg_side_camera.header.stamp, "top_camera_link", "side2top_handpose"));


        ///----------------------------------------///
        ///  這邊擇一開啟, 若開上面(if), 只會有top或side的CB會 board tf, 可能造成發布頻率不足(目前沒有只有一開始出現一次)
        ///  用來防止" Transform from handpose_odom to handpose_base_link was unavailable for the time requested. Using latest instead. "

        if((!top_tf_board_flag) && side_tf_board_flag){
            pub_ukf_side_camera.publish(odo_msg_side_camera);
            tf::Transform side_tf;
            side_tf.setOrigin(tf::Vector3(odo_msg_side_camera.pose.pose.position.x, odo_msg_side_camera.pose.pose.position.y, odo_msg_side_camera.pose.pose.position.z) );
            side_tf.setRotation( tf::Quaternion(odo_msg_side_camera.pose.pose.orientation.x, odo_msg_side_camera.pose.pose.orientation.y, odo_msg_side_camera.pose.pose.orientation.z, odo_msg_side_camera.pose.pose.orientation.w) );
            odom_broadcaster.sendTransform(tf::StampedTransform(side_tf, odo_msg_side_camera.header.stamp, "top_camera_link", "handpose_odom"));
            base_link_broadcaster.sendTransform(tf::StampedTransform(side_tf, odo_msg_side_camera.header.stamp, "top_camera_link", "handpose_base_link"));
            std::cout << "only side camera"<<std::endl;
        }

        // pub_ukf_side_camera.publish(odo_msg_side_camera);
        // tf::Transform side_tf;
        // side_tf.setOrigin(tf::Vector3(odo_msg_side_camera.pose.pose.position.x, odo_msg_side_camera.pose.pose.position.y, odo_msg_side_camera.pose.pose.position.z) );
        // side_tf.setRotation( tf::Quaternion(odo_msg_side_camera.pose.pose.orientation.x, odo_msg_side_camera.pose.pose.orientation.y, odo_msg_side_camera.pose.pose.orientation.z, odo_msg_side_camera.pose.pose.orientation.w) );
        // odom_broadcaster.sendTransform(tf::StampedTransform(side_tf, odo_msg_side_camera.header.stamp, "top_camera_link", "handpose_odom"));
        // base_link_broadcaster.sendTransform(tf::StampedTransform(side_tf, odo_msg_side_camera.header.stamp, "top_camera_link", "handpose_base_link"));
        
        ///----------------------------------------///

        tf::Quaternion tf_quaternion;
        tf::quaternionMsgToTF(odo_msg_side_camera.pose.pose.orientation, tf_quaternion);
        double roll_side, pitch_side, yaw_side;
        tf::Matrix3x3(tf_quaternion).getRPY(roll_side, pitch_side, yaw_side);
        if(start_record) {
            double side_now_time = odo_msg_side_camera.header.stamp.sec + odo_msg_side_camera.header.stamp.nsec*1e-9;
            sprintf(outmsg_side_camera,"%.7f %.7f %.7f %.7f %.7f %.7f",odo_msg_side_camera.pose.pose.position.x ,odo_msg_side_camera.pose.pose.position.y ,odo_msg_side_camera.pose.pose.position.z ,roll_side/M_PI*180,pitch_side/M_PI*180,yaw_side/M_PI*180);
            side_camera_pose << side_now_time - begin_time << " " << outmsg_side_camera << std::endl;
        }
    }
    catch (...) {
        cout<<"something wrong (not tf)"<< endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_adjust_with_ukf_new");
    ros::NodeHandle nh;

    Pose_adjust_tf pose_adjust_tf;
    // ros::Rate loop_rate(10);
    // cout<<"before while"<< endl;
    // ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    while (ros::ok()){
        ros::spinOnce();
		// loop_rate.sleep();
    }

    return 0;
}