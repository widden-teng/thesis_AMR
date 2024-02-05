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
#include <geometry_msgs/TransformStamped.h>
using namespace std;

class Tag_Pose{
public:
	ros::NodeHandle nh;
	ros::Subscriber sub_tag;
	ros::Subscriber sub_qr;
	ros::Subscriber sub_target;
	ros::Subscriber sub_drillup;
	ros::Subscriber sub_drillup2;
	ros::Subscriber sub_drilldown;
	ros::Subscriber sub_drilldown2;

	ros::Publisher pub_hole;
	ros::Publisher pub_target;
	ros::Publisher pub_traj1;
	ros::Publisher pub_traj2;

	tf::Transform T_robot_to_tag0;
	tf::Transform T_tag9_to_hole;
	tf::Transform T_tag1_to_ARbase;
	tf::Transform T_tag19_to_pointer;
	tf::Transform T_qr_to_cad;
	tf::Transform T_cad_to_ARbase;

	tf::Transform T_cam_to_tag0;
	tf::Transform T_cam_to_tag1;
	tf::Transform T_cam_to_tag9;
	tf::Transform T_cam_to_tag19;
	tf::Transform T_cam_to_qr;

	tf::Transform T_robot_to_hole;

	tf::Transform T_ARbase_to_target;
	tf::Transform T_cad_to_target;
	double tm_target_p[6];
	double tm_target_t[6];


	tf::Transform T_target;
	tf::Transform T_robot_traj;

	tf::TransformListener listener;

	Tag_Pose(){
		// puretag0
		tf::Quaternion q_robot2tag0(-0.0076459, -0.0059163, 0.70856, 0.70558);
		// tf::Vector3 v_robot2tag0(0.50451, 0.16056, 0.0051718);
		// tf::Vector3 v_robot2tag0(0.50451, 0.15606, 0.0001718);

		tf::Vector3 v_robot2tag0(0.50501, 0.16276, 0.0001718);

		// tf::Quaternion q_robot2tag0(-0.0076459, -0.0059163, 0.70856, 0.70558);
		// tf::Vector3 v_robot2tag0(0.50801, 0.15606, 0.0001718);
		//for hole
		// tf::Vector3 v_robot2tag0(0.5144511108398437, 0.16416395263671874, 0.001284);
		T_robot_to_tag0.setOrigin(v_robot2tag0);
		T_robot_to_tag0.setRotation(q_robot2tag0);

		tf::Quaternion q_tag9tohole;
		q_tag9tohole.setRPY(-M_PI/2,0,0);
		tf::Vector3 v_tag9tohole(0,0.075,-0.044);
		// q_tag9tohole.setRPY(0.0,M_PI/2,0);
		// tf::Vector3 v_tag9tohole(0.078,0.0,-0.043);
		T_tag9_to_hole.setOrigin(v_tag9tohole);
		T_tag9_to_hole.setRotation(q_tag9tohole);

		tf::Quaternion q_tag19_to_pointer;
		q_tag19_to_pointer.setRPY(M_PI/2,0,0);
		tf::Vector3 v_tag19_to_pointer(0,-0.111,-0.053);
		T_tag19_to_pointer.setOrigin(v_tag19_to_pointer);
		T_tag19_to_pointer.setRotation(q_tag19_to_pointer);

		tf::Quaternion q_tag1_to_ARbase;
		q_tag1_to_ARbase.setRPY(0,0,M_PI/2);
		tf::Vector3 v_tag1_to_ARbase(0.2416,-0.03,0.0);
		T_tag1_to_ARbase.setOrigin(v_tag1_to_ARbase);
		T_tag1_to_ARbase.setRotation(q_tag1_to_ARbase);

		tf::Quaternion q_qr_to_cad;
		// match
		// q_qr_to_cad.setRPY(1.6591 *M_PI/180,  -0.4119*M_PI/180,  -89.1530*M_PI/180);
		// tf::Vector3 v_qr_to_cad(-0.0889,0.0059,-0.0604);
		
		//AR
		// q_qr_to_cad.setRPY(-0.2591 *M_PI/180,  -0.4119*M_PI/180,  -89.1530*M_PI/180);
		// tf::Vector3 v_qr_to_cad(-0.092,0.0069,-0.0654);
		// q_qr_to_cad.setRPY(-0.0319 *M_PI/180,  1.5760*M_PI/180,  -90.1128*M_PI/180);
		// q_qr_to_cad.setRPY(-0.0319 *M_PI/180,  1.5760*M_PI/180,  -90.1128*M_PI/180);
		// tf::Vector3 v_qr_to_cad(-0.093,0.0029,-0.0725);
		//q_qr_to_cad.setRPY(-2.6933 *M_PI/180,  -1.2449*M_PI/180,  -90.012*M_PI/180);
		//tf::Vector3 v_qr_to_cad(-0.0962,0.0024,-0.0678);
  //   	q_qr_to_cad.setRPY(-0.236 *M_PI/180,  -0.5987*M_PI/180,  -89.2079*M_PI/180);
		// tf::Vector3 v_qr_to_cad(-0.1684,-0.0403,-0.0712);
		// q_qr_to_cad.setRPY(-0.3982 *M_PI/180,  -0.6635*M_PI/180,  -88.0806*M_PI/180);
		// tf::Vector3 v_qr_to_cad(-0.1669,-0.0456,-0.0716);

		q_qr_to_cad.setRPY(-4.599 *M_PI/180,  -0.4962*M_PI/180,  -88.0868*M_PI/180);
		tf::Vector3 v_qr_to_cad(-0.1725,-0.0454,-0.0568);
		T_qr_to_cad.setOrigin(v_qr_to_cad);
		T_qr_to_cad.setRotation(q_qr_to_cad);

		pub_hole=nh.advertise<geometry_msgs::Transform> ("/robot_to_hole", 1);
		pub_target=nh.advertise<geometry_msgs::Transform> ("/robot_target", 1);
		pub_traj1=nh.advertise<geometry_msgs::PoseStamped> ("/traj1", 1);
		pub_traj2=nh.advertise<geometry_msgs::PoseStamped> ("/traj2", 1);

		sub_tag=nh.subscribe("/tag_detections", 10, &Tag_Pose::tag_Callback,this);
		sub_qr=nh.subscribe("/qr_pose", 10, &Tag_Pose::qr_Callback,this);
		sub_target=nh.subscribe("/targetA", 10, &Tag_Pose::target_Callback_qr_ar,this);
		sub_drillup=nh.subscribe("/drillup", 10, &Tag_Pose::drillup_Callback,this);
		sub_drillup2=nh.subscribe("/drillup2", 10, &Tag_Pose::drillup2_Callback,this);
		sub_drilldown=nh.subscribe("/drilldown", 10, &Tag_Pose::drilldown_Callback,this);
		sub_drilldown2=nh.subscribe("/drilldown2", 10, &Tag_Pose::drilldown2_Callback,this);

	}
		
	void loop_confirm(geometry_msgs::Transform& msg_robottarget){
		// tf::Vector3 lv( msg_robottarget->pose.position.x,msg_robottarget->pose.position.z,msg_robottarget->pose.position.y);
	    
		tf::TransformListener listener;

		tf::Quaternion q_target2tool(0.0, 1.0 ,0.0, 0.0);
		// tf::Vector3 v_target2tool(0.0025, -0.0008, 0.2661);
		tf::Vector3 v_target2tool(0.00, 0.0, 0.266);
		// tf::Vector3 v_target2tool(0.000, 0.00, 0.0895);
		tf::Transform T_target2tool(q_target2tool,v_target2tool);

		ros::Rate rate(10.0);
		while (ros::ok()){
       		geometry_msgs::PoseStamped transform_pub;
       		tf::StampedTransform transform;
       		try{
       			ros::Time now = ros::Time::now();
    			listener.waitForTransform("/base", "/tool0",
                              		now, ros::Duration(3.0));
        		listener.lookupTransform("/base", "/tool0",  
                                  ros::Time(0), transform);
       		}
       		catch (tf::TransformException ex){
        		ROS_ERROR("%s",ex.what());
        		ros::Duration(1.0).sleep();
        		continue;
       		}

			//T_robot_traj = transform*T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_qr*T_qr_to_cad*T_cad_to_target*T_target2tool;
			T_robot_traj = T_target2tool*transform.inverse()*T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_qr*T_qr_to_cad;
			//T_robot_traj = T_robot_traj.inverse();

			//right to left
			tf::Quaternion q2(-T_robot_traj.getRotation().x(),-T_robot_traj.getRotation().y(),-T_robot_traj.getRotation().z(),T_robot_traj.getRotation().w());
			tf::Vector3 vec2(T_robot_traj.getOrigin().x(),T_robot_traj.getOrigin().y(),T_robot_traj.getOrigin().z());
			T_robot_traj.setOrigin(vec2);
			T_robot_traj.setRotation(q2);

			tm_target_t[0] = T_robot_traj.getOrigin().x();
	    	tm_target_t[1] = T_robot_traj.getOrigin().y();
	    	tm_target_t[2] = T_robot_traj.getOrigin().z();

	    	tf::Quaternion q(
	        	T_robot_traj.getRotation().x(),
	        	T_robot_traj.getRotation().y(),
	        	T_robot_traj.getRotation().z(),
	        	T_robot_traj.getRotation().w());
	    	tf::Matrix3x3 m(q);
	    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	    	m.getRPY(tm_target_t[3], tm_target_t[4], tm_target_t[5]);

       		ROS_INFO("T_robot_traj = %f %f %f %f %f %f\n",tm_target_t[0],tm_target_t[1],tm_target_t[2],tm_target_t[3]*180/M_PI,tm_target_t[4]*180/M_PI,tm_target_t[5]*180/M_PI);

	       	transform_pub.pose.position.x = T_robot_traj.getOrigin().x();
	       	transform_pub.pose.position.y = T_robot_traj.getOrigin().y();
	       	transform_pub.pose.position.z = T_robot_traj.getOrigin().z();
	       	transform_pub.pose.orientation.x = -T_robot_traj.getRotation().x();
	       	transform_pub.pose.orientation.y = -T_robot_traj.getRotation().y();
	       	transform_pub.pose.orientation.z = -T_robot_traj.getRotation().z();
	       	transform_pub.pose.orientation.w = -T_robot_traj.getRotation().w();

	       	// pub_traj1.publish(transform_pub);
	       	// pub_traj2.publish(transform_pub);

	   		float x = pow(msg_robottarget.translation.x - transform.getOrigin().x(), 2);
	   		//ROS_WARN("%f", x);
	   		float y = pow(msg_robottarget.translation.y - transform.getOrigin().y(), 2);
	   		float z = pow(msg_robottarget.translation.z - transform.getOrigin().z(), 2);
	   		float rx = pow(msg_robottarget.rotation.x - transform.getRotation().x(), 2);
	   		float ry = pow(msg_robottarget.rotation.y - transform.getRotation().y(), 2);
	   		float rz = pow(msg_robottarget.rotation.z - transform.getRotation().z(), 2);
	   		float rw = pow(msg_robottarget.rotation.w - transform.getRotation().w(), 2);

	   		float num_pos = x+y+z;
	   		float num_ori = rx+ry+rz+rw;
	   		rate.sleep();
	   		if(num_pos > 1e-6){
	   			continue;
	   		}
	   		if(num_ori < 1e-6){
	   			ROS_INFO("Target Reach!");
	   			break;
	   		}
	   		else{
	   			continue;
	   		}
	   
	       
	    }
	}

	void tag_Callback(const apriltags_ros::AprilTagDetectionArray::ConstPtr &msg){
		// ROS_ERROR("in callback\n");
		for(int i=0;i<msg->detections.size();i++){
			tf::Quaternion q(msg->detections[i].pose.pose.orientation.x,msg->detections[i].pose.pose.orientation.y,msg->detections[i].pose.pose.orientation.z,msg->detections[i].pose.pose.orientation.w);
			tf::Vector3 vec(msg->detections[i].pose.pose.position.x,msg->detections[i].pose.pose.position.y,msg->detections[i].pose.pose.position.z);
			if(msg->detections[i].id==0){ // robot base tag
				T_cam_to_tag0.setOrigin(vec);
				T_cam_to_tag0.setRotation(q);
			}
			else if(msg->detections[i].id==1){ // AR & bone model tag 
				T_cam_to_tag1.setOrigin(vec);
				T_cam_to_tag1.setRotation(q);
			}
			else if(msg->detections[i].id==9){ // poke th hole
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
				hole_target_set();

			}
			else if(msg->detections[i].id==19){ // spatial pointer
				T_cam_to_tag19.setOrigin(vec);
				T_cam_to_tag19.setRotation(q);
			}
		}
	}

	void qr_Callback(const geometry_msgs::TransformStamped::ConstPtr& qrPose)
	{
		tf::Quaternion q(qrPose->transform.rotation.x,qrPose->transform.rotation.y,qrPose->transform.rotation.z,qrPose->transform.rotation.w);
		tf::Vector3 vec(qrPose->transform.translation.x,qrPose->transform.translation.y,qrPose->transform.translation.z);
		T_cam_to_qr.setOrigin(vec);
		T_cam_to_qr.setRotation(q);

		// T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_qr*T_qr_to_cad;
		// geometry_msgs::Transform msg_robottarget;
		// msg_robottarget.translation.x=T_target.getOrigin().x();
		// msg_robottarget.translation.y=T_target.getOrigin().y();
		// msg_robottarget.translation.z=T_target.getOrigin().z();
		// msg_robottarget.rotation.x=T_target.getRotation().x();
		// msg_robottarget.rotation.y=T_target.getRotation().y();
		// msg_robottarget.rotation.z=T_target.getRotation().z();
		// msg_robottarget.rotation.w=T_target.getRotation().w();
		//pub_target.publish(msg_robottarget);
	}

	void target_Callback_qr(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
	{
		//unity output x=-x ry=-ry rz=-rz
		tf::Vector3 v( TargetPose->pose.position.x,TargetPose->pose.position.y,TargetPose->pose.position.z);
	    tm_target_p[0] = TargetPose->pose.position.x;
	    tm_target_p[1] = TargetPose->pose.position.y;
	    tm_target_p[2] = TargetPose->pose.position.z;

	    tf::Quaternion q(
	        TargetPose->pose.orientation.x,
	        TargetPose->pose.orientation.y,
	        TargetPose->pose.orientation.z,
	        TargetPose->pose.orientation.w);
	    tf::Matrix3x3 m(q);
	    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	    m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

	    ROS_INFO("targetA = %f %f %f %f %f %f\n",tm_target_p[0],tm_target_p[1],tm_target_p[2],tm_target_p[3]*180/M_PI,tm_target_p[4]*180/M_PI,tm_target_p[5]*180/M_PI);
	    tf::Quaternion q_target;
		q_target.setRPY(160 *M_PI/180,  0*M_PI/180,  90*M_PI/180);//old

	    T_cad_to_target.setOrigin(v);
		T_cad_to_target.setRotation(q*q_target);


		tf::Quaternion q_target2tool(0.0, 1.0 ,0.0, 0.0);
		
    	// q_target.setRPY(3.0 *M_PI/180,  0.0*M_PI/180,  0*M_PI/180);//multi
		// tf::Vector3 v_target2tool(0.0, -0.0, 0.0895);
		tf::Vector3 v_target2tool(0.0, -0.0, 0.266);
		tf::Transform T_target2tool(q_target2tool,v_target2tool);

		// T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_qr*T_qr_to_cad*T_cad_to_target*T_target2tool;
		// T_target=T_robot_to_tag0*T_cad_to_target*T_target2tool;
		// T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_qr*T_cad_to_target*T_target2tool;
		T_target=T_cad_to_target;

		geometry_msgs::Transform msg_robottarget;
		msg_robottarget.translation.x=T_target.getOrigin().x();
		msg_robottarget.translation.y=T_target.getOrigin().y();
		msg_robottarget.translation.z=T_target.getOrigin().z();
		msg_robottarget.rotation.x=T_target.getRotation().x();
		msg_robottarget.rotation.y=T_target.getRotation().y();
		msg_robottarget.rotation.z=T_target.getRotation().z();
		msg_robottarget.rotation.w=T_target.getRotation().w();
		pub_target.publish(msg_robottarget);

	}

	void target_Callback_qr_ar(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
	{
		if(TargetPose->header.frame_id!="qr_init"){
		//unity output x=-x ry=-ry rz=-rz
		//tf::Vector3 v( -TargetPose->pose.position.x,TargetPose->pose.position.y,TargetPose->pose.position.z);
   		//multi bone
   		tf::Vector3 v( TargetPose->pose.position.x,TargetPose->pose.position.z,TargetPose->pose.position.y);
	    tm_target_p[0] = -TargetPose->pose.position.x;
	    tm_target_p[1] = TargetPose->pose.position.z;
	    tm_target_p[2] = TargetPose->pose.position.y;

	    tf::Quaternion q(
	        TargetPose->pose.orientation.x,
	        TargetPose->pose.orientation.z,
	        TargetPose->pose.orientation.y,
	        TargetPose->pose.orientation.w);
	    // tf::Quaternion q_target_rotate;
	    // q_target_rotate.setRPY(-M_PI/2,0,0);
	    // q=q_target_rotate*q;
	    	tf::Matrix3x3 m(q);
	    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	    m[0][1]=-m[0][1];
	    m[0][2]=-m[0][2];
	    m[1][0]=-m[1][0];
	    m[2][0]=-m[2][0];
	    m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

	    // Eigen::Quaterniond qE(
	    // 	TargetPose->pose.orientation.w,
	    // 	TargetPose->pose.orientation.x,
	    //     TargetPose->pose.orientation.y,
	    //     TargetPose->pose.orientation.z
	    //     );
	    // Eigen::Vector3d euler = qE.toRotationMatrix().eulerAngles(2,1,0);
	    // std::cout<<euler*180/M_PI<<std::endl;
	    // euler=euler*180/M_PI;
	    
	    // std::cout<<qE.toRotationMatrix()<<std::endl;
	    q.setRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);
	    // m.getEulerYPR(tm_target_p[3], tm_target_p[4], tm_target_p[5]);
	    // m.getEulerZYX(tm_target_p[3], tm_target_p[4], tm_target_p[5]);
	    // m.getRPY(tm_target_p[3], tm_target_p[5], tm_target_p[4]);

	    ROS_INFO("targetA = %f %f %f %f %f %f\n",tm_target_p[0],tm_target_p[1],tm_target_p[2],tm_target_p[3]*180/M_PI,tm_target_p[4]*180/M_PI,tm_target_p[5]*180/M_PI);
	    T_cad_to_target.setOrigin(v);

	    tf::Quaternion q_target;
        q_target.setRPY(-3.0 *M_PI/180,  -1.0*M_PI/180,  0*M_PI/180);//multi
		T_cad_to_target.setRotation(q);
		// T_cad_to_target.setRotation(q*q_target);


		tf::Quaternion q_target2tool(0.0, 1.0 ,0.0, 0.0);
		// tf::Vector3 v_target2tool(0.0025, -0.0008, 0.2661);
		tf::Vector3 v_target2tool(0.0, -0.0, 0.266);
		// tf::Vector3 v_target2tool(0.000, 0.00, 0.0895);
		tf::Transform T_target2tool(q_target2tool,v_target2tool);

		tf::Quaternion q_cad_rotate;
		
		q_cad_rotate.setRPY(0,0,M_PI/2);
		tf::Vector3 v_cad_rotate(0.0, 0.0, 0.0);
		tf::Transform T_cad_rotate(q_cad_rotate,v_cad_rotate);
		// tf::Transform T_target_rotate(q_target_rotate,v_cad_rotate);

		// T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_tag1*T_tag1_to_ARbase*T_ARbase_to_target*T_target2tool;
		T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_qr*T_qr_to_cad*T_cad_to_target*T_target2tool;
		geometry_msgs::Transform msg_robottarget;
		msg_robottarget.translation.x=T_target.getOrigin().x();
		msg_robottarget.translation.y=T_target.getOrigin().y();
		msg_robottarget.translation.z=T_target.getOrigin().z();
		msg_robottarget.rotation.x=T_target.getRotation().x();
		msg_robottarget.rotation.y=T_target.getRotation().y();
		msg_robottarget.rotation.z=T_target.getRotation().z();
		msg_robottarget.rotation.w=T_target.getRotation().w();
		pub_target.publish(msg_robottarget);
		// tf::Vector3 v1(0.023,0.0125,0.094);
		// tf::Vector3 v2(-0.025,-0.0019,0.0923);
		// if(v!=v1&&v!=v2)
		// 	pub_target.publish(msg_robottarget);
		// else
		// 	printf("pre1 or pre2\n");

		// T_robot_traj = T_target2tool*transform.inverse()*T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_qr*T_qr_to_cad;
		// ROS_WARN("QR frame=%f %f %f\n",T_robot_traj.getOrigin().x(),T_robot_traj.getOrigin().y(),T_robot_traj.getOrigin().z());
		loop_confirm(msg_robottarget);
	}

	}


	void target_Callback(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
	{
		tf::Vector3 v(TargetPose->pose.position.x,TargetPose->pose.position.y,TargetPose->pose.position.z);
	    tm_target_p[0] = TargetPose->pose.position.x;
	    tm_target_p[1] = TargetPose->pose.position.y;
	    tm_target_p[2] = TargetPose->pose.position.z;

	    tf::Quaternion q(
	        TargetPose->pose.orientation.x,
	        TargetPose->pose.orientation.y,
	        TargetPose->pose.orientation.z,
	        TargetPose->pose.orientation.w);
	    tf::Matrix3x3 m(q);
	    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	    m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

	    ROS_INFO("targetA = %f %f %f %f %f %f\n",tm_target_p[0],tm_target_p[1],tm_target_p[2],tm_target_p[3],tm_target_p[4],tm_target_p[5]);
	    T_ARbase_to_target.setOrigin(v);
		T_ARbase_to_target.setRotation(q);


		tf::Quaternion q_target2tool(0.0, 1.0 ,0.0, 0.0);
		tf::Vector3 v_target2tool(0.0025, -0.0008, 0.2661);
		tf::Transform T_target2tool(q_target2tool,v_target2tool);

		T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_tag1*T_tag1_to_ARbase*T_ARbase_to_target*T_target2tool;
		geometry_msgs::Transform msg_robottarget;
		msg_robottarget.translation.x=T_target.getOrigin().x();
		msg_robottarget.translation.y=T_target.getOrigin().y();
		msg_robottarget.translation.z=T_target.getOrigin().z();
		msg_robottarget.rotation.x=T_target.getRotation().x();
		msg_robottarget.rotation.y=T_target.getRotation().y();
		msg_robottarget.rotation.z=T_target.getRotation().z();
		msg_robottarget.rotation.w=T_target.getRotation().w();
		pub_target.publish(msg_robottarget);

	}

	void drillup_Callback(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
	{
		// tf::Vector3 v(TargetPose->pose.position.x,TargetPose->pose.position.y,TargetPose->pose.position.z);
	 //    tm_target_p[0] = TargetPose->pose.position.x;
	 //    tm_target_p[1] = TargetPose->pose.position.y;
	 //    tm_target_p[2] = TargetPose->pose.position.z;

	 //    tf::Quaternion q(
	 //        TargetPose->pose.orientation.x,
	 //        TargetPose->pose.orientation.y,
	 //        TargetPose->pose.orientation.z,
	 //        TargetPose->pose.orientation.w);
	 //    tf::Matrix3x3 m(q);
	 //    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	 //    m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

	 //    ROS_INFO("drillup = %f %f %f %f %f %f\n",tm_target_p[0],tm_target_p[1],tm_target_p[2],tm_target_p[3],tm_target_p[4],tm_target_p[5]);
	 //    T_ARbase_to_target.setOrigin(v);
		// T_ARbase_to_target.setRotation(q);

		ROS_INFO("drillup");
		
		tf::Quaternion q_target2tool(0.0, 0.0 ,0.0, 1.0);
		tf::Vector3 v_target2tool(0.0, 0.0, -0.02);
		tf::Transform T_target2tool(q_target2tool,v_target2tool);

		// T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_tag1*T_tag1_to_ARbase*T_ARbase_to_target*T_target2tool;
		T_target=T_target*T_target2tool;
		geometry_msgs::Transform msg_robottarget;
		msg_robottarget.translation.x=T_target.getOrigin().x();
		msg_robottarget.translation.y=T_target.getOrigin().y();
		msg_robottarget.translation.z=T_target.getOrigin().z();
		msg_robottarget.rotation.x=T_target.getRotation().x();
		msg_robottarget.rotation.y=T_target.getRotation().y();
		msg_robottarget.rotation.z=T_target.getRotation().z();
		msg_robottarget.rotation.w=T_target.getRotation().w();
		pub_target.publish(msg_robottarget);

		loop_confirm(msg_robottarget);
	}

	void drillup2_Callback(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
	{

		ROS_INFO("drillup");
		tf::Quaternion q_target2tool(0.0, 0.0 ,0.0, 1.0);
		tf::Vector3 v_target2tool(0.0, 0.0, -0.02);
		tf::Transform T_target2tool(q_target2tool,v_target2tool);

		// T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_tag1*T_tag1_to_ARbase*T_ARbase_to_target*T_target2tool;
		T_target=T_target*T_target2tool;
		geometry_msgs::Transform msg_robottarget;
		msg_robottarget.translation.x=T_target.getOrigin().x();
		msg_robottarget.translation.y=T_target.getOrigin().y();
		msg_robottarget.translation.z=T_target.getOrigin().z();
		msg_robottarget.rotation.x=T_target.getRotation().x();
		msg_robottarget.rotation.y=T_target.getRotation().y();
		msg_robottarget.rotation.z=T_target.getRotation().z();
		msg_robottarget.rotation.w=T_target.getRotation().w();
		pub_target.publish(msg_robottarget);

		loop_confirm(msg_robottarget);
	}

	void drilldown_Callback(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
	{
		// tf::Vector3 v(TargetPose->pose.position.x,TargetPose->pose.position.y,TargetPose->pose.position.z);
	 //    tm_target_p[0] = TargetPose->pose.position.x;
	 //    tm_target_p[1] = TargetPose->pose.position.y;
	 //    tm_target_p[2] = TargetPose->pose.position.z;

	 //    tf::Quaternion q(
	 //        TargetPose->pose.orientation.x,
	 //        TargetPose->pose.orientation.y,
	 //        TargetPose->pose.orientation.z,
	 //        TargetPose->pose.orientation.w);
	 //    tf::Matrix3x3 m(q);
	 //    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	 //    m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

	 //    ROS_INFO("drilldown = %f %f %f %f %f %f\n",tm_target_p[0],tm_target_p[1],tm_target_p[2],tm_target_p[3],tm_target_p[4],tm_target_p[5]);
	 //    T_ARbase_to_target.setOrigin(v);
		// T_ARbase_to_target.setRotation(q);
		ROS_INFO("drilldown");
		

		tf::Quaternion q_target2tool(0.0, 0.0 ,0.0, 1.0);
		tf::Vector3 v_target2tool(0.0, 0.0, 0.02);
		tf::Transform T_target2tool(q_target2tool,v_target2tool);

		// T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_tag1*T_tag1_to_ARbase*T_ARbase_to_target*T_target2tool;
		T_target=T_target*T_target2tool;
		geometry_msgs::Transform msg_robottarget;
		msg_robottarget.translation.x=T_target.getOrigin().x();
		msg_robottarget.translation.y=T_target.getOrigin().y();
		msg_robottarget.translation.z=T_target.getOrigin().z();
		msg_robottarget.rotation.x=T_target.getRotation().x();
		msg_robottarget.rotation.y=T_target.getRotation().y();
		msg_robottarget.rotation.z=T_target.getRotation().z();
		msg_robottarget.rotation.w=T_target.getRotation().w();
		pub_target.publish(msg_robottarget);

		loop_confirm(msg_robottarget);
	}

	void drilldown2_Callback(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
	{
		ROS_INFO("drilldown");
		

		tf::Quaternion q_target2tool(0.0, 0.0 ,0.0, 1.0);
		tf::Vector3 v_target2tool(0.0, 0.0, 0.02);
		tf::Transform T_target2tool(q_target2tool,v_target2tool);

		// T_target=T_robot_to_tag0*T_cam_to_tag0.inverse()*T_cam_to_tag1*T_tag1_to_ARbase*T_ARbase_to_target*T_target2tool;
		T_target=T_target*T_target2tool;
		geometry_msgs::Transform msg_robottarget;
		msg_robottarget.translation.x=T_target.getOrigin().x();
		msg_robottarget.translation.y=T_target.getOrigin().y();
		msg_robottarget.translation.z=T_target.getOrigin().z();
		msg_robottarget.rotation.x=T_target.getRotation().x();
		msg_robottarget.rotation.y=T_target.getRotation().y();
		msg_robottarget.rotation.z=T_target.getRotation().z();
		msg_robottarget.rotation.w=T_target.getRotation().w();
		pub_target.publish(msg_robottarget);

		loop_confirm(msg_robottarget);
	}

	void hole_target_set(){
		//test rosrun tf static_transform_publisher 0 0 0.3 1 0 0 0 /hole /tt 100
		tf::Quaternion q_target;
		q_target.setRPY(1.0 *M_PI/180,  0.0*M_PI/180,  0*M_PI/180);

		tf::Quaternion q_hole2target(1.0, 0.0 ,0.0 ,0.0);
		tf::Vector3 v_hole2target(0.001, 0.0008, 0.2641);
		// tf::Vector3 v_hole2target(0.0, 0.0, 0.273);
		tf::Transform T_hole2target(q_target*q_hole2target,v_hole2target);
		T_target=T_robot_to_hole*T_hole2target;

		geometry_msgs::Transform msg_robottarget;
		msg_robottarget.translation.x=T_target.getOrigin().x();
		msg_robottarget.translation.y=T_target.getOrigin().y();
		msg_robottarget.translation.z=T_target.getOrigin().z();
		msg_robottarget.rotation.x=T_target.getRotation().x();
		msg_robottarget.rotation.y=T_target.getRotation().y();
		msg_robottarget.rotation.z=T_target.getRotation().z();
		msg_robottarget.rotation.w=T_target.getRotation().w();


		// tf::Quaternion q(msg_robottarget.rotation.x,msg_robottarget.rotation.y,msg_robottarget.rotation.z,msg_robottarget.rotation.w);
		// tf::Matrix3x3 m(q);
		// double deg[3];
		// m.getRPY(deg[0], deg[1], deg[2]);
		// deg[0]=-M_PI;
		// deg[1]=0.0;

		// Eigen::AngleAxisf yawAngle (deg[2], Eigen::Vector3f::UnitZ());
		// Eigen::AngleAxisf pitchAngle (deg[1], Eigen::Vector3f::UnitY());
		// Eigen::AngleAxisf rollAngle(deg[0], Eigen::Vector3f::UnitX());
		// Eigen::Quaternion<float> eq = yawAngle * pitchAngle *rollAngle;


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
