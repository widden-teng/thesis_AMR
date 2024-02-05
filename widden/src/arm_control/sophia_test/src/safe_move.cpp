
#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_robot_state.h"
#include "tm_driver/tm_ros_node.h"

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"
#include "robotiq_controller/Robotiq2FGripper_robot_output.h"

#include "tm_msgs/SetIO.h"
#include "tm_msgs/SetVelocity.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SetEvent.h"
#include "tm_msgs/SetPositions.h"

#include <Eigen/Dense>

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#ifdef USE_BOOST
  #include <boost/lexical_cast.hpp>
  #include <boost/thread/thread.hpp>
  #include <boost/thread/mutex.hpp>
  #include <boost/thread/condition_variable.hpp>
  #include <boost/chrono/chrono.hpp>
#else
  #include <thread>
  #include <mutex>
  #include <condition_variable>
  #include <chrono>
#endif

#define DEG2RAD M_PI/180.0
#define RAD2DEG 180.0/M_PI
#define REPEATABILITY 0.01//0.00005

#define GRIPPER_LENGTH 0.22

#define VMAX_ATT_GPR 0.08
#define VMAX_ATT 0.08
#define VMAX_REP 0.15
#define VMAX_ROT 20
#define VMAX_CC_GPR  0.03

#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 150
// #define JOINTLIMIT_SPD_456 200
#define JOINTLIMIT_SPD_456 150

#define STOP 0
#define PASS 1
#define REACH 2

#define SMOOTHSTOP 0
#define REACHPOINT 1
#define INTERRUPT  2

#define OBSTACLE_GPR false
#define V_TRAVEL 0.1

using namespace std;

typedef float Scalar;
const float EPS = 1e-6;
const float LAMBDA_MAX = 0.3;
const float EPSQ = 1e-15;


std::vector<double> g_robot_joint_angle(6);
double g_objposition[7];
int tm_target_type=0; // 1: random, 2: drilldown, 3: drillup
double tm_target_p[6];
double tm_target_j[6];
bool pose_fill = false;

ros::Publisher RobotState;

ros::ServiceClient vel_client;
ros::ServiceClient script_client;
ros::ServiceClient event_client;
ros::ServiceClient pos_client;

ros::Publisher pub_ctrl;
robotiq_controller::Robotiq2FGripper_robot_output gripper_command;

std::string exit_cmd = "ScriptExit()";
std::string VStart_cmd = "ContinueVJog()";
std::string VStop_cmd = "StopContinueVmode()";


bool ReflexxesPositionRun(RMLPositionInputParameters &InputState,
	std::vector<double> TargetPosition,
	std::vector<double> TargetVelocity,
	double SynTime)
{
	tm_msgs::SetVelocity vel_srv;

	double time_s;
	std::vector<double> FinalPosition;
	bool pass = true;
	std::vector<double> VelocityCommand(6);
	double blend = 0;

	ReflexxesAPI *RML = NULL;
	RMLPositionInputParameters  *IP = NULL;
	RMLPositionOutputParameters *OP = NULL;
	RMLPositionFlags Flags;
	int ResultValue = 0;

	double *T = new double[16];
	double *q = new double[6];

	RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
	IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);
	OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

	*IP = InputState;

	tm_reflexxes::initTermios(1);

	//  ********************************************************************/
	//  Assigning all RMLPositionInputParameters : 
	//  Current POS, VEL, ACC : set before call ReflexxesPositionRun
	//  Target POS, VEL       : set before call ReflexxesPositionRun
	//  Max VEL, ACC          : set after call ReflexxesPositionRun
	//  SelectionVector       : set after call ReflexxesPositionRun
	//  ********************************************************************
	for (int i = 0; i < NUMBER_OF_DOFS; ++i)
	{
		IP->MaxJerkVector->VecData[i] = 100;           //RMLTypeII not using, needed for validity
		IP->MaxVelocityVector->VecData[i] = 3.14; //0.3247
		IP->MaxAccelerationVector->VecData[i] = 3.14;
		IP->TargetPositionVector->VecData[i] = TargetPosition[i];
		IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];
		IP->SelectionVector->VecData[i] = true;
	}
	IP->MinimumSynchronizationTime = SynTime;


	if (IP->CheckForValidity())
		printf("Input values are valid!\n");
	else
	{
		printf("Input values are INVALID!\n");
		pass = false;
	}


	struct timeval tm1, tm2, tm3, tm4;
	double cycle_iteration = 1.0;


	gettimeofday(&tm3, NULL);

	while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED && ros::ok())
	{
		//********************************************************
		// The area execution in 25ms real time sharp

		gettimeofday(&tm1, NULL);

		ResultValue = RML->RMLPosition(*IP, OP, Flags);

		if (ResultValue < 0)
		{
			printf("An error occurred (%d).\n", ResultValue);
			break;
		}

		VelocityCommand = { OP->NewVelocityVector->VecData[0],
							OP->NewVelocityVector->VecData[1],
							OP->NewVelocityVector->VecData[2],
							OP->NewVelocityVector->VecData[3],
							OP->NewVelocityVector->VecData[4],
							OP->NewVelocityVector->VecData[5] };

		// TR.setMoveJointSpeedabs(vec, blend);
		vel_srv.request.motion_type = 1;
		vel_srv.request.velocity = VelocityCommand;
		// vel_client.call(vel_srv);
		if (vel_client.call(vel_srv))
		{
			if (vel_srv.response.ok) {
				// ROS_INFO_STREAM("SetPositions to robot");
			}
			else
				ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
		}
		else
		{
			ROS_ERROR_STREAM("Error SetPositions to robot");
			// return 1;
		}
		// usleep(100*1000);

		//***************************************************************

		if (tm_reflexxes::kbhit())
		{
			char c = getchar();
			if (c == 'q' || c == 'Q')
			{
				ROS_WARN("Smooth Stop Activate...");
				ReflexxesPositionSmoothStop(IP, 0.25);
				pass = false;
				break;
			}
		}

		*IP->CurrentPositionVector = *OP->NewPositionVector;
		*IP->CurrentVelocityVector = *OP->NewVelocityVector;

		gettimeofday(&tm2, NULL);
		long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);
		// usleep(24940 - time_compensation);
		if ((24940 - time_compensation) < 1)
			usleep(100);
		else
			usleep(24940 - time_compensation);

		//********************************************************
		// The area execution in 25ms real time sharp
	}

	gettimeofday(&tm4, NULL);
	long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

	ROS_INFO("========== Final state position based position =============");
	ROS_INFO("XYZ_pos : %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);
	print_info("Finished in %llu us", tt);

	tm_reflexxes::resetTermios();

	if (pass)
		InputState = *IP;

	delete  RML;
	delete  IP;
	delete  OP;
	delete[] T;
	delete[] q;

	return pass;
}

bool CheckJointLimit(double *q)
{
	bool valid = true;

	if (abs(q[0]) > 265 * DEG2RAD)
	{
		ROS_WARN("[Position] 1st joint position out of limit (270) : %lf", q[0] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[1]) > 175 * DEG2RAD)
	{
		ROS_WARN("[Position] 2nd joint position out of limit (180): %lf", q[1] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[2]) > 148 * DEG2RAD)
	{
		ROS_WARN("[Position] 3rd joint position out of limit (155): %lf", q[2] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[3]) > 175 * DEG2RAD)
	{
		ROS_WARN("[Position] 4th joint position out of limit (180): %lf", q[3] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[4]) > 175 * DEG2RAD)
	{
		ROS_WARN("[Position] 5th joint position out of limit (180): %lf", q[4] * RAD2DEG);
		valid = false;
	}
	else if (abs(q[5]) > 265 * DEG2RAD)
	{
		ROS_WARN("[Position] 6th joint position out of limit (180): %lf", q[5] * RAD2DEG);
		valid = false;
	}
	else
		valid = true;

	return valid;
}

bool CheckVelocityLimit(std::vector<double> qd, double &MaxScalingFactor)
{
	bool valid = true;
	double ExceedRatio[6] = { 0,0,0,0,0,0 };
	double MaxExceed = 1;
	short  ExceedJoint;

	for (int i = 0; i < 3; ++i)
	{
		if (abs(qd[i]) / JOINTLIMIT_SPD_123 > 1.0)
		{
			ExceedRatio[i] = abs(qd[i]) / JOINTLIMIT_SPD_123;
			ROS_WARN("[Velocity] %dth joint velocity exceed limit(150): %10.4lf", i + 1, qd[i] * RAD2DEG);
			valid = false;
		}
		else
			ExceedRatio[i] = 0.0;
	}

	for (int i = 3; i < 6; ++i)
	{
		if (abs(qd[i]) / JOINTLIMIT_SPD_456 > 1.0)
		{
			ExceedRatio[i] = abs(qd[i]) / JOINTLIMIT_SPD_456;
			ROS_WARN("[Velocity] %dth joint velocity exceed limit(200): %10.4lf", i + 1, qd[i] * RAD2DEG);
			valid = false;
		}
		else
			ExceedRatio[i] = 0.0;
	}

	if (!valid)
	{
		for (int i = 0; i < NUMBER_OF_DOFS; ++i)
		{
			if (ExceedRatio[i] > MaxExceed)
			{
				MaxExceed = ExceedRatio[i];
				ExceedJoint = i;
			}
		}
		MaxScalingFactor = MaxExceed;
		ROS_WARN("[Velocity] THE MOST EXCEED JOINT IS %dth joint, NEED SCALING %10.4lf", ExceedJoint + 1, MaxScalingFactor);
	}

	return valid;
}

bool GetQfromInverseKinematics(double* CartesianPosition, double *q_inv)
{
	Eigen::Matrix<float, 4, 4> T_;
	Eigen::AngleAxisf yawAngle(CartesianPosition[5], Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf pitchAngle(CartesianPosition[4], Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
	Eigen::Quaternion<float> q = yawAngle * pitchAngle *rollAngle;
	Eigen::Matrix<float, 3, 3> RotationMatrix = q.matrix();
	double *T = new double[16];


	T_ << 0., 0., 0., CartesianPosition[0],
		0., 0., 0., CartesianPosition[1],
		0., 0., 0., CartesianPosition[2],
		0., 0., 0., 1.;

	T_.block<3, 3>(0, 0) = RotationMatrix.block<3, 3>(0, 0);

	tm_jacobian::Matrix2DoubleArray(T_, T);
	int num_sol = tm_kinematics::inverse(T, q_inv);

	delete[] T;
	return CheckJointLimit(q_inv);
}

bool AR_Demo()
{
	bool run_succeed = true;
	bool table_to_box = false;
	bool skip_flag = false;
	int  pass = 0;
	int finalpose_flag = 1;
	std::vector<double>  final_position(6);
	double SynchronousTime = 7;
	std::vector<double> TargetPosition(6), TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6);
	double *T = new double[16];
	double *q = new double[6];
	std_msgs::Int32 robot_motion_states;

	RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
	// RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);



	// TM5.interface->stateRT->getQAct(CurrentPosition);
	for (int i = 0; i < NUMBER_OF_DOFS; ++i)
	{
		CurrentPosition[i] = g_robot_joint_angle[i];
	}

	for (int i = 0; i < NUMBER_OF_DOFS; ++i)
	{
		IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
		IP_position->CurrentVelocityVector->VecData[i] = 0.0;
		IP_position->CurrentAccelerationVector->VecData[i] = 0.0;
	}

	std::vector<double> ToolPose = { 0.8737,    0.1277,    1.6537,    -0.1775,    1.5723,    0.1935 };


	TargetVelocity = { 0, 0, 0, 0, 0, 0 };

	g_objposition[0] = 2.0;
	robot_motion_states.data = tm_target_type;

	while (ros::ok())
	{
		if (pose_fill)
		{
			for (int i = 0; i < NUMBER_OF_DOFS; ++i)
				TargetPosition[i] = tm_target_j[i];

			if (robot_motion_states.data == 1)
			{
				// Picking_above_pose (computational result)
				ROS_WARN("random");
				skip_flag = false;
				// SynchronousTime = 5;//1.5;
				SynchronousTime = 3;
				robot_motion_states.data = 2;
				if (!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
					break;


				RobotState.publish(robot_motion_states);
				ROS_WARN("Robot State = %d Complete", robot_motion_states.data);
			}
			else if (robot_motion_states.data == 2 || robot_motion_states.data == 3)
			{
				// Picking_above_pose (computational result)
				ROS_WARN("drill");
				skip_flag = false;
				// SynchronousTime = 5;//1.5;
				SynchronousTime = 3;
				robot_motion_states.data = 2;
				if (!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
					break;


				RobotState.publish(robot_motion_states);
				ROS_WARN("Robot State = %d Complete", robot_motion_states.data);
			}
			else {}

			pose_fill = false;
			sleep(1);
		}
	}



	ROS_WARN("AR Demo shutdown");
	delete IP_position;
	delete[] T;
	delete[] q;

	return 0;
}

std::vector<double> parse_cmd(char* cstr, const char* delim, double& res)
{
	std::vector<double> ret;
	char* pch;
	char* pch_save;
	pch = strtok_r(cstr, delim, &pch_save);
	if (pch != NULL)
	{
		while ((pch = strtok_r(NULL, delim, &pch_save)) != NULL)
		{
			//count++;
			if (ret.size() < 6)
			{
				ret.push_back(atof(pch));
			}
			else
			{
				res = atof(pch);
				break;
			}
		}
	}

	return ret;
}


void ToolPose_Callback(const tm_msgs::FeedbackState::ConstPtr& ToolPose)
{
	tm_end_effector_p[0] = ToolPose->tool_pose[0];
	tm_end_effector_p[1] = ToolPose->tool_pose[1];
	tm_end_effector_p[2] = ToolPose->tool_pose[2];
	tm_end_effector_p[3] = ToolPose->tool_pose[3];
	tm_end_effector_p[4] = ToolPose->tool_pose[4];
	tm_end_effector_p[5] = ToolPose->tool_pose[5];

	// tf::Quaternion q(
	//     ToolPose->pose.orientation.x,
	//     ToolPose->pose.orientation.y,
	//     ToolPose->pose.orientation.z,
	//     ToolPose->pose.orientation.w);
	// tf::Matrix3x3 m(q);

	// T_tm_end_effector <<   0., 0., 0., tm_end_effector_p[0],
	//         0., 0., 0., tm_end_effector_p[1],
	//         0., 0., 0., tm_end_effector_p[2],
	//         0., 0., 0., 1.;
	// Eigen::Matrix<double,3,3> rot;
	// // tf::matrixTFToEigen(m,rot);
	// // T_tm_end_effector.block<3,3>(0,0) = rot.block<3,3>(0,0);
	// //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	// m.getRPY(tm_end_effector_p[3], tm_end_effector_p[4], tm_end_effector_p[5]);

}

void ToolJoint_Callback(const tm_msgs::FeedbackState::ConstPtr& ToolPose)
{
	if (ToolPose->joint_pos.size() == 6) {
		g_robot_joint_angle[0] = ToolPose->joint_pos[0];
		g_robot_joint_angle[1] = ToolPose->joint_pos[1];
		g_robot_joint_angle[2] = ToolPose->joint_pos[2];
		g_robot_joint_angle[3] = ToolPose->joint_pos[3];
		g_robot_joint_angle[4] = ToolPose->joint_pos[4];
		g_robot_joint_angle[5] = ToolPose->joint_pos[5];
	}
}

void Target_Callback(const geometry_msgs::TransformStamped::ConstPtr& TargetPose)
{
	tm_target_p[0] = TargetPose->transform.translation.x;
	tm_target_p[1] = TargetPose->transform.translation.y;
	tm_target_p[2] = TargetPose->transform.translation.z;

	tf::Quaternion q(
		TargetPose->transform.rotation.x,
		TargetPose->transform.rotation.y,
		TargetPose->transform.rotation.z,
		TargetPose->transform.rotation.w);
	tf::Matrix3x3 m(q);
	//Roll: rx, Pitch: ry, Yaw: rz, unit: rad
	m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

	//if (strncmp(TargetPose->child_frame_id, "random", 6) == 0)
	if (TargetPose->child_frame_id == "random") {
		tm_target_type = 1;
	}
	else if (TargetPose->child_frame_id == "drilldown") {
		tm_target_type = 2;
	}
	else if (TargetPose->child_frame_id == "drillup") {
		tm_target_type = 3;
	}
	

	if (GetQfromInverseKinematics(tm_target_p, tm_target_j)) {
		pose_fill = true;
	}
	else {
		pose_fill = false;
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_tm5_reflexxes");
	ros::NodeHandle node_handle;

	ROS_INFO("Subscribe /tool_position");
	ros::CallbackQueue tool_pose;
	// ros::SubscribeOptions ops_toolpose = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/tool_pose", 10, ToolPose_Callback, ros::VoidPtr(), &tool_pose);
	ros::SubscribeOptions ops_toolpose = ros::SubscribeOptions::create<tm_msgs::FeedbackState>("/feedback_states", 10, ToolPose_Callback, ros::VoidPtr(), &tool_pose);
	ros::Subscriber sub_toolpose = node_handle.subscribe(ops_toolpose);
	ros::AsyncSpinner async_spinner_toolpose(1, &tool_pose);
	async_spinner_toolpose.start();

	ROS_INFO("Subscribe /robot_target");
	ros::CallbackQueue robot_target;
	ros::SubscribeOptions ops_robot_target = ros::SubscribeOptions::create<geometry_msgs::TransformStamped>("/robot_target", 10, Target_Callback, ros::VoidPtr(), &robot_target);
	ros::Subscriber sub_robot_target = node_handle.subscribe(ops_robot_target);
	ros::AsyncSpinner async_spinner_robot_target(1, &robot_target);
	async_spinner_robot_target.start();
	
	vel_client = node_handle.serviceClient<tm_msgs::SetVelocity>("tm_driver/set_velocity");
	script_client = node_handle.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
	event_client = node_handle.serviceClient<tm_msgs::SetEvent>("tm_driver/set_event");
	pos_client = node_handle.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");


	tm_msgs::SetPositions target_pose, target_pose_line;
	tm_msgs::SetEvent event_srv;
	tm_msgs::SendScript script_srv;

	target_pose.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
	target_pose.request.velocity = 2.0;//rad/s
	target_pose.request.acc_time = 0.2;
	target_pose.request.blend_percentage = 0;
	target_pose.request.fine_goal = false;

	target_pose_line.request.motion_type = tm_msgs::SetPositions::Request::LINE_T;
	target_pose_line.request.velocity = 0.3;//rad/s
	target_pose_line.request.acc_time = 0.1;
	target_pose_line.request.blend_percentage = 0;
	target_pose_line.request.fine_goal = false;

	std::vector<double> home = { 0.5134511108398437, 0.16386395263671874, 0.40, -M_PI, 0.0, M_PI / 2 };

	// ros::Publisher pub_estimation;
	ros::Rate r(30);
	int valid = 0;

	char cstr[512];
	char delim[] = " ,;\t";
	char c;
	while (ros::ok())
	{
		memset(cstr, 0, 512);
		fgets(cstr, 512, stdin);
		int n = (int)strlen(cstr);
		if (n > 0)
		{
			if (cstr[n - 1] == '\n')
				cstr[n - 1] = '\0';
		}
		if (strncmp(cstr, "quit", 4) == 0)
		{
			script_srv.request.id = "quit";
			script_srv.request.script = exit_cmd;

			script_client.call(script_srv);
			fgRun = false;

			print_info("quit");
			break;
		}
		else if (strncmp(cstr, "home", 4) == 0)
		{
			std::vector<double> vec1 = { 0,0,0,0,0,0 };

			target_pose.request.positions.clear();
			target_pose.request.positions = vec1;
			pos_client.call(target_pose);

			print_info("Back to home");
		}
		else if (strncmp(cstr, "ready", 5) == 0)
		{
			std::vector<double> vec1 = { 0.0,    -0.4777,    1.9319,    -1.4537,    1.5708,    0.0 };

			target_pose.request.positions.clear();
			target_pose.request.positions = vec1;
			pos_client.call(target_pose);

			print_info("Back to ready position");
		}
		else if (strncmp(cstr, "gstart", 6) == 0)
		{
			ROS_INFO("Activating gripper");
			reset_gripper();
			pub_ctrl.publish(gripper_command);
			sleep(1);
			init_gripper();
			pub_ctrl.publish(gripper_command);
			sleep(1);
			set_gripper(1);
			pub_ctrl.publish(gripper_command);
			sleep(1);
		}
		else if (strncmp(cstr, "gopen", 6) == 0)
		{
			set_gripper(0);
			pub_ctrl.publish(gripper_command);
		}
		else if (strncmp(cstr, "gclose", 7) == 0)
		{
			set_gripper(1);
			pub_ctrl.publish(gripper_command);
		}
		else if (strncmp(cstr, "gopen0", 7) == 0)
		{
			gripper_command.rACT = 1;
			gripper_command.rGTO = 1;
			gripper_command.rSP = 200;
			gripper_command.rFR = 0;
			gripper_command.rPR = 0;
			pub_ctrl.publish(gripper_command);

		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// do bin picking demo 
		else if (strncmp(cstr, "demo", 6) == 0)
		{
			// TmRobot.setDigitalOutputEE(0,true);
			set_gripper(1); // close gripper=1 
			pub_ctrl.publish(gripper_command);

			script_srv.request.id = "Vstart";
			script_srv.request.script = VStart_cmd;
			script_client.call(script_srv);
			print_info("joint velocity control mode ON...");

			AR_Demo();
			//StaticRobotAvoidnace(TmRobot);
			script_srv.request.id = "spdmodeoff";
			script_srv.request.script = VStop_cmd;
			script_client.call(script_srv);

			print_info("joint vlocity control mode OFF...");
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		else if (strncmp(cstr, "spdmodeoff", 10) == 0)
		{
			script_srv.request.id = "spdmodeoff";
			script_srv.request.script = VStop_cmd;

			script_client.call(script_srv);
			print_info("joint vlocity control mode OFF...");
		}
		else
		{
			std::string cmd{ cstr };
			std::cout << "send cmd: " << cmd << "\n";

			script_srv.request.id = "Parse_cmd";
			script_srv.request.script = cmd;
			script_client.call(script_srv);

			print_info("send cmd...");
		}
	}

	printf("[ info] TM_ROS: shutdown\n");

	return 0;
}






	while (ros::ok()) {
		//ROS_ERROR("start_cmd = %d",start_cmd);
		// printf("j=%lf %lf %lf %lf %lf %lf\n",tm_target_j[0],tm_target_j[1],tm_target_j[2],tm_target_j[3],tm_target_j[4],tm_target_j[5]);
		// double T[16];

		// tm_kinematics::forward(tm_target_j, T);

		// cout << start_cmd<<">>>> T06" << endl;
		// tm_jacobian::printMatrix(T,4,16);
		if (start_cmd != 0) {
			if (GetQfromInverseKinematics(tm_target_p, tm_target_j)) {
				// printf("j=%lf %lf %lf %lf %lf %lf\n",tm_target_j[0],tm_target_j[1],tm_target_j[2],tm_target_j[3],tm_target_j[4],tm_target_j[5]);
				double T[16];
				tm_kinematics::forward(tm_target_j, T);

				cout << start_cmd << ">>>> T06" << endl;
				tm_jacobian::printMatrix(T, 4, 16);
				target_pose.request.positions.clear();
				// std::vector<double> target={0,0,0,0,0,0};

				for (int i = 0; i < 6; i++) {
					// target[i]=tm_target_j[i];
					target_pose.request.positions.push_back(tm_target_p[i]);
				}

				// std::cin>>valid;
				// if(valid==-1){
				//     target_pose.request.positions=home;
				// }

				if (client.call(target_pose))
				{
					start_cmd = 0;
					if (target_pose.response.ok) ROS_INFO_STREAM("SetPositions to robot");
					else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
				}
				else
				{
					start_cmd = 0;
					ROS_ERROR_STREAM("Error SetPositions to robot");
					return 1;
				}
			}
			else {
				ROS_ERROR("out of workspace");
				start_cmd = 0;
			}


		}
		r.sleep();

	}

	return 0;
}

