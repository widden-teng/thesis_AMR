/*********************************************************************
 * tm_action_sponge.cpp
 *
 * Copyright (c) 2020, ISCI / National Chiao Tung University (NCTU)
 *
 * Author: Horn Lee (Horn860805@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 *
 * Author: Horn Lee
 */

/* Based on original source from Yun-Hsuan Tsai */

/*********************************************************************
 * tm_ros_wrapper.cpp
 *
 * Copyright 2016 Copyright 2016 Techman Robot Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************/

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
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

#include <stdio.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
//#include <my_msg_pkg/Path.h>
#include <fstream>
#include <ostream>
#include <algorithm>
#include <ros/package.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

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

#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951
#define REPEATABILITY 0.0045 //0.01//0.00005 //TODO: modify attractive force
#define DegreeThreshold 0.02 //TODO: modify attractive force

#define GRIPPER_LENGTH 0.27 //0.266

#define VMAX_ATT_GPR 0.08
#define VMAX_REP     0.15 
#define VMAX_CC_GPR  0.03

#define OBSTACLE_GPR false
#define V_TRAVEL 0.05

#define DANGEROUS_ZONE 0.3 
#define JOINTLIMIT_SPD_123 50 //150
#define JOINTLIMIT_SPD_456 50 //200

#define STOP 0
#define PASS 1
#define REACH 2

#define STEPSIZE 0.03

typedef float Scalar;
const float EPS = 1e-6;
const float LAMBDA_MAX = 0.3;
const float EPSQ = 1e-15;

using namespace std;

double last_point_x = 0.0;
double last_point_y = 0.0;
double last_point_z = 0.0;

double g_distance;
std::vector<double> g_obstacle_position(3);
std::vector<double> tool_obstacle_position(3);
std::vector<double> g_obstacle_velocity(3);
std::vector<double> g_constraint_position(3);
std::vector<double> g_robot_joint_angle(6);
std::vector<double> Virtual_robot_joint_angle(6);
std::vector<double> g_robot_tool_pos(6);

std::vector<geometry_msgs::Point> init_path;
std::vector<geometry_msgs::Pose> static_path;
std::vector<geometry_msgs::Pose> static_path_simulate;
std::vector<geometry_msgs::Point> dynamic_path;

ros::Publisher marker_pub, reset_monitoring_pub, Joints_pub, pub_robot_project;
visualization_msgs::Marker edge;

int init_path_num;
bool get_robot_joints = false;
bool get_init_path = false;
int g_dangerous = 1;
int g_dangerous_prev = 1;
bool get_dynamic_path = false;
int start_cmd = 0;
int start_simulate = 0;
bool stopwrite = true;

ros::ServiceClient vel_client;
ros::ServiceClient script_client;
ros::ServiceClient event_client;
ros::ServiceClient pos_client;

ros::Publisher pub_ctrl;
robotiq_controller::Robotiq2FGripper_robot_output gripper_command;

std::string exit_cmd = "ScriptExit()";
std::string VStart_cmd = "ContinueVJog()";
std::string VStop_cmd = "StopContinueVmode()";

ofstream end_pose, distance_error, virtual_joints, real_joints;

bool ReflexxesPositionRun(  RMLPositionInputParameters &InputState, 
                            std::vector<double> TargetPosition,
                            std::vector<double> TargetVelocity, 
                            double SynTime);

void reset_gripper()
{
    gripper_command.rACT = 0;
    gripper_command.rGTO = 0;
    gripper_command.rSP  = 0;
    gripper_command.rFR  = 0;   
}

void init_gripper()
{
    gripper_command.rACT = 1;
    gripper_command.rGTO = 1;
    gripper_command.rSP  = 200;
    gripper_command.rFR  = 0;
}

void set_gripper(int mode)
{
    if(mode == 0)   //Open
    {
        gripper_command.rACT = 1;
        gripper_command.rGTO = 1;
        gripper_command.rSP  = 200;
        gripper_command.rFR  = 0;
        gripper_command.rPR = 170;
    }
    if(mode == 1)   //Close
    {
        gripper_command.rACT = 1;
        gripper_command.rGTO = 1;
        gripper_command.rSP  = 200;
        gripper_command.rFR  = 0;
        gripper_command.rPR = 230;
    }
    if(mode == 2)   //full open
    {
        gripper_command.rACT = 1;
        gripper_command.rGTO = 1;
        gripper_command.rSP  = 200;
        gripper_command.rFR  = 0;
        gripper_command.rPR = 100;
    }
    if(mode == 3)   //bar Close
    {
        gripper_command.rACT = 1;
        gripper_command.rGTO = 1;
        gripper_command.rSP  = 200;
        gripper_command.rFR  = 0;
        gripper_command.rPR = 200;
    }
}

bool CheckJointLimit(double *q)
{
    bool valid = true;

    if(abs(q[0]) > 270*DEG2RAD) //265
    {
        ROS_WARN("[Position] 1st joint position out of limit (270) : %lf",q[0]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[1]) > 1.57) //175*DEG2RAD
    {
        ROS_WARN("[Position] 2nd joint position out of limit (180): %lf",q[1]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[2]) > 155*DEG2RAD) //148
    {
        ROS_WARN("[Position] 3rd joint position out of limit (155): %lf",q[2]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[3]) > 180*DEG2RAD) //175
    {
        ROS_WARN("[Position] 4th joint position out of limit (180): %lf",q[3]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[4]) > 180*DEG2RAD) //175
    {
        ROS_WARN("[Position] 5th joint position out of limit (180): %lf",q[4]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[5]) > 270*DEG2RAD) //265
    {
        ROS_WARN("[Position] 6th joint position out of limit (180): %lf",q[5]*RAD2DEG );
        valid = false;
    }
    else
        valid = true;

    return valid;
}

bool CheckVelocityLimit(std::vector<double> qd, double &MaxScalingFactor)
{
    bool valid = true;
    double ExceedRatio[6] = {0,0,0,0,0,0};
    double MaxExceed = 1;
    short  ExceedJoint;

    for (int i = 0; i < 3; ++i)
    {
        if (abs(qd[i])/JOINTLIMIT_SPD_123 > 1.0)
        {
            ExceedRatio[i] = abs(qd[i])/JOINTLIMIT_SPD_123;
            ROS_WARN("[Velocity] %dth joint velocity exceed limit(150): %10.4lf",i+1,qd[i]*RAD2DEG);
            valid = false;
        }
        else
            ExceedRatio[i] = 0.0;
    }

    for (int i = 3; i < 6; ++i)
    {
        if (abs(qd[i])/JOINTLIMIT_SPD_456 > 1.0)
        {
            ExceedRatio[i] = abs(qd[i])/JOINTLIMIT_SPD_456;
            ROS_WARN("[Velocity] %dth joint velocity exceed limit(200): %10.4lf",i+1,qd[i]*RAD2DEG);
            valid = false;
        }
        else
            ExceedRatio[i] = 0.0;
    }

    if(!valid)
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            if(ExceedRatio[i] > MaxExceed)
            {
                MaxExceed = ExceedRatio[i];
                ExceedJoint = i;
            }
        }
        MaxScalingFactor = MaxExceed;
        ROS_WARN("[Velocity] THE MOST EXCEED JOINT IS %dth joint, NEED SCALING %10.4lf",ExceedJoint+1,MaxScalingFactor);
    }

    return valid;
}

void ReflexxesSmoothStop(   RMLVelocityInputParameters &InputState, 
                            double SynTime)
{
    tm_msgs::SetVelocity vel_srv;

    double blend = 0, time_s;
    std::vector<double> vec;

    ReflexxesAPI *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags Flags;
    int ResultValue = 0;
    bool pass = true;
    struct timeval tm1,tm2, tm3, tm4;

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    OP = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);
    *IP = InputState;


    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library*/

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
        IP->MaxAccelerationVector->VecData[i] = 0.5*40;
        IP->TargetVelocityVector->VecData[i] = 0.0;
        if(IP->CurrentVelocityVector->VecData[i] != 0.0)    
            IP->SelectionVector->VecData[i] = true;
        else
            IP->SelectionVector->VecData[i] = false;

    }
    IP->MinimumSynchronizationTime = SynTime;

    // ********************************************************************


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
        printf("Input values are INVALID!\n");

    Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;

    gettimeofday(&tm3, NULL);
    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

        ResultValue =  RML->RMLVelocity(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }
        vec = { OP->NewVelocityVector->VecData[0],
                OP->NewVelocityVector->VecData[1],
                OP->NewVelocityVector->VecData[2],
                OP->NewVelocityVector->VecData[3],
                OP->NewVelocityVector->VecData[4],
                OP->NewVelocityVector->VecData[5]};

        // TR.setMoveJointSpeedabs(vec, blend);
        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = vec;

        std_msgs::Float64MultiArray OutputJoints;
        OutputJoints.data.resize(6);
        for(int i = 0; i < 6; i++){
            OutputJoints.data[i] = OP->NewPositionVector->VecData[i];
        }
        Joints_pub.publish(OutputJoints);

        vel_client.call(vel_srv);
        //**********************
        // Print out commands

        // time_s = TR.interface->stateRT->getTime();
        // printf("[ %lf ] pos:  ",time_s );

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf("%10.4lf ", OP->NewPositionVector->VecData[i]);

        printf(" | spd: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
        
        printf("\n");

        //**********************

        *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
        *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
        *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);   
        if((24940 - time_compensation) < 1)
            usleep(100);
        else
            usleep(24940 - time_compensation);         
        // usleep(24940 - time_compensation);  

        // The area execution in 25ms real time sharp
        //********************************************************
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    std::vector<double> FinalPosition(6);
    // time_s = TR.interface->stateRT->getQAct(FinalPosition);
    printf("=============== Final state of Smooth Stop =========================\n");
    // printf("[ %lf ]  ", time_s);

    FinalPosition[0] = g_robot_joint_angle[0];
    FinalPosition[1] = g_robot_joint_angle[1];
    FinalPosition[2] = g_robot_joint_angle[2];
    FinalPosition[3] = g_robot_joint_angle[3];
    FinalPosition[4] = g_robot_joint_angle[4];
    FinalPosition[5] = g_robot_joint_angle[5];
    for (int i = 0; i < NUMBER_OF_DOFS; ++i){
        printf(" %10.4lf ",FinalPosition[i]);
    }
    printf("\n");
    print_info("Smooth stop finish in %llu us", tt);
    InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;
}

void ReflexxesSmoothStop_simulate(   RMLVelocityInputParameters &InputState, 
                            double SynTime)
{
    tm_msgs::SetVelocity vel_srv;

    double blend = 0, time_s;
    std::vector<double> vec;

    ReflexxesAPI *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags Flags;
    int ResultValue = 0;
    bool pass = true;
    struct timeval tm1,tm2, tm3, tm4;

    RML = new ReflexxesAPI(6, CYCLE_TIME_IN_SECONDS);
    IP = new RMLVelocityInputParameters(6);
    OP = new RMLVelocityOutputParameters(6);
    *IP = InputState;


    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library*/

    for (int i = 0; i < 6; ++i)
    {
        IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
        IP->MaxAccelerationVector->VecData[i] = 0.5*40;
        IP->TargetVelocityVector->VecData[i] = 0.0;
        if(IP->CurrentVelocityVector->VecData[i] != 0.0)    
            IP->SelectionVector->VecData[i] = true;
        else
            IP->SelectionVector->VecData[i] = false;

    }
    IP->MinimumSynchronizationTime = SynTime;

    // ********************************************************************


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
        printf("Input values are INVALID!\n");

    Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;

    gettimeofday(&tm3, NULL);
    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

        ResultValue =  RML->RMLVelocity(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }
        vec = { OP->NewVelocityVector->VecData[0],
                OP->NewVelocityVector->VecData[1],
                OP->NewVelocityVector->VecData[2],
                OP->NewVelocityVector->VecData[3],
                OP->NewVelocityVector->VecData[4],
                OP->NewVelocityVector->VecData[5]};

        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = vec;
        //vel_client.call(vel_srv);

        for (int i = 0; i < 6; ++i)
            printf("%10.4lf ", OP->NewPositionVector->VecData[i]);

        printf(" | spd: ");

        for (int i = 0; i < 6; ++i)
            printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
        
        printf("\n");

        std_msgs::Float64MultiArray OutputJoints;
        OutputJoints.data.resize(6);
        for(int i = 0; i < 6; i++){
            OutputJoints.data[i] = OP->NewPositionVector->VecData[i];
        }
        Joints_pub.publish(OutputJoints);

        //**********************

        *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
        *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
        *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);   
        if((24940 - time_compensation) < 1)
            usleep(100);
        else
            usleep(24940 - time_compensation);         

        // The area execution in 25ms real time sharp
        //********************************************************
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    std::vector<double> FinalPosition(6);
    // time_s = TR.interface->stateRT->getQAct(FinalPosition);
    printf("=============== Final state of Smooth Stop =========================\n");
    // printf("[ %lf ]  ", time_s);

    FinalPosition[0] = Virtual_robot_joint_angle[0];
    FinalPosition[1] = Virtual_robot_joint_angle[1];
    FinalPosition[2] = Virtual_robot_joint_angle[2];
    FinalPosition[3] = Virtual_robot_joint_angle[3];
    FinalPosition[4] = Virtual_robot_joint_angle[4];
    FinalPosition[5] = Virtual_robot_joint_angle[5];
    for (int i = 0; i < 6; ++i){
        printf(" %10.4lf ",FinalPosition[i]);
    }
    printf("\n");
    print_info("Smooth stop finish in %llu us", tt);
    InputState = *IP;

    delete  RML;
    delete  IP;
    delete  OP;
}

//  ********************************************************************/
//  fn        : ReflexxesVelocityRun()  -- For p2p motion
//  brief     : Use RML API to execute given velocity in simulation.  
//  param[in] : &InputState, Current State of robot under RML.
//  param[in] : TargetVelocity, The velocity when reach target position.
//  param[in] : SynTime, The time for execute the trajectory.
//  param[out]: 1:pass, 2:reach goal position, 0:smooth stop
//  ********************************************************************
int ReflexxesVelocityRun(   RMLVelocityInputParameters &InputState, 
                            std::vector<double> TargetVelocity, 
                            std::vector<double> TargetPosition,
                            std::vector<double> EffVelocity,
                            double SynTime)
{
    tm_msgs::SetVelocity vel_srv;

    ReflexxesAPI *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags Flags;
    int ResultValue = 0;
    int pass = PASS;
    double DistanceToGoal[3];
    double DegreeToGoal[3];
    std::vector<double> VelocityCommand(6);
    double blend = 0;
    
    double *T = new double [16];
    double *q = new double [6];

    tm_reflexxes::initTermios(1);

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP  = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    OP  = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);
    *IP = InputState;


    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP->MaxJerkVector->VecData[i]         = 100; //RMLTypeII not using, needed for validity
        IP->MaxAccelerationVector->VecData[i] = 0.5*40;
        IP->TargetVelocityVector->VecData[i]  = TargetVelocity[i];

        if(IP->CurrentVelocityVector->VecData[i] != TargetVelocity[i])    
            IP->SelectionVector->VecData[i] = true;
        else
            IP->SelectionVector->VecData[i] = false;
    }
    IP->MinimumSynchronizationTime = SynTime;

    // ********************************************************************


    if (IP->CheckForValidity())
    {
        //printf("Input values are valid!\n");
    }
    else
    {
        printf("Input values are INVALID!\n");
    }
    Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;
    


    struct timeval tm1,tm2, tm3, tm4;

    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

        ResultValue =  RML->RMLVelocity(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }

        VelocityCommand = { OP->NewVelocityVector->VecData[0],
                            OP->NewVelocityVector->VecData[1],
                            OP->NewVelocityVector->VecData[2],
                            OP->NewVelocityVector->VecData[3],
                            OP->NewVelocityVector->VecData[4],
                            OP->NewVelocityVector->VecData[5]};
        
        // TR.setMoveJointSpeedabs(VelocityCommand, blend);
        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = VelocityCommand;

        std_msgs::Float64MultiArray OutputJoints;
        OutputJoints.data.resize(6);

        for (int i = 0; i < NUMBER_OF_DOFS; ++i){
            q[i] = OP->NewPositionVector->VecData[i];
            OutputJoints.data[i] = q[i];
        }
        Joints_pub.publish(OutputJoints);

        // char outmsg_tag[200];
        // sprintf(outmsg_tag,"%.7f %.7f %.7f %.7f %.7f %.7f",OutputJoints.data[0]*RAD2DEG, OutputJoints.data[1]*RAD2DEG, OutputJoints.data[2]*RAD2DEG, OutputJoints.data[3]*RAD2DEG, OutputJoints.data[4]*RAD2DEG, OutputJoints.data[5]*RAD2DEG);
        // real_joints << outmsg_tag << endl;

        if (vel_client.call(vel_srv))                             
        {
            if (vel_srv.response.ok){
                
            } 
                // ROS_INFO_STREAM("SetPositions to robot");
            else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
        }
        else
        {
            ROS_ERROR_STREAM("Error SetPositions to robot, Q");
            // return 1;
        }

        //***************************************************************
        // Print out commands
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        //ROS_INFO("tool0 XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

        //***************************************************************

        for (int i = 0; i < 3; ++i)
        {
            if(abs(EffVelocity[i]) == 0)
                DistanceToGoal[i] = 0;
            else
                DistanceToGoal[i] = abs(T[i*4+3] -TargetPosition[i]);
        }

        tf::Matrix3x3 mout(T[0], T[4], T[8], T[1], T[5], T[9], T[2], T[6], T[10]);
        std::vector<double> out(3);
        mout.getRPY(out[0], out[1], out[2]);

        tf::Quaternion q_goal(TargetPosition[3], TargetPosition[4], TargetPosition[5], TargetPosition[6]);
        tf::Matrix3x3 Rot_out(q_goal);
        std::vector<double> Goal(3);
        Rot_out.getRPY(Goal[0], Goal[1], Goal[2]);
        // ROS_WARN("Degree: %10.4lf %10.4lf %10.4lf Target: %10.4lf %10.4lf %10.4lf", out[0]*RAD2DEG, out[1]*RAD2DEG, out[2]*RAD2DEG, Goal[0]*RAD2DEG, Goal[1]*RAD2DEG, Goal[2]*RAD2DEG);
        
        for (int i = 0; i < 3; ++i)
        {
            // if(abs(EffVelocity[i+3]) == 0)
            //     DegreeToGoal[i] = 0;
            // else{
                DegreeToGoal[i] = abs((out[i]) -(Goal[i]));
                if(DegreeToGoal[i] > 180*DEG2RAD)
                    DegreeToGoal[i] = abs(DegreeToGoal[i]-360*DEG2RAD);
            //}
        }
        // ROS_WARN("DistanceToGoal: %10.4lf %10.4lf %10.4lf DegreeToGoal: %10.4lf %10.4lf %10.4lf", DistanceToGoal[0], DistanceToGoal[1], DistanceToGoal[2], DegreeToGoal[0], DegreeToGoal[1], DegreeToGoal[2]);

        if( (DistanceToGoal[0] < REPEATABILITY) && (DistanceToGoal[1] < REPEATABILITY) && (DistanceToGoal[2] < REPEATABILITY))// && (DegreeToGoal[0] < DegreeThreshold) && (DegreeToGoal[1] < DegreeThreshold) && (DegreeToGoal[2] < DegreeThreshold))
        {
            // ROS_ERROR("DistanceToGoal: %10.4lf %10.4lf %10.4lf", DistanceToGoal[0], DistanceToGoal[1], DistanceToGoal[2]);
            ROS_ERROR("DistanceToGoal: %10.4lf %10.4lf %10.4lf DegreeToGoal: %10.4lf %10.4lf %10.4lf", DistanceToGoal[0], DistanceToGoal[1], DistanceToGoal[2], DegreeToGoal[0]*RAD2DEG, DegreeToGoal[1]*RAD2DEG, DegreeToGoal[2]*RAD2DEG);

            char outmsg_tag[200];
            sprintf(outmsg_tag,"%.7f %.7f %.7f %.7f %.7f %.7f",DistanceToGoal[0], DistanceToGoal[1], DistanceToGoal[2], DegreeToGoal[0]*RAD2DEG, DegreeToGoal[1]*RAD2DEG, DegreeToGoal[2]*RAD2DEG);
            distance_error << outmsg_tag << endl;
            /*
            char outmsg_real[200];
            sprintf(outmsg_real,"%.7f %.7f %.7f %.7f %.7f %.7f",OutputJoints.data[0]*RAD2DEG, OutputJoints.data[1]*RAD2DEG, OutputJoints.data[2]*RAD2DEG, OutputJoints.data[3]*RAD2DEG, OutputJoints.data[4]*RAD2DEG, OutputJoints.data[5]*RAD2DEG);
            real_joints << outmsg_real << endl;
            */
            pass = REACH;
            break;
        }

        if(!CheckJointLimit(q))
        {
            pass = STOP;
            break; 
        }

        if (tm_reflexxes::kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                ROS_WARN("Smooth Stop Activate...");
                ReflexxesSmoothStop(*IP, 0.25);
                pass = STOP;
                break;
            }
        }

        *IP->CurrentPositionVector     =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector     =  *OP->NewVelocityVector;
        *IP->CurrentAccelerationVector =  *OP->NewAccelerationVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        // usleep(24940 - time_compensation);  
        if((24940 - time_compensation) < 1)
            usleep(100);
        else
            usleep(24940 - time_compensation);

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

//    ROS_INFO("========== Final state velocity based position =============");
//    ROS_INFO("pass = %d  tool0 : %10.4lf %10.4lf %10.4lf", pass, T[3], T[7], T[11]);
//    ROS_INFO("Joint position  : %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", q[0]*RAD2DEG,q[1]*RAD2DEG,q[2]*RAD2DEG,q[3]*RAD2DEG,q[4]*RAD2DEG,q[5]*RAD2DEG);
//    ROS_INFO("Finished in %llu us", tt);

    tm_reflexxes::resetTermios();

    InputState = *IP;
    
    delete  RML;
    delete  IP;
    delete  OP;
    delete [] T;
    delete [] q;

    return pass;
}

int ReflexxesVelocityRun_simulate(   RMLVelocityInputParameters &InputState, 
                            std::vector<double> TargetVelocity, 
                            std::vector<double> TargetPosition,
                            std::vector<double> EffVelocity,
                            double SynTime)
{
    tm_msgs::SetVelocity vel_srv;

    ReflexxesAPI *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags Flags;
    int ResultValue = 0;
    int pass = PASS;
    double DistanceToGoal[3];
    double DegreeToGoal[3];
    std::vector<double> VelocityCommand(6);
    double blend = 0;
    
    double *T = new double [16];
    double *q = new double [6];

    tm_reflexxes::initTermios(1);

    RML = new ReflexxesAPI(6, CYCLE_TIME_IN_SECONDS);
    IP  = new RMLVelocityInputParameters(6);
    OP  = new RMLVelocityOutputParameters(6);
    *IP = InputState;

    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library
    for (int i = 0; i < 6; ++i)
    {
        IP->MaxJerkVector->VecData[i]         = 100; //RMLTypeII not using, needed for validity
        IP->MaxAccelerationVector->VecData[i] = 0.5*40;
        IP->TargetVelocityVector->VecData[i]  = TargetVelocity[i];

        if(IP->CurrentVelocityVector->VecData[i] != TargetVelocity[i])    
            IP->SelectionVector->VecData[i] = true;
        else
            IP->SelectionVector->VecData[i] = false;
    }
    IP->MinimumSynchronizationTime = SynTime;

    // ********************************************************************

    if (IP->CheckForValidity())
    {
        //printf("Input values are valid!\n");
    }
    else
    {
        printf("Input values are INVALID!\n");
    }
    Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;
    


    struct timeval tm1,tm2, tm3, tm4;

    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp
        gettimeofday(&tm1, NULL); 

        ResultValue =  RML->RMLVelocity(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }

        VelocityCommand = { OP->NewVelocityVector->VecData[0],
                            OP->NewVelocityVector->VecData[1],
                            OP->NewVelocityVector->VecData[2],
                            OP->NewVelocityVector->VecData[3],
                            OP->NewVelocityVector->VecData[4],
                            OP->NewVelocityVector->VecData[5]};

        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = VelocityCommand;
        //***************************************************************
        // Print out commands
        std_msgs::Float64MultiArray OutputJoints;
        OutputJoints.data.resize(6);
        for (int i = 0; i < 6; ++i){
            q[i] = OP->NewPositionVector->VecData[i];
            OutputJoints.data[i] = q[i];
        }
        Joints_pub.publish(OutputJoints);

        // if(stopwrite == true)
        // {
        //     char outmsg_tag[200];
        //     sprintf(outmsg_tag,"%.7f %.7f %.7f %.7f %.7f %.7f",OutputJoints.data[0]*RAD2DEG, OutputJoints.data[1]*RAD2DEG, OutputJoints.data[2]*RAD2DEG, OutputJoints.data[3]*RAD2DEG, OutputJoints.data[4]*RAD2DEG, OutputJoints.data[5]*RAD2DEG);
        //     virtual_joints << outmsg_tag << endl;
        // }

        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        //ROS_INFO("tool0 XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

        //***************************************************************

        for (int i = 0; i < 3; ++i)
        {
            if(abs(EffVelocity[i]) == 0)
                DistanceToGoal[i] = 0;
            else
                DistanceToGoal[i] = abs(T[i*4+3] -TargetPosition[i]);
        }

        tf::Matrix3x3 mout(T[0], T[4], T[8], T[1], T[5], T[9], T[2], T[6], T[10]);
        std::vector<double> out(3);
        mout.getRPY(out[0], out[1], out[2]);

        tf::Quaternion q_goal(TargetPosition[3], TargetPosition[4], TargetPosition[5], TargetPosition[6]);
        tf::Matrix3x3 Rot_out(q_goal);
        std::vector<double> Goal(3);
        Rot_out.getRPY(Goal[0], Goal[1], Goal[2]);
        // ROS_WARN("Degree: %10.4lf %10.4lf %10.4lf Target: %10.4lf %10.4lf %10.4lf", out[0]*RAD2DEG, out[1]*RAD2DEG, out[2]*RAD2DEG, Goal[0]*RAD2DEG, Goal[1]*RAD2DEG, Goal[2]*RAD2DEG);
        
        for (int i = 0; i < 3; ++i)
        {
            // if(abs(EffVelocity[i+3]) == 0)
            //     DegreeToGoal[i] = 0;
            // else{
                DegreeToGoal[i] = abs((out[i]) -(Goal[i]));
                if(DegreeToGoal[i] > 180*DEG2RAD)
                    DegreeToGoal[i] = abs(DegreeToGoal[i]-360*DEG2RAD);
            //}
        }
        // ROS_WARN("DistanceToGoal: %10.4lf %10.4lf %10.4lf DegreeToGoal: %10.4lf %10.4lf %10.4lf", DistanceToGoal[0], DistanceToGoal[1], DistanceToGoal[2], DegreeToGoal[0], DegreeToGoal[1], DegreeToGoal[2]);

        if((DistanceToGoal[0] < REPEATABILITY) && (DistanceToGoal[1] < REPEATABILITY) && (DistanceToGoal[2] < REPEATABILITY))// && (DegreeToGoal[0] < DegreeThreshold) && (DegreeToGoal[1] < DegreeThreshold) && (DegreeToGoal[2] < DegreeThreshold))
        {
            ROS_ERROR("DistanceToGoal: %10.4lf %10.4lf %10.4lf DegreeToGoal: %10.4lf %10.4lf %10.4lf", DistanceToGoal[0], DistanceToGoal[1], DistanceToGoal[2], DegreeToGoal[0]*RAD2DEG, DegreeToGoal[1]*RAD2DEG, DegreeToGoal[2]*RAD2DEG);
            
            // char outmsg_tag[200];
            // sprintf(outmsg_tag,"%.7f %.7f %.7f",DistanceToGoal[0], DistanceToGoal[1], DistanceToGoal[2]);
            // distance_error << outmsg_tag << endl;
            /*
            if(stopwrite == true)
            {
                char outmsg_virtual[200];
                sprintf(outmsg_virtual,"%.7f %.7f %.7f %.7f %.7f %.7f",OutputJoints.data[0]*RAD2DEG, OutputJoints.data[1]*RAD2DEG, OutputJoints.data[2]*RAD2DEG, OutputJoints.data[3]*RAD2DEG, OutputJoints.data[4]*RAD2DEG, OutputJoints.data[5]*RAD2DEG);
                virtual_joints << outmsg_virtual << endl;
            }
            */
            pass = REACH;
            break;
        }

        if(!CheckJointLimit(q))
        {
            pass = STOP;
            break; 
        }

        if (tm_reflexxes::kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                ROS_WARN("Smooth Stop Activate...");
                ReflexxesSmoothStop_simulate(*IP, 0.25);
                pass = STOP;
                break;
            }
        }

        *IP->CurrentPositionVector     =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector     =  *OP->NewVelocityVector;
        *IP->CurrentAccelerationVector =  *OP->NewAccelerationVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        // usleep(24940 - time_compensation);  
        if((24940 - time_compensation) < 1)
            usleep(100);
        else
            usleep(24940 - time_compensation);

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

//    ROS_INFO("========== Final state velocity based position =============");
//    ROS_INFO("pass = %d  tool0 : %10.4lf %10.4lf %10.4lf", pass, T[3], T[7], T[11]);
//    ROS_INFO("Joint position  : %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", q[0]*RAD2DEG,q[1]*RAD2DEG,q[2]*RAD2DEG,q[3]*RAD2DEG,q[4]*RAD2DEG,q[5]*RAD2DEG);
//    ROS_INFO("Finished in %llu us", tt);

    tm_reflexxes::resetTermios();

    InputState = *IP;
    
    delete  RML;
    delete  IP;
    delete  OP;
    delete [] T;
    delete [] q;

    return pass;
}

bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ)
{
    Eigen::VectorXd sigma;  //vector of singular values
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

    sigma = svd.singularValues();
    int m = sigma.rows();

    for (int i = 0; i < m ; ++i)
    {
        if(sigma(i) > EPS)
            sigma(i) = 1.0/ sigma(i);
        else
            sigma(i) = 0.0;
    }

    invJ = svd.matrixV() * sigma.asDiagonal() * svd.matrixU().transpose();

    return true;
}

bool OnlineVFCGeneration(   Eigen::Vector3d &TaskVelocity,
                            std::vector<double> ConstrainPoint, 
                            double *q) 
{
    double Vmax_cc = 1;
    double Survelliance_cc = 2/0.27;  //dangerous zone = 0.27m
    double ShapingFactor_cc = 8;
    double *T3 = new double [16];
    double ConstrainedPlane_z  = 0.0;    //table
    double dis_constrain = 0;
    double CartesianInfluence;
    std::vector<double> ConstrainedPoint(3); //object 0.359, 0.1115, 0.1646

    ConstrainedPoint = g_constraint_position;//g_obstacle_position;

    tm_jacobian::Forward_Kinematics_3(q,T3,-0.15); //-0.15
    Eigen::Vector3d Constrain2TaskJoint;

    Constrain2TaskJoint <<  T3[3] -ConstrainedPoint[0],
                            T3[7] -ConstrainedPoint[1],
                            T3[11]-ConstrainedPoint[2];    // for z plane : table
    
    double Distance2ConstrainedPoint = sqrt(Constrain2TaskJoint.dot(Constrain2TaskJoint));

    if(Distance2ConstrainedPoint > T3[11]) //table
    {
        dis_constrain = T3[11];
        CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));
        TaskVelocity << 0., 0., CartesianInfluence;
//        ROS_WARN("Constrain : table");
    }
    else 
    {
        Survelliance_cc = 2/0.37;
        dis_constrain = Distance2ConstrainedPoint;
        CartesianInfluence  = Vmax_cc / (1 + exp((dis_constrain*Survelliance_cc-1)*ShapingFactor_cc));
        TaskVelocity = CartesianInfluence*(Constrain2TaskJoint/dis_constrain);
//        ROS_WARN("Constrain : point");
    }

//    ROS_INFO("Distance table     : %10.4lf",T3[11]);
//    ROS_INFO("Distance point     : %10.4lf",Distance2ConstrainedPoint);
//    ROS_INFO("joint 3 position   : %10.4lf %10.4lf %10.4lf [%10.4lf]",T3[3],T3[7],T3[11], CartesianInfluence);
//    ROS_INFO("VirtualForce       : %10.4lf %10.4lf %10.4lf ",TaskVelocity(0),TaskVelocity(1),TaskVelocity(2));

    delete [] T3;
    return true;
}

bool GetQdfromVirtualForceConstrain(std::vector<double> CurrentPosition,
                                    std::vector<double> EFF_Velocity, 
                                    std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6, 1> JointSpeed;
    Eigen::Matrix<double, 3, 1> EFFSpeed;

    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::Vector3d VirtualForce;
    Eigen::Vector3d Constrained_Velocity;

    Eigen::Vector2d qd12_original;
    std::vector<double> ConstrainedPoint(3);
    double *q_now = new double [6];
    
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        q_now[i] = CurrentPosition[i];

    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    qd12_original(0) = qd[0];
    qd12_original(1) = qd[1];

    if(OnlineVFCGeneration(VirtualForce,ConstrainedPoint,q_now)) //constrain generated
    {
        Eigen::Matrix<double, 3, 4> Jacobian_3456     = Jacobian_123456.block<3,4>(0,2);
        Eigen::Matrix<double, 3, 2> Jacobian_12       = Jacobian_123456.block<3,2>(0,0);
        Eigen::Matrix<double, 3, 2> TaskJacobian_3    = tm_jacobian::Forward_Linear_Jacobian_3(q);
        Eigen::Vector3d OriginalForce = TaskJacobian_3* qd12_original;

        Eigen::MatrixXd TaskJacobian_3_inv;
        if(!pinv_SVD(TaskJacobian_3, TaskJacobian_3_inv))
            return false;

        Eigen::Vector3d temp;
        temp << abs(OriginalForce(0))*VirtualForce(0),
                abs(OriginalForce(1))*VirtualForce(1),
                abs(OriginalForce(2))*VirtualForce(2);

        for (int i = 0; i < 3; ++i)
        {
            if(temp(i)*OriginalForce(i) < 0)
                Constrained_Velocity(i) = OriginalForce(i) + temp(i);
            else
                Constrained_Velocity(i) = OriginalForce(i);
        }
        //Eigen::Vector3d Constrained_Velocity = OriginalForce + temp;

//        ROS_INFO("OriginalForce      : %10.4lf %10.4lf %10.4lf ",OriginalForce(0),OriginalForce(1),OriginalForce(2));
//        ROS_INFO("Constrained_Velocty: %10.4lf %10.4lf %10.4lf ",Constrained_Velocity(0),Constrained_Velocity(1),Constrained_Velocity(2));

        Eigen::VectorXd qd_12 = TaskJacobian_3_inv * Constrained_Velocity;
        Eigen::Matrix<double, 3, 1> linear_velocity_temp = EFFSpeed - Jacobian_12*qd_12;
        Eigen::MatrixXd jacobian_3456_inv;
        if(!pinv_SVD(Jacobian_3456,jacobian_3456_inv))
            return false;
        Eigen::VectorXd qd_3456 = jacobian_3456_inv*linear_velocity_temp;

        qd[0] = qd_12(0);
        qd[1] = qd_12(1);
        qd[2] = qd_3456(0);
        qd[3] = qd_3456(1);
        qd[4] = qd_3456(2);
        qd[5] = qd_3456(3);
    
//        ROS_WARN("qd_12   :     %10.4lf %10.4lf",qd_12(0)*RAD2DEG,qd_12(1)*RAD2DEG);
//        ROS_WARN("qd_3456 :     %10.4lf %10.4lf %10.4lf %10.4lf",qd_3456(0)*RAD2DEG,qd_3456(1)*RAD2DEG,qd_3456(2)*RAD2DEG,qd_3456(3)*RAD2DEG);
    }
    
//    ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);

    /*if(CheckVelocityLimit(qd,ScalingFactor))
        return true;
    else
    {
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            qd[i] = qd[i]/ScalingFactor;
        }
        ROS_WARN("Scaled qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);
        return CheckVelocityLimit(qd,ScalingFactor);
    }*/
}

bool GetQdfromLinearJacobian(   std::vector<double> CurrentPosition,
                                std::vector<double> EFF_Velocity, 
                                std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6,1> JointSpeed;
    Eigen::Matrix<double, 5, 1> EFFSpeed;

    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2],    EFF_Velocity[3],    EFF_Velocity[4];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q, GRIPPER_LENGTH);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 5, 6> Jacobian_12345   = Geometry_Jacobian.block<5,6>(0,0);

    Eigen::MatrixXd jacobian_12345_inv;
    pinv_SVD(Jacobian_12345,jacobian_12345_inv);
    JointSpeed = jacobian_12345_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    //ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);
}

bool GetQdfromLinearJacobian_free456(std::vector<double> CurrentPosition,
                                std::vector<double> EFF_Velocity, 
                                std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6,1> JointSpeed;
    Eigen::Matrix<double, 3, 1> EFFSpeed;

    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q, GRIPPER_LENGTH);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_12345   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::MatrixXd jacobian_12345_inv;
    pinv_SVD(Jacobian_12345,jacobian_12345_inv);
    JointSpeed = jacobian_12345_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    //ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);
}

bool GetQdfromLinearJacobian_fix456(   std::vector<double> CurrentPosition,
                                std::vector<double> EFF_Velocity, 
                                std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6,1> JointSpeed;
    Eigen::Matrix<double, 6, 1> EFFSpeed;

    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2],    EFF_Velocity[3],    EFF_Velocity[4],    EFF_Velocity[5];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q, GRIPPER_LENGTH);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 6, 6> Jacobian_12345   = Geometry_Jacobian.block<6,6>(0,0);

    Eigen::MatrixXd jacobian_12345_inv;
    pinv_SVD(Jacobian_12345,jacobian_12345_inv);
    JointSpeed = jacobian_12345_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    //ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);
}

bool GetQdfromLinearJacobian3(   std::vector<double> CurrentPosition,
                                std::vector<double> EFF_Velocity, 
                                std::vector<double>& qd)
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6,1> JointSpeed_ori;
    Eigen::Matrix<double , 3,1> JointSpeed_456;
    Eigen::Matrix<double, 3, 1> EFFSpeed;
    Eigen::Matrix<double, 3, 1> EFFSpeed12;

    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q, GRIPPER_LENGTH);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123   = Geometry_Jacobian.block<3,6>(0,0);
    Eigen::Matrix<double, 3, 2> Jacobian_12   = Geometry_Jacobian.block<3,2>(0,0);
    Eigen::Matrix<double, 3, 3> Jacobian_456   = Geometry_Jacobian.block<3,3>(0,3);

    Eigen::MatrixXd jacobian_123_inv;
    pinv_SVD(Jacobian_123,jacobian_123_inv);
    JointSpeed_ori = jacobian_123_inv * EFFSpeed;

    Eigen::Matrix<double , 2,1> qd12_ori;
    qd12_ori << JointSpeed_ori(0), JointSpeed_ori(1);
    EFFSpeed12 = Jacobian_12 * qd12_ori;

    Eigen::Matrix<double, 3, 1> EFF_velocity_diff = EFFSpeed - EFFSpeed12;
    Eigen::MatrixXd Jacobian_456_inv;
    pinv_SVD(Jacobian_456,Jacobian_456_inv);
    JointSpeed_456 = Jacobian_456_inv * EFF_velocity_diff;

    qd[0] = JointSpeed_ori(0);
    qd[1] = JointSpeed_ori(1);
    qd[2] = 0;
    qd[3] = JointSpeed_456(0);
    qd[4] = JointSpeed_456(1);
    qd[5] = JointSpeed_456(2);


    // for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    //     qd[i] = JointSpeed(i);

    //ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);
}

bool GetQdfromInverseJacobian(std::vector<double> CurrentPosition,std::vector<double> EFF_Velocity, std::vector<double>& qd)
{

    Eigen::Matrix<float, 6, 1> home,q;
    home << 0, -PI*0.5, 0, PI*0.5, 0, 0;
    Eigen::Matrix<float,6,1> effspd,jointspd;

    home   << 0, -PI*0.5, 0, PI*0.5, 0, 0;
    effspd << EFF_Velocity[0], EFF_Velocity[1], EFF_Velocity[2], EFF_Velocity[3], EFF_Velocity[4], EFF_Velocity[5];
    q      << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    q += home;

    Eigen::Matrix<float, 6, 6> Inverse_Jacobian = tm_jacobian::Inverse_Jacobian(q);
    jointspd = Inverse_Jacobian*effspd;
    //cout << ">>>> Inverse jacobian" << endl;
    //tm_jacobian::printMatrix(Inverse_Jacobian);

    tm_jacobian::Matrix2DoubleVector(jointspd,qd);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);
}

double default_RX = 3.13;
double default_RY = 0;

bool OnlineAttractiveForceGeneration(   std::vector<double>& action_qd,
                                        double *TCP_position,             // robot TCP position       : for min_distance
                                        double *q,                        // robot joint position     : for jacobian
                                        std::vector<double> GoalPoint,    // eff to obstacle goal     : for att
                                        std::vector<double> &EFF_Velocity,// eff velocity             : for att
                                        std::vector<double> PassThrough_Velocity) 
{
    bool   succeed = false;
    double Vmax_att = VMAX_ATT_GPR;
    double Survelliance_att = 0.15;  //0.15
    double ShapingFactor_att = 10;  //5

    Eigen::Vector3d Eff2Goal;
    Eff2Goal << GoalPoint[0]-TCP_position[3], GoalPoint[1]-TCP_position[7], GoalPoint[2]-TCP_position[11];
    double dis_goal = sqrt(pow(Eff2Goal(0),2) + pow(Eff2Goal(1),2) + pow(Eff2Goal(2),2));

    tf::Matrix3x3 mout(TCP_position[0], TCP_position[4], TCP_position[8], TCP_position[1], TCP_position[5], TCP_position[9], TCP_position[2], TCP_position[6], TCP_position[10]);
    Eigen::Vector3d now;
    mout.getRPY(now(0), now(1), now(2));

    tf::Quaternion q2(GoalPoint[3], GoalPoint[4], GoalPoint[5], GoalPoint[6]);
    tf::Matrix3x3 Rout(q2);
    Eigen::Vector3d Goal;
    Rout.getRPY(Goal(0), Goal(1), Goal(2));

    Eigen::Vector3d Deg2Goal;
    for(int i = 0; i < 3; i++){
        Deg2Goal(i) = Goal(i) - now(i);

        if(Deg2Goal(i) > 180*DEG2RAD)
            Deg2Goal(i) = Deg2Goal(i)-360*DEG2RAD;
        else if(Deg2Goal(i) < -180*DEG2RAD)
            Deg2Goal(i) = Deg2Goal(i)+360*DEG2RAD;
    }
    // ROS_INFO("DEG2GOAL: %7f %7f %7f", Deg2Goal(0)*RAD2DEG, Deg2Goal(1)*RAD2DEG, Deg2Goal(2)*RAD2DEG);

    double deg_goal = sqrt( pow(Deg2Goal(0),2) + pow(Deg2Goal(1),2) + pow(Deg2Goal(2),2));
    // dis_goal /= STEPSIZE;
    
    Eigen::Vector3d AttractiveVecotor = Eff2Goal/dis_goal;
    Eigen::Vector3d AttractiveVecotor_deg = Deg2Goal/deg_goal;
    
    double AttractiveForce = Vmax_att - Vmax_att*exp(-(dis_goal*ShapingFactor_att)/Survelliance_att);

    //double AttractiveForce_deg = Vmax_att - Vmax_att*exp(-(deg_goal*20)/Survelliance_att);

    double AttractiveForce_deg_x = 0.65*sin(Deg2Goal(0));
    double AttractiveForce_deg_y = 0.65*sin(Deg2Goal(1));
    double AttractiveForce_deg_z = 0.65*sin(Deg2Goal(2));

    Eigen::Vector3d AttractiveVelocity = AttractiveForce*AttractiveVecotor;

    double AttractiveVelocity_deg_x = -(abs(AttractiveForce_deg_x)*AttractiveVecotor_deg(0));
    double AttractiveVelocity_deg_y = abs(AttractiveForce_deg_y)*AttractiveVecotor_deg(1);
    double AttractiveVelocity_deg_z = abs(AttractiveForce_deg_z)*AttractiveVecotor_deg(2);

    /*** Pass Through Velocity ***/
    Eigen::Vector3d PS_vel_vec;
    PS_vel_vec << PassThrough_Velocity[0], PassThrough_Velocity[1], PassThrough_Velocity[2];
    double PS_vel_norm = sqrt( pow(PS_vel_vec(0), 2) + pow(PS_vel_vec(1), 2) + pow(PS_vel_vec(2), 2) );

    Eigen::Vector3d VelocitySmoothVector = PS_vel_vec/PS_vel_norm;

    double VelocitySmoothForce = Vmax_att*exp(-(dis_goal*ShapingFactor_att)/Survelliance_att);

    Eigen::Vector3d PathSmoothVelocity;
    if(PassThrough_Velocity[0] == 0 && PassThrough_Velocity[1] == 0 && PassThrough_Velocity[2] == 0)
        PathSmoothVelocity << 0, 0, 0;
    else
        PathSmoothVelocity = VelocitySmoothForce*VelocitySmoothVector; 

    // ROS_INFO("Path Smooth Velocity: %10.4lf, %10.4lf, %10.4lf ", PathSmoothVelocity(0), PathSmoothVelocity(1), PathSmoothVelocity(2));
    /*****************************/
    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    EFF_Velocity = {AttractiveVelocity(0)+PathSmoothVelocity(0), AttractiveVelocity(1)+PathSmoothVelocity(1), AttractiveVelocity(2)+PathSmoothVelocity(2), 0,0,0}; 
    // EFF_Velocity = {AttractiveVelocity(0)+PathSmoothVelocity(0), AttractiveVelocity(1)+PathSmoothVelocity(1), AttractiveVelocity(2)+PathSmoothVelocity(2), AttractiveVelocity_deg_x,AttractiveVelocity_deg_y, AttractiveVelocity_deg_z}; 
    // ROS_INFO("AttractiveVecotor_deg: x: %10.4lf y: %10.4lf z: %10.4lf",AttractiveVecotor_deg(0), AttractiveVecotor_deg(1), AttractiveVecotor_deg(2));
    // ROS_INFO("Attractive Velocity: x: %10.4lf y: %10.4lf z: %10.4lf",AttractiveVelocity_deg_x, AttractiveVelocity_deg_y, AttractiveVelocity_deg_z);
    double AttractiveScalar = sqrt(AttractiveVelocity.dot(AttractiveVelocity));

    succeed = GetQdfromLinearJacobian_fix456(CurrentPosition,EFF_Velocity,action_qd);

    return succeed;
}

bool OnlineRepulsiveForceGeneration(    std::vector<double>& Repulsive_qd,
                                        double *TCP_position,                   // robot TCP position       : for min_distance
                                        double *q,                              // robot joint position     : for jacobian
                                        std::vector<double> &EFF_Velocity)      // eff velocity             : for att
{
    bool succeed = false;
    double Vmax_rep = VMAX_REP;
    double Survelliance_rep = 2/DANGEROUS_ZONE;//0.2;
    double ShapingFactor_rep = 8;
    std::vector<double> ObstaclePoint(3),tool_ObstaclePoint(3); 
    
    ObstaclePoint = g_obstacle_position;
    tool_ObstaclePoint = tool_obstacle_position;

    Eigen::Vector3d Obstacle2Eff;
    Obstacle2Eff << TCP_position[3]-ObstaclePoint[0], TCP_position[7]-ObstaclePoint[1], TCP_position[11]-ObstaclePoint[2];
    // Obstacle2Eff << tool_ObstaclePoint[0]-ObstaclePoint[0], tool_ObstaclePoint[1]-ObstaclePoint[1], tool_ObstaclePoint[2]-ObstaclePoint[2];
    double dis_repuslive = sqrt(Obstacle2Eff.dot(Obstacle2Eff));
    Eigen::Vector3d RepulsiveVector   = Obstacle2Eff/dis_repuslive;

    double RepulsiveForce  = Vmax_rep / (1 + exp((dis_repuslive*Survelliance_rep-1)*ShapingFactor_rep));
    Eigen::Vector3d RepulsiveVelocity  = RepulsiveForce *RepulsiveVector;

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    EFF_Velocity = {RepulsiveVelocity(0), RepulsiveVelocity(1), RepulsiveVelocity(2), 0,0,0}; 
    //OS_INFO("ObstaclePoint      : %10.4lf %10.4lf %10.4lf",ObstaclePoint[0],ObstaclePoint[1],ObstaclePoint[2]);
//    ROS_WARN("Repulsive  Velocity: %10.4lf ", sqrt(RepulsiveVelocity.dot(RepulsiveVelocity)));
    
    succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    // succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    // succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,Repulsive_qd);   

    return succeed;

}

bool Direction(int index, int axis, std::vector<geometry_msgs::Pose> path)
{
    if(axis == 0){
        if((path[index-1].position.x - path[index].position.x) < 0 && (path[index].position.x - path[index+1].position.x) < 0)
            return true;
        else if((path[index-1].position.x - path[index].position.x) >= 0 && (path[index].position.x - path[index+1].position.x) >= 0)
            return true;
        else
            return false;
    }
    else if(axis == 1){
        if((path[index-1].position.y - path[index].position.y) < 0 && (path[index].position.y - path[index+1].position.y) < 0)
            return true;
        else if((path[index-1].position.y - path[index].position.y) >= 0 && (path[index].position.y - path[index+1].position.y) >= 0)
            return true;
        else
            return false;
    }
    else if(axis == 2){
        if((path[index-1].position.z - path[index].position.z) < 0 && (path[index].position.z - path[index+1].position.z) < 0)
            return true;
        else if((path[index-1].position.z - path[index].position.z) >= 0 && (path[index].position.z - path[index+1].position.z) >= 0)
            return true;
        else
            return false;
    }

}

bool Hexagon_Demo()
{
    
    bool run_succeed = true;
    int  pass = 0;
    double SynchronousTime = 1;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6), PassThrough_velocity;
    double *T = new double [16];
    double *q = new double [6];
    // std::vector<geometry_msgs::Pose> exec_path;
    int exec_index = 0;
    int state_curr = 1, state_prev = 1;

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    // while(get_init_path == false){
    //     ROS_WARN("Not Get Path !!!");
    //     sleep(1);
    // }
    // get_init_path == true;

    // TM5.interface->stateRT->getQAct(CurrentPosition);
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        CurrentPosition[i] = g_robot_joint_angle[i];
    }

    // printf("1:%10.4lf, 2:%10.4lf, 3:%10.4lf, 4:%10.4lf, 5%10.4lf, 6:%10.4lf\n", g_robot_joint_angle[0]*RAD2DEG, g_robot_joint_angle[1]*RAD2DEG, g_robot_joint_angle[2]*RAD2DEG, g_robot_joint_angle[3]*RAD2DEG, g_robot_joint_angle[4]*RAD2DEG, g_robot_joint_angle[5]*RAD2DEG);
    
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    //std::vector<double> InitialPosition = {0.00 * DEG2RAD,    -22.72 * DEG2RAD,    100.00 * DEG2RAD,    12.9 * DEG2RAD,    90.00 * DEG2RAD,    0.00 * DEG2RAD};
    //std::vector<double> FinalPosition = {53.0* DEG2RAD,    -4.5* DEG2RAD,    80.13* DEG2RAD,    13.9* DEG2RAD,    90.0* DEG2RAD,    50.0* DEG2RAD};

    //TargetVelocity = {0, 0, 0, 0, 0, 0};

    // if(!ReflexxesPositionRun(*IP_position, InitialPosition, TargetVelocity, 1))
    // {
    //     ROS_WARN("Smooth stop activate...");
    //     return 0;
    // }
    // TM5.interface->stateRT->getQAct(CurrentPosition);
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        CurrentPosition[i] = g_robot_joint_angle[i];
    }
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_velocity->CurrentPositionVector->VecData[i]     = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
        IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
    }

    PassThrough_velocity = {0, 0, 0};

    /*** APF Guidance Control ***/
    while(ros::ok()){
        // TM5.interface->stateRT->getQAct(CurrentPosition);
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            CurrentPosition[i] = g_robot_joint_angle[i];
        }
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
            q[i] = CurrentPosition[i];
        }
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

        // Pass through velocity decision
        if(exec_index == 0 || exec_index == static_path.size()-1){
            PassThrough_velocity = {0, 0, 0};
        }
        else{
            for(int i = 0; i < 3; i++){
                if(i == 0){
                    if(Direction(exec_index, i, static_path)){
                        PassThrough_velocity[i] = ((static_path[exec_index-1].position.x - static_path[exec_index].position.x) + (static_path[exec_index].position.x - static_path[exec_index+1].position.x));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].x - exec_path[exec_index].x) + (exec_path[exec_index].x - exec_path[exec_index+1].x));
                        PassThrough_velocity[i] *= -1;
                    }
                }
                else if(i == 1){
                    if(Direction(exec_index, i, static_path)){
                        PassThrough_velocity[i] = ((static_path[exec_index-1].position.y - static_path[exec_index].position.y) + (static_path[exec_index].position.y - static_path[exec_index+1].position.y));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].y - exec_path[exec_index].y) + (exec_path[exec_index].y - exec_path[exec_index+1].y));
                        PassThrough_velocity[i] *= -1;
                    }
                }
                else if(i == 2){
                    if(Direction(exec_index, i, static_path)){
                        PassThrough_velocity[i] = ((static_path[exec_index-1].position.z - static_path[exec_index].position.z) + (static_path[exec_index].position.z - static_path[exec_index+1].position.z));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].z - exec_path[exec_index].z) + (exec_path[exec_index].z - exec_path[exec_index+1].z));
                        PassThrough_velocity[i] *= -1;
                    }
                }

            }
        }

        // ROS_ERROR("Pose step x: %lf, Pose step y: %lf, Pose step z: %lf", pose_step.x, pose_step.y, pose_step.z);
        tf::Quaternion q3(static_path[exec_index].orientation.x, static_path[exec_index].orientation.y, static_path[exec_index].orientation.z, static_path[exec_index].orientation.w);
        tf::Matrix3x3 mout(q3);
        std::vector<double> out(3);
        mout.getRPY(out[0], out[1], out[2]);

        TargetPosition = {static_path[exec_index].position.x, static_path[exec_index].position.y, static_path[exec_index].position.z, static_path[exec_index].orientation.x, static_path[exec_index].orientation.y, static_path[exec_index].orientation.z, static_path[exec_index].orientation.w};
        ROS_WARN("TargetPosition: %lf, %lf, %lf, %lf, %lf, %lf", static_path[exec_index].position.x, static_path[exec_index].position.y, static_path[exec_index].position.z, out[0]*RAD2DEG, out[1]*RAD2DEG, out[2]*RAD2DEG);

        // TargetPosition = {exec_path[exec_index].x, exec_path[exec_index].y, exec_path[exec_index].z, 0, 0, 0};
        //ROS_WARN("Target x: %lf, y: %lf, z: %lf", exec_path[exec_index].x, exec_path[exec_index].y, exec_path[exec_index].z);

        /*** Robot State Assignment ***/
        // TM5.interface->stateRT->getQAct(CurrentPosition);
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            CurrentPosition[i] = g_robot_joint_angle[i];
        }
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
        }
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

        /*** Motion Generation ***/
        if(OnlineAttractiveForceGeneration(Attractive_qd, T, q, TargetPosition, TargetVelocity, PassThrough_velocity))
        {
            // if(OnlineRepulsiveForceGeneration(Repulsive_qd, T, q, RepulsiveVelocity))
            // {
                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    qd[i] = Attractive_qd[i];
                    // qd[i] = Attractive_qd[i] + Repulsive_qd[i];

                ROS_INFO("gripper : %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

                tf::Matrix3x3 mout(T[0], T[4], T[8], T[1], T[5], T[9], T[2], T[6], T[10]);
                std::vector<double> out(3);
                mout.getRPY(out[0], out[1], out[2]);
                char outmsg_tag[200];
                sprintf(outmsg_tag,"%.7f %.7f %.7f %.7f %.7f %.7f",T[3], T[7], T[11], out[0]*RAD2DEG, out[1]*RAD2DEG, out[2]*RAD2DEG);
                end_pose << outmsg_tag << endl;
                //ROS_ERROR_STREAM("Writing pose!");

                pass = ReflexxesVelocityRun(*IP_velocity, qd, TargetPosition, TargetVelocity ,0.2);

                if(pass == REACH)
                {

                    exec_index++;
                }
                else if(pass == STOP)
                    break;
                else{}
            // }
            // else
            // {
            //     ROS_WARN("Repulsive force generate fail, smooth stop activate...");
            //     tm_reflexxes::ReflexxesSmoothStop(TM5,*IP_velocity,0.25);
            //     break;
            // }
        }
        else
        {
            ROS_WARN("Attrative force generate fail, smooth stop activate...");
            ReflexxesSmoothStop(*IP_velocity,0.25);
            break;
        }

        if(exec_index == static_path.size()){    // Path finish
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
            // if(!ReflexxesPositionRun(*IP_position, FinalPosition, TargetVelocity, 1))
            // {
            //     ROS_WARN("Smooth stop activate...");
            //     return 0;
            // }
            // TM5.interface->stateRT->getQAct(CurrentPosition);
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                CurrentPosition[i] = g_robot_joint_angle[i];
            }
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            {
                IP_velocity->CurrentPositionVector->VecData[i]     = CurrentPosition[i];
                IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
                IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
            }

            ROS_WARN("Path finished, smooth stop activate...");
            start_cmd = 0;
            ReflexxesSmoothStop(*IP_velocity,0.25);
            break;
        }
    }

    ROS_WARN("Hexagon_Demo shutdown");
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "tm5_init";
    pub_robot_project.publish(msg);
    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q;

    return 0;
}

bool Hexagon_Demo_simulate()
{
    bool run_succeed = true;
    int  pass = 0;
    double SynchronousTime = 1;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6), PassThrough_velocity;
    double *T = new double [16];
    double *q = new double [6];

    int exec_index = 0;
    int state_curr = 1, state_prev = 1;

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(6);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(6);

    for (int i = 0; i < 6; ++i)
    {
        CurrentPosition[i] = Virtual_robot_joint_angle[i];
    }

    // printf("1: %10.4lf, 2: %10.4lf, 3: %10.4lf, 4: %10.4lf, 5 %10.4lf, 6: %10.4lf \n", Virtual_robot_joint_angle[0]*RAD2DEG, Virtual_robot_joint_angle[1]*RAD2DEG, Virtual_robot_joint_angle[2]*RAD2DEG, Virtual_robot_joint_angle[3]*RAD2DEG, Virtual_robot_joint_angle[4]*RAD2DEG, Virtual_robot_joint_angle[5]*RAD2DEG);


    for (int i = 0; i < 6; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    for (int i = 0; i < 6; ++i)
    {
        CurrentPosition[i] = Virtual_robot_joint_angle[i];
    }
    for (int i = 0; i < 6; ++i)
    {
        IP_velocity->CurrentPositionVector->VecData[i]     = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
        IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
    }

    PassThrough_velocity = {0, 0, 0};

    /*** APF Guidance Control ***/
    
    while(ros::ok()){
        // TM5.interface->stateRT->getQAct(CurrentPosition);
        for (int i = 0; i < 6; ++i)
        {
            CurrentPosition[i] = Virtual_robot_joint_angle[i];
        }
        for (int i = 0; i < 6; ++i)
        {
            IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
            q[i] = CurrentPosition[i];
        }
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        /*** Node Decision ***/

        ROS_WARN("Next pt : %lf, %lf, %lf, %lf, %lf, %lf, %lf", static_path_simulate[exec_index].position.x, static_path_simulate[exec_index].position.y, static_path_simulate[exec_index].position.z, static_path_simulate[exec_index].orientation.x, static_path_simulate[exec_index].orientation.y, static_path_simulate[exec_index].orientation.z, static_path_simulate[exec_index].orientation.w);

        
        // Pass through velocity decision
        if(exec_index == 0 || exec_index == static_path_simulate.size()-1){
            PassThrough_velocity = {0, 0, 0};
        }
        else{
            for(int i = 0; i < 3; i++){
                if(i == 0){
                    if(Direction(exec_index, i, static_path_simulate)){
                        PassThrough_velocity[i] = ((static_path_simulate[exec_index-1].position.x - static_path_simulate[exec_index].position.x) + (static_path_simulate[exec_index].position.x - static_path_simulate[exec_index+1].position.x));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].x - exec_path[exec_index].x) + (exec_path[exec_index].x - exec_path[exec_index+1].x));
                        PassThrough_velocity[i] *= -1;
                    }
                }
                else if(i == 1){
                    if(Direction(exec_index, i, static_path_simulate)){
                        PassThrough_velocity[i] = ((static_path_simulate[exec_index-1].position.y - static_path_simulate[exec_index].position.y) + (static_path_simulate[exec_index].position.y - static_path_simulate[exec_index+1].position.y));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].y - exec_path[exec_index].y) + (exec_path[exec_index].y - exec_path[exec_index+1].y));
                        PassThrough_velocity[i] *= -1;
                    }
                }
                else if(i == 2){
                    if(Direction(exec_index, i, static_path_simulate)){
                        PassThrough_velocity[i] = ((static_path_simulate[exec_index-1].position.z - static_path_simulate[exec_index].position.z) + (static_path_simulate[exec_index].position.z - static_path_simulate[exec_index+1].position.z));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].z - exec_path[exec_index].z) + (exec_path[exec_index].z - exec_path[exec_index+1].z));
                        PassThrough_velocity[i] *= -1;
                    }
                }

            }
        }

        // ROS_ERROR("Pose step x: %lf, Pose step y: %lf, Pose step z: %lf", pose_step.x, pose_step.y, pose_step.z);
        tf::Quaternion q3(static_path_simulate[exec_index].orientation.x, static_path_simulate[exec_index].orientation.y, static_path_simulate[exec_index].orientation.z, static_path_simulate[exec_index].orientation.w);
        tf::Matrix3x3 mout(q3);
        std::vector<double> out(3);
        mout.getRPY(out[0], out[1], out[2]);

        TargetPosition = {static_path_simulate[exec_index].position.x, static_path_simulate[exec_index].position.y, static_path_simulate[exec_index].position.z, static_path_simulate[exec_index].orientation.x, static_path_simulate[exec_index].orientation.y, static_path_simulate[exec_index].orientation.z, static_path_simulate[exec_index].orientation.w};
        ROS_WARN("TargetPosition: %lf, %lf, %lf, %lf, %lf, %lf", static_path_simulate[exec_index].position.x, static_path_simulate[exec_index].position.y, static_path_simulate[exec_index].position.z, out[0]*RAD2DEG, out[1]*RAD2DEG, out[2]*RAD2DEG);

        /*** Robot State Assignment ***/
        // TM5.interface->stateRT->getQAct(CurrentPosition);
        for (int i = 0; i < 6; ++i)
        {
            CurrentPosition[i] = Virtual_robot_joint_angle[i];
        }
        for (int i = 0; i < 6; ++i)
        {
            IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
        }
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

        if(exec_index == static_path_simulate.size()-1){
            stopwrite = false;
        }
        /*** Motion Generation ***/
        if(OnlineAttractiveForceGeneration(Attractive_qd, T, q, TargetPosition, TargetVelocity, PassThrough_velocity))
        {
            // if(OnlineRepulsiveForceGeneration(Repulsive_qd, T, q, RepulsiveVelocity))
            // {
                for (int i = 0; i < 6; ++i)
                    qd[i] = Attractive_qd[i];
                    // qd[i] = Attractive_qd[i] + Repulsive_qd[i];
                
                ROS_INFO("gripper : %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

                pass = ReflexxesVelocityRun_simulate(*IP_velocity, qd, TargetPosition, TargetVelocity ,0.1);
                //ROS_ERROR_STREAM("Writing pose!");
                if(pass == REACH)
                {
                    exec_index++;
                }
                else if(pass == STOP)
                    break;
                else{}
    
        }
        else
        {
            ROS_WARN("Attrative force generate fail, smooth stop activate...");
            ReflexxesSmoothStop_simulate(*IP_velocity,0.25);
            break;
        }

        if(exec_index == static_path_simulate.size()){    // Path finish
            for (int i = 0; i < 6; ++i)
            {
                CurrentPosition[i] = Virtual_robot_joint_angle[i];
            }
            for (int i = 0; i < 6; ++i)
            {
                IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
                IP_position->CurrentVelocityVector->VecData[i] = 0.0;
                IP_position->CurrentAccelerationVector->VecData[i] = 0.0;
            }

            for (int i = 0; i < 6; ++i)
            {
                CurrentPosition[i] = Virtual_robot_joint_angle[i];
            }
            for (int i = 0; i < 6; ++i)
            {
                IP_velocity->CurrentPositionVector->VecData[i]     = CurrentPosition[i];
                IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
                IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
            }

            ROS_WARN("Path finished, smooth stop activate...");
            ReflexxesSmoothStop_simulate(*IP_velocity,0.25);
            
            break;
        }
    }
    start_simulate = 0;
    get_robot_joints = false;
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "tm5_init";
    ros::Duration(1).sleep();
    pub_robot_project.publish(msg);
    ROS_WARN("Simulate shutdown");

    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q;

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

void distance_callback(const std_msgs::Float32::ConstPtr& distance)
{
    g_distance = distance->data;
    //ROS_INFO("minimum_distance = %10.3f",distance->data);
}

void position_callback(const geometry_msgs::PointStamped::ConstPtr& closest_pt)
{
    g_obstacle_position[0] = closest_pt->point.x;
    g_obstacle_position[1] = closest_pt->point.y;
    g_obstacle_position[2] = closest_pt->point.z;
    //ROS_INFO("x =  %10.3f, y =  %10.3f, z =  %10.3f", closest_pt->point.x, closest_pt->point.y, closest_pt->point.z);
}

void tool_position_callback(const geometry_msgs::PointStamped::ConstPtr& closest_pt)
{
    tool_obstacle_position[0] = closest_pt->point.x;
    tool_obstacle_position[1] = closest_pt->point.y;
    tool_obstacle_position[2] = closest_pt->point.z;
    //ROS_INFO("x =  %10.3f, y =  %10.3f, z =  %10.3f", closest_pt->point.x, closest_pt->point.y, closest_pt->point.z);
}


void velocity_callback(const geometry_msgs::Twist::ConstPtr& velocity)
{
    g_obstacle_velocity[0] = velocity->linear.x;
    g_obstacle_velocity[1] = velocity->linear.y;
    g_obstacle_velocity[2] = velocity->linear.z;
    //ROS_INFO("vx = %10.3f, vy = %10.3f, vz = %10.3f", velocity->linear.x, velocity->linear.y, velocity->linear.z);
}

void constraint_callback(const geometry_msgs::PointStamped::ConstPtr& closest_bpt)
{
    g_constraint_position[0] = closest_bpt->point.x;
    g_constraint_position[1] = closest_bpt->point.y;
    g_constraint_position[2] = closest_bpt->point.z;
}

void path_state_callback(const std_msgs::Int32::ConstPtr& msg)
{
    // g_dangerous_prev = g_dangerous;
    g_dangerous = msg->data;
}

void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  if(msg->joint_pos.size() == 6){
    g_robot_joint_angle[0] = msg->joint_pos[0];
    g_robot_joint_angle[1] = msg->joint_pos[1];
    g_robot_joint_angle[2] = msg->joint_pos[2];
    g_robot_joint_angle[3] = msg->joint_pos[3];
    g_robot_joint_angle[4] = msg->joint_pos[4];
    g_robot_joint_angle[5] = msg->joint_pos[5];
  }
}

void TMmsgCallback_simulate(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if(msg->data.size() == 6){
    for(int i = 0; i < 6; i++){
        if(msg->data[i] > 180){
            if(i == 1){
                Virtual_robot_joint_angle[i] = (msg->data[i]-360)*DEG2RAD;
            }
            else
                Virtual_robot_joint_angle[i] = -(msg->data[i]-360)*DEG2RAD;
        }
        else if(msg->data[i] < -180){
            if(i == 1){
                Virtual_robot_joint_angle[i] = (msg->data[i]+360)*DEG2RAD;
            }
            else
                Virtual_robot_joint_angle[i] = -(msg->data[i]+360)*DEG2RAD;
        }
        else{
            if(i == 1){
                Virtual_robot_joint_angle[i] = msg->data[i]*DEG2RAD;
            }
            else
                Virtual_robot_joint_angle[i] = -(msg->data[i]*DEG2RAD);
        } 
    }
    get_robot_joints = true;
  }
}

void drawPath(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "base";
    edge.header.stamp = ros::Time::now();
    edge.ns = "smoothPath";
    edge.id = 5;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = edge.scale.y = edge.scale.z = 0.006;

    edge.color.g = 1;
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}
/*
void showSmoothPath(std::vector<geometry_msgs::Pose> static_path)
{
    visualization_msgs::Marker v_start, v_end;
    v_start.type = v_end.type = visualization_msgs::Marker::POINTS;
    v_start.header.frame_id = v_end.header.frame_id = "base";
    v_start.header.stamp = v_end.header.stamp = ros::Time::now();
    v_start.ns = v_end.ns = "start/end vertices";
    v_start.id = 0;
    v_end.id = 1;
    v_start.action = v_end.action = visualization_msgs::Marker::ADD;

    v_start.color.a = 1.0f;
    v_start.color.g = 1.0f;
    v_start.scale.x = v_start.scale.y = v_start.scale.z = 0.03;
    v_end.scale.x = v_end.scale.y = v_end.scale.z = 0.03;


    v_end.color.a = 1.0f;
    v_end.color.r = 1.0f;

    geometry_msgs::Point next;
    geometry_msgs::Point curr;

    for (int i = 0; i < static_path.size() - 1; ++i) {
        curr = static_path[i];
        next = static_path[i + 1];
        drawPath(curr, next);
    }
    curr = static_path[0];
    next = static_path[static_path.size() - 1];
    v_start.points.push_back(curr);
    v_end.points.push_back(next);
    marker_pub.publish(v_start);
    marker_pub.publish(v_end);
    v_start.points.clear();
    v_end.points.clear();
}
*/
void get_path_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    static_path.clear();
    geometry_msgs::Pose pose;
    for(int i = 0; i < msg->poses.size(); i++)
    {
        pose.position.x = msg->poses[i].position.x;
        pose.position.y = msg->poses[i].position.y;
        pose.position.z = msg->poses[i].position.z;
        pose.orientation.x = msg->poses[i].orientation.x;
        pose.orientation.y = msg->poses[i].orientation.y;
        pose.orientation.z = msg->poses[i].orientation.z;
        pose.orientation.w = msg->poses[i].orientation.w;

        static_path.push_back(pose);
    }
    for(int i = 0; i < static_path.size(); i++){
        tf::Quaternion q3(static_path[i].orientation.x, static_path[i].orientation.y, static_path[i].orientation.z, static_path[i].orientation.w);
        tf::Matrix3x3 mout(q3);
        std::vector<double> out(3);
        mout.getRPY(out[0], out[1], out[2]);

        tf::Quaternion newQuaternion;
        newQuaternion.setRPY(out[0], out[1], 90*DEG2RAD);
        static_path[i].orientation.x = newQuaternion.getX();
        static_path[i].orientation.y = newQuaternion.getY();
        static_path[i].orientation.z = newQuaternion.getZ();
        static_path[i].orientation.w = newQuaternion.getW();
    }

    printf("Total smooth path length : %lu\n", static_path.size());
    for(int i = 0; i < static_path.size(); i++){
        tf::Quaternion q4(static_path[i].orientation.x, static_path[i].orientation.y, static_path[i].orientation.z, static_path[i].orientation.w);
        tf::Matrix3x3 Nout(q4);
        std::vector<double> out(3);
        Nout.getRPY(out[0], out[1], out[2]);
        printf("x: %10.4lf, y: %10.4lf, z: %10.4lf rx: %10.4lf, ry: %10.4lf, rz: %10.4lf \n", static_path[i].position.x, static_path[i].position.y, static_path[i].position.z, out[0]*RAD2DEG, out[1]*RAD2DEG, out[2]*RAD2DEG);
    }

    // showSmoothPath(static_path);
    edge.points.clear();
    start_cmd = 1;
    start_simulate = 0;
}

void get_path_callback_simulate(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    static_path_simulate.clear();
    geometry_msgs::Pose pose;
    for(int i = 0; i < msg->poses.size(); i++)
    {
        pose.position.x = msg->poses[i].position.x;
        pose.position.y = msg->poses[i].position.y;
        pose.position.z = msg->poses[i].position.z;
        pose.orientation.x = msg->poses[i].orientation.x;
        pose.orientation.y = msg->poses[i].orientation.y;
        pose.orientation.z = msg->poses[i].orientation.z;
        pose.orientation.w = msg->poses[i].orientation.w;

        static_path_simulate.push_back(pose);
    }

    for(int i = 0; i < static_path_simulate.size(); i++){
        tf::Quaternion q3(static_path_simulate[i].orientation.x, static_path_simulate[i].orientation.y, static_path_simulate[i].orientation.z, static_path_simulate[i].orientation.w);
        tf::Matrix3x3 mout(q3);
        std::vector<double> out(3);
        mout.getRPY(out[0], out[1], out[2]);

        tf::Quaternion newQuaternion;
        newQuaternion.setRPY(out[0], out[1], 90*DEG2RAD);
        static_path_simulate[i].orientation.x = newQuaternion.getX();
        static_path_simulate[i].orientation.y = newQuaternion.getY();
        static_path_simulate[i].orientation.z = newQuaternion.getZ();
        static_path_simulate[i].orientation.w = newQuaternion.getW();
    }

    printf("Total smooth path length : %lu\n", static_path_simulate.size());
    for(int i = 0; i < static_path_simulate.size(); i++){
        tf::Quaternion q4(static_path_simulate[i].orientation.x, static_path_simulate[i].orientation.y, static_path_simulate[i].orientation.z, static_path_simulate[i].orientation.w);
        tf::Matrix3x3 Nout(q4);
        std::vector<double> out(3);
        Nout.getRPY(out[0], out[1], out[2]);
        printf("x: %10.4lf, y: %10.4lf, z: %10.4lf rx: %10.4lf, ry: %10.4lf, rz: %10.4lf \n", static_path_simulate[i].position.x, static_path_simulate[i].position.y, static_path_simulate[i].position.z, out[0]*RAD2DEG, out[1]*RAD2DEG, out[2]*RAD2DEG);
    }

    // showSmoothPath(static_path_simulate);
    edge.points.clear();
    start_simulate = 1;
    start_cmd = 0;
}

int main(int argc, char **argv)
{
    // std::string host;

    // #ifdef USE_BOOST
    //     boost::condition_variable data_cv;
    //     boost::condition_variable data_cv_rt;
    // #else
    //     std::condition_variable data_cv;
    //     std::condition_variable data_cv_rt;
    // #endif

    ros::init(argc, argv, "tm_action");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(2);

    marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 40);
    //reset_monitoring_pub = node_handle.advertise<std_msgs::Int32>("/reset_local_replan", 40);
    Joints_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/tm5_position", 1000);
    pub_robot_project = node_handle.advertise<geometry_msgs::PoseStamped>("/tm5",1);

    ros::Subscriber receive_path = node_handle.subscribe("/MovingPath", 1, get_path_callback);
    ros::Subscriber receive_path_simulate = node_handle.subscribe("/MovingPath_simulate", 1, get_path_callback_simulate);
    ros::Subscriber receive_joints = node_handle.subscribe("/RobotJoints", 1000, TMmsgCallback_simulate);
    ros::Subscriber robot_status_sub = node_handle.subscribe("feedback_states", 1000,TMmsgCallback);

    vel_client = node_handle.serviceClient<tm_msgs::SetVelocity>("tm_driver/set_velocity");
    script_client = node_handle.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    event_client = node_handle.serviceClient<tm_msgs::SetEvent>("tm_driver/set_event");
    pos_client = node_handle.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    pub_ctrl = node_handle.advertise<robotiq_controller::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput",10);

    tm_msgs::SetPositions pos_srv;
    tm_msgs::SetEvent event_srv;
    tm_msgs::SendScript script_srv;

    spinner.start();
    bool fgRun = false;

    std::string path = ros::package::getPath("sophia_test");
    end_pose.open (path+"/src/tool_pose/0207/tool_pose0207-3.txt", std::ifstream::out);
    distance_error.open (path+"/src/tool_pose/0207/error0207-3.txt", std::ifstream::out);

    virtual_joints.open (path+"/src/Joint_values/virtual_value.txt", std::ifstream::out);
    real_joints.open (path+"/src/Joint_values/real_value.txt", std::ifstream::out);

    char cstr[512] = "";
    char delim[] = " ,;\t";
    char c;
    

    ros::Rate r(10);
    while (ros::ok())
    {
        if(start_cmd==1){
            unsigned char ch = 0;
            //std_msgs::Int32 reset_local_replan_msg;
            //reset_local_replan_msg.data = 1;

            set_gripper(1); 
            pub_ctrl.publish(gripper_command);

            script_srv.request.id = "Vstart";
            script_srv.request.script = VStart_cmd;
            script_client.call(script_srv);
            print_info("joint velocity control mode ON...");

            //reset_monitoring_pub.publish(reset_local_replan_msg);
            Hexagon_Demo();

            script_srv.request.id = "spdmodeoff";
            script_srv.request.script = VStop_cmd;
            script_client.call(script_srv);
            print_info("joint vlocity control mode OFF...");


        }
        else if(start_simulate==1 && get_robot_joints){

            unsigned char ch = 0;

            script_srv.request.id = "Vstart";
            script_srv.request.script = VStart_cmd;
            script_client.call(script_srv);
            print_info("simulation control mode ON...");

            Hexagon_Demo_simulate();

            script_srv.request.id = "spdmodeoff";
            script_srv.request.script = VStop_cmd;
            script_client.call(script_srv);
            print_info("simulation control mode OFF...");
            // flag =1;
        }

        r.sleep();
    }
    
    //ros::waitForShutdown();
    
    printf("[ info] TM_ROS: shutdown\n");

    return 0;
}
