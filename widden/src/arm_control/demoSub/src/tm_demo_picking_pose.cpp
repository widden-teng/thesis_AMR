/*********************************************************************
 * tm_action_picking_safe.cpp
 *
 * Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
 * Author: Howard Chen (s880367@gmail.com)
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
 * Author: Howard Chen
 */


// [2]   Picking_above_pose (computational result)
// [3]   Object_pose (computational result)
// [4]   After_picking_pose
// [5]   Prepare_to_place_pose + placing_pose



#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_robot_state.h"
#include "tm_driver/tm_ros_node.h"

#include <my_msg_pkg/pose_estimation_data.h>

#include "tm_reflexxes/tm_reflexxes.h"
#include "tm_kinematics/tm_kin.h"
#include "robotiq_controller/Robotiq2FGripper_robot_output.h"

#include "tm_msgs/SetIO.h"
#include "tm_msgs/SetVelocity.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SetEvent.h"
#include "tm_msgs/SetPositions.h"

#include <eigen3/Eigen/Dense>

#include <stdio.h>
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
#include "std_msgs/Bool.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

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

#define IMAGE_CAPTURE_POSE 1
#define GRASP_READY_POSE   2
#define OBJECT_POSITION    3
#define GRASP_AFTER_POSE   4
#define PLACING_POSE       5
#define PLACING_POSE       5
#define PLACING_POSE_DOWN  6
#define MID_POSE           7
#define CHECK_POSE         8
#define IMAGE_CAPTURE_MOVING 1

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

double last_point_x = 0.0;
double last_point_y = 0.0;
double last_point_z = 0.0;

double g_distance;
std::vector<double> g_obstacle_position(3);
std::vector<double> tool_obstacle_position(3);
std::vector<double> g_obstacle_velocity(3);
std::vector<double> g_constraint_position(3);
std::vector<double> g_robot_joint_angle(6);

Eigen::Matrix<double,4,4> T_top;
Eigen::Matrix<double,4,4> T_bottom;

double g_objposition[7];
bool position_fill = false;
bool rotation_fill = false;
bool arrive_flag = false;
int g_ObjClass;

ros::Publisher RobotState;



using namespace std;

ros::ServiceClient vel_client;
ros::ServiceClient script_client;
ros::ServiceClient event_client;
ros::ServiceClient pos_client;

ros::Publisher pub_ctrl;
robotiq_controller::Robotiq2FGripper_robot_output gripper_command;

std::string exit_cmd = "ScriptExit()";
std::string VStart_cmd = "ContinueVJog()";
std::string VStop_cmd = "StopContinueVmode()";


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

    if(abs(q[0]) > 265*DEG2RAD)
    {
        ROS_WARN("[Position] 1st joint position out of limit (270) : %lf",q[0]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[1]) > 175*DEG2RAD)
    {
        ROS_WARN("[Position] 2nd joint position out of limit (180): %lf",q[1]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[2]) > 148*DEG2RAD)
    {
        ROS_WARN("[Position] 3rd joint position out of limit (155): %lf",q[2]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[3]) > 175*DEG2RAD)
    {
        ROS_WARN("[Position] 4th joint position out of limit (180): %lf",q[3]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[4]) > 175*DEG2RAD)
    {
        ROS_WARN("[Position] 5th joint position out of limit (180): %lf",q[4]*RAD2DEG );
        valid = false;
    }
    else if(abs(q[5]) > 265*DEG2RAD)
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

void ReflexxesPositionSmoothStop(   RMLPositionInputParameters *InputState, 
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
    IP  = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    OP  = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);

    *IP->CurrentPositionVector     = *InputState->CurrentPositionVector;
    *IP->CurrentVelocityVector     = *InputState->CurrentVelocityVector;
    *IP->CurrentAccelerationVector = *InputState->CurrentAccelerationVector;


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
        usleep(24940 - time_compensation);  

        // The area execution in 25ms real time sharp
        //********************************************************
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    // std::vector<double> FinalPosition;
    // time_s = TR.interface->stateRT->getQAct(FinalPosition);
    // printf("=============== Final state of Smooth Stop =========================\n");
    // printf("[ %lf ]  ", time_s);

    // for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    //     printf(" %10.4lf ",FinalPosition[i]);
    // printf("\n");
    print_info("Smooth stop finish in %llu us", tt);

    *InputState->CurrentPositionVector     = *IP->CurrentPositionVector;
    *InputState->CurrentVelocityVector     = *IP->CurrentVelocityVector;
    *InputState->CurrentAccelerationVector = *IP->CurrentAccelerationVector;

    delete  RML;
    delete  IP;
    delete  OP;
}

bool ReflexxesPositionRun(  RMLPositionInputParameters &InputState, 
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

    ReflexxesAPI *RML = NULL    ;
    RMLPositionInputParameters  *IP = NULL;
    RMLPositionOutputParameters *OP = NULL;
    RMLPositionFlags Flags;
    int ResultValue = 0;

    double *T = new double [16];
    double *q = new double [6];

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP  = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP  = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

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
        IP->MaxJerkVector->VecData[i]         = 100;           //RMLTypeII not using, needed for validity
        IP->MaxVelocityVector->VecData[i]     = 3.14; //0.3247
        IP->MaxAccelerationVector->VecData[i] = 3.14;
        IP->TargetPositionVector->VecData[i]  = TargetPosition[i]; 
        IP->TargetVelocityVector->VecData[i]  = TargetVelocity[i];
        IP->SelectionVector->VecData[i]       = true;
    }
    IP->MinimumSynchronizationTime = SynTime;


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
    {
        printf("Input values are INVALID!\n");
        pass = false;
    }


    struct timeval tm1,tm2, tm3, tm4;
    double cycle_iteration = 1.0;


    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED && ros::ok())
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

        ResultValue =  RML->RMLPosition(*IP, OP, Flags );

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
        // Print out commands

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];

        //tm_jacobian::Forward_Kinematics_3(q,T);
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        ROS_INFO("gripper XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

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

        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;

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

    ROS_INFO("========== Final state position based position =============");
    ROS_INFO("XYZ_pos : %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);
    print_info("Finished in %llu us", tt);

    tm_reflexxes::resetTermios();
    
    if(pass)
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

bool GetQdfromLinearJacobian(   std::vector<double> CurrentPosition,
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
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    //ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    return CheckVelocityLimit(qd,ScalingFactor);
}

bool GetQdfromRotationForce(   std::vector<double> CurrentPosition,
                                std::vector<double> TcpandGoalPoint,
                                std::vector<double> EFF_Velocity, 
                                std::vector<double> Rotation_Speed, 
                                std::vector<double>& qd, 
                                double distance
                                )
{
    Eigen::Matrix<float , 6, 1> home,q;
    Eigen::Matrix<double , 6,1> JointSpeed;
    Eigen::Matrix<double, 6, 1> RotSpeed;
    Eigen::Matrix<double, 3, 1> EFFSpeed;


    home     <<                  0,            -PI*0.5,                  0,             PI*0.5,                  0,                  0;
    q        << CurrentPosition[0], CurrentPosition[1], CurrentPosition[2], CurrentPosition[3], CurrentPosition[4], CurrentPosition[5];
    EFFSpeed <<    EFF_Velocity[0],    EFF_Velocity[1],    EFF_Velocity[2];
    RotSpeed <<                  0,                  0,                  0,                  0,                  0,  Rotation_Speed[5];
    q += home;



    if(distance > 0.4 )
    {
        if(q(5)*RAD2DEG > 2 || q(5)*RAD2DEG < -2)
        {   double RecoverRoute = sqrt(pow((TcpandGoalPoint[0]-TcpandGoalPoint[3]),2)+pow((TcpandGoalPoint[1]-TcpandGoalPoint[4]),2)+pow((TcpandGoalPoint[2]-TcpandGoalPoint[5]),2));
            double RecoverTime = RecoverRoute/V_TRAVEL;
            double RecoverQd = (0 - q(5))/RecoverTime;;
            RotSpeed<< 0,0,0,0,0,RecoverQd;
        }
        else
        {
            RotSpeed<< 0,0,0,0,0,0;
        }   
    }
    else if ( q(5)*RAD2DEG > 90 )
    {
        RotSpeed<< 0,0,0,0,0,0;
    }



    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q, GRIPPER_LENGTH);// Forward_Jacobian_d(q);
    Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::MatrixXd jacobian_123456_inv;
    pinv_SVD(Jacobian_123456,jacobian_123456_inv);
    JointSpeed = jacobian_123456_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i)+RotSpeed(i);

    ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

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

bool OnlineAttractiveForceGeneration(   std::vector<double>& action_qd,
                                        double *TCP_position,             // robot TCP position       : for min_distance
                                        double *q,                        // robot joint position     : for jacobian
                                        std::vector<double> GoalPoint,    // eff to obstacle goal     : for att
                                        std::vector<double> &EFF_Velocity) // eff velocity             : for att
{
    bool   succeed = false;
    double Vmax_att = VMAX_ATT_GPR;
    double Survelliance_att = 0.15;
    double ShapingFactor_att = 5;

    Eigen::Vector3d Eff2Goal;
    Eff2Goal << GoalPoint[0]-TCP_position[3], GoalPoint[1]-TCP_position[7], GoalPoint[2]-TCP_position[11];
    double dis_goal = sqrt( pow(Eff2Goal(0),2) + pow(Eff2Goal(1),2) + pow(Eff2Goal(2),2)     );

    Eigen::Vector3d AttractiveVecotor = Eff2Goal/dis_goal;

    double AttractiveForce = Vmax_att - Vmax_att*exp(-(dis_goal*ShapingFactor_att)/Survelliance_att); 

    Eigen::Vector3d AttractiveVelocity = AttractiveForce*AttractiveVecotor; 

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    EFF_Velocity = {AttractiveVelocity(0), AttractiveVelocity(1), AttractiveVelocity(2), 0,0,0}; 
    double AttractiveScalar = sqrt(AttractiveVelocity.dot(AttractiveVelocity));
//    ROS_INFO("Attractive Velocity: %10.4lf ", AttractiveScalar);
    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,action_qd);
    //succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,action_qd);  
    //succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,action_qd);
    succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,action_qd);

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
    //Obstacle2Eff << TCP_position[3]-ObstaclePoint[0], TCP_position[7]-ObstaclePoint[1], TCP_position[11]-ObstaclePoint[2];
    Obstacle2Eff << tool_ObstaclePoint[0]-ObstaclePoint[0], tool_ObstaclePoint[1]-ObstaclePoint[1], tool_ObstaclePoint[2]-ObstaclePoint[2];
    double dis_repuslive = sqrt(Obstacle2Eff.dot(Obstacle2Eff));
    Eigen::Vector3d RepulsiveVector   = Obstacle2Eff/dis_repuslive;

    double RepulsiveForce  = Vmax_rep / (1 + exp((dis_repuslive*Survelliance_rep-1)*ShapingFactor_rep));
    Eigen::Vector3d RepulsiveVelocity  = RepulsiveForce *RepulsiveVector;

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    EFF_Velocity = {RepulsiveVelocity(0), RepulsiveVelocity(1), RepulsiveVelocity(2), 0,0,0}; 
    //OS_INFO("ObstaclePoint      : %10.4lf %10.4lf %10.4lf",ObstaclePoint[0],ObstaclePoint[1],ObstaclePoint[2]);
//    ROS_WARN("Repulsive  Velocity: %10.4lf ", sqrt(RepulsiveVelocity.dot(RepulsiveVelocity)));
    
    //succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    //succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,Repulsive_qd);   
    succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,Repulsive_qd);   

    return succeed;

}

bool OnlineRotationForceGeneration(     std::vector<double>& Rotation_qd,
                                        std::vector<double> GoalPoint,          // for recover motion
                                        double *TCP_position,                   // robot TCP position       : for min_distance
                                        double *q)                              // robot joint position     : for jacobian 
{
    
    bool succeed = false;
    int sign = 1;
    double Vmax_rot = VMAX_ROT;
    double Survelliance_rot = 0.15;
    double ShapingFactor_rot = 5;   //
    std::vector<double> ObstaclePoint(3),tool_ObstaclePoint(3),TopPoint(3),BottomPoint(3);
    Eigen::Matrix4d T_TopPoint,T_BottomPoint,T;

    T << TCP_position[0],   TCP_position[1],   TCP_position[2],   TCP_position[3], 
         TCP_position[4],   TCP_position[5],   TCP_position[6],   TCP_position[7], 
         TCP_position[8],   TCP_position[9],   TCP_position[10],  TCP_position[11], 
         TCP_position[12],  TCP_position[13],  TCP_position[14],  TCP_position[15];

    T_TopPoint = T * T_top;
    T_BottomPoint = T * T_bottom;
    TopPoint = {T_TopPoint(0,3),T_TopPoint(1,3),T_TopPoint(2,3)};
    BottomPoint = {T_BottomPoint(0,3),T_BottomPoint(1,3),T_BottomPoint(2,3)};


    ObstaclePoint = g_obstacle_position;
    tool_ObstaclePoint = tool_obstacle_position;
    // ROS_INFO("TCP_position : %10.4lf %10.4lf %10.4lf", TCP_position[3],TCP_position[7],TCP_position[11]);
    // ROS_INFO("TCP_position : %10.4lf %10.4lf %10.4lf", T(0,3),T(1,3),T(2,3));
    // ROS_INFO("TopPoint : %10.4lf %10.4lf %10.4lf",TopPoint[0],TopPoint[1],TopPoint[2]);
    // ROS_INFO("BottomPoint : %10.4lf %10.4lf %10.4lf",BottomPoint[0],BottomPoint[1],BottomPoint[2]);
    // geometry_msgs::PointStamped pt;
    // pt.header.frame_id = "/base";
    // pt.point.x = TopPoint[0];
    // pt.point.y = TopPoint[1];
    // pt.point.z = TopPoint[2];
    // top_pt.publish(pt);

    // pt.point.x = BottomPoint[0];
    // pt.point.y = BottomPoint[1];
    // pt.point.z = BottomPoint[2];
    // bottom_pt.publish(pt);


    Eigen::Vector3d Obstacle2Eff,Obstacle2Top,Obstacle2Bottom,Obstacle2Tool;
    Obstacle2Eff << TCP_position[3]-ObstaclePoint[0], TCP_position[7]-ObstaclePoint[1], TCP_position[11]-ObstaclePoint[2];
    Obstacle2Tool << tool_ObstaclePoint[0]-ObstaclePoint[0], tool_ObstaclePoint[1]-ObstaclePoint[1], tool_ObstaclePoint[2]-ObstaclePoint[2];
    Obstacle2Top << TopPoint[0] - ObstaclePoint[0], TopPoint[1] - ObstaclePoint[1], TopPoint[2] - ObstaclePoint[2];
    Obstacle2Bottom << BottomPoint[0] - ObstaclePoint[0], BottomPoint[1] - ObstaclePoint[1], BottomPoint[2] - ObstaclePoint[2];


    double dis_ee = sqrt(Obstacle2Eff.dot(Obstacle2Eff));
    double dis_Top = sqrt(Obstacle2Top.dot(Obstacle2Top));
    double dis_Bottom = sqrt(Obstacle2Bottom.dot(Obstacle2Bottom));
    double dis_test = sqrt(Obstacle2Tool.dot(Obstacle2Tool));

    double rotation_factor = dis_Top - dis_Bottom;
    ROS_WARN("rotation_factor = %10.4lf", rotation_factor);
    if (rotation_factor>0)
        sign = 1;
    else
        sign = -1;

    double RotationForce  =sign*( Vmax_rot - Vmax_rot*exp(-(abs(rotation_factor)*ShapingFactor_rot)/Survelliance_rot) );


    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    // std::vector<double> CurrentPosition = {joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5]}; 
    //ROS_INFO("ObstaclePoint      : %10.4lf %10.4lf %10.4lf",ObstaclePoint[0],ObstaclePoint[1],ObstaclePoint[2]);
    //ROS_WARN("Repulsive  Velocity: %10.4lf ", sqrt(RepulsiveVelocity.dot(RepulsiveVelocity)));
    //ROS_WARN("Current q = %10.4lf  , %10.4lf  , %10.4lf  , %10.4lf  , %10.4lf  , %10.4lf ",q[0], q[1], q[2], q[3], q[4], q[5]);


//還要調整旋轉factor大小
    
    std::vector<double> RotationSpeed = { 0 , 0 , 0 , 0 , 0 , 0 };
    if(dis_test < 0.7 && abs(rotation_factor )> 0.03){
        // RotationSpeed[5] = 10*DEG2RAD;
        RotationSpeed[5] = RotationForce*DEG2RAD;
    }
    ROS_WARN("rotation : now q = %10.4lf ,RotationForce = %10.4lf", q[5]*RAD2DEG , RotationForce);
    std::vector<double> EFF_Velocity = { 0 , 0 , 0 , 0 , 0 , 0 };
    std::vector<double> TcpandGoalPoint = {TCP_position[3],TCP_position[7],TCP_position[11],GoalPoint[0],GoalPoint[1],GoalPoint[2]}; // for rotation recover computing
    succeed = GetQdfromRotationForce(CurrentPosition,TcpandGoalPoint,EFF_Velocity,RotationSpeed,Rotation_qd,dis_ee);  

    return succeed;

}



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

        // TR.setMoveJointSpeedabs(vec, blend);
        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = VelocityCommand;
        // vel_client.call(vel_srv);
        // usleep(100*1000);
        if (vel_client.call(vel_srv))                             
        {
            if (vel_srv.response.ok){

            } 
                // ROS_INFO_STREAM("SetPositions to robot");
            else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
        }
        else
        {
            ROS_ERROR_STREAM("Error SetPositions to robot");
            // return 1;
        }


        //***************************************************************
        // Print out commands

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];
        
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        ROS_INFO("tool0 XYZ_pos: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

        //***************************************************************

        for (int i = 0; i < 3; ++i)
        {
            if(abs(EffVelocity[i]) == 0)
                DistanceToGoal[i] = 0;
            else
                DistanceToGoal[i] = abs(T[i*4+3] -TargetPosition[i]);
        }

        if( (DistanceToGoal[0] < REPEATABILITY) && (DistanceToGoal[1] < REPEATABILITY) && (DistanceToGoal[2] < REPEATABILITY))
        {
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
        // usleep(1000); 
        if((24940 - time_compensation) < 1)
            usleep(100);
        else
            usleep(24940 - time_compensation);   

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

   ROS_INFO("========== Final state velocity based position =============");
   ROS_INFO("pass = %d  tool0 : %10.4lf %10.4lf %10.4lf", pass, T[3], T[7], T[11]);
   ROS_INFO("Joint position  : %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", q[0]*RAD2DEG,q[1]*RAD2DEG,q[2]*RAD2DEG,q[3]*RAD2DEG,q[4]*RAD2DEG,q[5]*RAD2DEG);
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



void Image1PTP()
{
    int timer = 0;

    tm_msgs::SendScript script_srv;
    tm_msgs::SetPositions pos_srv;

    script_srv.request.id = "spdmodeoff";
    script_srv.request.script = VStop_cmd;
    script_client.call(script_srv);
    
    print_info("joint vlocity control mode OFF...");

    std::vector<double> TargetPosition = {0.7616,    0.1689,    1.4458,    -0.0612,    1.5654,    -0.8033};

    pos_srv.request.motion_type = 1;
    pos_srv.request.positions = TargetPosition;
    pos_srv.request.velocity = 50;
    pos_srv.request.acc_time = 0.2;
    pos_srv.request.blend_percentage = 0;
    pos_srv.request.fine_goal = false;
    pos_client.call(pos_srv);

    print_info("Back to position 5");

    while(abs(TargetPosition[0]-g_robot_joint_angle[0]) > 0.01 || abs(TargetPosition[1]-g_robot_joint_angle[1]) > 0.01 ||
          abs(TargetPosition[2]-g_robot_joint_angle[2]) > 0.01 || abs(TargetPosition[3]-g_robot_joint_angle[3]) > 0.01 ||
          abs(TargetPosition[4]-g_robot_joint_angle[4]) > 0.01 || abs(TargetPosition[5]-g_robot_joint_angle[5]) > 0.01 )
    {
        usleep(200*1000);
        timer++;
        if(timer > 50){
            pos_client.call(pos_srv);
            ROS_WARN("Re-calling service");
            timer = 0;
        }
    }


    script_srv.request.id = "Vstart";
    script_srv.request.script = VStart_cmd;
    script_client.call(script_srv);
    print_info("joint velocity control mode ON...");
}

bool Hexagon_Demo(int pick_num, RMLPositionInputParameters &IP_Position_Main)
{
    bool run_succeed = true;
    int  pass = 0;
    double SynchronousTime = 7;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6),Rotation_qd(6);
    double *T = new double [16];
    double *q = new double [6];
    // double tool_radius = 0.15;
    double tool_radius = 0.1; //TM5 target

    pick_num--;
    pick_num = (pick_num % 2) + 1;

    int round = 0;

    T_top <<    1, 0, 0, 0,
                0, 1, 0, tool_radius,
                0, 0, 1, 0,
                0, 0, 0, 1;

    T_bottom << 1, 0, 0,  0,
                0, 1, 0, -tool_radius,
                0, 0, 1,  0,
                0, 0, 0,  1;
    

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

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

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    std::vector<double> InitialPosition = {0.2595,    -0.1595,    1.4565,    -1.3401,    1.2991,    -0.0};

    std::vector<double> RecoverPosition = {0.2595,    -0.1595,    1.4565,    -1.3401,    1.2991,    -0.0};

    TargetVelocity = {0, 0, 0, 0, 0, 0};
    if(!ReflexxesPositionRun(*IP_position, InitialPosition, TargetVelocity, SynchronousTime))
    {
        ROS_WARN("Smooth stop activate...");
        return 0;
    }

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

    //Test for hexagon
    int state = 2;
    double x_plane  = 0.68;

    double left_y   = -0.3360;
    double mid_y    = -0.027;
    double right_y  = 0.264;

    double upp_z    = 0.81;
    double up_z     = 0.755;

    double mid_z    = 0.55;

    double down_z   = 0.505;
    double downn_z  = 0.38;
    while(ros::ok())
    {
        // TM5.interface->stateRT->getQAct(CurrentPosition);
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            CurrentPosition[i] = g_robot_joint_angle[i];
        }
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
            //q[i] = joint_pos[i];
        }
 


// 我的 triangle
        if(state == 1)      //s1
        {
            TargetPosition = {x_plane, mid_y, upp_z, 0, 0, 0};
            RecoverPosition = {0.2595,    -0.1595,    1.4565,    -1.3401,    1.2991,    -0.0};
        }
        else if(state == 2) //s2
        {
            TargetPosition = {x_plane, right_y, mid_z, 0, 0, 0};
            
        }
        else if(state == 3) //s3
        {
            TargetPosition = {x_plane, left_y, mid_z, 0, 0, 0};
        }
        else if(state == 4) //s4
        {
            TargetPosition = {x_plane, mid_y, upp_z, 0, 0, 0};
        }
        else if(state == 5) //s5
        {
            TargetPosition = {x_plane, right_y, mid_z, 0, 0, 0};
        }
        else if(state == 6) //s6
        {
            TargetPosition = {x_plane, left_y, mid_z, 0, 0, 0};
        }
        else{}



        if(state != 7)  //point-to-point motion
        {
            //tm_kinematics::forward(q,T);
            tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

            if(OnlineAttractiveForceGeneration(Attractive_qd, T, q, TargetPosition, TargetVelocity))
            {
                if(OnlineRepulsiveForceGeneration(Repulsive_qd, T, q, RepulsiveVelocity))
                {
                    if(OnlineRotationForceGeneration(Rotation_qd,TargetPosition, T, q))
                    {    
                        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                            qd[i] = Attractive_qd[i] + Repulsive_qd[i] + Rotation_qd[i];
                        

                        // ROS_INFO("gripper : %10.4lf %10.4lf %10.4lf Atrractive: %10.4lf %10.4lf %10.4lf Repulsive: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11],TargetVelocity[0],TargetVelocity[1],TargetVelocity[2],RepulsiveVelocity[0],RepulsiveVelocity[1],RepulsiveVelocity[2] );
                        pass = ReflexxesVelocityRun(*IP_velocity, qd, TargetPosition, TargetVelocity ,0.2);

                        if(pass == REACH)
                        {
                            if(state == 1 || state == 4)
                            {
                                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                                {
                                    IP_position->CurrentPositionVector->VecData[i]     = IP_velocity->CurrentPositionVector->VecData[i];
                                    IP_position->CurrentVelocityVector->VecData[i]     = IP_velocity->CurrentVelocityVector->VecData[i];
                                    IP_position->CurrentAccelerationVector->VecData[i] = IP_velocity->CurrentAccelerationVector->VecData[i];
                                }

                                //TargetVelocity = {0,0,0,0,0,0};
                                // if(!ReflexxesPositionRun(*IP_position, RecoverPosition, qd, 0.5))
                                //     break;
                                
                                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                                {
                                    IP_velocity->CurrentPositionVector->VecData[i]     = IP_position->CurrentPositionVector->VecData[i];
                                    IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i];
                                    IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i];
                                }
                                // break;
                                round++;

                            }

                            if(pick_num == 1){  //3 round
                                if(round == 3){
                                    // IP_Position_Main = *IP_position;
                                    break;
                                }
                            }
                            else if(pick_num == 2){
                                if(round == 1){
                                    // IP_Position_Main = *IP_position;
                                    break;
                                }
                            }

                            state++;
                            // break;
                        }
                        else if(pass == STOP)
                            break;
                        else{}
                    }
                    else
                    {
                        ROS_WARN("Rotation force generate fail, smooth stop activate...");
                        ReflexxesSmoothStop(*IP_velocity,0.25);
                        break;
                    }
                }
                else
                {
                    ROS_WARN("Repulsive force generate fail, smooth stop activate...");
                    ReflexxesSmoothStop(*IP_velocity,0.25);
                    break;
                }
            }
            else
            {
                ROS_WARN("Attrative force generate fail, smooth stop activate...");
                ReflexxesSmoothStop(*IP_velocity,0.25);
                break;
            }
        }
        else    // initialization
        {
            state = 1;
        }        
    }
    // ReflexxesSmoothStop(*IP_velocity,0.25);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_Position_Main.CurrentPositionVector->VecData[i]     = IP_velocity->CurrentPositionVector->VecData[i];
        IP_Position_Main.CurrentVelocityVector->VecData[i]     = IP_velocity->CurrentVelocityVector->VecData[i];
        IP_Position_Main.CurrentAccelerationVector->VecData[i] = IP_velocity->CurrentAccelerationVector->VecData[i];
    }

    ROS_WARN("Hexagon_Demo shutdown");

    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q;

    return true;
}


bool BinPicking_Demo3()
{
    // TM5.setJointSpdModeON();
    // print_info("joint velocity control mode ON...");

    bool run_succeed = true;
    bool table_to_box = false;
    bool skip_flag = false;
    int  pass = 0;
    int finalpose_flag = 1;
    std::vector<double>  final_position(6);
    double SynchronousTime = 7;
    std::vector<double> TargetPosition(6), TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6);
    double *T = new double [16];
    double *q = new double [6];
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
    
    std::vector<double> origin = {0.0086,    -0.5819,    2.5447,    -1.6535,    1.5097,    0.0248};
    std::vector<double> upgrasp = {-0.9697,    0.1395,    1.1058,    0.3344,    1.545,    0.5817};
    //std::vector<double> vec3 = {-1.1554,    0.6884,    1.7523,    -0.9856,    1.5643,    -0.9065};
    //std::vector<double> vec4 = {-1.1304,    0.3562,    1.4949,    -0.2138,    1.4975,    0.4965};
    std::vector<double> upplace = {0.1791,    -0.0778,    1.4174,    0.2456,    1.5668,    0.1993};
    //std::vector<double> vec6 = {0.1819,    0.0878,    2.0569,    -0.5112,    1.6253,    0.1503};
    //std::vector<double> vec7 = {0.0086,    -0.5819,    2.5447,    -1.6535,    1.5097,    0.0248};
    std::vector<double> pick1 = {-1.1097,    0.5437,    1.4273,    -0.2307,    1.3292,    -0.9163};   //shortp_p
    std::vector<double> pick2 = {-0.8952,    0.5639,    1.4357,    -0.3005,    1.2518,    -1.1982};   //hand_w
    std::vector<double> pick3 = {-1.1219,    0.5817,    1.4205,    -0.4555,    1.5645,    -1.5019};   //bend_p
    std::vector<double> pick4 = {-1.1055,    0.3513,    1.7876,    -0.5758,    1.55,    0.4798};	  //hand_w
    std::vector<double> pick5 = {-1.2748,    0.4828,    1.5727,    -0.5112,    1.5476,    -0.4056};   //bend_p


    std::vector<double> place1 = {0.4426,    0.0414,    2.1225,    -0.6182,    1.5294,    0.4384};
    std::vector<double> place2 = {0.2716,    0.1049,    2.0747,    -0.5836,    1.5464,    0.2847};
	std::vector<double> place3 = {0.1129,    0.051,    2.1501,    -0.6611,    1.5294,    0.1075};
	std::vector<double> place4 = {0.3833,    0.0391,    2.1487,    -0.6276,    1.5289,    0.4384};
	std::vector<double> place5 = {0.1822,    0.0403,    2.1796,    -0.6985,    1.5296,    0.2122};




    TargetVelocity = {0, 0, 0, 0, 0, 0};

    g_objposition[0] = 2.0;
    robot_motion_states.data = 1;

    while(ros::ok())
    {
    	if(robot_motion_states.data == 1)
    	{
    		SynchronousTime = 5;
            robot_motion_states.data = 2;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;

            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);



            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

    	}
	    else if(robot_motion_states.data == 2) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 3;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 3) 
        {
                SynchronousTime = 5;
                robot_motion_states.data = 4;
                if(!ReflexxesPositionRun(*IP_position, pick1, TargetVelocity, SynchronousTime))
                    break;  
                set_gripper(1); 
                pub_ctrl.publish(gripper_command);

                sleep(2);

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 4) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 5;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 5) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 6;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 6) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 7;
            if(!ReflexxesPositionRun(*IP_position, place1, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 7) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 8;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 8) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 9;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 9) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 10;
            if(!ReflexxesPositionRun(*IP_position, pick2, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 10) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 11;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 11) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 12;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 12) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 13;
            if(!ReflexxesPositionRun(*IP_position, place2, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 13) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 14;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
//add



//grasp4 grasp5
        else if(robot_motion_states.data == 14) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 15;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 15) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 16;
            if(!ReflexxesPositionRun(*IP_position, pick3, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 16) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 17;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 17) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 18;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 18) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 20;
            if(!ReflexxesPositionRun(*IP_position, place3, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 20) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 21;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 21)
        {

            SynchronousTime = 5;
            robot_motion_states.data = 22;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 22)
        {

            SynchronousTime = 5;
            robot_motion_states.data = 23;
            if(!ReflexxesPositionRun(*IP_position, pick4, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 23)
        {

            SynchronousTime = 3;
            robot_motion_states.data = 24;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data ==24) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 25;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 25) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 26;
            if(!ReflexxesPositionRun(*IP_position, place4, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 26) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 27;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 27) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 28;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 28) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 29;
            if(!ReflexxesPositionRun(*IP_position, pick5, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 29) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 30;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 30) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 31;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 31) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 32;
            if(!ReflexxesPositionRun(*IP_position, place5, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 32) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 33;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }


//add
        else if(robot_motion_states.data == 33) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 34;
            if(!ReflexxesPositionRun(*IP_position, origin, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else{

	    break;
		}
    }
    ROS_WARN("Bin Picking Demo shutdown");
    delete IP_position;
    delete [] T;
    delete [] q;
    return 0;
}
bool BinPicking_Demo4()
{
    // TM5.setJointSpdModeON();
    // print_info("joint velocity control mode ON...");

    bool run_succeed = true;
    bool table_to_box = false;
    bool skip_flag = false;
    int  pass = 0;
    int finalpose_flag = 1;
    std::vector<double>  final_position(6);
    double SynchronousTime = 7;
    std::vector<double> TargetPosition(6), TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6);
    double *T = new double [16];
    double *q = new double [6];
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

    std::vector<double> origin = {0.0086,    -0.5819,    2.5447,    -1.6535,    1.5097,    0.0248};
    std::vector<double> upgrasp = {-1.1304,    0.3562,    1.4949,    -0.2138,    1.4975,    0.4965};
    //std::vector<double> vec3 = {-1.1554,    0.6884,    1.7523,    -0.9856,    1.5643,    -0.9065};
    //std::vector<double> vec4 = {-1.1304,    0.3562,    1.4949,    -0.2138,    1.4975,    0.4965};
    std::vector<double> upplace = {0.1169,    0.1148,    2.1384,    -0.7205,    1.5296,    0.1075};
    //std::vector<double> vec6 = {0.1819,    0.0878,    2.0569,    -0.5112,    1.6253,    0.1503};
    //std::vector<double> vec7 = {0.0086,    -0.5819,    2.5447,    -1.6535,    1.5097,    0.0248};
    std::vector<double> pick1 = {-1.3638,    0.713,    1.7685,    -1.0915,    1.8396,    -1.1517};  //shortp_p
    std::vector<double> pick2 = {-1.3825,    0.6779,    1.9907,    -1.4832,    1.571,    0.1656};   //hand_w
    std::vector<double> pick3 = {-1.3205,    0.7838,    1.523,    -0.6882,    1.4947,    -0.9488};   //bend_p
    std::vector<double> pick4 = {-1.258,    0.8315,    1.4399,    -0.6625,    1.6846,    -1.7244};	  //hand_w
    std::vector<double> pick5 = {-1.2748,    0.4828,    1.5727,    -0.5112,    1.5476,    -0.4056};   //bend_p


    std::vector<double> place1 = {0.4716,    0.0663,    2.1607,    -0.6571,    1.5231,    0.4388};

    std::vector<double> place2 = {0.4716,    0.0663,    2.1607,    -0.6571,    1.5231,    0.4388};
    std::vector<double> place3 = {0.1169,    0.1148,    2.1384,    -0.7205,    1.5296,    0.1075};

    std::vector<double> place4 = {0.1169,    0.1148,    2.1384,    -0.7205,    1.5296,    0.1075};
    std::vector<double> upbox = {1.4898,    -0.0738,    1.7937,    -0.188,    1.5308,    -0.0852};
    std::vector<double> box={1.5094,    -0.5534,    1.635,    -0.6105,    1.5308,    -0.085};



    TargetVelocity = {0, 0, 0, 0, 0, 0};

    g_objposition[0] = 2.0;
    robot_motion_states.data = 1;

    while(ros::ok())
    {
        if(robot_motion_states.data == 1)
        {
            SynchronousTime = 1;
            robot_motion_states.data = 2;
            if(!ReflexxesPositionRun(*IP_position, origin, TargetVelocity, SynchronousTime))
                break;

            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);



            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

        }
        else if(robot_motion_states.data == 2) 
        {

            SynchronousTime = 10;
            robot_motion_states.data = 3;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        if(robot_motion_states.data == 3) 
        {
            if(arrive_flag)
            {
                SynchronousTime = 5;
                robot_motion_states.data = 4;
                if(!ReflexxesPositionRun(*IP_position, pick1, TargetVelocity, SynchronousTime))
                    break;  
                set_gripper(1); 
                pub_ctrl.publish(gripper_command);

                sleep(2);
                arrive_flag = false;
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

            }
        }
        else if(robot_motion_states.data == 4) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 5;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 5) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 6;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 6) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 7;
            if(!ReflexxesPositionRun(*IP_position, place1, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 7) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 8;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 8) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 9;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 9) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 10;
            if(!ReflexxesPositionRun(*IP_position, pick2, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 10) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 11;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 11) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 12;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 12) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 13;
            if(!ReflexxesPositionRun(*IP_position, place2, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 13) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 14;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
//add



//grasp4 grasp5
        else if(robot_motion_states.data == 14) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 15;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 15) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 16;
            if(!ReflexxesPositionRun(*IP_position, pick3, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 16) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 17;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 17) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 18;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 18) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 20;
            if(!ReflexxesPositionRun(*IP_position, place3, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 20) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 21;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 21)
        {

            SynchronousTime = 5;
            robot_motion_states.data = 22;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 22)
        {

            SynchronousTime = 5;
            robot_motion_states.data = 23;
            if(!ReflexxesPositionRun(*IP_position, pick4, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 23)
        {

            SynchronousTime = 3;
            robot_motion_states.data = 24;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data ==24) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 25;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 25) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 26;
            if(!ReflexxesPositionRun(*IP_position, place4, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 26) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 27;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 27) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 28;
            if(!ReflexxesPositionRun(*IP_position, origin, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 28) 
        {
            if(arrive_flag){
            SynchronousTime = 3;
            robot_motion_states.data = 29;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
        }
        else if(robot_motion_states.data == 29) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 30;
            if(!ReflexxesPositionRun(*IP_position, place4, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 30) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 31;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 31) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 32;
            if(!ReflexxesPositionRun(*IP_position, upbox, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 32) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 33;
            if(!ReflexxesPositionRun(*IP_position, box, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 33) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 34;
            if(!ReflexxesPositionRun(*IP_position, upbox, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
         else if(robot_motion_states.data == 34) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 35;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 35) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 36;
            if(!ReflexxesPositionRun(*IP_position, place3, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 36) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 37;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 37) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 38;
            if(!ReflexxesPositionRun(*IP_position, upbox, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 38) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 39;
            if(!ReflexxesPositionRun(*IP_position, box, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 39) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 40;
            if(!ReflexxesPositionRun(*IP_position, upbox, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
         else if(robot_motion_states.data == 40) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 41;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 41) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 42;
            if(!ReflexxesPositionRun(*IP_position, place2, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 42) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 43;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 43) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 44;
            if(!ReflexxesPositionRun(*IP_position, upbox, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 44) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 45;
            if(!ReflexxesPositionRun(*IP_position, box, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 45) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 46;
            if(!ReflexxesPositionRun(*IP_position, upbox, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
         else if(robot_motion_states.data == 46) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 47;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 47) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 48;
            if(!ReflexxesPositionRun(*IP_position, place1, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 48) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 49;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 49) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 50;
            if(!ReflexxesPositionRun(*IP_position, upbox, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 50) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 51;
            if(!ReflexxesPositionRun(*IP_position, box, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 51) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 52;
            if(!ReflexxesPositionRun(*IP_position, upbox, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
         else if(robot_motion_states.data == 52) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 53;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 54) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 55;
            if(!ReflexxesPositionRun(*IP_position, origin, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else{

	    break;
		}
    }
    ROS_WARN("Bin Picking Demo shutdown");
    delete IP_position;
    delete [] T;
    delete [] q;
    return 0;
}


bool BinPicking_Demo2()
{
    // TM5.setJointSpdModeON();
    // print_info("joint velocity control mode ON...");

    bool run_succeed = true;
    bool table_to_box = false;
    bool skip_flag = false;
    int  pass = 0;
    int finalpose_flag = 1;
    std::vector<double>  final_position(6);
    double SynchronousTime = 7;
    std::vector<double> TargetPosition(6), TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6);
    double *T = new double [16];
    double *q = new double [6];
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
    
    std::vector<double> origin = {0.0086,    -0.5819,    2.5447,    -1.6535,    1.5097,    0.0248};
    std::vector<double> upgrasp = {-1.1304,    0.3562,    1.4949,    -0.2138,    1.4975,    0.4965};
    //std::vector<double> vec3 = {-1.1554,    0.6884,    1.7523,    -0.9856,    1.5643,    -0.9065};
    //std::vector<double> vec4 = {-1.1304,    0.3562,    1.4949,    -0.2138,    1.4975,    0.4965};
    std::vector<double> upplace = {0.1819,    -0.1121,    1.8178,    -0.1152,    1.6254,    0.1503};
    //std::vector<double> vec6 = {0.1819,    0.0878,    2.0569,    -0.5112,    1.6253,    0.1503};
    //std::vector<double> vec7 = {0.0086,    -0.5819,    2.5447,    -1.6535,    1.5097,    0.0248};
    std::vector<double> pick1 = {-1.1687,    0.8239,    1.4191,    -0.6121,    1.5647,    0.3339};
    std::vector<double> pick2 = {-1.0681,    0.8800,    1.3167,    -0.5395,    1.615,    0.1075};
    std::vector<double> pick3 = {-1.2776,    0.794,    1.4816,    -0.7009,    1.5127,    0.326};


    std::vector<double> place1 = {0.4426,    0.0414,    2.1225,    -0.6182,    1.5294,    0.4384};
    std::vector<double> place2 = {0.1819,    0.0878,    2.0569,    -0.5112,    1.6253,    0.1503};
	std::vector<double> place3 = {0.1129,    0.051,    2.1501,    -0.6611,    1.5294,    0.1075};




    TargetVelocity = {0, 0, 0, 0, 0, 0};

    g_objposition[0] = 2.0;
    robot_motion_states.data = 1;

    while(ros::ok())
    {
    	if(robot_motion_states.data == 1)
    	{
    		SynchronousTime = 1;
            robot_motion_states.data = 2;
            if(!ReflexxesPositionRun(*IP_position, origin, TargetVelocity, SynchronousTime))
                break;

            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);



            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

    	}
	    else if(robot_motion_states.data == 2) 
        {

            SynchronousTime = 10;
            robot_motion_states.data = 3;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 3) 
        {
            if(arrive_flag)
            {
                SynchronousTime = 5;
                robot_motion_states.data = 4;
                if(!ReflexxesPositionRun(*IP_position, pick1, TargetVelocity, SynchronousTime))
                    break;  
                set_gripper(1); 
                pub_ctrl.publish(gripper_command);

                sleep(2);

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
        }
        else if(robot_motion_states.data == 4) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 5;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 5) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 6;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 6) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 7;
            if(!ReflexxesPositionRun(*IP_position, place1, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 7) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 8;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 8) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 9;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 9) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 10;
            if(!ReflexxesPositionRun(*IP_position, pick2, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 10) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 11;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 11) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 12;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 12) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 13;
            if(!ReflexxesPositionRun(*IP_position, place2, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 13) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 14;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }

        else if(robot_motion_states.data == 14) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 15;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 15) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 16;
            if(!ReflexxesPositionRun(*IP_position, pick3, TargetVelocity, SynchronousTime))
                break;  
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
            sleep(2);

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 16) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 17;
            if(!ReflexxesPositionRun(*IP_position, upgrasp, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 17) 
        {

            SynchronousTime = 5;
            robot_motion_states.data = 18;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 18) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 20;
            if(!ReflexxesPositionRun(*IP_position, place3, TargetVelocity, SynchronousTime))
                break;  
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            sleep(1);
            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else if(robot_motion_states.data == 20) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 21;
            if(!ReflexxesPositionRun(*IP_position, upplace, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }


        else if(robot_motion_states.data == 21) 
        {

            SynchronousTime = 3;
            robot_motion_states.data = 22;
            if(!ReflexxesPositionRun(*IP_position, origin, TargetVelocity, SynchronousTime))
                break;  

            RobotState.publish(robot_motion_states);
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
        }
        else{

	    break;
		}
    }
    ROS_WARN("Bin Picking Demo shutdown");
    delete IP_position;
    delete [] T;
    delete [] q;
    return 0;
}





bool BinPicking_Demo()
{
    // TM5.setJointSpdModeON();
    // print_info("joint velocity control mode ON...");

    bool run_succeed = true;
    bool table_to_box = false;
    bool skip_flag = false;
    int  pass = 0;
    int finalpose_flag = 1;
    std::vector<double>  final_position(6);
    double SynchronousTime = 7;
    std::vector<double> TargetPosition(6), TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6);
    double *T = new double [16];
    double *q = new double [6];
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
    
    //0523 箱子image_pose
    std::vector<double> ImageCapture1    = {0.7616,    0.1689,    1.4458,    -0.0612,    1.5654,    -0.8033};
    //0524 桌角image_pose
    std::vector<double> ImageCapture2    = {0.5347,    -0.5977,    2.1283,    0.044,    1.5644,    0.5381};

    //0524中間檢查點
    std::vector<double> RecoverPosition = {1.0773,    -0.0594,    1.7915,    -0.1471,    1.571,    -0.4834};

    std::vector<double> BoxAbovePosition = {0.9997,    0.2791,    1.5423,    -0.4047,    1.6835,    -0.5932};  //0523 above box
    std::vector<double> ToolPose = {0.8737,    0.1277,    1.6537,    -0.1775,    1.5723,    0.1935};


    TargetVelocity = {0, 0, 0, 0, 0, 0};

    g_objposition[0] = 2.0;
    robot_motion_states.data = 1;

    while(ros::ok())
    {
        if(g_objposition[0] == 1.0 && position_fill && rotation_fill)
        {
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                TargetPosition[i] = g_objposition[i+1];
            if(robot_motion_states.data == 1)  
            {
                // Picking_above_pose (computational result)
                ROS_WARN("HeyHey");
                skip_flag = false;
                // SynchronousTime = 5;//1.5;
                SynchronousTime = 3;
                robot_motion_states.data = 2;
                if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;

                if(g_ObjClass == 1)     //Object1
                    set_gripper(1);
                else if(g_ObjClass == 3)     //Object3
                    set_gripper(2);
                pub_ctrl.publish(gripper_command);

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 2) 
            {
                // Object_pose (computational result)
                // SynchronousTime = 5;//2;
                ROS_WARN("HoHo");
                SynchronousTime = 3;
                robot_motion_states.data = 3;
                if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;  
                // getchar();
                // set_gripper(0); 
                if(g_ObjClass == 1)     //Object1
                    set_gripper(0);
                else if(g_ObjClass == 3)     //Object3
                    set_gripper(3);
                pub_ctrl.publish(gripper_command);

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            }
            else if(robot_motion_states.data == 3)
            {
                // After_picking_pose
                // If move from box to table : Box_above_pose
                // If move from table to box : Picking_above_pose
                robot_motion_states.data = 4;

                if (!table_to_box)
                {
                    // SynchronousTime = 0.7;
                    SynchronousTime = 3;
                    // if(!ReflexxesPositionRun(TM5, *IP_position, BoxAbovePosition, TargetVelocity, SynchronousTime))
                    //     break;

                    if(g_ObjClass == 1){
                        if(!ReflexxesPositionRun(*IP_position, ImageCapture1, TargetVelocity, SynchronousTime))
                            break;
                    }
                    else if(g_ObjClass == 3){
                        if(!ReflexxesPositionRun(*IP_position, BoxAbovePosition, TargetVelocity, SynchronousTime))
                            break;
                        if(!ReflexxesPositionRun(*IP_position, ToolPose, TargetVelocity, SynchronousTime))
                            break;
                    }
                    

                }
                else{
                    
                    // SynchronousTime = 0.8;
                    SynchronousTime = 3;
                    if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                        break;
                    if(!ReflexxesPositionRun(*IP_position, ImageCapture2, TargetVelocity, SynchronousTime))
                        break;
                }
                sleep(1);

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                // ROS_INFO("Enter to CONTINUE");
                // getchar();

            }
            else if(robot_motion_states.data == 4)
            {
                // Prepare_to_place_pose + placing_pose
                // If move from box to table : Table_above_pose + Final_pose
                // If move from table to box : RecoverPosition + Box_above_pose + Final_pose

                SynchronousTime = 5;
                robot_motion_states.data = 5;

                if(!table_to_box)
                {
                    // ReflexxesPositionSafetyRun
                    // if(ReflexxesPositionSafetyRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime) == SMOOTHSTOP)
                    //     break;
                    Hexagon_Demo(finalpose_flag, *IP_position);
                    // usleep(300 * 1000);
                    // //table五筒
                    // for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    // {
                    //     CurrentPosition[i] = g_robot_joint_angle[i];
                    // }

                    // for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    // {
                    //     IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
                    //     IP_position->CurrentVelocityVector->VecData[i] = 0.0;
                    //     IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

                    //     IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
                    //     IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
                    //     IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
                    // }
                    usleep(100 * 1000);

                    

                    if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                        break;

                    
                    if(finalpose_flag == 1)
                        final_position = {0.115,    -0.3286,    2.2883,    -0.4245,    1.5736,    0.1127};

                    else if(finalpose_flag == 2)
                        final_position = {0.454,    -0.3543,    2.2871,    -0.3951,    1.5628,    0.4491};

                    else if(finalpose_flag == 3)
                        final_position = {0.115,    -0.3286,    2.2883,    -0.4245,    1.5736,    0.1127};

                    else if(finalpose_flag == 4)
                        final_position = {0.454,    -0.3543,    2.2871,    -0.3951,    1.5628,    0.4491};

                    else if(finalpose_flag == 5)
                        final_position = {0.115,    -0.3286,    2.2883,    -0.4245,    1.5736,    0.1127};

                    SynchronousTime = 1.5;
                    // SynchronousTime = 5;
                    if(!ReflexxesPositionRun(*IP_position, final_position, TargetVelocity, SynchronousTime))
                        break;
                    finalpose_flag = finalpose_flag + 1;
                    //change mode
                    if(finalpose_flag >= 6)
                    {
                        table_to_box = false;
                        skip_flag = true;
                        finalpose_flag = 1;
                    }
                    

                }
                else
                {
                    // SynchronousTime = 1.5;
                    SynchronousTime = 5;
                    
                    if(!ReflexxesPositionRun(*IP_position, RecoverPosition, TargetVelocity, SynchronousTime))
                        break;

                    // SynchronousTime = 0.5;
                    SynchronousTime = 5;
                    if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                        break;

                    //箱子內彷初始排法
                    if(finalpose_flag == 1)
                        final_position = {1.1699,    0.2217,    1.764,    -0.3986,    1.5584,    -0.4213};


                    else if(finalpose_flag == 2)
                        final_position = {1.0263,    0.3543,    1.5631,    -0.3327,    1.5567,    -0.5779};


                    else if(finalpose_flag == 3)
                        final_position = {1.2296,    0.414,    1.4451,    -0.2681,    1.557,    -0.3595};


                    else if(finalpose_flag == 4)
                        final_position = {1.0934,    0.5824,    1.1938,    -0.1855,    1.5565,    -0.4913};


                    else if(finalpose_flag == 5)
                        final_position = {1.11,    0.3693,    1.453,    -0.2227,    1.5567,    -0.4753};


                    // SynchronousTime = 0.5;
                    SynchronousTime = 2;
                    if(!ReflexxesPositionRun(*IP_position, final_position, TargetVelocity, SynchronousTime))
                        break;
                    
                    finalpose_flag = finalpose_flag + 1;

                    //中止program
                    if(finalpose_flag >= 6)
                    {
                        table_to_box = false;
                        finalpose_flag = 1;
                        // set_gripper(1); 
                        if(g_ObjClass == 1)     //Object1
                            set_gripper(1);
                        else if(g_ObjClass == 3)     //Object3
                            set_gripper(2);
                        pub_ctrl.publish(gripper_command);
                        sleep(1);
                        // SynchronousTime = 0.5;
                        SynchronousTime = 1.5;
                        if(!ReflexxesPositionRun(*IP_position, ImageCapture1, TargetVelocity, SynchronousTime))
                            break;
                        SynchronousTime = 1.5;
                        if(!ReflexxesPositionRun(*IP_position, ImageCapture2, TargetVelocity, SynchronousTime))
                            break;
                        return(0);

                    }

                }

                usleep(500 * 1000);

                //close gripper
                // set_gripper(1); 
                if(g_ObjClass == 1)     //Object1
                    set_gripper(1);
                else if(g_ObjClass == 3)     //Object3
                    set_gripper(2);
                pub_ctrl.publish(gripper_command);

                usleep(500 * 1000);

                // if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                //             break;       

                

                
                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);

                
                // ROS_INFO("Enter to CONTINUE");
                // getchar();
                
            }
            else if(robot_motion_states.data == 5)
            {
                // Mid_point
                // If move from box to table : Image1
                // If move from table to box : Image1
                robot_motion_states.data = 8;

                if (!table_to_box)
                {
                    // SynchronousTime = 0.7;
                    SynchronousTime = 2;
                    if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                        break;

                }
                else
                {
                    
                    // SynchronousTime = 0.8;
                    SynchronousTime = 2;
                    if(!ReflexxesPositionRun(*IP_position, TargetPosition, TargetVelocity, SynchronousTime))
                    break;
                }
                

                RobotState.publish(robot_motion_states);
                ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
                // ROS_INFO("Enter to CONTINUE");
                // getchar();
                robot_motion_states.data = 1;
            }
            

            else{}

            g_objposition[0] = 0.0;
            position_fill = false;
            rotation_fill = false;
            sleep(1);
        }
        else if(g_objposition[0] == 2.0)
        {

            // robot_motion_states.data = IMAGE_CAPTURE_MOVING;
            // RobotState.publish(robot_motion_states);



            // SynchronousTime = 1.5;
            SynchronousTime = 5;
            if(!table_to_box)
            {
                Image1PTP();
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
                // if(!ReflexxesPositionRun(*IP_position, ImageCapture1, TargetVelocity, SynchronousTime))
                //     break;
            }
            else
            {
                
                if(!skip_flag)
                {

                    if(!ReflexxesPositionRun(*IP_position, ImageCapture1, TargetVelocity, SynchronousTime))
                            break;
                }
                else
                {
                    skip_flag = false;
                }

                if(!ReflexxesPositionRun(*IP_position, ImageCapture2, TargetVelocity, SynchronousTime))
                    break;
            }
            robot_motion_states.data = IMAGE_CAPTURE_POSE;
            ROS_WARN("Robot State = %d Complete",robot_motion_states.data);
            RobotState.publish(robot_motion_states);
            g_objposition[0] = 0.0;
            skip_flag = true;
        }
    }



    ROS_WARN("Bin Picking Demo shutdown");
    delete IP_position;
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

void ObjPosition_callback(const geometry_msgs::Vector3::ConstPtr& array)
{
    g_objposition[1] = array->x;
    g_objposition[2] = array->y;
    g_objposition[3] = array->z;
    position_fill = true;
    ROS_INFO("g_objposition[1] = %f",g_objposition[1] );
    ROS_INFO("g_objposition[2] = %f",g_objposition[2] );
    ROS_INFO("g_objposition[3] = %f",g_objposition[3] );
}

void ObjRotation_callback(const geometry_msgs::Vector3::ConstPtr& array)
{
    g_objposition[4] = array->x;
    g_objposition[5] = array->y;
    g_objposition[6] = array->z;
    rotation_fill = true;
    ROS_INFO("g_objposition[4] = %f",g_objposition[4] );
    ROS_INFO("g_objposition[5] = %f",g_objposition[5] );
    ROS_INFO("g_objposition[6] = %f",g_objposition[6] );
}

void MotionLock_callback(const std_msgs::Int32::ConstPtr& cmd)
{
    g_objposition[0] = cmd->data;
    ROS_INFO("g_objposition[0] = %f",g_objposition[0] );
}

void arriveCB(const std_msgs::Bool::ConstPtr& flag)
{
    arrive_flag = flag->data;

}
void distance_callback(const std_msgs::Float32::ConstPtr& distance)
{
    g_distance = distance->data;
    ROS_ERROR("minimum_distance = %10.3f",distance->data);
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

void ObjClass_callback(const std_msgs::Int32::ConstPtr& objclass)
{
    g_ObjClass = objclass->data;
    ROS_INFO("g_ObjClass = %d",g_ObjClass );
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

    ros::init(argc, argv, "tm_action_picking");
    ros::NodeHandle node_handle;

    RobotState = node_handle.advertise<std_msgs::Int32>("/robot_motion_states",10);
    

    ros::AsyncSpinner spinner(7);
    ros::Subscriber position_sub    = node_handle.subscribe("/eff/kinect_merge/closest_pt_tracking", 1,position_callback);
    ros::Subscriber tool_position_sub   = node_handle.subscribe("/pc1_pt", 1, tool_position_callback);
    ros::Subscriber ObjClass_sub  = node_handle.subscribe("object_class", 1,ObjClass_callback);
    ros::Subscriber constraint_sub  = node_handle.subscribe("/body/kinect_merge/closest_pt_tracking", 1,constraint_callback);
    ros::Subscriber ObjPosition_sub = node_handle.subscribe("/object_position", 1,ObjPosition_callback);
    ros::Subscriber ObjRotation_sub = node_handle.subscribe("/object_rotation", 1,ObjRotation_callback);
    ros::Subscriber MotionLock_sub  = node_handle.subscribe("/lock_motion", 1,MotionLock_callback);
    ros::Subscriber robot_status_sub = node_handle.subscribe("feedback_states", 1000,TMmsgCallback);
    ros::Subscriber arrive_sub = node_handle.subscribe("arrive_states", 1, arriveCB);

    vel_client = node_handle.serviceClient<tm_msgs::SetVelocity>("tm_driver/set_velocity");
    script_client = node_handle.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    event_client = node_handle.serviceClient<tm_msgs::SetEvent>("tm_driver/set_event");
    pos_client = node_handle.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    pub_ctrl = node_handle.advertise<robotiq_controller::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput",10);

    tm_msgs::SetPositions pos_srv;
    tm_msgs::SetEvent event_srv;
    tm_msgs::SendScript script_srv;

    spinner.start();

    // if (!(ros::param::get("~robot_ip", host)))
    // {
    //     if (argc > 1)
    //         host = argv[1];
    //     else
    //         exit(1);
    // }

    // const int STDIN = 0;
    // int sockfd = -1;
    bool fgRun = false;

    // for (int i = 0; i < argc; i++)
    // {
    //     printf("[DEBUG] arg%d:= %s\n", i, argv[i]);
    // }
    // host = argv[1];
    // printf("[ INFO] host: %s\n", host.c_str());


    // TmDriver TmRobot(data_cv, data_cv_rt, host, 0);

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
        else if (strncmp(cstr, "start", 5) == 0)
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
        else if (strncmp(cstr, "halt", 4) == 0)
        {
            print_info("halt");
            break;
        }
        else if(strncmp(cstr, "home", 4) == 0)
        {
            std::vector<double> vec1 = {0,0,0,0,0,0};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to home");
        }
        else if(strncmp(cstr, "ready", 5) == 0)
        {
            std::vector<double> vec1 = {0.0,    -0.4777,    1.9319,    -1.4537,    1.5708,    0.0};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to ready position");
        }
        else if(strncmp(cstr, "gopen", 6) == 0)
        {
            set_gripper(0); 
            pub_ctrl.publish(gripper_command);
        }
        else if(strncmp(cstr, "gclose", 7) == 0)
        {
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);
        }
        else if(strncmp(cstr, "gopen0", 7) == 0)
        {
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 0; 
            pub_ctrl.publish(gripper_command);
            
        }
        else if(strncmp(cstr, "test1", 6) == 0)
        {
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 50; 
            pub_ctrl.publish(gripper_command);


            // 此為gclose
            // gripper_command.rACT = 1;
            // gripper_command.rGTO = 1;
            // gripper_command.rSP  = 200;
            // gripper_command.rFR  = 0;
            // gripper_command.rPR = 170;
            
        }
        else if(strncmp(cstr, "test2", 6) == 0)
        {
            gripper_command.rACT = 1;
            gripper_command.rGTO = 1;
            gripper_command.rSP  = 200;
            gripper_command.rFR  = 0;
            gripper_command.rPR  = 150; 
            pub_ctrl.publish(gripper_command);
            
        }
        else if(strncmp(cstr, "1", 1) == 0)
        {
            std::vector<double> vec1 = {0.9945,    0.7611,    0.4229,    0.3955,    1.5723,    -0.5669};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 1");
        }
        else if(strncmp(cstr, "2", 1) == 0)
        {
            std::vector<double> vec1 = {1.0334,    0.429,    0.9594,    0.1907,    1.5728,    -0.5289};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 2");
        }
        else if(strncmp(cstr, "3", 1) == 0)
        {
            std::vector<double> vec1 = {1.1081,    0.7451,    0.4516,    0.3822,    1.5728,    -0.4536};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 3");
        }
        else if(strncmp(cstr, "4", 1) == 0)
        {
            std::vector<double> vec1 = {1.174,    0.4032,    0.9925,    0.1781,    1.5692,    -0.3844};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 4");
        }
        else if(strncmp(cstr, "5", 1) == 0)
        {
            std::vector<double> vec1 = {1.0418,    0.3586,    0.6946,    0.5091,    1.5725,    -0.5205};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 5");
        }
    
        else if(strncmp(cstr, "image1", 6) == 0)
        {
            std::vector<double> vec1 = {0.7616,    0.1689,    1.4458,    -0.0612,    1.5654,    -0.8033};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 5");
        }
        else if(strncmp(cstr, "upplace", 7) == 0)
        {
            std::vector<double> vec1 = {0.1169,    0.1148,    2.1384,    -0.7205,    1.5296,    0.1075};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("upplace");
        }
        else if(strncmp(cstr, "place1", 6) == 0)
        {
            std::vector<double> vec1 = {0.4716,    0.0663,    2.1607,    -0.6571,    1.5231,    0.4388};;

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 10;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("place1");
        }
        else if(strncmp(cstr, "place2", 6) == 0)
        {
            std::vector<double> vec1 = {0.4716,    0.0663,    2.1607,    -0.6571,    1.5231,    0.4388};


            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("place2");
        }
        else if(strncmp(cstr, "place3", 6) == 0)
        {
            std::vector<double> vec1 = {0.1169,    0.1148,    2.1384,    -0.7205,    1.5296,    0.1075};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("place3");
        }
        else if(strncmp(cstr, "place4", 6) == 0)
        {
            std::vector<double> vec1 = {0.1169,    0.1148,    2.1384,    -0.7205,    1.5296,    0.1075};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("place3");
        }
        else if(strncmp(cstr, "origin", 6) == 0)
        {
            std::vector<double> origin = {0.0086,    -0.5819,    2.5447,    -1.6535,    1.5097,    0.0248};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = origin;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("ugrasp");
        }
        else if(strncmp(cstr, "ugrasp", 6) == 0)
        {
            std::vector<double> upgrasp = {-1.1304,    0.3562,    1.4949,    -0.2138,    1.4975,    0.4965};

            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = upgrasp;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("ugrasp");
        }

        else if (strncmp(cstr, "demov2", 6) == 0)
        {
            // TmRobot.setDigitalOutputEE(0,true);

            script_srv.request.id = "Vstart";
            script_srv.request.script = VStart_cmd;
            script_client.call(script_srv);
            print_info("joint velocity control mode ON...");

            BinPicking_Demo2();
            //StaticRobotAvoidnace(TmRobot);
            script_srv.request.id = "spdmodeoff";
            script_srv.request.script = VStop_cmd;
            script_client.call(script_srv);
            
            print_info("joint vlocity control mode OFF...");
            


        }

        else if (strncmp(cstr, "demov3", 6) == 0)
        {
            // TmRobot.setDigitalOutputEE(0,true);

            script_srv.request.id = "Vstart";
            script_srv.request.script = VStart_cmd;
            script_client.call(script_srv);
            print_info("joint velocity control mode ON...");

            BinPicking_Demo3();
            //StaticRobotAvoidnace(TmRobot);
            script_srv.request.id = "spdmodeoff";
            script_srv.request.script = VStop_cmd;
            script_client.call(script_srv);
            
            print_info("joint vlocity control mode OFF...");
            


        }
        else if (strncmp(cstr, "demov4", 6) == 0)
        {
            // TmRobot.setDigitalOutputEE(0,true);

            script_srv.request.id = "Vstart";
            script_srv.request.script = VStart_cmd;
            script_client.call(script_srv);
            print_info("joint velocity control mode ON...");

            BinPicking_Demo4();
            //StaticRobotAvoidnace(TmRobot);
            script_srv.request.id = "spdmodeoff";
            script_srv.request.script = VStop_cmd;
            script_client.call(script_srv);
            
            print_info("joint vlocity control mode OFF...");
            


        }

//1201end
        else if(strncmp(cstr, "image2", 6) == 0)
        {
            std::vector<double> vec1 = {0.5347,    -0.5977,    2.1283,    0.044,    1.5644,    0.5381};
            pos_srv.request.motion_type = 1;
            pos_srv.request.positions = vec1;
            pos_srv.request.velocity = 30;
            pos_srv.request.acc_time = 0.2;
            pos_srv.request.blend_percentage = 0;
            pos_srv.request.fine_goal = false;
            pos_client.call(pos_srv);

            print_info("Back to position 5");
        }

    
        else if (strncmp(cstr, "gotest", 6) == 0)
        {
            // TmRobot.setDigitalOutputEE(0,true);
            set_gripper(1); 
            pub_ctrl.publish(gripper_command);

            script_srv.request.id = "Vstart";
            script_srv.request.script = VStart_cmd;
            script_client.call(script_srv);
            print_info("joint velocity control mode ON...");

            BinPicking_Demo();
            //StaticRobotAvoidnace(TmRobot);
            script_srv.request.id = "spdmodeoff";
            script_srv.request.script = VStop_cmd;
            script_client.call(script_srv);
            
            print_info("joint vlocity control mode OFF...");
            


        }
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
    //ros::waitForShutdown();

    printf("[ info] TM_ROS: shutdown\n");

    // script_srv.request.id = "spdmodeoff";
    // script_srv.request.script = VStop_cmd;

    // script_client.call(script_srv);
    // print_info("joint vlocity control mode OFF...");

    return 0;
}
