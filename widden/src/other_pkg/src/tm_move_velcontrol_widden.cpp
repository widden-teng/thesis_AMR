
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
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
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

#include "robotiq_controller/Robotiq2FGripper_robot_output.h"

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
#define REPEATABILITY 0.0028 //0.01//0.00005 //TODO: modify attractive force ; 用來判斷是否到目標點

#define GRIPPER_LENGTH 0.17//0.266

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
std::vector<double> g_robot_tool_pos(6);

std::vector<geometry_msgs::Point> init_path;
std::vector<geometry_msgs::Point> static_path;
std::vector<geometry_msgs::Point> dynamic_path;


ros::Publisher marker_pub, reset_monitoring_pub;
visualization_msgs::Marker edge;

int init_path_num;
bool get_init_path = false;
int g_dangerous = 1;
int g_dangerous_prev = 1;
bool get_dynamic_path = false;
int start_cmd = 0;

ros::ServiceClient vel_client;
ros::ServiceClient script_client;
ros::ServiceClient event_client;
ros::ServiceClient pos_client;

ros::Publisher pub_ctrl;
robotiq_controller::Robotiq2FGripper_robot_output gripper_command;

std::string exit_cmd = "ScriptExit()";
std::string VStart_cmd = "ContinueVJog()";
std::string VStop_cmd = "StopContinueVmode()";

ofstream end_pose_txt, distance_error;

bool ReflexxesPositionRun(  RMLPositionInputParameters &InputState, 
                            std::vector<double> TargetPosition,
                            std::vector<double> TargetVelocity, 
                            double SynTime);

//使gripper各項數值歸零(將夾爪閉合)
void reset_gripper()
{
    // rSP是夾爪開合距離; rFR是多少力夾爪就不會動; 兩者數值皆為0~255
    gripper_command.rACT = 0;
    gripper_command.rGTO = 0;
    gripper_command.rATR = 0;
    gripper_command.rPR  = 0;
    gripper_command.rSP  = 0;
    gripper_command.rFR  = 0;  
}

//初始化夾爪(將夾爪開啟)
void init_gripper()
{
    gripper_command.rACT = 1;
    gripper_command.rGTO = 1;
    gripper_command.rATR = 0;
    gripper_command.rPR  = 0;
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
        gripper_command.rPR = 0;
    }
    if(mode == 1)   //Close
    {
        gripper_command.rACT = 1;
        gripper_command.rGTO = 1;
        gripper_command.rSP  = 200;
        gripper_command.rFR  = 0;
        gripper_command.rPR = 80;
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
    //將夾爪資訊pub給Robotiq2FGripperRobotOutput
    pub_ctrl.publish(gripper_command); 
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
        
        // vel_srv 的type為 tm_msgs::SetVelocity
        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = vec;
        // 將速度傳送給"tm_driver/set_velocity"這個server
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
        

        // The area execution in 25ms real time sharp
        //********************************************************
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    std::vector<double> FinalPosition(6);
    // time_s = TR.interface->stateRT->getQAct(FinalPosition);
    printf("=============== Final state of Smooth Stop =========================\n");
    // printf("[ %lf ]  ", time_s);

    // FinalPosition 為現在的姿態
    FinalPosition[0] = g_robot_joint_angle[0];
    FinalPosition[1] = g_robot_joint_angle[1];
    FinalPosition[2] = g_robot_joint_angle[2];
    FinalPosition[3] = g_robot_joint_angle[3];
    FinalPosition[4] = g_robot_joint_angle[4];
    FinalPosition[5] = g_robot_joint_angle[5];
    for (int i = 0; i < NUMBER_OF_DOFS; ++i){
        printf(" %10.4lf ",FinalPosition[i]);
    }
    printf("=============== Final g_robot_tool_pos =========================\n");
    for (int i = 0; i < NUMBER_OF_DOFS; ++i){
        printf(" %10.4lf ",g_robot_tool_pos[i]);
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
// TargetVelocity 為 qd; EffVelocity 為 TargetVelocity
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

        //很下面會更新IP->CurrentVelocityVector
        if(IP->CurrentVelocityVector->VecData[i] != TargetVelocity[i])    
            IP->SelectionVector->VecData[i] = true;
        else
            IP->SelectionVector->VecData[i] = false;
    }
    IP->MinimumSynchronizationTime = SynTime;

    // ********************************************************************

    // 如果fail不知道會改啥？? 我猜會讓下面跑不了
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

    //ResultValue type為 int , default為0
    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

        // 不知道這邊RML->RMLVelocity生成的OP有沒有考慮到IP的速度(現在的速度)
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

        // vel_srv 的type為 tm_msgs::SetVelocity
        vel_srv.request.motion_type = 1;
        vel_srv.request.velocity = VelocityCommand;
        // 將速度傳送給"tm_driver/set_velocity"這個server
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

        //***************************計算誤差*************************
        // Print out commands

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            q[i] = OP->NewPositionVector->VecData[i];
        
        // T 的type為 double* ([16])，第一次為空的(這邊第一次用)
        // 給卡式的資訊, noap(4*4)
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

        if( (DistanceToGoal[0] < REPEATABILITY) && (DistanceToGoal[1] < REPEATABILITY) && (DistanceToGoal[2] < REPEATABILITY))
        {
            ROS_ERROR("DistanceToGoal: %10.4lf %10.4lf %10.4lf", DistanceToGoal[0], DistanceToGoal[1], DistanceToGoal[2]);
            
            char outmsg_tag[200];
            //將與目標點的差異寫入txt
            sprintf(outmsg_tag,"%.7f %.7f %.7f",DistanceToGoal[0], DistanceToGoal[1], DistanceToGoal[2]);
            distance_error << outmsg_tag << endl;

            pass = REACH;
            break;
        }
        //***************************************************************

        // 判斷是否有超過joint的上下線
        if(!CheckJointLimit(q))
        {
            pass = STOP;
            break; 
        }
        // 若有輸入東西,則進判斷; q的話強制停止
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
        // 更新IP資訊
        *IP->CurrentPositionVector     =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector     =  *OP->NewVelocityVector;
        *IP->CurrentAccelerationVector =  *OP->NewAccelerationVector;

        gettimeofday(&tm2, NULL);
        // time_compensation 為while到現在的時間(包含計算送出現在所需移動速度...)
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        
        //用來避免一致發送指令給TM5(頻率太高發送的內容一樣), 至少25ms
        if((24940 - time_compensation) < 1)
            // usleep為毫秒
            usleep(100);
        else
            usleep(24940 - time_compensation);

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    // tt 為總共跑完的時間（一個點到另一個點的時間)
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
        IP->MaxVelocityVector->VecData[i]     = 0.3247; //3.14;
        IP->MaxAccelerationVector->VecData[i] = 3.14;
        IP->TargetPositionVector->VecData[i]  = TargetPosition[i]; 
        IP->TargetVelocityVector->VecData[i]  = TargetVelocity[i];
        IP->SelectionVector->VecData[i]       = true;
    }
    IP->MinimumSynchronizationTime = SynTime;


    if (IP->CheckForValidity())
        printf("Input values are valid, Good!\n");
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
                //ROS_INFO_STREAM("SetPositions to robot");
            }
            else 
                ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
        }
        else
        {
            ROS_ERROR_STREAM("Error SetPositions to robot, QQ");
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
        // EPS 的type為 const float, 值為 1e-6
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

//fix456 代表只考慮X, Y, X; 沒有考慮R, P, Y 
// EFF_Velocity ＝ AttractiveVelocity ＋ PathSmoothVelocity
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

    // 用來找Jacobian
    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_gripper(q, GRIPPER_LENGTH);// Forward_Jacobian_d(q);
    //matrix.block<p,q>(i,j), 一個p行q列的子矩陣，該定義表示從原矩陣中第(i, j)開始，獲取一個p行q列的子矩陣, 且內容不變
    // 這邊會取6*6的原因為有考慮到R, P, Y (但為0, 0, 0)
    Eigen::Matrix<double, 6, 6> Jacobian_12345   = Geometry_Jacobian.block<6,6>(0,0);

    Eigen::MatrixXd jacobian_12345_inv;
    pinv_SVD(Jacobian_12345,jacobian_12345_inv);
    JointSpeed = jacobian_12345_inv * EFFSpeed;
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        qd[i] = JointSpeed(i);

    //ROS_INFO("Generatd qd = %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf", qd[0]*RAD2DEG, qd[1]*RAD2DEG, qd[2]*RAD2DEG, qd[3]*RAD2DEG, qd[4]*RAD2DEG, qd[5]*RAD2DEG);

    double ScalingFactor;
    //怕速度太快, 這邊沒看！！！！！！！！！！！！！！！！！！！！！！！！！！
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


//EFF_Velocity 為 TargetVelocity
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
    //計算現在末端點與本次點的距離差(僅考慮x, y, z)
    Eff2Goal << GoalPoint[0]-TCP_position[3], GoalPoint[1]-TCP_position[7], GoalPoint[2]-TCP_position[11];
    double dis_goal = sqrt( pow(Eff2Goal(0),2) + pow(Eff2Goal(1),2) + pow(Eff2Goal(2),2)     );
    // dis_goal /= STEPSIZE;
    
    //AttractiveVecotor為吸引力的單位向量
    Eigen::Vector3d AttractiveVecotor = Eff2Goal/dis_goal;
    //帶入吸引力公式
    double AttractiveForce = Vmax_att - Vmax_att*exp(-(dis_goal*ShapingFactor_att)/Survelliance_att);

    Eigen::Vector3d AttractiveVelocity = AttractiveForce*AttractiveVecotor; 

    /*** Pass Through Velocity ***/
    Eigen::Vector3d PS_vel_vec;
    PS_vel_vec << PassThrough_Velocity[0], PassThrough_Velocity[1], PassThrough_Velocity[2];
    double PS_vel_norm = sqrt( pow(PS_vel_vec(0), 2) + pow(PS_vel_vec(1), 2) + pow(PS_vel_vec(2), 2) );

    Eigen::Vector3d VelocitySmoothVector = PS_vel_vec/PS_vel_norm;

    double VelocitySmoothForce = Vmax_att*exp(-(dis_goal*ShapingFactor_att)/Survelliance_att);

    //PathSmoothVelocity, VelocitySmoothForce 用來使經過中間點時速度不為0
    Eigen::Vector3d PathSmoothVelocity;
    if(PassThrough_Velocity[0] == 0 && PassThrough_Velocity[1] == 0 && PassThrough_Velocity[2] == 0)
        PathSmoothVelocity << 0, 0, 0;
    else
        PathSmoothVelocity = VelocitySmoothForce*VelocitySmoothVector; 

    // ROS_INFO("Path Smooth Velocity: %10.4lf, %10.4lf, %10.4lf ", PathSmoothVelocity(0), PathSmoothVelocity(1), PathSmoothVelocity(2));
    /*****************************/

    std::vector<double> CurrentPosition = {q[0], q[1], q[2], q[3], q[4], q[5]}; 
    EFF_Velocity = {AttractiveVelocity(0)+PathSmoothVelocity(0), AttractiveVelocity(1)+PathSmoothVelocity(1), AttractiveVelocity(2)+PathSmoothVelocity(2), 0,0,0}; 
    double AttractiveScalar = sqrt(AttractiveVelocity.dot(AttractiveVelocity)); //沒用到, 跟dis_goal 一樣(備註掉)
//    ROS_INFO("Attractive Velocity: %10.4lf ", AttractiveScalar);
    // succeed = tm_jacobian::GetQdfromInverseJacobian(CurrentPosition,EFF_Velocity,action_qd);
    // succeed = GetQdfromLinearJacobian(CurrentPosition,EFF_Velocity,action_qd);

    //計算inv_jacobian 並根據EFF_Velocity 計算出 JointSpeed (action_qd)
    // succeed的值為CheckVelocityLimit()
    succeed = GetQdfromLinearJacobian_fix456(CurrentPosition,EFF_Velocity,action_qd);
        //ROS_WARN("fix 456");

    /*** For White Box Demo **/
    // if(TCP_position[11] > 0.25 && q[2] > 25*DEG2RAD){
    //     succeed = GetQdfromLinearJacobian_fix456(CurrentPosition,EFF_Velocity,action_qd);
    //     ROS_WARN("fix 456");
    // }
    // else if(q[2] <= 25*DEG2RAD && action_qd[2] < 0){
    //     succeed = GetQdfromLinearJacobian3(CurrentPosition,EFF_Velocity,action_qd);
    //     ROS_WARN("J3 protection");
    // }
    // else{
    //     succeed = GetQdfromLinearJacobian_free456(CurrentPosition,EFF_Velocity,action_qd);  
    //     ROS_WARN("free 456");
    // }
    /*************************/

    // if(q[2] <= 25*DEG2RAD && action_qd[2] < 0){
    //     succeed = GetQdfromLinearJacobian3(CurrentPosition,EFF_Velocity,action_qd);
    //     ROS_WARN("J3 protection");
    // }


    //succeed = GetQdfromCartesianConstrain(CurrentPosition,EFF_Velocity,action_qd);
    // succeed = GetQdfromVirtualForceConstrain(CurrentPosition,EFF_Velocity,action_qd);

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

bool Direction(int index, int axis, std::vector<geometry_msgs::Point> path)
{
    //僅有特定x, y, z 各自前後點的方向一致才會回傳true
    if(axis == 0){
        if((path[index-1].x - path[index].x) < 0 && (path[index].x - path[index+1].x) < 0)
            return true;
        else if((path[index-1].x - path[index].x) >= 0 && (path[index].x - path[index+1].x) >= 0)
            return true;
        else
            return false;
    }
    else if(axis == 1){
        if((path[index-1].y - path[index].y) < 0 && (path[index].y - path[index+1].y) < 0)
            return true;
        else if((path[index-1].y - path[index].y) >= 0 && (path[index].y - path[index+1].y) >= 0)
            return true;
        else
            return false;
    }
    else if(axis == 2){
        if((path[index-1].z - path[index].z) < 0 && (path[index].z - path[index+1].z) < 0)
            return true;
        else if((path[index-1].z - path[index].z) >= 0 && (path[index].z - path[index+1].z) >= 0)
            return true;
        else
            return false;
    }

}

bool Hexagon_Demo()
{
    
    bool run_succeed = true; //沒用到
    int  pass = 0;
    double SynchronousTime = 1; //沒用到
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6), PassThrough_velocity;
    double *T = new double [16];
    double *q = new double [6];
    std::vector<geometry_msgs::Point> exec_path;
    int exec_index = 0;
    int state_curr = 1, state_prev = 1;

    // geometry_msgs::Point start_pose, end_pose, pose_step;
    // start_pose.x = 30;
    // start_pose.y = 90;
    // start_pose.z = -30;

    // end_pose.x = -30;
    // end_pose.y = 90;
    // end_pose.z = 0;
    // bool switching_path = false;

    //RMLPositionInputParameters 定義在tm_reflexxes2/ReflexxesTypeII//include//RMLPositionInputParameters.h
    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    // while(get_init_path == false){
    //     ROS_WARN("Not Get Path !!!");
    //     sleep(1);
    // }
    // get_init_path == true;

    // TM5.interface->stateRT->getQAct(CurrentPosition);
    
    //g_robot_joint_angle 的type為 std::vector<double>(全域變數)，於TMmsgCallback中不斷更新
    // 用來將現在位置存入CurrentPosition
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        CurrentPosition[i] = g_robot_joint_angle[i];
    }

    //  IP_position, IP_velocity 的初始化(給當下點 ＆ 使速度與加速度為0)
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

    // 以下兩個for與上面的功能一樣（trivial)
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
    // PassThrough_velocity 的type為 std::vector<double>, 此處為重製
    PassThrough_velocity = {0, 0, 0};

    //dynamic_path 的type為 std::vector<geometry_msgs::Point>
    //static_path 會在 get_path_callback中更新 (Global)
    dynamic_path = static_path;

    /*** APF Guidance Control ***/
    while(ros::ok()){
        // TM5.interface->stateRT->getQAct(CurrentPosition);
        
        //不斷更新現在的位置
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            CurrentPosition[i] = g_robot_joint_angle[i];
        }
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
            // q 的type為 double* ([6])
            q[i] = CurrentPosition[i];
        }
        // T 的type為 double* ([16])，第一次為空的(這邊第一次用)
        // 給卡式的資訊, noap(4*4)
        // 類似用來得到夾爪位置的TF
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        

        /*** Path Swithcing (Initial or Dynamic) ***/
        //state_prev與state_curr default為1
        state_prev = state_curr;
        //g_dangerous default為 1 (全域變數); path_state_callback中會根據msg更改, path_state_callback沒有使用
        state_curr = g_dangerous;
         
        if(g_dangerous == 1){   // Do initial path
            //exec_path 的type為 std::vector<geometry_msgs::Point>
            exec_path = static_path;
            ROS_WARN("Do initial path, go!");
        }
        // get_dynamic_path 的type為bool, defalut為 fasle (Global), 本來會在dynamic_path_callback中呼叫，但備註解掉 ！！！一定不會進！！！！
        else if(g_dangerous == 2 && get_dynamic_path == true){   // Do dynamic path
            // Assign dynamic Path
            // get_dynamic_path = false;
            if(dynamic_path.size() >= 1)
                exec_path = dynamic_path;
            ROS_WARN("Do dynamic path");

            // if(exec_index != exec_path.size()-1)
            //     exec_index = exec_index+1;

            if(exec_index < exec_path.size()-2 && exec_index > 1){
                Eigen::Vector3d vec_a, vec_b;

                vec_a << exec_path[exec_index].x - T[3] , exec_path[exec_index].y - T[7] , exec_path[exec_index].z - T[11];
                vec_b << exec_path[exec_index+1].x - exec_path[exec_index].x, exec_path[exec_index+1].y - exec_path[exec_index].y, exec_path[exec_index+1].z - exec_path[exec_index].z;
            
                double angle = acos((vec_a.dot(vec_b)) / (sqrt(vec_a.dot(vec_a))*sqrt(vec_b.dot(vec_b)))) * (180/M_PI);

                if(angle > 90){
                    exec_index++;
                }
            }

            // exec_index = nearest node in dynamic path
            // switching_path = true;
        }

        /*** Node Decision ***/

        // state_prev 一定為1, state_curr為根據g_dangerous變數更改（path_state_callback中會根據msg更改）; path_state_callback沒有使用
        // !!!!!!!!一定一樣!!!!!!!!!!!
        if(state_prev != state_curr){  // Initial Path to Dynamic Path
            double shortest_dis, dis_tmp;

            ROS_ERROR("Path state switch");
            shortest_dis = sqrt(pow(exec_path[0].x - T[3], 2) + pow(exec_path[0].y - T[7], 2) + pow(exec_path[0].z - T[11], 2));
            exec_index = 0;
            for(int i = 1; i < exec_path.size(); i++){
                dis_tmp = sqrt(pow(exec_path[i].x - T[3], 2) + pow(exec_path[i].y - T[7], 2) + pow(exec_path[i].z - T[11], 2));
                if(dis_tmp < shortest_dis){
                    shortest_dis = dis_tmp;
                    if(i != exec_path.size()-1)
                        exec_index = i+1;
                    else
                        exec_index = i;
                }
            }
            if(exec_index < exec_path.size()-2 && exec_index > 1){
                Eigen::Vector3d vec_a, vec_b;

                vec_a << exec_path[exec_index].x - T[3] , exec_path[exec_index].y - T[7] , exec_path[exec_index].z - T[11];
                vec_b << exec_path[exec_index+1].x - exec_path[exec_index].x, exec_path[exec_index+1].y - exec_path[exec_index].y, exec_path[exec_index+1].z - exec_path[exec_index].z;
            
                double angle = acos((vec_a.dot(vec_b)) / (sqrt(vec_a.dot(vec_a))*sqrt(vec_b.dot(vec_b)))) * (180/M_PI);

                if(angle > 85){
                    exec_index++;
                }
            }
        }   
        ROS_WARN("Next pt : %lf, %lf, %lf", exec_path[exec_index].x, exec_path[exec_index].y, exec_path[exec_index].z);

        // else if(state_prev == 2 && state_curr == 1){  //Dynamic Path to Initial Path
        //     shortest_dis = sqrt(pow(exec_path[0].x - T[3], 2) + pow(exec_path[0].y - T[7], 2) + pow(exec_path[0].z - T[11], 2));
        //     exec_index = 0;
        //     for(int i = 1; i < exec_path.size(); i++){
        //         dis_tmp = sqrt(pow(exec_path[i].x - T[3], 2) + pow(exec_path[i].y - T[7], 2) + pow(exec_path[i].z - T[11], 2));
        //         if(dis_tmp < shortest_dis){
        //             shortest_dis = dis_tmp;
        //             exec_index = i;
        //         }
        //     }
        // }

        // else if(g_dangerous_prev == true && g_dangerous == true)    // Dynamic Path, check whether path update

        // Pass through velocity decision
        // 若為起點或終點，速度為0
        if(exec_index == 0 || exec_index == exec_path.size()-1){
            PassThrough_velocity = {0, 0, 0};
        }
        else{
            for(int i = 0; i < 3; i++){
                if(i == 0){
                    // Direction()   x方向前後點(3個)的方向一致才會為true
                    // 方向一致PassThrough_velocity才不為0
                    if(Direction(exec_index, i, exec_path)){
                        //PassThrough_velocity 的type為 std::vector<double> 
                        PassThrough_velocity[i] = ((exec_path[exec_index-1].x - exec_path[exec_index].x) + (exec_path[exec_index].x - exec_path[exec_index+1].x));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].x - exec_path[exec_index].x) + (exec_path[exec_index].x - exec_path[exec_index+1].x));
                        PassThrough_velocity[i] *= -1;
                    }
                }
                else if(i == 1){
                    if(Direction(exec_index, i, exec_path)){
                        PassThrough_velocity[i] = ((exec_path[exec_index-1].y - exec_path[exec_index].y) + (exec_path[exec_index].y - exec_path[exec_index+1].y));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].y - exec_path[exec_index].y) + (exec_path[exec_index].y - exec_path[exec_index+1].y));
                        PassThrough_velocity[i] *= -1;
                    }
                }
                else if(i == 2){
                    if(Direction(exec_index, i, exec_path)){
                        PassThrough_velocity[i] = ((exec_path[exec_index-1].z - exec_path[exec_index].z) + (exec_path[exec_index].z - exec_path[exec_index+1].z));
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

        //TargetPosition 為現在的點
        TargetPosition = {exec_path[exec_index].x, exec_path[exec_index].y, exec_path[exec_index].z, 0, 0, 0};
        //ROS_WARN("Target x: %lf, y: %lf, z: %lf", exec_path[exec_index].x, exec_path[exec_index].y, exec_path[exec_index].z);

        /*** Robot State Assignment ***/
        // TM5.interface->stateRT->getQAct(CurrentPosition);
        
        //一直更新現在為姿態, 此處與while一開始做的一樣
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            CurrentPosition[i] = g_robot_joint_angle[i];
        }
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
            // q 的type為 double* ([6])
            q[i] = IP_velocity->CurrentPositionVector->VecData[i];
        }
        tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);
        //

        /*** Motion Generation ***/

        // Attractive_qd, TargetVelocity 的type為 std::vector<double> ; Attractive_qd與TargetVelocity 會被更改
        // TargetVelocity 變成 AttractiveVelocity ＋ PathSmoothVelocity  (OnlineAttractiveForceGeneration中為EFF_Velocity)
        // Attractive_qd 變為由上述的TargetVelocity與現在位置計算後 joint_speed; 
        if(OnlineAttractiveForceGeneration(Attractive_qd, T, q, TargetPosition, TargetVelocity, PassThrough_velocity))
        {
            // if(OnlineRepulsiveForceGeneration(Repulsive_qd, T, q, RepulsiveVelocity))
            // {
                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    qd[i] = Attractive_qd[i];
                    // qd[i] = Attractive_qd[i] + Repulsive_qd[i];

                //ROS_INFO("gripper : %10.4lf %10.4lf %10.4lf Atrractive: %10.4lf %10.4lf %10.4lf Repulsive: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11],TargetVelocity[0],TargetVelocity[1],TargetVelocity[2],RepulsiveVelocity[0],RepulsiveVelocity[1],RepulsiveVelocity[2] );
                // ROS_INFO("gripper : %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11]);

                // char outmsg_tag[200];
                // sprintf(outmsg_tag,"%.7f %.7f %.7f",T[3], T[7], T[11]);
                // end_pose_txt << outmsg_tag << endl;
                //ROS_ERROR_STREAM("Writing pose!");

                // ReflexxesVelocityRun用來將joint_speed 經過joint limit，按鍵Q, (現在的速度) 來將速度傳給手臂
                // IP_velocity 為現在的速度（經計算執行的
                pass = ReflexxesVelocityRun(*IP_velocity, qd, TargetPosition, TargetVelocity ,0.2);

                // ReflexxesVelocityRun 是否到達點(其中一個，不是步)
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

        if(exec_index == exec_path.size()){    // Path finish
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
                IP_velocity->CurrentVelocityVector->VecData[i]     = IP_position->CurrentVelocityVector->VecData[i]; //0.0
                IP_velocity->CurrentAccelerationVector->VecData[i] = IP_position->CurrentAccelerationVector->VecData[i]; //0.0
            }

            ROS_WARN("Path finished, smooth stop activate...");
            start_cmd = 0;
            ReflexxesSmoothStop(*IP_velocity,0.25);
            break;
        }
    }

    ROS_WARN("Hexagon_Demo shutdown");
    delete IP_position;
    delete IP_velocity;
    delete [] T;
    delete [] q;

    return 0;
}

bool Hexagon_Demo_Reverse() //沒用到
{
    
    bool run_succeed = true;
    int  pass = 0;
    double SynchronousTime = 1;
    std::vector<double> TargetPosition, TargetVelocity, CurrentPosition(6), qd(6), Attractive_qd(6), Repulsive_qd(6), RepulsiveVelocity(6), PassThrough_velocity;
    double *T = new double [16];
    double *q = new double [6];
    std::vector<geometry_msgs::Point> exec_path;
    int exec_index = 0;
    int state_curr = 1, state_prev = 1;

    geometry_msgs::Point start_pose, end_pose, pose_step;
    start_pose.x = 30;
    start_pose.y = 90;
    start_pose.z = -30;

    end_pose.x = -30;
    end_pose.y = 90;
    end_pose.z = 0;
    // bool switching_path = false;

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

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }
    std::vector<double> InitialPosition = {-0.2481,    0.7712,    1.2664,    -0.4671,    1.5567,    -0.2638};
    std::vector<double> FinalPosition = {0.3343,    -0.4224,    2.3233,    -0.3237,    1.5618,    0.3197};

    TargetVelocity = {0, 0, 0, 0, 0, 0};

    // if(!ReflexxesPositionRun(TM5, *IP_position, InitialPosition, TargetVelocity, SynchronousTime))
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

    dynamic_path = static_path;

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
        

        /*** Path Swithcing (Initial or Dynamic) ***/
        state_prev = state_curr;
        state_curr = g_dangerous;
         
        if(g_dangerous == 1){   // Do initial path
            exec_path = static_path;
            ROS_WARN("Do initial path");
            std::reverse(exec_path.begin(), exec_path.end());
        }
        else if(g_dangerous == 2 && get_dynamic_path == true){   // Do dynamic path
            // Assign dynamic Path
            // get_dynamic_path = false;
            if(dynamic_path.size() >= 1)
                exec_path = dynamic_path;
            ROS_WARN("Do dynamic path");
            std::reverse(exec_path.begin(), exec_path.end());

            if(exec_index < exec_path.size()-2 && exec_index > 1){
                Eigen::Vector3d vec_a, vec_b;

                vec_a << exec_path[exec_index].x - T[3] , exec_path[exec_index].y - T[7] , exec_path[exec_index].z - T[11];
                vec_b << exec_path[exec_index+1].x - exec_path[exec_index].x, exec_path[exec_index+1].y - exec_path[exec_index].y, exec_path[exec_index+1].z - exec_path[exec_index].z;
            
                double angle = acos((vec_a.dot(vec_b)) / (sqrt(vec_a.dot(vec_a))*sqrt(vec_b.dot(vec_b)))) * (180/M_PI);

                if(angle > 85){
                    exec_index++;
                }
            }

            // exec_index = nearest node in dynamic path
            // switching_path = true;
        }
        

        /*** Node Decision ***/
        // if(g_dangerous == false && g_dangerous_prev == false)   // Initial Path
        // double shortest_dis, dis_tmp;

        // TM5.interface->stateRT->getQAct(CurrentPosition);
        // for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        // {
        //     // IP_velocity->CurrentPositionVector->VecData[i] = CurrentPosition[i];
        //     q[i] = CurrentPosition[i];
        // }
        // tm_jacobian::Forward_Kinematics_gripper(q,T,GRIPPER_LENGTH);

        if(state_prev != state_curr){  // Initial Path to Dynamic Path
            double shortest_dis, dis_tmp;

            ROS_ERROR("Path state switch");
            shortest_dis = sqrt(pow(exec_path[0].x - T[3], 2) + pow(exec_path[0].y - T[7], 2) + pow(exec_path[0].z - T[11], 2));
            exec_index = 0;
            for(int i = 1; i < exec_path.size(); i++){
                dis_tmp = sqrt(pow(exec_path[i].x - T[3], 2) + pow(exec_path[i].y - T[7], 2) + pow(exec_path[i].z - T[11], 2));
                if(dis_tmp < shortest_dis){
                    shortest_dis = dis_tmp;
                    if(i != exec_path.size()-1)
                        exec_index = i+1;
                    else
                        exec_index = i-1; 
                }
            }

            if(exec_index < exec_path.size()-2 && exec_index > 1){
                Eigen::Vector3d vec_a, vec_b;

                vec_a << exec_path[exec_index].x - T[3] , exec_path[exec_index].y - T[7] , exec_path[exec_index].z - T[11];
                vec_b << exec_path[exec_index+1].x - exec_path[exec_index].x, exec_path[exec_index+1].y - exec_path[exec_index].y, exec_path[exec_index+1].z - exec_path[exec_index].z;
            
                double angle = acos((vec_a.dot(vec_b)) / (sqrt(vec_a.dot(vec_a))*sqrt(vec_b.dot(vec_b)))) * (180/M_PI);

                if(angle > 85){
                    exec_index++;
                }
            }
        }  
        ROS_WARN("Next pt : %lf, %lf, %lf", exec_path[exec_index].x, exec_path[exec_index].y, exec_path[exec_index].z);

        // else if(state_prev == 2 && state_curr == 1){  //Dynamic Path to Initial Path
        //     shortest_dis = sqrt(pow(exec_path[0].x - T[3], 2) + pow(exec_path[0].y - T[7], 2) + pow(exec_path[0].z - T[11], 2));
        //     exec_index = 0;
        //     for(int i = 1; i < exec_path.size(); i++){
        //         dis_tmp = sqrt(pow(exec_path[i].x - T[3], 2) + pow(exec_path[i].y - T[7], 2) + pow(exec_path[i].z - T[11], 2));
        //         if(dis_tmp < shortest_dis){
        //             shortest_dis = dis_tmp;
        //             exec_index = i;
        //         }
        //     }
        // }

        // else if(g_dangerous_prev == true && g_dangerous == true)    // Dynamic Path, check whether path update

        // Pass through velocity decision
        if(exec_index == 0 || exec_index == exec_path.size()-1){
            PassThrough_velocity = {0, 0, 0};
        }
        else{
            for(int i = 0; i < 3; i++){
                if(i == 0){
                    if(Direction(exec_index, i, exec_path)){
                        PassThrough_velocity[i] = ((exec_path[exec_index-1].x - exec_path[exec_index].x) + (exec_path[exec_index].x - exec_path[exec_index+1].x));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].x - exec_path[exec_index].x) + (exec_path[exec_index].x - exec_path[exec_index+1].x));
                        PassThrough_velocity[i] *= -1;
                    }
                }
                else if(i == 1){
                    if(Direction(exec_index, i, exec_path)){
                        PassThrough_velocity[i] = ((exec_path[exec_index-1].y - exec_path[exec_index].y) + (exec_path[exec_index].y - exec_path[exec_index+1].y));
                        PassThrough_velocity[i] *= -1;
                    }
                    else{
                        PassThrough_velocity[i] = 0.0;
                        // PassThrough_velocity[i] = ((exec_path[exec_index-1].y - exec_path[exec_index].y) + (exec_path[exec_index].y - exec_path[exec_index+1].y));
                        PassThrough_velocity[i] *= -1;
                    }
                }
                else if(i == 2){
                    if(Direction(exec_index, i, exec_path)){
                        PassThrough_velocity[i] = ((exec_path[exec_index-1].z - exec_path[exec_index].z) + (exec_path[exec_index].z - exec_path[exec_index+1].z));
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

        TargetPosition = {exec_path[exec_index].x, exec_path[exec_index].y, exec_path[exec_index].z, 0, 0, 0};
        // ROS_WARN("Target x: %lf, y: %lf, z: %lf", exec_path[exec_index].x, exec_path[exec_index].y, exec_path[exec_index].z);

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

                ROS_INFO("gripper : %10.4lf %10.4lf %10.4lf Atrractive: %10.4lf %10.4lf %10.4lf Repulsive: %10.4lf %10.4lf %10.4lf", T[3], T[7], T[11],TargetVelocity[0],TargetVelocity[1],TargetVelocity[2],RepulsiveVelocity[0],RepulsiveVelocity[1],RepulsiveVelocity[2] );
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

        if(exec_index == exec_path.size()){    // Path finish
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
            if(!ReflexxesPositionRun(*IP_position, FinalPosition, TargetVelocity, 1))
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

            ROS_WARN("Path finished, smooth stop activate...");
            ReflexxesSmoothStop(*IP_velocity,0.25);
            break;
        }
    }


    ROS_WARN("Hexagon_Demo shutdown");
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

// void init_path_callback(const my_msg_pkg::Path::ConstPtr& i_path)
// {
//     static_path = i_path->node;
//     init_path_num = i_path->pt_count;
//     get_init_path = true; 
//     ROS_WARN("Get initial Path!!");
// }

// void dynamic_path_callback(const my_msg_pkg::Path::ConstPtr& msg)
// {
//     dynamic_path = msg->node;
//     get_dynamic_path = true;
//     // init_path_num = i_path->pt_count;
//     // get_init_path = true; 
//     // ROS_WARN("Get dynamic Path!!");
// }

void path_state_callback(const std_msgs::Int32::ConstPtr& msg)
{
    // g_dangerous_prev = g_dangerous;
    g_dangerous = msg->data;
}

void TMmsgCallback(const tm_msgs::FeedbackState::ConstPtr& msg)
{
  //吃六軸角度
  if(msg->joint_pos.size() == 6){
    g_robot_joint_angle[0] = msg->joint_pos[0];
    g_robot_joint_angle[1] = msg->joint_pos[1];
    g_robot_joint_angle[2] = msg->joint_pos[2];
    g_robot_joint_angle[3] = msg->joint_pos[3];
    g_robot_joint_angle[4] = msg->joint_pos[4];
    g_robot_joint_angle[5] = msg->joint_pos[5];
  }


  // if(start_cmd){
  //   tf::Quaternion q_tool(0.0, 1.0 ,0.0, 0.0);
  //   tf::Vector3 v_tool(0.0, -0.0, 0.266); //0.266
  //   tf::Transform T_tool(q_tool,v_tool);
    
    //有考慮到末端工具之6D pose?
    g_robot_tool_pos[0] = msg->tool_pose[0];
    g_robot_tool_pos[1] = msg->tool_pose[1];
    g_robot_tool_pos[2] = msg->tool_pose[2];
    g_robot_tool_pos[3] = msg->tool_pose[3];
    g_robot_tool_pos[4] = msg->tool_pose[4];
    g_robot_tool_pos[5] = msg->tool_pose[5];

    char outmsg_tag[200];
    //sprintf用來將各種型態的東西格式化輸出成字串，此為outmsg_tag
    sprintf(outmsg_tag,"%.7f %.7f %.7f",g_robot_tool_pos[0],g_robot_tool_pos[1],g_robot_tool_pos[2]);
    end_pose_txt << outmsg_tag << endl;
  //   tf::Quaternion q3;
  //   q3.setRPY(g_robot_tool_pos[3], g_robot_tool_pos[4], g_robot_tool_pos[5]);
  //   tf::Vector3 vec3(g_robot_tool_pos[0], g_robot_tool_pos[1], g_robot_tool_pos[2]);
  //   tf::Transform T_path;
  //   T_path.setOrigin(vec3);
  //   T_path.setRotation(q3);

  //   T_path=T_path*T_tool;

  //   char outmsg_tag[200];
  //   sprintf(outmsg_tag,"%.7f %.7f %.7f",T[3], T[7], T[11]);
  //   end_pose << outmsg_tag << endl;
  //   ROS_ERROR_STREAM("Writing pose!");
  // }
  
}

void drawPath(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    //edge type為visualization_msgs::Marker (全域變數)
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "base";
    edge.header.stamp = ros::Time::now();
    edge.ns = "smoothPath";
    edge.id = 5;
    edge.action = visualization_msgs::Marker::ADD;
    //Marker中的pose 為 geometry_msgs/Pose 
    edge.pose.orientation.w = 1;

    edge.scale.x = edge.scale.y = edge.scale.z = 0.006;

    edge.color.g = 1;
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}

//用來pub 從/MovingPath(get_path_callback)中得到的點並經過Marker連起來的資訊（會一直更新，不是一開始就全部連起來，一次+一個點與edge) !!!這邊有點奇怪，有點像是秀出線後只保留點
void showSmoothPath(std::vector<geometry_msgs::Point> static_path)
{
    //marker有各種type，但之後的參數設置都一樣
    visualization_msgs::Marker v_start, v_end;
    v_start.type = v_end.type = visualization_msgs::Marker::POINTS;
    //設定Marker的Header
    v_start.header.frame_id = v_end.header.frame_id = "base";
    v_start.header.stamp = v_end.header.stamp = ros::Time::now();
    //設定Marker的namespace(與C++一般的相同)
    v_start.ns = v_end.ns = "start/end vertices";
    //id用於與ns聯合起來，形成唯一的id(可以將各個標誌物區分開來，使得程序可以對指定的標誌物進行操作)
    v_start.id = 0;
    v_end.id = 1;
    // 此Marker的功(有ADD，MODIFY，DELETE， DELETEALL)
    v_start.action = v_end.action = visualization_msgs::Marker::ADD;

    // 設定Marker的顏色(r,g,b,a) 範圍[0.0 ~ 1.0]
    v_start.color.a = 1.0f;
    v_start.color.g = 1.0f;
    // 設定Marker的大小，default為 1, 1, 1
    v_start.scale.x = v_start.scale.y = v_start.scale.z = 0.03;

    v_end.color.a = 1.0f;
    v_end.color.r = 1.0f;
    v_end.scale.x = v_end.scale.y = v_end.scale.z = 0.03;

    geometry_msgs::Point next;
    geometry_msgs::Point curr;

    //用來不斷更新線(看有幾個點)
    for (int i = 0; i < static_path.size() - 1; ++i) {
        curr = static_path[i];
        next = static_path[i + 1];
        drawPath(curr, next);
    }
    curr = static_path[0];
    next = static_path[static_path.size() - 1];
    v_start.points.push_back(curr);
    v_end.points.push_back(next);
    //只傳第一個點與最後一個點， 感覺有點怪怪的
    marker_pub.publish(v_start);
    marker_pub.publish(v_end);
    // 保持v_start與v_end 中的point只有一個點
    v_start.points.clear();
    v_end.points.clear();
}

//主要靠這個控給點的速度控制 
void get_path_callback(const geometry_msgs::PoseArray::ConstPtr& msg)   //geometry_msgs::PoseArray包含header(std_msgs/Header)與poses(geometry_msgs/Pose)
{
    // static_path 型態為 std::vector<geometry_msgs::Point>
    // vector::clear為刪除該vector中的元素
    static_path.clear();
    geometry_msgs::Point pose;
    // 把msg從poseArray轉成pose型態並存入static_path這個vector中
    for(int i = 0; i < msg->poses.size(); i++)
    {
        pose.x = msg->poses[i].position.x;
        pose.y = msg->poses[i].position.y;
        pose.z = msg->poses[i].position.z;

        static_path.push_back(pose);
    }

    printf("Total smooth path length : %lu\n", static_path.size());
    for(int i = 0; i < static_path.size(); i++){
        printf("x: %10.4lf, y: %10.4lf, z: %10.4lf \n", static_path[i].x, static_path[i].y, static_path[i].z);
    }


    showSmoothPath(static_path);
    //清除從drawPath更新的edge(全域變數，只有在drawPath與此用到)
    edge.points.clear();
    //get_init_path的type為bool(全域變數), default為false
    get_init_path = true;
    //start_cmd的type為int(全域變數), default為0
    start_cmd = 1;
    /*
    while(abs(g_robot_tool_pos[0]-pose.x)>0.01 || abs(g_robot_tool_pos[1]-pose.y)>0.01 || abs(g_robot_tool_pos[2]-pose.z)>0.01){  
    }
    printf("shit!!!!!!!!!");
    */
    // linux下的sleep為秒
    sleep(4); 
    // mode 0:open, 1:close, 2:full open, 3:bar close
    set_gripper(1);
}

// void read_static_path()
// {
//     string line;
//     // ifstream static_path_file ("/home/tm5-server/Documents/human_robot/src/rrt-planning/path/white_box/white_box_lower_path.txt");
//     ifstream static_path_file ("/home/yu/Documents/human/src/rrt-planning/path/camera_shell_path.txt");
//     geometry_msgs::Point point;

//     static_path.clear();

//     if (static_path_file.is_open())
//     {
//         while ( getline (static_path_file,line) )
//         {
//           stringstream iss(line);
//           // cout << line << '\n';
//           // sscanf(line, "%lf,%lf,%lf,", &point.x, &point.y, &point.z);
//           iss >> point.x;
//           iss.ignore(1, ',');
//           iss >> point.y;
//           iss.ignore(1, ',');
//           iss >> point.z;
//           iss.ignore(1, ',');
//           static_path.push_back(point);
//         }
//         static_path_file.close();
//     }

//     else
//         cout << "Unable to open file";

//     printf("Total smooth path length : %d\n", static_path.size());
//     for(int i = 0; i < static_path.size(); i++){
//         printf("x: %10.4lf, y: %10.4lf, z: %10.4lf \n", static_path[i].x, static_path[i].y, static_path[i].z);
//     }
//     showSmoothPath(static_path);
//     edge.points.clear();
//     get_init_path = true;
// }

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
    ros::AsyncSpinner spinner(10); //開10個thread

    //用來pub 從/MovingPath(get_path_callback)中得到的點並經過Marker連起來的資訊（會一直更新，不是一開始就全部連起來，一次+一個點與edge) !!!這邊有點奇怪，有點像是秀出線後只保留點
    marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 40);
    //reset_monitoring_pub = node_handle.advertise<std_msgs::Int32>("/reset_local_replan", 40);


    //ros::Subscriber position_sub   = node_handle.subscribe("/eff/kinect_merge/closest_pt_tracking", 1,position_callback);
    //ros::Subscriber tool_position_sub   = node_handle.subscribe("/pc1_pt", 1, tool_position_callback);
    //ros::Subscriber constraint_sub = node_handle.subscribe("/body/kinect_merge/closest_pt_tracking", 1,constraint_callback);
    //ros::Subscriber path_sub = node_handle.subscribe("online_path", 1, init_path_callback);
    //ros::Subscriber path_state_sub = node_handle.subscribe("/path_state", 1, path_state_callback);
    //ros::Subscriber dynamic_path_sub = node_handle.subscribe("online_replan_path", 1, dynamic_path_callback);
    
    //主要靠這個控給點的速度控制
    ros::Subscriber receive_path = node_handle.subscribe("/MovingPath", 1, get_path_callback);
    // 用來儲存手臂個軸與末端點姿態(將其寫入txt檔與存成全域變數)
    ros::Subscriber robot_status_sub = node_handle.subscribe("feedback_states", 1000,TMmsgCallback);
    // 會在ReflexxesVelocityRun中使用到(將計算過後的最終速度傳給手臂)
    vel_client = node_handle.serviceClient<tm_msgs::SetVelocity>("tm_driver/set_velocity");
    script_client = node_handle.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    //沒用到
    event_client = node_handle.serviceClient<tm_msgs::SetEvent>("tm_driver/set_event");
    pos_client = node_handle.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    //用來控制夾爪的publiher
    pub_ctrl = node_handle.advertise<robotiq_controller::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput",10);
    spinner.start();
    sleep(1);
    
    //使gripper各項數值歸零(將夾爪閉合)
    reset_gripper();
    // gripper_command 的type為 robotiq_controller::Robotiq2FGripper_robot_output (全域變數)
    pub_ctrl.publish(gripper_command); 
    printf("1\n");
    sleep(1);
    
    //初始話夾爪(將夾爪開啟)
    init_gripper();
    pub_ctrl.publish(gripper_command); 
    printf("2\n");
    sleep(1);
    
    

    tm_msgs::SetPositions pos_srv; //沒用到 (型態類似夾爪參數)
    tm_msgs::SetEvent event_srv;    //沒用到
    tm_msgs::SendScript script_srv;

    

    // if (!(ros::param::get("~robot_ip", host)))
    // {
    //     if (argc > 1)
    //         host = argv[1];
    //     else
    //         exit(1);
    // }

    // const int STDIN = 0;
    // int sockfd = -1;
    bool fgRun = false; //沒用到(備註解掉)

    // for (int i = 0; i < argc; i++)
    // {
    //     printf("[DEBUG] arg%d:= %s\n", i, argv[i]);
    // }
    // host = argv[1];
    // printf("[ INFO] host: %s\n", host.c_str());


    // TmDriver TmRobot(data_cv, data_cv_rt, host, 0);

    //獲取sophia_test pkg 位置與宣告開啟的檔案(用於寫入)
    std::string path = ros::package::getPath("sophia_test");
    //tool_pose.txt 紀錄末端點軌跡
    end_pose_txt.open (path+"/src/tool_pose/tool_pose_yun.txt", std::ifstream::out);
    //error.txt紀錄末端點誤差 , 於ReflexxesVelocityRun中紀錄與實際該到點的誤差(會受REPEATABILITY影響)
    distance_error.open (path+"/src/tool_pose/error_yun.txt", std::ifstream::out);
    

    char cstr[512]; //沒用到(備註解掉)
    char delim[] = " ,;\t"; //沒用到(備註解掉)
    char c; //沒用到(備註解掉)
    ros::Rate r(10);
    while (ros::ok())
    {
        // memset(cstr, 0, 512);
        // fgets(cstr, 512, stdin);
        // int n = (int)strlen(cstr);
        // if (n > 0)
        // {
        //     if (cstr[n - 1] == '\n')
        //         cstr[n - 1] = '\0';
        // }
        // if (strncmp(cstr, "quit", 4) == 0)
        // {
        //     script_srv.request.id = "quit";
        //     script_srv.request.script = exit_cmd;
            
        //     script_client.call(script_srv);
        //     fgRun = false;

        //     print_info("quit");
        //     break;
        // }
        // else if (strncmp(cstr, "spdmodeoff", 10) == 0)
        // {
        //     script_srv.request.id = "spdmodeoff";
        //     script_srv.request.script = VStop_cmd;

        //     script_client.call(script_srv);
        //     print_info("joint vlocity control mode OFF...");
        // }
        // else
        // {
        //     std::string cmd{ cstr };
        //     std::cout << "send cmd: " << cmd << "\n";

        //     script_srv.request.id = "Parse_cmd";
        //     script_srv.request.script = cmd;
        //     script_client.call(script_srv);

        //     print_info("send cmd...");
        // }
        
        // get_init_path 等到get_path_callback 後自動會變true (default為false)
        // start_cmd 有兩個地方: 1.等到get_path_callback 後自動會變1; 2. Hexagon_Demo 會變0 (default為0), 代表到終點
        // 當get_path_callback將點的位置存下後會執行，並於Hexagon_Demo執行完後結束
        if(start_cmd!=0 && get_init_path){
            unsigned char ch = 0; //沒用到
            //std_msgs::Int32 reset_local_replan_msg;
            //reset_local_replan_msg.data = 1;

            //set_gripper(1); 

      
            // script_srv 的type為 tm_msgs::SendScript
            //此mode為速度控制的mode
            script_srv.request.id = "Vstart";
            //VStart_cmd 的type為std::string, 值為 "ContinueVJog()";
            script_srv.request.script = VStart_cmd;
            script_client.call(script_srv);
            print_info("joint velocity control mode ON...");

            //reset_monitoring_pub.publish(reset_local_replan_msg);
            
            //這個會卡死，直到所有點都跑完
            Hexagon_Demo();

            // 這個應該是結束的訊號
            script_srv.request.id = "spdmodeoff";
            script_srv.request.script = VStop_cmd;
            script_client.call(script_srv);
            print_info("joint vlocity control mode OFF...");
        }

        r.sleep();
    }
    //ros::waitForShutdown();

    printf("[ info] TM_ROS: shutdown\n");

    return 0;
}
