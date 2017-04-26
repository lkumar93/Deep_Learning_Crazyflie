#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "pid.hpp"

#define KEYCODE_RA 0x43
#define KEYCODE_LA 0x44
#define KEYCODE_UA 0x41
#define KEYCODE_DA 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_T 0x74
#define KEYCODE_L 0x6C
#define KEYCODE_P 0x70

#define KP_X 10.0
#define KI_X 2.0
#define KD_X 2.0

#define KP_Y 10.0
#define KI_Y 2.0
#define KD_Y 2.0

#define KP_Z 1800000.0
#define KI_Z 1800.0
#define KD_Z 18000.0

#define INTEGRATOR_MAX_Z 1000.0
#define INTEGRATOR_MIN_Z -1000.0

#define MAX_OUTPUT_Z 60000.0
#define MIN_OUTPUT_Z 10000.0

#define INTEGRATOR_MAX_X 0.1
#define INTEGRATOR_MIN_X -0.1

#define MAX_OUTPUT_X 10.0
#define MIN_OUTPUT_X -10.0

#define INTEGRATOR_MAX_Y 0.1
#define INTEGRATOR_MIN_Y -0.1

#define MAX_OUTPUT_Y 10.0
#define MIN_OUTPUT_Y -10.0

#define WAITING 0
#define TAKE_OFF 1
#define POS_CTRL 2
#define LAND 3
#define EMERGENCY 4

#include <thread> 

using namespace geometry_msgs;
using namespace std;

int kfd = 0;
struct termios cooked, raw;



class CrazyfliePositionController
 {

 public:
   CrazyfliePositionController();
   void keyLoop();
   void getGroundTruth(const geometry_msgs::PoseStampedConstPtr& OptiTrackPacket);
   void cmdSubscriber(const geometry_msgs::TwistConstPtr& cmd_pos);
   void stateSubscriber(const std_msgs::Int32ConstPtr& cmd_state);
   void takeoff(const ros::TimerEvent&);
   void land(const ros::TimerEvent&);
   void pos_ctrl(const ros::TimerEvent&);
   void emergency(const ros::TimerEvent&);
   void pidReset();
   void run();

  
 private:
   
   ros::NodeHandle nh_;
   double yawrate, thrust;
   
   ros::Publisher vel_pub_, point_pub_;
   ros::Subscriber ground_truth_sub_,cmd_sub_,state_sub_;

   geometry_msgs::Twist cmd;
   ros::Timer takeoff_timer,land_timer, pos_ctrl_timer, emergency_timer,keyloop_timer;
 
   // In m

   float initial_position_x;
   float initial_position_y;
   float initial_position_z;

   float current_position_x;
   float current_position_y;
   float current_position_z;

   double goal_x, goal_y, goal_z;

   bool initialized;

   int state;

   PID pidX;
   PID pidY;
   PID pidZ;

   
 };


CrazyfliePositionController::CrazyfliePositionController():
  goal_x(0.0),
  goal_y(0.0),
  yawrate(0.0),
  goal_z(0.15),
  current_position_x(0.0),
  current_position_y(0.0),
  current_position_z(0.0),
  initial_position_x(0.0),
  initial_position_y(0.0),
  initial_position_z(0.0),
  thrust(0.0),
  state(WAITING),
  pidX(KP_X, KD_X, KD_X, MIN_OUTPUT_X, MAX_OUTPUT_X, INTEGRATOR_MIN_X, INTEGRATOR_MAX_X, "x"),
  pidY(KP_Y, KD_Y, KD_Y, MIN_OUTPUT_Y, MAX_OUTPUT_Y, INTEGRATOR_MIN_Y, INTEGRATOR_MAX_Y, "y"),
  pidZ(KP_Z, KD_Z, KD_Z, MIN_OUTPUT_Z, MAX_OUTPUT_Z, INTEGRATOR_MIN_Z, INTEGRATOR_MAX_Z, "z")
{
  vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("crazyflie/deep_learning/cmd_vel", 1);
  point_pub_ =  nh_.advertise<geometry_msgs::PointStamped>("crazyflie/position", 1);
  initialized = false;
  ground_truth_sub_ = nh_.subscribe("/crazyflie/ground_truth/pose_3d" , 10, &CrazyfliePositionController::getGroundTruth, this);
  cmd_sub_ = nh_.subscribe("crazyflie/deep_learning/cmd_pos" , 10, &CrazyfliePositionController::cmdSubscriber, this);
  state_sub_ = nh_.subscribe("crazyflie/deep_learning/cmd_state" , 10, &CrazyfliePositionController::stateSubscriber, this);
  takeoff_timer = nh_.createTimer(ros::Duration(0.02), &CrazyfliePositionController::takeoff, this);
  land_timer = nh_.createTimer(ros::Duration(0.02), &CrazyfliePositionController::land, this);
  pos_ctrl_timer = nh_.createTimer(ros::Duration(0.02), &CrazyfliePositionController::pos_ctrl, this);
  emergency_timer = nh_.createTimer(ros::Duration(0.02), &CrazyfliePositionController::emergency, this);
}

void CrazyfliePositionController::pidReset()
    {
        pidX.reset();
        pidZ.reset();
        pidZ.reset();
    }

void CrazyfliePositionController::getGroundTruth(const geometry_msgs::PoseStampedConstPtr& OptiTrackPacket)
{

	current_position_x = OptiTrackPacket->pose.position.x;
	current_position_y = OptiTrackPacket->pose.position.y;
	current_position_z = OptiTrackPacket->pose.position.z;

	geometry_msgs::PointStamped position_msg;
	position_msg.header.frame_id = OptiTrackPacket->header.frame_id;
	position_msg.header.stamp = OptiTrackPacket->header.stamp;
	position_msg.point.x = current_position_x;
	position_msg.point.y = current_position_z;	
	position_msg.point.z = current_position_y;	

	point_pub_.publish(position_msg);

	if(!initialized)
	{
		initial_position_x = current_position_x;
		initial_position_y = current_position_y;
		initial_position_z = current_position_z;
		initialized = true;
	}

        //ROS_INFO("Z- %f, Y - %f ,X- %f, yawrate - %f",current_position_z,current_position_y, current_position_x, yawrate);
}

void CrazyfliePositionController::cmdSubscriber(const geometry_msgs::TwistConstPtr& cmd_pos)
{

	goal_x = cmd_pos->linear.x;
	goal_y = cmd_pos->linear.y;
	goal_z = cmd_pos->linear.z;
      
}

void CrazyfliePositionController::stateSubscriber(const std_msgs::Int32ConstPtr& cmd_state)
{
	state = cmd_state->data;
	if(state == TAKE_OFF)
	{
		initial_position_x = current_position_x;
		initial_position_y = current_position_y;
		initial_position_z = current_position_z;

	}
	//ROS_INFO("%d",state);
}

void CrazyfliePositionController::takeoff(const ros::TimerEvent& e)
{
   if(initialized && state==TAKE_OFF)
   {
	float dt = e.current_real.toSec() - e.last_real.toSec();

 	if (current_position_z > initial_position_z + 0.1  || thrust > 25000)
         {
            pidReset();
            pidZ.setIntegral(thrust / pidZ.ki());
            state = POS_CTRL;
            thrust = 0;
        }
        else
        {
	    ROS_DEBUG("TAKING OFF");
            thrust += 10000 * dt;
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.y = 0.0;
            cmd.linear.z = thrust;
            vel_pub_.publish(cmd);
        }

   }

}

void CrazyfliePositionController::land(const ros::TimerEvent& e)
{
   if(initialized && state==LAND)
   {
	    ROS_DEBUG("LANDING");
	    cmd.linear.x = pidX.update(current_position_x, initial_position_x + goal_x);
            cmd.linear.y = pidY.update(current_position_y, initial_position_y + goal_y);
            cmd.angular.y = yawrate;
            cmd.linear.z = pidZ.update(current_position_z, initial_position_z + 0.05);
            vel_pub_.publish(cmd);

	    if(current_position_z <= initial_position_z + 0.05)
	    {
		state = EMERGENCY;
	    }
   }

}

void CrazyfliePositionController::run()
{
	 ros::spin();
}



void CrazyfliePositionController::pos_ctrl(const ros::TimerEvent& e)
{
   if(initialized && state==POS_CTRL)
   {
	    ROS_INFO("POSITION CONTROL :");
	    cmd.linear.x = pidX.update(current_position_x, initial_position_x + goal_x);
            cmd.linear.y = pidY.update(current_position_y, initial_position_y + goal_y);
            cmd.angular.y = yawrate;
            cmd.linear.z = pidZ.update(current_position_z, initial_position_z + goal_z);
	    ROS_INFO("POSITION CONTROL : x= %f, y= %f, z= %f ",cmd.linear.x,cmd.linear.y,cmd.linear.z);
            vel_pub_.publish(cmd);
   }

}

void CrazyfliePositionController::emergency(const ros::TimerEvent& e)
{
   if(state==EMERGENCY)
   {
	    ROS_DEBUG("EMERGENCY"); 
	    cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.y = 0.0;
            cmd.linear.z = 0.0;
            vel_pub_.publish(cmd);
	    pidReset();
   }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "crazyflie_pos_ctrl_node");
  CrazyfliePositionController crazyflie_position_controller;
  crazyflie_position_controller.run();

  return(0);
}

