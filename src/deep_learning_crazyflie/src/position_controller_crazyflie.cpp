#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "pid.hpp"
#include "deep_learning_crazyflie/TunePID.h"

#define KP_X 40.0
#define KI_X 2.0
#define KD_X 20.0

#define KP_Y 40.0
#define KI_Y 2.0
#define KD_Y 20.0

//#define KP_Z 1800000.0
//#define KI_Z 1800.0
//#define KD_Z 18000.0

#define KP_Z 5000.0
#define KI_Z 3500.0
#define KD_Z 6000.0

#define INTEGRATOR_MAX_Z 1000.0
#define INTEGRATOR_MIN_Z -1000.0

#define MAX_OUTPUT_Z 55000.0
#define MIN_OUTPUT_Z 20000.0

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
#define CALIBRATE 5

#define X_TUNE 0
#define Y_TUNE 1
#define Z_TUNE 2



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
   bool pid_tuner(deep_learning_crazyflie::TunePID::Request &request,
	deep_learning_crazyflie::TunePID::Response &response);
   void pidReset();
   void pidSet();
   void run();

  
 private:
   
   ros::NodeHandle nh_;
   double yawrate, thrust;
   
   ros::Publisher vel_pub_, point_stamped_pub_, point_pub_, error_pub_;
   ros::Subscriber ground_truth_sub_,cmd_sub_,state_sub_;

   ros::ServiceServer pid_tuner_service;

   geometry_msgs::Twist cmd;
   ros::Timer takeoff_timer,land_timer, pos_ctrl_timer, emergency_timer,keyloop_timer;
 
   // In m

   double initial_position_x, initial_position_y, initial_position_z;
   double current_position_x, current_position_y, current_position_z;

   double kp_x, ki_x, kd_x;
   double kp_y, ki_y, kd_y;
   double kp_z, ki_z, kd_z;

   double goal_x, goal_y, goal_z;
   double error_x, error_y, error_z;

   bool initialized, calibrated, inflight;

   int state, prev_state;

   PID pidX, pidY, pidZ;
   
 };


CrazyfliePositionController::CrazyfliePositionController():
  goal_x(0.0),
  goal_y(0.0),
  yawrate(0.0),
  goal_z(0.2),
  current_position_x(0.0),
  current_position_y(0.0),
  current_position_z(0.0),
  initial_position_x(0.0),
  initial_position_y(0.0),
  initial_position_z(0.0),
  thrust(0.0),
  kp_x(KP_X),
  ki_x(KI_X),
  kd_x(KD_X),
  kp_y(KP_Y),
  ki_y(KI_Y),
  kd_y(KD_Y),
  kp_z(KP_Z),
  ki_z(KI_Z),
  kd_z(KD_Z),
  calibrated(false),
  initialized(false),
  inflight(false),
  state(WAITING),
  prev_state(WAITING),
  pidX(KP_X, KD_X, KD_X, MIN_OUTPUT_X, MAX_OUTPUT_X, INTEGRATOR_MIN_X, INTEGRATOR_MAX_X, "x"),
  pidY(KP_Y, KD_Y, KD_Y, MIN_OUTPUT_Y, MAX_OUTPUT_Y, INTEGRATOR_MIN_Y, INTEGRATOR_MAX_Y, "y"),
  pidZ(KP_Z, KD_Z, KD_Z, MIN_OUTPUT_Z, MAX_OUTPUT_Z, INTEGRATOR_MIN_Z, INTEGRATOR_MAX_Z, "z")
{
  vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("crazyflie/deep_learning/cmd_vel", 1);
  point_pub_ =  nh_.advertise<geometry_msgs::Point>("crazyflie/ground_truth/position", 1);
  error_pub_ =  nh_.advertise<geometry_msgs::Point>("crazyflie/ground_truth/position_error", 1);
  point_stamped_pub_ =  nh_.advertise<geometry_msgs::PointStamped>("crazyflie/ground_truth/position_stamped", 1);
  ground_truth_sub_ = nh_.subscribe("/crazyflie/ground_truth/pose_3d" , 1, &CrazyfliePositionController::getGroundTruth, this);
  state_sub_ = nh_.subscribe("crazyflie/deep_learning/cmd_state" , 1, &CrazyfliePositionController::stateSubscriber, this);
  cmd_sub_ = nh_.subscribe("crazyflie/deep_learning/cmd_pos" , 1, &CrazyfliePositionController::cmdSubscriber, this);
  takeoff_timer = nh_.createTimer(ros::Duration(0.02), &CrazyfliePositionController::takeoff, this);
  pos_ctrl_timer = nh_.createTimer(ros::Duration(0.02), &CrazyfliePositionController::pos_ctrl, this);
  land_timer = nh_.createTimer(ros::Duration(0.02), &CrazyfliePositionController::land, this);
  emergency_timer = nh_.createTimer(ros::Duration(0.001), &CrazyfliePositionController::emergency, this);
  pid_tuner_service = nh_.advertiseService("tune_pid", &CrazyfliePositionController::pid_tuner, this);
}

void CrazyfliePositionController::pidReset()
{
    ROS_INFO("Resetting PID's history");
    pidX.reset();
    pidY.reset();
    pidZ.reset();
}

void CrazyfliePositionController::pidSet()
{
    ROS_INFO("====================================================");
    ROS_INFO("SETTING PID");
    ROS_INFO("kp_x = %f, ki_x = %f, kd_x =%f", kp_x, ki_x, kd_x);
    ROS_INFO("kp_y = %f, ki_y = %f, kd_y =%f", kp_y, ki_y, kd_y);
    ROS_INFO("kp_z = %f, ki_z = %f, kd_z =%f", kp_z, ki_z, kd_z);
    ROS_INFO("====================================================");

    pidX.setKp(kp_x);
    pidX.setKi(ki_x);
    pidX.setKd(kd_x);

    pidY.setKp(kp_y);
    pidY.setKi(ki_y);
    pidY.setKd(kd_y);

    pidZ.setKp(kp_z);
    pidZ.setKi(ki_z);
    pidZ.setKd(kd_z);

    pidReset();
}

bool CrazyfliePositionController::pid_tuner(deep_learning_crazyflie::TunePID::Request &request,
deep_learning_crazyflie::TunePID::Response &response)
{

	if(request.param == Z_TUNE)
	{
		kp_z = request.kp;
		ki_z = request.ki;
		kd_z = request.kd;
		ROS_INFO("PID TUNING: kp_z = %f, ki_z = %f, kd_z =%f", kp_z, ki_z, kd_z);
	}

	else if (request.param == X_TUNE)
	{
		kp_x = request.kp;
		ki_x = request.ki;
		kd_x = request.kd;
		ROS_INFO("PID TUNING: kp_x = %f, ki_x = %f, kd_x =%f", kp_x, ki_x, kd_x);
	}

	else if (request.param == Y_TUNE)
	{
		kp_y = request.kp;
		ki_y = request.ki;
		kd_y = request.kd;
		ROS_INFO("PID TUNING: kp_y = %f, ki_y = %f, kd_y =%f", kp_y, ki_y, kd_y);
	}

	pidSet();

	response.success = true;

	return true;
}

void CrazyfliePositionController::getGroundTruth(const geometry_msgs::PoseStampedConstPtr& OptiTrackPacket)
{
	current_position_x = OptiTrackPacket->pose.position.y;
	current_position_y = -OptiTrackPacket->pose.position.x;
	current_position_z = -OptiTrackPacket->pose.position.z;

	geometry_msgs::PointStamped position_msg;
	position_msg.header.frame_id = OptiTrackPacket->header.frame_id;
	position_msg.header.stamp = OptiTrackPacket->header.stamp;
	position_msg.point.x = current_position_x;
	position_msg.point.y = current_position_y;	
	position_msg.point.z = current_position_z;	

	point_stamped_pub_.publish(position_msg);
	point_pub_.publish(position_msg.point);

	error_z = initial_position_z + goal_z - current_position_z;
	error_y = initial_position_y + goal_y - current_position_y;
	error_x = initial_position_x + goal_x - current_position_x;

	geometry_msgs::Point error_msg;
	error_msg.x = error_x;
	error_msg.y = error_y;
	error_msg.z = error_z;

	error_pub_.publish(error_msg);
	
	if(!initialized)
		initialized = true;
}

void CrazyfliePositionController::cmdSubscriber(const geometry_msgs::TwistConstPtr& cmd_pos)
{
	goal_x = cmd_pos->linear.x;
	goal_y = cmd_pos->linear.y;
	goal_z = cmd_pos->linear.z;

	ROS_INFO("Goal X = %f, Goal Y = %f, Goal Z = %f", goal_x, goal_y,goal_z);

	yawrate =  cmd_pos->angular.y;

}

void CrazyfliePositionController::stateSubscriber(const std_msgs::Int32ConstPtr& cmd_state)
{

	if(cmd_state->data == CALIBRATE)
	{
		if(initialized)
		{
			initial_position_x = current_position_x;
			initial_position_y = current_position_y;
			initial_position_z = current_position_z;
			pidSet();
			calibrated = true;
		}
		else
		{
			ROS_INFO("Not Initalized");
		}
		
		state = WAITING;
	}
	else
	{
		state = cmd_state->data;
	}

	prev_state = state;

}

void CrazyfliePositionController::takeoff(const ros::TimerEvent& e)
{
   if(state==TAKE_OFF)
   {
 	if(calibrated)
	{
		float dt = e.current_real.toSec() - e.last_real.toSec();

	 	if (current_position_z > initial_position_z + 0.05  || thrust > 50000)
		 {
		    ROS_INFO("Crazyflie is now inflight");
		    pidSet();
		    
		    if(pidZ.ki() != 0.0)
		    	pidZ.setIntegral(thrust / pidZ.ki());
		    
		    inflight = true;
		    state = POS_CTRL;
		    thrust = 0;
		}
		else
		{
		    thrust += 25000 * dt;
		    cmd.linear.x = 0.0;
		    cmd.linear.y = 0.0;
		    cmd.angular.y = 0.0;
		    cmd.linear.z = thrust;
		    vel_pub_.publish(cmd);
		    ROS_INFO("TAKING OFF : Increasing Thrust = %f", thrust);
		}
	}

	else
	{
		ROS_INFO("Not Calibrated");
	}

   }

}

void CrazyfliePositionController::land(const ros::TimerEvent& e)
{
   if( state==LAND)
   {
	if(!inflight)
	{
		ROS_INFO("Crazyflie is not in flight, landing not required");
		return;
	}	

 	if(calibrated)
	{
	    float dt = e.current_real.toSec() - e.last_real.toSec();


	    if(current_position_z <= initial_position_z + 0.1)
	    {

		if(thrust > MIN_OUTPUT_Z)
		{
		    thrust -= 10000 * dt;
		    cmd.linear.x = 0.0;
		    cmd.linear.y = 0.0;
		    cmd.angular.y = 0.0;
		    cmd.linear.z = thrust;
		    vel_pub_.publish(cmd);
		    ROS_INFO("LANDING : Decreasing Thrust = %f", thrust);
		}

		else
		{
		        cmd.linear.x = 0.0;
		        cmd.linear.y = 0.0;
		        cmd.angular.y = 0.0;
		        cmd.linear.z = 0.0;
		        vel_pub_.publish(cmd);
			inflight = false;
			calibrated = false;
			state = WAITING;
			ROS_INFO("LANDED");
		}

	    }

	   else
	   {
		    cmd.linear.x = pidX.update(current_position_x, initial_position_x + goal_x);
		    cmd.linear.y = pidY.update(current_position_y, initial_position_y + goal_y);
		    cmd.angular.y = 0.0;
		    cmd.linear.z = pidZ.update(current_position_z, initial_position_z + 0.05);

		    ROS_INFO("Will land soon: Pitch = %f, Roll = %f, Thrust = %f ", cmd.linear.x, cmd.linear.y, cmd.linear.z);
		    vel_pub_.publish(cmd);
           }

	}

	else
	{
		ROS_INFO("Not Calibrated");
	}

   }

}

void CrazyfliePositionController::run()
{
	 ros::spin();
}



void CrazyfliePositionController::pos_ctrl(const ros::TimerEvent& e)
{
   if(state==POS_CTRL )
   {
	if(!inflight)
	{
		ROS_INFO("Execute Take Off before controlling position");
		return;
	}
	
 	if(calibrated)
	{
	    cmd.linear.x = pidX.update(current_position_x, initial_position_x + goal_x);
            cmd.linear.y = pidY.update(current_position_y, initial_position_y + goal_y);
            cmd.angular.y = yawrate;
            cmd.linear.z = pidZ.update(current_position_z, initial_position_z + goal_z);

	    ROS_INFO("POSITION CONTROL CMD : Pitch= %f, Roll= %f, Thrust= %f ",cmd.linear.x,cmd.linear.y,cmd.linear.z);
	    ROS_INFO("GOAL POSITION : x= %f, y= %f, z= %f ",initial_position_x + goal_x,initial_position_y + goal_y,initial_position_z + goal_z);
	    ROS_INFO("CURRENT POSITION : x= %f, y= %f, z= %f ",current_position_x,current_position_y ,current_position_z);
	    ROS_INFO("ERROR POSITION : x= %f, y= %f, z= %f ",error_x,error_y,error_z);
	    ROS_INFO(" ");

            vel_pub_.publish(cmd);
	}

	else
	{
		ROS_INFO("Not Calibrated");
	}

   }
}

void CrazyfliePositionController::emergency(const ros::TimerEvent& e)
{
   if(state==EMERGENCY)
   {
	    ROS_INFO("EMERGENCY STOP"); 
	    cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.y = 0.0;
            cmd.linear.z = 0.0;
            vel_pub_.publish(cmd);
            vel_pub_.publish(cmd);
            vel_pub_.publish(cmd);
	    pidReset();
	    state = WAITING;
	    calibrated = false;
	    inflight = false;
	    initialized = false;
   }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "crazyflie_position_control_node");
  CrazyfliePositionController crazyflie_position_controller;
  crazyflie_position_controller.run();

  return(0);
}

