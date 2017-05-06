//
// THIS FILE CONTAINS THE CRAZYFLIE POSITION CONTROLLER NODE	
//
// COPYRIGHT BELONGS TO THE AUTHOR OF THIS CODE AND WOLFGANG HOENING, USC
//
// AUTHOR : LAKSHMAN KUMAR
// AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
// EMAIL : LKUMAR93@TERPMAIL.UMD.EDU
// LINKEDIN : WWW.LINKEDIN.COM/IN/LAKSHMANKUMAR1993
//
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THE GPLv3 LICENSE
// THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF
// THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS 
// PROHIBITED.
// 
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
// BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
// CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
// CONDITIONS.
//


///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include "pid.hpp"
#include "deep_learning_crazyflie/TunePID.h"
#include "deep_learning_crazyflie/Status.h"

///////////////////////////////////////////
//
//	DEFINITIONS
//
///////////////////////////////////////////

#define KP_X 40.0
#define KI_X 3.0
#define KD_X 20.0

#define KP_Y 40.0
#define KI_Y 3.0
#define KD_Y 20.0

#define KP_Z 5000.0
#define KI_Z 3500.0
#define KD_Z 6500.0

#define INTEGRATOR_MAX_Z 1000.0
#define INTEGRATOR_MIN_Z -1000.0

#define MAX_OUTPUT_Z 55000.0
#define MIN_OUTPUT_Z 20000.0

#define INTEGRATOR_MAX_X 0.2
#define INTEGRATOR_MIN_X -0.2

#define MAX_OUTPUT_X 20.0
#define MIN_OUTPUT_X -20.0

#define INTEGRATOR_MAX_Y 0.2
#define INTEGRATOR_MIN_Y -0.2

#define MAX_OUTPUT_Y 20.0
#define MIN_OUTPUT_Y -20.0

#define WAITING 0
#define TAKE_OFF 1
#define POS_CTRL 2
#define LAND 3
#define EMERGENCY 4
#define CALIBRATE 5
#define REINFORCEMENT_LEARNING 6

#define X_TUNE 0
#define Y_TUNE 1
#define Z_TUNE 2

#define TAKEOFF_GAIN 25000
#define LANDING_GAIN 50000

#define GOAL_REACH_THRESHOLD_INIT 0.01
#define GOAL_REACH_THRESHOLD_MAX 0.01

#define OUT_OF_BOUNDS_THRESHOLD 0.7

///////////////////////////////////////////
//
//	NAMESPACES
//
///////////////////////////////////////////

using namespace geometry_msgs;
using namespace std;

///////////////////////////////////////////
//
//	CLASS
//
///////////////////////////////////////////

class CrazyfliePositionController
 {

 public:
   CrazyfliePositionController();
   void getGroundTruth(const geometry_msgs::PoseStampedConstPtr& OptiTrackPacket);
   void cmdSubscriber(const geometry_msgs::TwistConstPtr& cmd_pos);
   void stateSubscriber(const std_msgs::Int32ConstPtr& cmd_state);
   void takeoff(const ros::TimerEvent&);
   void land(const ros::TimerEvent&);
   void pos_ctrl(const ros::TimerEvent&);
   void emergency(const ros::TimerEvent&);
   bool pid_tuner(deep_learning_crazyflie::TunePID::Request &request,
	deep_learning_crazyflie::TunePID::Response &response);
   bool requestStatus(deep_learning_crazyflie::Status::Request &request,
	deep_learning_crazyflie::Status::Response &response);
   void pidReset();
   void pidSet();
   void run();
   void publish_cmd(double pitch_cmd, double roll_cmd, double thrust_cmd, double yawrate_cmd = 0.0);
   void set_goal(double pos_x, double pos_y, double pos_z, double yawrate_goal = 0.0);

  
 private:
   
   ros::NodeHandle nh_;
   double yawrate, thrust;
   
   ros::Publisher vel_pub_, point_stamped_pub_, point_pub_, error_pub_, error_stamped_pub_,state_stamped_pub_;
   ros::Subscriber ground_truth_sub_,cmd_sub_,state_sub_;

   ros::ServiceServer pid_tuner_service,status_request_service;

   geometry_msgs::Twist cmd;
   ros::Timer takeoff_timer,land_timer, pos_ctrl_timer, emergency_timer;

   // In m
   double initial_position_x, initial_position_y, initial_position_z;
   double current_position_x, current_position_y, current_position_z;

   double kp_x, ki_x, kd_x;
   double kp_y, ki_y, kd_y;
   double kp_z, ki_z, kd_z;

   double goal_x, goal_y, goal_z;
   double error_x, error_y, error_z;

   bool initialized, calibrated, inflight;

   bool goal_reached_threshold;

   int count;

   int state;

   std::string status;

   PID pidX, pidY, pidZ;
   
 };

///////////////////////////////////////////
//
//	MEMBER FUNCTIONS
//
///////////////////////////////////////////

CrazyfliePositionController::CrazyfliePositionController():
  goal_x(0.0),
  goal_y(0.0),
  yawrate(0.0),
  goal_z(0.25),
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
  status("NOT CALIBRATED"),
  calibrated(false),
  initialized(false),
  inflight(false),
  state(WAITING),
  goal_reached_threshold(GOAL_REACH_THRESHOLD_INIT),
  count(0),
  pidX(KP_X, KD_X, KD_X, MIN_OUTPUT_X, MAX_OUTPUT_X, INTEGRATOR_MIN_X, INTEGRATOR_MAX_X, "x"),
  pidY(KP_Y, KD_Y, KD_Y, MIN_OUTPUT_Y, MAX_OUTPUT_Y, INTEGRATOR_MIN_Y, INTEGRATOR_MAX_Y, "y"),
  pidZ(KP_Z, KD_Z, KD_Z, MIN_OUTPUT_Z, MAX_OUTPUT_Z, INTEGRATOR_MIN_Z, INTEGRATOR_MAX_Z, "z")
{
  vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("crazyflie/deep_learning/cmd_vel", 1);
  state_stamped_pub_ =  nh_.advertise<geometry_msgs::TwistStamped>("crazyflie/deep_learning/state_stamped", 1);
  point_pub_ =  nh_.advertise<geometry_msgs::Point>("crazyflie/ground_truth/position", 1);
  error_pub_ =  nh_.advertise<geometry_msgs::Point>("crazyflie/ground_truth/position_error", 1);
  error_stamped_pub_ =  nh_.advertise<geometry_msgs::PointStamped>("crazyflie/ground_truth/position_error_stamped", 1);
  point_stamped_pub_ =  nh_.advertise<geometry_msgs::PointStamped>("crazyflie/ground_truth/position_stamped", 1);
  ground_truth_sub_ = nh_.subscribe("/crazyflie/ground_truth/pose_3d" , 1, &CrazyfliePositionController::getGroundTruth, this);
  state_sub_ = nh_.subscribe("crazyflie/deep_learning/cmd_state" , 1, &CrazyfliePositionController::stateSubscriber, this);
  cmd_sub_ = nh_.subscribe("crazyflie/deep_learning/cmd_pos" , 1, &CrazyfliePositionController::cmdSubscriber, this);
  takeoff_timer = nh_.createTimer(ros::Duration(0.01), &CrazyfliePositionController::takeoff, this);
  pos_ctrl_timer = nh_.createTimer(ros::Duration(0.01), &CrazyfliePositionController::pos_ctrl, this);
  land_timer = nh_.createTimer(ros::Duration(0.01), &CrazyfliePositionController::land, this);
  emergency_timer = nh_.createTimer(ros::Duration(0.001), &CrazyfliePositionController::emergency, this);
  pid_tuner_service = nh_.advertiseService("crazyflie/tune_pid", &CrazyfliePositionController::pid_tuner, this);
  status_request_service = nh_.advertiseService("crazyflie/request_status", &CrazyfliePositionController::requestStatus, this);
}

void CrazyfliePositionController::publish_cmd(double pitch_cmd, double roll_cmd, double thrust_cmd, double yawrate_cmd)
{
    cmd.linear.x = pitch_cmd;
    cmd.linear.y = roll_cmd;
    cmd.angular.y = yawrate_cmd;
    cmd.linear.z = thrust_cmd;
    vel_pub_.publish(cmd);
}

void CrazyfliePositionController::set_goal(double pos_x, double pos_y, double pos_z, double yawrate_goal)
{
	goal_x = pos_x;
	goal_y = pos_y;
	goal_z = pos_z;
	yawrate = yawrate_goal;
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

   // pidReset();
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

bool CrazyfliePositionController::requestStatus(deep_learning_crazyflie::Status::Request &request,
	deep_learning_crazyflie::Status::Response &response)
{
	response.status = status ;

	return true;
}

void CrazyfliePositionController::getGroundTruth(const geometry_msgs::PoseStampedConstPtr& OptiTrackPacket)
{
	current_position_x = OptiTrackPacket->pose.position.y;
	current_position_y = -OptiTrackPacket->pose.position.x;
	current_position_z = -OptiTrackPacket->pose.position.z;

	geometry_msgs::PointStamped position_msg;
	position_msg.header.frame_id = OptiTrackPacket->header.frame_id;
	position_msg.header.stamp = ros::Time::now();
	position_msg.point.x = current_position_x;
	position_msg.point.y = current_position_y;	
	position_msg.point.z = current_position_z;	

	point_stamped_pub_.publish(position_msg);
	point_pub_.publish(position_msg.point);

	error_z = initial_position_z + goal_z - current_position_z;
	error_y = initial_position_y + goal_y - current_position_y;
	error_x = initial_position_x + goal_x - current_position_x;

	geometry_msgs::PointStamped error_msg;
	error_msg.header.frame_id = OptiTrackPacket->header.frame_id;
	error_msg.header.stamp = ros::Time::now();
	error_msg.point.x = error_x;
	error_msg.point.y = error_y;
	error_msg.point.z = error_z;

	error_stamped_pub_.publish(error_msg);
	error_pub_.publish(error_msg.point);

	geometry_msgs::TwistStamped stamped_state_msg;
	stamped_state_msg.header.frame_id = OptiTrackPacket->header.frame_id;
	stamped_state_msg.header.stamp = ros::Time::now();
	stamped_state_msg.twist.angular.x = error_x;
	stamped_state_msg.twist.angular.y = error_y;
	stamped_state_msg.twist.angular.z = error_z;

	stamped_state_msg.twist.linear.x = current_position_x - initial_position_x;
	stamped_state_msg.twist.linear.y = current_position_y - initial_position_y;
	stamped_state_msg.twist.linear.z = current_position_z - initial_position_z;

	state_stamped_pub_.publish(stamped_state_msg);

	if(calibrated) 
	    	if(abs(error_x) > OUT_OF_BOUNDS_THRESHOLD || abs(error_y) > OUT_OF_BOUNDS_THRESHOLD || abs(error_z) > OUT_OF_BOUNDS_THRESHOLD)
	    	{
			status = "OUT OF BOUNDS";
			state = EMERGENCY;
			ROS_INFO("OUT OF BOUNDS - EMERGENCY STOP");
	    	}
	
	if(!initialized)
		initialized = true;
}

void CrazyfliePositionController::cmdSubscriber(const geometry_msgs::TwistConstPtr& cmd_pos)
{
	set_goal(cmd_pos->linear.x,cmd_pos->linear.y,cmd_pos->linear.z,cmd_pos->angular.y);

	ROS_INFO("GOAL CHANGED : Goal X = %f, Goal Y = %f, Goal Z = %f", goal_x, goal_y,goal_z);
	status ="GOAL CHANGED";

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
			pidReset();
			set_goal(0.0,0.0,0.25);
			count = 0;
			calibrated = true;
			status = "CALIBRATED";
		}
		else
		{
			ROS_INFO("Not Initalized");
			status = "NOT INITIALIZED";
		}
		
		state = WAITING;
	}
	else
	{
		state = cmd_state->data;
	}

}

void CrazyfliePositionController::takeoff(const ros::TimerEvent& e)
{
   if(state==TAKE_OFF)
   {

	set_goal(0.0,0.0,0.25);

 	if(calibrated)
	{
		float dt = e.current_real.toSec() - e.last_real.toSec();

	 	if (current_position_z > initial_position_z + 0.05  || thrust > 50000)
		 {
		    ROS_INFO("Crazyflie is now inflight");
		    pidReset();
		    pidSet();
		    
		    if(pidZ.ki() != 0.0)
		    	pidZ.setIntegral(thrust / pidZ.ki());
		    
		    inflight = true;
		    status = "INFLIGHT";
		    state = POS_CTRL;
		    thrust = 0;
		}
		else
		{
		    thrust += TAKEOFF_GAIN * dt;
		    publish_cmd(0.0,0.0,thrust);
		    ROS_INFO("TAKING OFF : Increasing Thrust = %f", thrust);
		    status = "TAKING OFF";
		}
	}

	else
	{
		ROS_INFO("Not Calibrated");
		status = "NOT CALIBRATED";
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

	    set_goal(goal_x,goal_y,0.0);

	    if(current_position_z <= initial_position_z + 0.1 )
	    {
		if(cmd.linear.z > MIN_OUTPUT_Z+5000 && current_position_z >= initial_position_z + 0.05)
		{
		    publish_cmd(0,0,cmd.linear.z-(LANDING_GAIN*dt));
		    ROS_INFO("LANDING : Decreasing Thrust = %f", cmd.linear.z);
		    status = "LANDING";
		}

		else
		{
			publish_cmd(0.0,0.0,0.0);
			inflight = false;
			calibrated = false;
			state = WAITING;
			status = "LANDED";
			ROS_INFO("LANDED");

		}
	    }
	   else
	   {
		    double pitch_cmd = pidX.update(current_position_x, initial_position_x + goal_x);
		    double roll_cmd = pidY.update(current_position_y, initial_position_y + goal_y);
		    double thrust_cmd = pidZ.update(current_position_z, initial_position_z + 0.05);

		    publish_cmd(pitch_cmd, roll_cmd, thrust_cmd);

		    ROS_INFO("Will land soon: Pitch = %f, Roll = %f, Thrust = %f ", cmd.linear.x, cmd.linear.y, cmd.linear.z);

		    status = "LANDING";
           }
	}

	else
	{
		ROS_INFO("Not Calibrated");
		status = "NOT CALIBRATED";
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
	    double pitch_cmd = pidX.update(current_position_x, initial_position_x + goal_x);
            double roll_cmd = pidY.update(current_position_y, initial_position_y + goal_y);
            double thrust_cmd = pidZ.update(current_position_z, initial_position_z + goal_z);

	    publish_cmd(pitch_cmd,roll_cmd,thrust_cmd);

	    ROS_INFO("POSITION CONTROL CMD : Pitch= %f, Roll= %f, Thrust= %f ",cmd.linear.x,cmd.linear.y,cmd.linear.z);
	    ROS_INFO("GOAL POSITION : x= %f, y= %f, z= %f ",initial_position_x + goal_x,initial_position_y + goal_y,initial_position_z + goal_z);
	    ROS_INFO("CURRENT POSITION : x= %f, y= %f, z= %f ",current_position_x,current_position_y ,current_position_z);
	    ROS_INFO("ERROR POSITION : x= %f, y= %f, z= %f ",error_x,error_y,error_z);
	    ROS_INFO(" ");

	    if(abs(error_x)<= goal_reached_threshold && abs(error_y) <= goal_reached_threshold && abs(error_z) <= goal_reached_threshold && abs(pidX.velocity()) <= goal_reached_threshold && abs(pidY.velocity()) <= goal_reached_threshold && abs(pidZ.velocity()) <= goal_reached_threshold)
	     {
		status = "GOAL REACHED";
		goal_reached_threshold = GOAL_REACH_THRESHOLD_INIT;
		count = 0;
	     }
	    else
  	    {	    
		    count++;
		    if(count > 500)
		    {
			if(goal_reached_threshold < GOAL_REACH_THRESHOLD_MAX)
				goal_reached_threshold+=0.01/100.0;
		    }
	    }

	}

	else
	{
		ROS_INFO("Not Calibrated");
		status = "NOT CALIBRATED";
	}

   }
}

void CrazyfliePositionController::emergency(const ros::TimerEvent& e)
{
   if(state==EMERGENCY)
   {
	    ROS_INFO("EMERGENCY STOP"); 
	    publish_cmd(0.0,0.0,0.0);
	    thrust = 0;
	    pidReset();
	    state = WAITING;
 	    status="NOT CALIBRATED";
	    calibrated = false;
	    inflight = false;
	    initialized = false;
   }
}

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crazyflie_position_control_node");
  CrazyfliePositionController crazyflie_position_controller;
  crazyflie_position_controller.run();

  return(0);
}

