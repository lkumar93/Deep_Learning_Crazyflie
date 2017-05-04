#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "deep_learning_crazyflie/TunePID.h"
#include "deep_learning_crazyflie/Status.h"
#include <thread>

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
#define KEYCODE_O 0x6F
#define KEYCODE_I 0x69
#define KEYCODE_C 0x63
#define KEYCODE_E 0x65

#define POSITION_MODE 0
#define ORIENTATION_MODE 1
#define PID_TUNING_MODE 2

#define WAITING 0
#define TAKE_OFF 1
#define POS_CTRL 2
#define LAND 3
#define EMERGENCY 4
#define CALIBRATE 5

#define X_TUNE 0
#define Y_TUNE 1
#define Z_TUNE 2

#define KP_Z_INIT 5000.0
#define KI_Z_INIT 3500.0
#define KD_Z_INIT 6000.0

#define KP_Z_OFFSET 100.0
#define KI_Z_OFFSET 100.0
#define KD_Z_OFFSET 100.0

#define KP_X_INIT 40.0
#define KI_X_INIT 2.0
#define KD_X_INIT 30.0

#define KP_Y_INIT 40.0
#define KI_Y_INIT 2.0
#define KD_Y_INIT 30.0

#define KP_XY_OFFSET 0.25
#define KI_XY_OFFSET 0.25
#define KD_XY_OFFSET 0.25




using namespace geometry_msgs;
using namespace std;


class TeleopCrazyflie
 {

 public:
   TeleopCrazyflie();
   void keyLoop();
   void executeTrajectory();
 
 private:
   
   ros::NodeHandle nh_;
   double roll, pitch, yawrate, thrust, goal_x, goal_y, goal_z, kp, ki, kd;
   int mode, state, tune_param, prev_tune_param;
   ros::Publisher vel_pub_, cmd_pub_, state_pub_;
   ros::ServiceClient pid_tuning_client, status_request_client;
   double kp_z_prev, ki_z_prev, kd_z_prev;
   double kp_y_prev, ki_y_prev, kd_y_prev;
   double kp_x_prev, ki_x_prev, kd_x_prev;
   std::string status;
   
 };

TeleopCrazyflie::TeleopCrazyflie():
  roll(0.0),
  pitch(0.0),
  yawrate(0.0),
  thrust(0.0),
  mode(ORIENTATION_MODE),
  state(WAITING),
  goal_x(0.0),
  goal_y(0.0),
  goal_z(0.25),
  kp(KP_Z_INIT),
  kd(KD_Z_INIT),
  ki(KI_Z_INIT),
  status("NOT CALIBRATED"),
  kp_z_prev(KP_Z_INIT),
  ki_z_prev(KI_Z_INIT),
  kd_z_prev(KD_Z_INIT),
  kp_x_prev(KP_X_INIT),
  ki_x_prev(KI_X_INIT),
  kd_x_prev(KD_X_INIT),
  kp_y_prev(KP_Y_INIT),
  ki_y_prev(KI_Y_INIT),
  kd_y_prev(KD_Y_INIT),
  tune_param(Z_TUNE),
  prev_tune_param(Z_TUNE)
{
  vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("crazyflie/deep_learning/cmd_vel", 1);
  state_pub_ =  nh_.advertise<std_msgs::Int32>("crazyflie/deep_learning/cmd_state", 1);
  cmd_pub_ =  nh_.advertise<geometry_msgs::Twist>("crazyflie/deep_learning/cmd_pos", 1);
  pid_tuning_client = nh_.serviceClient<deep_learning_crazyflie::TunePID>("crazyflie/tune_pid");
  status_request_client = nh_.serviceClient<deep_learning_crazyflie::Status>("crazyflie/request_status");

}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
   tcsetattr(kfd, TCSANOW, &cooked);
   ros::shutdown();
   exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crazyflie_teleop_node");
  TeleopCrazyflie teleop_crazyflie;

  signal(SIGINT,quit);

  teleop_crazyflie.keyLoop();
  
  return(0);
}

void TeleopCrazyflie::executeTrajectory()
{
     std_msgs::Int32 state_msg;

     bool execute_trajectory = true;
     bool point_1_reached = false;
     bool point_2_reached = false;
     bool point_3_reached = false;
     bool point_4_reached = false;
     bool point_5_reached = false;
     ROS_INFO("Starting Trajectory");

     while(execute_trajectory)
     {

	if(state == EMERGENCY)
		return;
	
        deep_learning_crazyflie::Status srv;

        if(!status_request_client.call(srv))
        {
		ROS_INFO("Failed to call service request_status");
        }
        else
       {
		 status = srv.response.status;
       }

	if(status == "NOT CALIBRATED" )
	{
		ROS_INFO("Calibrate to execute trajectory");
		break;
	}
	
	else if	(status == "NOT INITIALIZED")
	{
		ROS_INFO("Mocap Data not initialized");
		break;
	}	

	else if(status == "CALIBRATED")
	{
		state = TAKE_OFF;
		state_msg.data = state;
      		state_pub_.publish(state_msg);
		
		ROS_INFO("Taking Off");
	}
	else if(status == "OUT OF BOUNDS")
	{
		ROS_INFO("Out of Bounds. Stopping Crazyflie");
		state = EMERGENCY;
		thrust = 0;
		roll = 0;
		pitch = 0;
		yawrate = 0;
		state_msg.data = state;
		state_pub_.publish(state_msg);
		execute_trajectory = false;
		break;
	}
	else
	{

   		geometry_msgs::Twist twist;

		if(status == "GOAL REACHED")
		{

		    if(!point_1_reached)
		    {
			    ros::Duration(3).sleep();
			    ROS_INFO("Going Right");
			    twist.linear.z = 0.25;
			    twist.linear.x = 0.0;
			    twist.linear.y = 0.25;
			    cmd_pub_.publish(twist); 
			    ros::Duration(5).sleep();
			    point_1_reached = true;
		    }

		    else if(!point_2_reached)
		    {
			    ROS_INFO("Going Forward");
			    twist.linear.z = 0.25;
			    twist.linear.x = 0.25;
			    twist.linear.y = 0.25;
			    cmd_pub_.publish(twist); 
			    ros::Duration(5).sleep();
			    point_2_reached = true;
		    }

		    else if(!point_3_reached)
		    {
			    ROS_INFO("Going Left");
			    twist.linear.z = 0.25;
			    twist.linear.x = 0.25;
			    twist.linear.y = -0.25;
			    cmd_pub_.publish(twist); 
			    ros::Duration(5).sleep();
			    point_3_reached = true;
		    }

		    else if(!point_4_reached)
		    {
			    ROS_INFO("Going Backward");
			    twist.linear.z = 0.25;
			    twist.linear.x = -0.25;
			    twist.linear.y = -0.25;
			    cmd_pub_.publish(twist); 
			    ros::Duration(5).sleep();
			    point_4_reached = true;
		    }

		    else if(!point_5_reached)
		    {
			    ROS_INFO("Going Home");
			    twist.linear.z = 0.25;
			    twist.linear.x = 0.0;
			    twist.linear.y = 0.0;
			    cmd_pub_.publish(twist); 
			    ros::Duration(5).sleep();
			    point_5_reached = true;
		    }

		    else
		    {
			ROS_INFO("Landing");
			state = LAND;
			state_msg.data = state;
	      		state_pub_.publish(state_msg);
			execute_trajectory = false;
		    }
		}
	}

     }

    ROS_INFO("Stopping Trajectory");
    return;
}

void TeleopCrazyflie::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  double thrust_offset,roll_offset,pitch_offset,yawrate_offset;
  double goal_x_offset,goal_y_offset,goal_z_offset;
  double kp_offset, kd_offset, ki_offset;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the crazyflie.");

  puts("P - Position Mode");
  puts("O - Orientation Mode");
  puts("I - PID Tuning Mode");
  puts("C - Calibrate");
  puts("T - Take Off");
  puts("L - Land");
  puts("E - Execute Trajectory");  

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

   ROS_INFO("value: 0x%02X\n", c);

   thrust_offset = 1000 ;
   roll_offset = 1.0;
   yawrate_offset = 1.0;
   pitch_offset = 1.0;
   goal_z_offset = 0.05;
   goal_y_offset = 0.05;
   goal_x_offset = 0.05;

   kp_offset = KP_Z_OFFSET;
   kd_offset = KD_Z_OFFSET;
   ki_offset = KI_Z_OFFSET;

   if( tune_param == X_TUNE || tune_param == Y_TUNE)
   {
	   kp_offset = KP_XY_OFFSET;
	   ki_offset = KI_XY_OFFSET;
	   kd_offset = KD_XY_OFFSET;
   }

   //ros::Duration(0.05).sleep();

    std_msgs::Int32 state_msg;


    switch(c)
    {
      case KEYCODE_T:
        ROS_INFO("TAKE OFF");

	if(mode == POSITION_MODE || mode == PID_TUNING_MODE)
	{
		state = TAKE_OFF;
		state_msg.data = state;
      		state_pub_.publish(state_msg);
	}
	else
		ROS_INFO("Take Off available only in Position Mode");
	dirty = true;
        break;

      case KEYCODE_C:
        ROS_INFO("CALIBRATE");
	state = CALIBRATE;	
	dirty = true;
	state_msg.data = state;
	state_pub_.publish(state_msg);
        break;


      case KEYCODE_L:
        ROS_INFO("LAND");

	if(mode == POSITION_MODE || mode == PID_TUNING_MODE)
	{
		state = LAND;
		state_msg.data = state;
      		state_pub_.publish(state_msg);
	}

	else
		ROS_INFO("Landing available only in Position Mode");

	dirty = true;
        break;

      case KEYCODE_P:
        ROS_INFO("ACTIVATE POSITION MODE");
        mode = POSITION_MODE;
	state = WAITING;
	state_msg.data = state;
	state_pub_.publish(state_msg);
	dirty = true;
        break;

      case KEYCODE_E:
        ROS_INFO("Executing Trajectory");
	{
	std::thread TrajectoryServer(&TeleopCrazyflie::executeTrajectory,this);
	TrajectoryServer.join();}
        break;


      case KEYCODE_O:
        ROS_INFO("ACTIVATE ORIENTATION MODE");
        mode = ORIENTATION_MODE;
	state = WAITING;
	state_msg.data = state;
	state_pub_.publish(state_msg);
	dirty = true;
        break;

      case KEYCODE_I:
        ROS_INFO("ACTIVATE PID TUNING MODE");
        mode = PID_TUNING_MODE;
	state = WAITING;
	state_msg.data = state;
	state_pub_.publish(state_msg);
	dirty = true;
        break;

      case KEYCODE_Q:
        ROS_INFO("EMERGENCY STOP");
	state = EMERGENCY;
        thrust = 0;
	roll = 0;
	pitch = 0;
        yawrate = 0;
        dirty = true;
	state_msg.data = state;
	state_pub_.publish(state_msg);
        break;

      case KEYCODE_LA:
        ROS_DEBUG("LEFT");

	if(mode == ORIENTATION_MODE)
        	roll -= roll_offset;

	else if(mode == POSITION_MODE)
	        goal_y -= goal_y_offset;	
	
	else if(mode == PID_TUNING_MODE)
	        kd -= kd_offset;

	dirty = true;
        break;

      case KEYCODE_RA:
        ROS_DEBUG("RIGHT");
	if(mode == ORIENTATION_MODE)
        	roll += roll_offset;

	else if(mode == POSITION_MODE)
	        goal_y += goal_y_offset;

	else if(mode == PID_TUNING_MODE)
	        kd += kd_offset;

	dirty = true;
        break;

      case KEYCODE_UA:
        ROS_DEBUG("UP");
	if(mode == ORIENTATION_MODE)
        	pitch += pitch_offset;

	else if(mode == POSITION_MODE)
	        goal_x += goal_x_offset;

	else if(mode == PID_TUNING_MODE)
	        kp += kp_offset;

	dirty = true;
        break;

      case KEYCODE_DA:

        ROS_DEBUG("DOWN");
	if(mode == ORIENTATION_MODE)
       		pitch -= pitch_offset;

	else if(mode == POSITION_MODE)
	        goal_x -= goal_x_offset;

	else if(mode == PID_TUNING_MODE)
	        kp -= kp_offset;

	dirty = true;
        break;

      case KEYCODE_A:
        ROS_DEBUG("LEFT_A");
        yawrate -= yawrate_offset;
	if(mode == PID_TUNING_MODE)
	{
		if(tune_param <= X_TUNE)
	        	tune_param = X_TUNE;
		else
			tune_param=  tune_param-1;
	}
	dirty = true;
        break;


      case KEYCODE_D:
        ROS_DEBUG("RIGHT_D");
        yawrate += yawrate_offset;
	if(mode == PID_TUNING_MODE)
	{
		if(tune_param >= Z_TUNE)
	        	tune_param = Z_TUNE;
		else
			tune_param = tune_param + 1;

	}
	dirty = true;
        break;

      case KEYCODE_W:
        ROS_DEBUG("UP_W");
	if(mode == ORIENTATION_MODE)
        	thrust += thrust_offset;

	else if(mode == POSITION_MODE)
	        goal_z += goal_z_offset;

	else if(mode == PID_TUNING_MODE)
	        ki += ki_offset;

	dirty = true;
        break;

      case KEYCODE_S:
        ROS_DEBUG("DOWN_S");
	if(mode == ORIENTATION_MODE)
        	thrust -= thrust_offset;
	else if(mode == POSITION_MODE)
	        goal_z -= goal_z_offset;
	else if(mode == PID_TUNING_MODE)
	        ki -= ki_offset;

	dirty = true;
        break;

    }
   
    geometry_msgs::Twist twist;

    twist.angular.y = yawrate;

    if(mode == ORIENTATION_MODE)
    {
	    twist.linear.z = thrust;
	    twist.linear.x = pitch;
	    twist.linear.y = roll;
    }

    else if(mode == POSITION_MODE)
    {
	    twist.linear.z = goal_z;
	    twist.linear.x = goal_x;
	    twist.linear.y = goal_y;
     }	

    else if(mode == PID_TUNING_MODE)
	{
	
		if(prev_tune_param == tune_param)
		{
			if(tune_param == Z_TUNE)
			{
				kp_z_prev = kp;
				kd_z_prev = kd;
				ki_z_prev = ki;
			}

			else if(tune_param == X_TUNE)
			{
				kp_x_prev = kp;
				kd_x_prev = kd;
				ki_x_prev = ki;
			}

			else if(tune_param == Y_TUNE)
			{
				kp_y_prev = kp;
				kd_y_prev = kd;
				ki_y_prev = ki;
			}
		}

		else
		{
			if(tune_param == Z_TUNE)
			{
				kp = kp_z_prev;
				kd = kd_z_prev;
				ki = ki_z_prev;
			}

			else if(tune_param == X_TUNE)
			{
				kp = kp_x_prev;
				kd = kd_x_prev;
				ki = ki_x_prev;
			}

			else if(tune_param == Y_TUNE)
			{
				kp = kp_y_prev;
				kd = kd_y_prev;
				ki = ki_y_prev;
			}
		}

		deep_learning_crazyflie::TunePID srv;

		srv.request.kp = kp;
		srv.request.ki = ki;
		srv.request.kd = kd;
		srv.request.param = tune_param;

		if(!pid_tuning_client.call(srv))
		{
			ROS_INFO("Failed to call service tune_pid");
		}

		if(tune_param == Z_TUNE)
			ROS_INFO("PID TUNING: Tune Param = Z, Kp - %f, Ki - %f , Kd - %f",kp,ki,kd);

		if(tune_param == Y_TUNE)
			ROS_INFO("PID TUNING: Tune Param = Y, Kp - %f, Ki - %f , Kd - %f",kp,ki,kd);

		if(tune_param == X_TUNE)
			ROS_INFO("PID TUNING: Tune Param = X, Kp - %f, Ki - %f , Kd - %f",kp,ki,kd);

		prev_tune_param = tune_param;

	}





    if(dirty ==true)
    {
      thrust_offset = 0.0;
      roll_offset = 0.0;   
      yawrate_offset = 0.0;
      pitch_offset = 0.0;
      goal_x_offset = 0.0;
      goal_y_offset = 0.0;   
      goal_z_offset = 0.0;
      kp_offset = 0.0;
      kd_offset = 0.0;
      ki_offset = 0.0;

      dirty=false;
    }

      if(state!= CALIBRATE)
      {
	   if(mode == ORIENTATION_MODE)
	   {
	    	vel_pub_.publish(twist); 
	   }

	   else if(mode == POSITION_MODE)
	   {
		cmd_pub_.publish(twist); 
	   }

	   if(mode == POSITION_MODE)
		ROS_INFO("POSITION MODE: X - %f, Y - %f ,Z - %f,yawrate - %f", goal_x ,goal_y, goal_z,yawrate);

	   else if(mode == ORIENTATION_MODE)
	   	ROS_INFO("ORIENTATION MODE: Thrust - %f, roll - %f ,pitch - %f,yawrate - %f",thrust,roll,pitch,yawrate);
     }

  }
  return;
}

