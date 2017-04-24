#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
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

#define KP_X 2.0
#define KI_X 0.0
#define KD_X 0.0

#define KP_Y 2.0
#define KI_Y 0.0
#define KD_Y 0.0

#define KP_Z 2.0
#define KI_Z 0.0
#define KD_Z 0.0

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


using namespace geometry_msgs;
using namespace std;

int kfd = 0;
struct termios cooked, raw;



class TeleopCrazyflie
 {

 public:
   TeleopCrazyflie();
   void keyLoop();
   void getGroundTruth(const geometry_msgs::PoseStampedConstPtr& OptiTrackPacket);
   void takeoff(const ros::TimerEvent&);
   void land(const ros::TimerEvent&);
   void pos_ctrl(const ros::TimerEvent&);
   void emergency(const ros::TimerEvent&);
   void pidReset();
 
 private:
   
   ros::NodeHandle nh_;
   double yawrate, thrust;
   
   ros::Publisher vel_pub_;
   ros::Subscriber ground_truth_sub_;

   geometry_msgs::Twist cmd;

   ros::Timer takeoff_timer,land_timer, pos_ctrl_timer, emergency_timer;


   // In m

   float initial_position_x;
   float initial_position_y;
   float initial_position_z;

   float current_position_x;
   float current_position_y;
   float current_position_z;

   double goal_x, goal_y, goal_z;

   bool initialized;

   bool state;

   PID pidX;
   PID pidY;
   PID pidZ;

   
 };


TeleopCrazyflie::TeleopCrazyflie():
  goal_x(0.0),
  goal_y(0.0),
  yawrate(0.0),
  goal_z(0.15),
  thrust(0.0),
  state(WAITING),
  pidX(KP_X, KD_X, KD_X, MIN_OUTPUT_X, MAX_OUTPUT_X, INTEGRATOR_MIN_X, INTEGRATOR_MAX_X, "x"),
  pidY(KP_Y, KD_Y, KD_Y, MIN_OUTPUT_Y, MAX_OUTPUT_Y, INTEGRATOR_MIN_Y, INTEGRATOR_MAX_Y, "y"),
  pidZ(KP_Z, KD_Z, KD_Z, MIN_OUTPUT_Z, MAX_OUTPUT_Z, INTEGRATOR_MIN_Z, INTEGRATOR_MAX_Z, "z")
{
  vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("crazyflie/deep_learning/cmd_vel", 1);
  initialized = false;
  ground_truth_sub_ = nh_.subscribe("crazyflie/ground_truth/pose_3d" , 1, &TeleopCrazyflie::getGroundTruth, this);
  takeoff_timer = nh_.createTimer(ros::Duration(0.1), &TeleopCrazyflie::takeoff, this);
  land_timer = nh_.createTimer(ros::Duration(0.1), &TeleopCrazyflie::land, this);
  pos_ctrl_timer = nh_.createTimer(ros::Duration(0.1), &TeleopCrazyflie::pos_ctrl, this);
  emergency_timer = nh_.createTimer(ros::Duration(0.1), &TeleopCrazyflie::emergency, this);
}

void TeleopCrazyflie::pidReset()
    {
        pidX.reset();
        pidZ.reset();
        pidZ.reset();
    }

void TeleopCrazyflie::getGroundTruth(const geometry_msgs::PoseStampedConstPtr& OptiTrackPacket)
{

	//Convert mm to meters, current_position_z is height
	current_position_x = OptiTrackPacket->pose.position.x/1000.0;
	current_position_y = OptiTrackPacket->pose.position.z/1000.0;
	current_position_z = OptiTrackPacket->pose.position.y/1000.0;

	if(!initialized)
	{
		initial_position_x = current_position_x;
		initial_position_y = current_position_y;
		initial_position_z = current_position_z;
	}
}

void TeleopCrazyflie::takeoff(const ros::TimerEvent& e)
{
   if(initialized && state==TAKE_OFF)
   {
	float dt = e.current_real.toSec() - e.last_real.toSec();

 	if (current_position_z > initial_position_z + 0.05 || thrust > 50000)
         {
            pidReset();
            pidZ.setIntegral(thrust / pidZ.ki());
            state = POS_CTRL;
            thrust = 0;
        }
        else
        {
            thrust += 10000 * dt;
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.y = 0.0;
            cmd.linear.z = thrust;
            vel_pub_.publish(cmd);
        }

   }

}

void TeleopCrazyflie::land(const ros::TimerEvent& e)
{
   if(initialized && state==LAND)
   {
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


void TeleopCrazyflie::pos_ctrl(const ros::TimerEvent& e)
{
   if(initialized && state==POS_CTRL)
   {
	    cmd.linear.x = pidX.update(current_position_x, initial_position_x + goal_x);
            cmd.linear.y = pidY.update(current_position_y, initial_position_y + goal_y);
            cmd.angular.y = yawrate;
            cmd.linear.z = pidZ.update(current_position_z, initial_position_z + goal_z);
            vel_pub_.publish(cmd);
   }

}

void TeleopCrazyflie::emergency(const ros::TimerEvent& e)
{
   if(state==EMERGENCY)
   {
	    cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.y = 0.0;
            cmd.linear.z = 0.0;
            vel_pub_.publish(cmd);
	    pidReset();
   }

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
  double goal_z_offset,goal_y_offset,goal_x_offset,yawrate_offset;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the crazyflie.");
  puts("T - TakeOff.");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

   ROS_INFO("value: 0x%02X\n", c);
   //ros::Duration(0.05).sleep();
   ROS_INFO("Z- %f, Y - %f ,X- %f, yawrate - %f",current_position_z,current_position_y, current_position_x, yawrate);

   yawrate_offset = 1.0;
   goal_z_offset = 0.05;
   goal_y_offset = 0.05;
   goal_x_offset = 0.05;

    switch(c)
    {
      case KEYCODE_T:
        ROS_DEBUG("TAKING OFF");
	state = TAKE_OFF;
        break;

      case KEYCODE_L:
        ROS_DEBUG("LAND");
	state = LAND;
        break;

      case KEYCODE_P:
        ROS_DEBUG("POSITION CONTROL");
	state = POS_CTRL;
        break;

      case KEYCODE_Q:
        ROS_DEBUG("STOP");
	state = EMERGENCY;
        break;

      case KEYCODE_LA:
        ROS_DEBUG("LEFT");
        goal_y -= goal_y_offset;
        ROS_INFO("goal_y = %f", goal_y);
        break;

      case KEYCODE_RA:
        ROS_DEBUG("RIGHT");
        goal_y += goal_y_offset;
        ROS_INFO("goal_y = %f", goal_y);
        break;

      case KEYCODE_UA:
        ROS_DEBUG("UP");
        goal_x += goal_x_offset;
        ROS_INFO("goal_x = %f", goal_x);
        break;

      case KEYCODE_DA:
        ROS_DEBUG("DOWN");
        goal_x -= goal_x_offset;
        ROS_INFO("goal_x = %f", goal_x);
        break;

      case KEYCODE_A:
        ROS_DEBUG("LEFT_A");
        yawrate -= yawrate_offset;
        ROS_INFO("yawrate = %f", yawrate);
        break;

      case KEYCODE_D:
        ROS_DEBUG("RIGHT_D");
        yawrate += yawrate_offset;
        ROS_INFO("yawrate = %f", yawrate);
        break;

      case KEYCODE_W:
        ROS_DEBUG("UP_W");
        goal_z += goal_z_offset;
        ROS_INFO("goal_z = %f", goal_z);
        break;

      case KEYCODE_S:
        ROS_DEBUG("DOWN_S");
        goal_z -= goal_z_offset;
        ROS_INFO("goal_z = %f", goal_z);
        break;

    }
  }
  return;
}



void quit(int sig)
{
   tcsetattr(kfd, TCSANOW, &cooked);
   ros::shutdown();
   exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crazyflie_pos_ctrl_node");
  TeleopCrazyflie teleop_crazyflie;

  signal(SIGINT,quit);

  teleop_crazyflie.keyLoop();
  
  return(0);
}

