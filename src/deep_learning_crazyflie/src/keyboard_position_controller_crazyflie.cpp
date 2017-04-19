#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "pid.hpp"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_UA 0x41
#define KEYCODE_DA 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64

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
#define INTEGRATOR_MAX_Z -1000.0

#define MAX_OUTPUT_Z 60000
#define MIN_OUTPUT_Z 10000

#define INTEGRATOR_MAX_X 0.1
#define INTEGRATOR_MAX_X -0.1

#define MAX_OUTPUT_X 10
#define MIN_OUTPUT_X -10

#define INTEGRATOR_MAX_Y 0.1
#define INTEGRATOR_MAX_Y -0.1

#define MAX_OUTPUT_Y 10
#define MIN_OUTPUT_Y -10

using namespace geometry_msgs;
using namespace std;


class TeleopCrazyflie
 {

 public:
   TeleopCrazyflie();
   void keyLoop();
   void getGroundTruth(const geometry_msgs::PoseStampedConstPtr&)

 
 private:
   
   ros::NodeHandle nh_;
   double roll, pitch, yawrate, thrust;
   ros::Publisher vel_pub_;
   ros::Subscriber ground_truth_sub_;

   float initial_position_x;
   float initial_position_y;
   float initial_position_z;

   float current_position_x;
   float current_position_y;
   float current_position_z;

   bool initialized;
   
 };

TeleopCrazyflie::getGroundTruth(const geometry_msgs::PoseStampedConstPtr& OptiTrackPacket)
{
	current_position_x = OptiTrackPacket->pose.position.x;
	current_position_y = OptiTrackPacket->pose.position.y;
	current_position_z = OptiTrackPacket->pose.position.z;

	if(!initialized)
	{
		initial_position_x = current_position_x;
		initial_position_y = current_position_y;
		initial_position_z = current_position_z;
	}
}

TeleopCrazyflie::TeleopCrazyflie():
  roll(0.0),
  pitch(0.0),
  yawrate(0.0),
  thrust(32767.0)
{
  vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("crazyflie/deep_learning/cmd_vel", 1);
  initialized = false;
  ground_truth_sub_ = nh_.subscribe("crazyflie/ground_truth/pose_3d" , 1, &TeleopCrazyflie::getGroundTruth, this);
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

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the crazyflie.");

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
   ROS_INFO("Thrust - %f, roll - %f ,pitch - %f,yawrate - %f",thrust,roll,pitch,yawrate);
   thrust_offset = 1000 ;
   roll_offset = 1.0;
   yawrate_offset = 1.0;
   pitch_offset = 1.0;
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        roll -= roll_offset;
	dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        roll += roll_offset;
	dirty = true;
        break;
      case KEYCODE_UA:
        ROS_DEBUG("UP");
        pitch += pitch_offset;
	dirty = true;
        break;
      case KEYCODE_DA:
        ROS_DEBUG("DOWN");
        pitch -= pitch_offset;
	dirty = true;
        break;
      case KEYCODE_A:
        ROS_DEBUG("LEFT_A");
        yawrate -= yawrate_offset;
	dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("RIGHT_D");
        yawrate += yawrate_offset;
	dirty = true;
        break;
      case KEYCODE_W:
        ROS_DEBUG("UP_W");
        thrust += thrust_offset;
	dirty = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("DOWN_S");
        thrust -= thrust_offset;
	dirty = true;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("STOP");
        thrust = 0;
	roll = 0;
	pitch = 0;
        yawrate = 0;
        dirty = true;
        break;
    }
   
    geometry_msgs::Twist twist;
    twist.linear.z = thrust;
    twist.linear.x = pitch;
    twist.linear.y = roll;
    twist.angular.y = yawrate;

    if(dirty ==true)
    {
      thrust_offset = 0.0;
      roll_offset = 0.0;   
      yawrate_offset = 0.0;
      pitch_offset = 0.0;
      vel_pub_.publish(twist);    
      dirty=false;
    }
  }
  return;
}

