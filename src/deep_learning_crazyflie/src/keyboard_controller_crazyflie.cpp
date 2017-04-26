#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

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
#define KEYCODE_O 0x79

#define POSITION_MODE 0
#define ORIENTATION_MODE 1

#define WAITING 0
#define TAKE_OFF 1
#define POS_CTRL 2
#define LAND 3
#define EMERGENCY 4

using namespace geometry_msgs;
using namespace std;


class TeleopCrazyflie
 {

 public:
   TeleopCrazyflie();
   void keyLoop();
 
 private:
   
   ros::NodeHandle nh_;
   double roll, pitch, yawrate, thrust, goal_x, goal_y, goal_z;
   int mode, state;
   ros::Publisher vel_pub_, cmd_pub_, state_pub_;
   
 };

TeleopCrazyflie::TeleopCrazyflie():
  roll(0.0),
  pitch(0.0),
  yawrate(0.0),
  thrust(32767.0),
  mode(ORIENTATION_MODE),
  state(WAITING),
  goal_x(0.0),
  goal_y(0.0),
  goal_z(0.15)
{
  vel_pub_ =  nh_.advertise<geometry_msgs::Twist>("crazyflie/deep_learning/cmd_vel", 1);
  state_pub_ =  nh_.advertise<std_msgs::Int32>("crazyflie/deep_learning/cmd_state", 1);
  cmd_pub_ =  nh_.advertise<geometry_msgs::Twist>("crazyflie/deep_learning/cmd_pos", 1);

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
  double goal_x_offset,goal_y_offset,goal_z_offset;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the crazyflie.");

  puts("P - Position Mode");
  puts("O - Orientation Mode");
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
   goal_z_offset = 0.05;
   goal_y_offset = 0.05;
   goal_x_offset = 0.05;
 
    switch(c)
    {
      case KEYCODE_T:
        ROS_INFO("TAKING OFF");
	if(mode == POSITION_MODE)
		state = TAKE_OFF;
	dirty = true;
        break;

      case KEYCODE_L:
        ROS_INFO("LAND");
	if(mode == POSITION_MODE)
		state = LAND;
	dirty = true;
        break;

      case KEYCODE_P:
        ROS_INFO("POSITION MODE");
        mode = POSITION_MODE;
	dirty = true;
        break;

      case KEYCODE_O:
        ROS_INFO("ORIENTATION MODE");
        mode = ORIENTATION_MODE;
	dirty = true;
        break;

      case KEYCODE_Q:
        ROS_INFO("STOP");
	state = EMERGENCY;
        thrust = 0;
	roll = 0;
	pitch = 0;
        yawrate = 0;
        dirty = true;
        break;

      case KEYCODE_LA:
        ROS_DEBUG("LEFT");

	if(mode == ORIENTATION_MODE)
        	roll -= roll_offset;
	else
	        goal_y -= goal_y_offset;
		
	dirty = true;
        break;

      case KEYCODE_RA:
        ROS_DEBUG("RIGHT");
	if(mode == ORIENTATION_MODE)
        	roll += roll_offset;
	else
	        goal_y += goal_y_offset;

	dirty = true;
        break;

      case KEYCODE_UA:
        ROS_DEBUG("UP");
	if(mode == ORIENTATION_MODE)
        	pitch += pitch_offset;

	else
	        goal_x += goal_x_offset;

	dirty = true;
        break;

      case KEYCODE_DA:

        ROS_DEBUG("DOWN");
	if(mode == ORIENTATION_MODE)
       		pitch -= pitch_offset;

	else
	        goal_x -= goal_x_offset;

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
	if(mode == ORIENTATION_MODE)
        	thrust += thrust_offset;

	else
	        goal_z += goal_z_offset;

	dirty = true;
        break;

      case KEYCODE_S:
        ROS_DEBUG("DOWN_S");
	if(mode == ORIENTATION_MODE)
        	thrust -= thrust_offset;
	else
	        goal_z -= goal_z_offset;

	dirty = true;
        break;

    }
   
    geometry_msgs::Twist twist;
    std_msgs::Int32 state_msg;

    twist.angular.y = yawrate;

    if(mode == ORIENTATION_MODE)
    {
	    twist.linear.z = thrust;
	    twist.linear.x = pitch;
	    twist.linear.y = roll;
    }

    else
    {
	    twist.linear.z = goal_z;
	    twist.linear.x = goal_x;
	    twist.linear.y = goal_y;

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

      dirty=false;
    
    }

      if(mode == ORIENTATION_MODE)
      	vel_pub_.publish(twist); 

      else
     {
        cmd_pub_.publish(twist); 
	state_msg.data = state;
	ROS_INFO("%d",state_msg.data);
        state_pub_.publish(state_msg);
     }

  }
  return;
}

