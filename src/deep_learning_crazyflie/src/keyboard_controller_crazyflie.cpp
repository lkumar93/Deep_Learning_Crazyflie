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
#define KEYCODE_O 0x6F
#define KEYCODE_I 0x69
#define KEYCODE_C 0x63

#define POSITION_MODE 0
#define ORIENTATION_MODE 1
#define PID_TUNING_MODE 2

#define WAITING 0
#define TAKE_OFF 1
#define POS_CTRL 2
#define LAND 3
#define EMERGENCY 4
#define Z_TUNE 5
#define Y_TUNE 6
#define X_TUNE 7
#define CALIBRATE 8

#define KP_Z_INIT 5000.0
#define KI_Z_INIT 3500.0
#define KD_Z_INIT 6000.0

#define KP_Z_OFFSET 100.0
#define KI_Z_OFFSET 100.0
#define KD_Z_OFFSET 100.0

#define KP_X_INIT 40.0
#define KI_X_INIT 2.0
#define KD_X_INIT 20.0

#define KP_Y_INIT 40.0
#define KI_Y_INIT 2.0
#define KD_Y_INIT 20.0

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
 
 private:
   
   ros::NodeHandle nh_;
   double roll, pitch, yawrate, thrust, goal_x, goal_y, goal_z, kp, ki, kd;
   int mode, state, tune_param, prev_tune_param;
   ros::Publisher vel_pub_, cmd_pub_, state_pub_;

   double kp_z_prev, ki_z_prev, kd_z_prev;
   double kp_y_prev, ki_y_prev, kd_y_prev;
   double kp_x_prev, ki_x_prev, kd_x_prev;

   
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
  goal_z(0.2),
  kp(KP_Z_INIT),
  kd(KD_Z_INIT),
  ki(KI_Z_INIT),
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
  double kp_offset, kd_offset, ki_offset;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the crazyflie.");

  puts("P - Position Mode");
  puts("O - Orientation Mode");
  puts("I - PID Tuning Mode");
  puts("C - Calibrate");

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


    switch(c)
    {
      case KEYCODE_T:
        ROS_INFO("TAKING OFF");
	if(mode == POSITION_MODE || mode == PID_TUNING_MODE)
		state = TAKE_OFF;
	//else
	//	ROS_INFO("To Take Off, Enable Position Mode");
	dirty = true;
        break;

      case KEYCODE_C:
        ROS_INFO("CALIBRATE");
	state = CALIBRATE;	
	dirty = true;
        break;


      case KEYCODE_L:
        ROS_INFO("LAND");
	if(mode == POSITION_MODE || mode == PID_TUNING_MODE)
		state = LAND;
	//else
	//	ROS_INFO("To Land, Enable Position Mode");
	dirty = true;
        break;

      case KEYCODE_P:
        ROS_INFO("POSITION MODE");
        mode = POSITION_MODE;
	state = WAITING;
	dirty = true;
        break;

      case KEYCODE_O:
        ROS_INFO("ORIENTATION MODE");
	state = WAITING;
        mode = ORIENTATION_MODE;
	dirty = true;
        break;

      case KEYCODE_I:
        ROS_INFO("PID TUNING MODE");
        mode = PID_TUNING_MODE;
	state = tune_param;
	goal_x = 0.0;
	goal_y = 0.0;
	goal_z = 0.2;
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
	else if(mode == POSITION_MODE)
	        goal_y -= goal_y_offset;		
	else 
	        kd -= kd_offset;

	dirty = true;
        break;

      case KEYCODE_RA:
        ROS_DEBUG("RIGHT");
	if(mode == ORIENTATION_MODE)
        	roll += roll_offset;
	else if(mode == POSITION_MODE)
	        goal_y += goal_y_offset;
	else 
	        kd += kd_offset;

	dirty = true;
        break;

      case KEYCODE_UA:
        ROS_DEBUG("UP");
	if(mode == ORIENTATION_MODE)
        	pitch += pitch_offset;

	else if(mode == POSITION_MODE)
	        goal_x += goal_x_offset;

	else 
	        kp += kp_offset;

	dirty = true;
        break;

      case KEYCODE_DA:

        ROS_DEBUG("DOWN");
	if(mode == ORIENTATION_MODE)
       		pitch -= pitch_offset;

	else if(mode == POSITION_MODE)
	        goal_x -= goal_x_offset;

	else 
	        kp -= kp_offset;

	dirty = true;
        break;

      case KEYCODE_A:
        ROS_DEBUG("LEFT_A");
        yawrate -= yawrate_offset;
	if(mode == PID_TUNING_MODE)
	{
		if(tune_param == Z_TUNE)
	        	tune_param = Z_TUNE;
		else
			tune_param=  tune_param-1;

		state = tune_param;

	}
        break;

	dirty = true;
      case KEYCODE_D:
        ROS_DEBUG("RIGHT_D");
        yawrate += yawrate_offset;
	if(mode == PID_TUNING_MODE)
	{
		if(tune_param == X_TUNE)
	        	tune_param = X_TUNE;
		else
			tune_param = tune_param + 1;

		state = tune_param;
	
	}
	dirty = true;
        break;

      case KEYCODE_W:
        ROS_DEBUG("UP_W");
	if(mode == ORIENTATION_MODE)
        	thrust += thrust_offset;

	else if(mode == POSITION_MODE)
	        goal_z += goal_z_offset;

	else 
	        ki += ki_offset;

	dirty = true;
        break;

      case KEYCODE_S:
        ROS_DEBUG("DOWN_S");
	if(mode == ORIENTATION_MODE)
        	thrust -= thrust_offset;
	else if(mode == POSITION_MODE)
	        goal_z -= goal_z_offset;
	else 
	        ki -= ki_offset;

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

	    if(mode == PID_TUNING_MODE)
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

		        twist.angular.x = kp;
		        twist.angular.y = ki;
		        twist.angular.z = kd;

    			prev_tune_param = tune_param;
		}


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


      state_msg.data = state;
      state_pub_.publish(state_msg);

      if(state!= CALIBRATE)
      {
	      if(mode == ORIENTATION_MODE)
	      {
	      	vel_pub_.publish(twist); 
	      }

	      else
	     {
		cmd_pub_.publish(twist); 
	     }

	   if(mode == POSITION_MODE)
		ROS_INFO("X - %f, Y - %f ,Z - %f,yawrate - %f", goal_x ,goal_y, goal_z,yawrate);

	   else if(mode == ORIENTATION_MODE)
	   	ROS_INFO("Thrust - %f, roll - %f ,pitch - %f,yawrate - %f",thrust,roll,pitch,yawrate);

	   else
		ROS_INFO("Tune Param = %d, Kp - %f, Ki - %f , Kd - %f",tune_param,kp,ki,kd);
     }

  }
  return;
}

