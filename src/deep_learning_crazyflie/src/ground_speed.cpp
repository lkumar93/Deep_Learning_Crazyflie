#include "ros/ros.h"
// #include "std_msgs/UInt16.h"
#include "sensor_msgs/Range.h"
#include "mavros_msgs/OpticalFlowRad.h"
#include "sensor_msgs/Imu.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;
using namespace sensor_msgs;
using namespace mavros_msgs;

float gyro_x = 0.0;
float gyro_y = 0.0;

float acc_x = 0.0;
float acc_y = 0.0;

float vel_x = 0.0;
float vel_y = 0.0;

float timer = 0.0;

float dt  = 0.0;

float opt_flow_x = 0.0;
float opt_flow_y = 0.0;

float ground_speed_x = 0.0;
float ground_speed_y = 0.0;

float imu_speed_x = 0.0;
float imu_speed_y = 0.0;

float ground_speed_x_ratio = 0.0;
float ground_speed_y_ratio = 0.0;

ros::Publisher groundspeed_pub;

void callback(const Imu::ConstPtr& Imumsg, const OpticalFlowRad::ConstPtr& Opticalflowmsg)
{
   
   mavros_msgs::OpticalFlowRad m;

   gyro_x = Imumsg->angular_velocity.x;
   gyro_y = Imumsg->angular_velocity.y;

   acc_x = Imumsg->linear_acceleration.x;
   acc_y = Imumsg->linear_acceleration.y;

   opt_flow_x = Opticalflowmsg->integrated_x;
   opt_flow_y = Opticalflowmsg->integrated_y;

   dt = (Opticalflowmsg->integration_time_us)/1000000; 

 
   if(abs(gyro_y*dt*379.05) > 1)	
   	ground_speed_x = ((opt_flow_x) + (gyro_y*dt*379.05));
   else
	ground_speed_x = opt_flow_x ; 
 
   if(abs(gyro_x*dt*366.6) > 1)
   	ground_speed_y = (opt_flow_y - (gyro_x*dt*366.6));
   else
	ground_speed_y = opt_flow_y ;

   imu_speed_x = vel_y + gyro_y;
   imu_speed_y = vel_x + gyro_x;

   ground_speed_x_ratio = opt_flow_x/gyro_y ;
   ground_speed_y_ratio = opt_flow_y/gyro_x ;
   
   ROS_INFO("opt_flow_x is %f", opt_flow_x);
   ROS_INFO("opt_flow_y is %f", opt_flow_y);

    ROS_INFO("gyro_x is %f", gyro_x);
   ROS_INFO("gyro_y is %f", gyro_y);	

   ROS_INFO("vel_x is %f", vel_x);
   ROS_INFO("vel_y is %f", vel_y);	

   ROS_INFO("imu_speed_x is %f", imu_speed_x);
   ROS_INFO("imu_speed_y is %f", imu_speed_y);

   //ROS_INFO("compass angle is %f", compass_angle);
  
   m.integrated_x = ground_speed_x;
   m.integrated_y = ground_speed_y;

   m.integrated_xgyro = gyro_y*131;
   m.integrated_ygyro = gyro_x*131;

   m.integrated_zgyro = opt_flow_x;
   m.distance = opt_flow_y;
   
   m.header.stamp=ros::Time::now();
   
   groundspeed_pub.publish(m);
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "groundspeed_node");
  
  ros::NodeHandle n;

  groundspeed_pub = n.advertise<mavros_msgs::OpticalFlowRad>("ground_speed", 10);

  message_filters::Subscriber<mavros_msgs::OpticalFlowRad> OpticalFlow_sub(n, "OpticalFlowXY", 1);
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(n, "crazyflie/imu", 1);
 
  typedef sync_policies::ApproximateTime< sensor_msgs::Imu, mavros_msgs::OpticalFlowRad> MySyncPolicy;
  
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, OpticalFlow_sub);
  
  sync.registerCallback(boost::bind(&callback, _1,_2)); 
  
  ros::spin();
   
  return 0;
}
