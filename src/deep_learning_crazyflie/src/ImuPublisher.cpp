#include "ros/ros.h"
// #include "std_msgs/UInt16.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "std_msgs/UInt8.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <std_msgs/String.h>

#define GRAVITY 9.80665

using namespace message_filters;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

ros::Publisher imu_pub;
ros::Publisher accel_comp_pub;
ros::Publisher filtered_height_pub;

double InitialHeight = 0.0; // Average sealevel pressure in mbar
double InitialAcceleration = 0.0;
double Accelmsg_Timestamp_Prev = 0.0;
int HeightAveragingCount = 0;
int HeightAveragingThreshold = 100;
double vz_est = 0.0;
double hz_est = 0.0;
double height = 0.0;

String frame_id;
double acc_x,acc_y,acc_z;
double acc_x_comp,acc_y_comp,acc_z_comp;
double gyro_x,gyro_y,gyro_z;
double qx,qy,qz,qw;
double grav_x,grav_y,grav_z;
double base_z_acc = 0.0;


sensor_msgs::Imu imu_msg;
geometry_msgs::Vector3Stamped acc_comp_msg;
geometry_msgs::Vector3Stamped filtered_height_msg;

bool is_init = false;


void imu_callback(const Vector3Stamped::ConstPtr& Gyromsg, const Vector3Stamped::ConstPtr& Accelmsg, const QuaternionStamped::ConstPtr& Orientationmsg )
{

  acc_x = Accelmsg->vector.x;
  acc_y = Accelmsg->vector.y;
  acc_z = Accelmsg->vector.z ;

  gyro_x = Gyromsg->vector.x ;
  gyro_y = Gyromsg->vector.y ;
  gyro_z = Gyromsg->vector.z ;
  
  qx = Orientationmsg->quaternion.x;
  qy = Orientationmsg->quaternion.y;
  qz = Orientationmsg->quaternion.z;
  qw = Orientationmsg->quaternion.w;

  frame_id.data = Gyromsg->header.frame_id;
  imu_msg.header.frame_id = frame_id.data;
  imu_msg.header.stamp =  ros::Time::now();
  
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.orientation.w = qw;

  imu_msg.angular_velocity.x = gyro_x ;
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = gyro_z;

  imu_msg.linear_acceleration.x = acc_x;
  imu_msg.linear_acceleration.y = acc_y;
  imu_msg.linear_acceleration.z = acc_z;

  imu_pub.publish(imu_msg);

//  double gravX = 2*(qx*qz - qw*qy);
//  double gravY = 2*(qw*qx - qy*qz);
//  double gravZ = qw*qw-qx*qx-qy*qy+qz*qz;

//  double acc_z_comp = (acc_x * gravX + acc_y * gravY + acc_z * gravZ) -0.93337;

//  Eigen::Vector3d GravityVector;

//  GravityVector[0] = 0.0;
//  GravityVector[1] = 0.0;
//  GravityVector[2] = 9.81; 

//  Eigen::Vector3d AccelerationVector, AccelerationVector_GravityCompensated;

//  AccelerationVector[0] = Accelmsg->vector.x;
//  AccelerationVector[1] = Accelmsg->vector.y;
//  AccelerationVector[2] = Accelmsg->vector.z;

//  Eigen::Affine3d transformation_matrix = Eigen::Affine3d::Identity();

//  transformation_matrix.rotate(q);

//  AccelerationVector_GravityCompensated = (transformation_matrix.rotation()*AccelerationVector) - GravityVector;

//  geometry_msgs::Vector3Stamped msg2;

//  msg2.header.frame_id = Gyromsg->header.frame_id;
//  msg2.header.stamp = Gyromsg->header.stamp;

//  msg2.vector.x = AccelerationVector_GravityCompensated[0];
//  msg2.vector.y = AccelerationVector_GravityCompensated[1];
//  msg2.vector.z = acc_z_comp;

//  accel_pub.publish(msg2);

//    if(HeightAveragingCount < HeightAveragingThreshold )
//	{
//		InitialHeight += Heightmsg->vector.z;

//		HeightAveragingCount++;

//		if (HeightAveragingCount == HeightAveragingThreshold)
//		{
//			InitialHeight = InitialHeight/HeightAveragingThreshold;
//			Accelmsg_Timestamp_Prev = Accelmsg->header.stamp.toSec() ;
//			
//		}
//				
//		return;
//	}

//	double dt = Accelmsg->header.stamp.toSec() - Accelmsg_Timestamp_Prev ;

// 	height = Heightmsg->vector.z - InitialHeight;

//	vz_est += (AccelerationVector_GravityCompensated[2])*dt;
//	hz_est +=  vz_est*dt;

//	float k_vz = -0.15;
//	float k_h_est= -0.92;

//	vz_est += k_vz*(hz_est -height);
//	hz_est += k_h_est*(hz_est- height);

//	geometry_msgs::Vector3Stamped msg3;

// 	msg3.header.frame_id = Gyromsg->header.frame_id;
//  	msg3.header.stamp = Gyromsg->header.stamp;

//        msg3.vector.z =height ;

//	filtered_height_pub.publish(msg3);

//	Accelmsg_Timestamp_Prev = Accelmsg->header.stamp.toSec() ;

//	ROS_INFO("Height is %f and Acceleration is %f ", hz_est, AccelerationVector_GravityCompensated[2]);
   
}

void gravity_callback(const Vector3Stamped::ConstPtr& Gravmsg)
{
  //Convert units in g to m/s^2

  grav_x = GRAVITY*Gravmsg->vector.x;
  grav_y = GRAVITY*Gravmsg->vector.y;
  grav_z = GRAVITY*Gravmsg->vector.z;

  acc_x_comp = acc_x - grav_x;
  acc_y_comp = acc_y - grav_y;
  acc_z_comp = acc_z - grav_z;

  acc_comp_msg.header.frame_id = frame_id.data;
  acc_comp_msg.header.stamp = ros::Time::now();
  acc_comp_msg.vector.x = acc_x_comp;
  acc_comp_msg.vector.y = acc_y_comp;
  acc_comp_msg.vector.z = acc_z_comp;

  accel_comp_pub.publish(acc_comp_msg);
  
}

void isinit_callback(const UInt8::ConstPtr& Isinitmsg)
{
  if(Isinitmsg->data == 1)
  {
	is_init = true;
  }
}

void height_callback(const Vector3Stamped::ConstPtr& Heightmsg)
{
   if(HeightAveragingCount < HeightAveragingThreshold )
	{
		InitialHeight += Heightmsg->vector.z;

		HeightAveragingCount++;

		if (HeightAveragingCount == HeightAveragingThreshold)
		{
			InitialHeight = InitialHeight/HeightAveragingThreshold;
		}
				
		return;
	}

   height = Heightmsg->vector.z - InitialHeight;

   filtered_height_msg.header.frame_id = frame_id.data;
   filtered_height_msg.header.stamp = ros::Time::now();
   filtered_height_msg.vector.z = height;

   filtered_height_pub.publish(filtered_height_msg);
}

void accbasez_callback(const Vector3Stamped::ConstPtr& accelmsg)
{
  base_z_acc = accelmsg->vector.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_imu_node");
  
  ros::NodeHandle n;

  imu_pub = n.advertise<sensor_msgs::Imu>("crazyflie/imu", 1);
  accel_comp_pub = n.advertise<geometry_msgs::Vector3Stamped>("crazyflie/accel/gravity_compensated", 1);
  filtered_height_pub = n.advertise<geometry_msgs::Vector3Stamped>("crazyflie/height/filtered", 1);

  ros::Subscriber gravity_direction_sub = n.subscribe("crazyflie/gravity_direction",1, gravity_callback);
  ros::Subscriber is_init_sub = n.subscribe("crazyflie/sensors/initialized",1, isinit_callback);
  ros::Subscriber accel_base_z_sub = n.subscribe("crazyflie/accel/base_z",1, accbasez_callback);
  ros::Subscriber filtered_height_sub = n.subscribe("crazyflie/height",1, height_callback);

  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyro_sub(n, "crazyflie/gyro", 1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub(n, "crazyflie/accel", 1);
  message_filters::Subscriber<geometry_msgs::QuaternionStamped> orientation_sub(n, "crazyflie/orientation/quaternion", 1);
 
  typedef sync_policies::ApproximateTime< geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, geometry_msgs::QuaternionStamped> MySyncPolicy;
  
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gyro_sub, accel_sub, orientation_sub);
  
  sync.registerCallback(boost::bind(&imu_callback, _1, _2, _3)); 
  
  ros::spin();
   
  return 0;
}
