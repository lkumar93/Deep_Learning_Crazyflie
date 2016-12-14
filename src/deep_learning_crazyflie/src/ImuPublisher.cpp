#include "ros/ros.h"
// #include "std_msgs/UInt16.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace message_filters;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

ros::Publisher imu_pub;
ros::Publisher accel_pub;
ros::Publisher filtered_height_pub;


double InitialHeight = 0.0; // Average sealevel pressure in mbar
double InitialAcceleration = 0.0;
double Accelmsg_Timestamp_Prev = 0.0;
int HeightAveragingCount = 0;
int HeightAveragingThreshold = 100;
double vz_est = 0.0;
double hz_est = 0.0;
double height = 0.0;

void callback(const Vector3Stamped::ConstPtr& Gyromsg, const Vector3Stamped::ConstPtr& Accelmsg, const Vector3Stamped::ConstPtr& Orientationmsg, const Vector3Stamped::ConstPtr& Heightmsg )
{

  float roll = Orientationmsg->vector.y;
  float pitch = Orientationmsg->vector.x;
  float yaw = Orientationmsg->vector.z;

  double acc_x = Accelmsg->vector.x;
  double acc_y = Accelmsg->vector.y;
  double acc_z = Accelmsg->vector.z ;

  double gyro_x = Gyromsg->vector.x ;
  double gyro_y = Gyromsg->vector.y ;
  double gyro_z = Gyromsg->vector.z ;
  

  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

  Eigen::Quaterniond q(pitchAngle*rollAngle*yawAngle ) ;

  double qx = q.x();
  double qy = q.y();
  double qz = q.z();
  double qw = q.w();

  sensor_msgs::Imu msg;

  msg.header.frame_id = Gyromsg->header.frame_id;
  msg.header.stamp = Gyromsg->header.stamp;
  
  msg.orientation.x = qx;
  msg.orientation.y = qy;
  msg.orientation.z = qz;
  msg.orientation.w = qw;

  msg.angular_velocity.x = gyro_x ;
  msg.angular_velocity.y = gyro_y;
  msg.angular_velocity.z = gyro_z;

  msg.linear_acceleration.x = acc_x;
  msg.linear_acceleration.y = acc_y;
  msg.linear_acceleration.z = acc_z;

  double gravX = 2*(qx*qz - qw*qy);
  double gravY = 2*(qw*qx - qy*qz);
  double gravZ = qw*qw-qx*qx-qy*qy+qz*qz;

  double acc_z_comp = (acc_x * gravX + acc_y * gravY + acc_z * gravZ) -0.93337;


  imu_pub.publish(msg);

  Eigen::Vector3d GravityVector;

  GravityVector[0] = 0.0;
  GravityVector[1] = 0.0;
  GravityVector[2] = 9.81; 

  Eigen::Vector3d AccelerationVector, AccelerationVector_GravityCompensated;

  AccelerationVector[0] = Accelmsg->vector.x;
  AccelerationVector[1] = Accelmsg->vector.y;
  AccelerationVector[2] = Accelmsg->vector.z;

  Eigen::Affine3d transformation_matrix = Eigen::Affine3d::Identity();

  transformation_matrix.rotate(q);

  AccelerationVector_GravityCompensated = (transformation_matrix.rotation()*AccelerationVector) - GravityVector;

  geometry_msgs::Vector3Stamped msg2;

  msg2.header.frame_id = Gyromsg->header.frame_id;
  msg2.header.stamp = Gyromsg->header.stamp;

  msg2.vector.x = AccelerationVector_GravityCompensated[0];
  msg2.vector.y = AccelerationVector_GravityCompensated[1];
  msg2.vector.z = acc_z_comp;

  accel_pub.publish(msg2);

    if(HeightAveragingCount < HeightAveragingThreshold )
	{
		InitialHeight += Heightmsg->vector.z;

		HeightAveragingCount++;

		if (HeightAveragingCount == HeightAveragingThreshold)
		{
			InitialHeight = InitialHeight/HeightAveragingThreshold;
			Accelmsg_Timestamp_Prev = Accelmsg->header.stamp.toSec() ;
			
		}
				
		return;
	}

	double dt = Accelmsg->header.stamp.toSec() - Accelmsg_Timestamp_Prev ;

 	height = Heightmsg->vector.z - InitialHeight;

	vz_est += (AccelerationVector_GravityCompensated[2])*dt;
	hz_est +=  vz_est*dt;

	float k_vz = -0.15;
	float k_h_est= -0.92;

	vz_est += k_vz*(hz_est -height);
	hz_est += k_h_est*(hz_est- height);

	geometry_msgs::Vector3Stamped msg3;

 	msg3.header.frame_id = Gyromsg->header.frame_id;
  	msg3.header.stamp = Gyromsg->header.stamp;

        msg3.vector.z =height ;

	filtered_height_pub.publish(msg3);

	Accelmsg_Timestamp_Prev = Accelmsg->header.stamp.toSec() ;

	ROS_INFO("Height is %f and Acceleration is %f ", hz_est, AccelerationVector_GravityCompensated[2]);
   
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_imu_node");
  
  ros::NodeHandle n;

  imu_pub = n.advertise<sensor_msgs::Imu>("crazyflie/imu", 1);
  accel_pub = n.advertise<geometry_msgs::Vector3Stamped>("crazyflie/accel/gravity_compensated", 1);
  filtered_height_pub = n.advertise<geometry_msgs::Vector3Stamped>("crazyflie/height/filtered", 1);

  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyro_sub(n, "crazyflie/gyro", 1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub(n, "crazyflie/accel", 1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> orientation_sub(n, "crazyflie/orientation", 1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> height_sub(n, "crazyflie/height", 1);
 
  typedef sync_policies::ApproximateTime< geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
  
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gyro_sub, accel_sub, orientation_sub, height_sub);
  
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4)); 
  
  ros::spin();
   
  return 0;
}
