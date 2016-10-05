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

ros::Publisher imu_pub;
ros::Publisher accel_pub;

void callback(const Vector3Stamped::ConstPtr& Gyromsg, const Vector3Stamped::ConstPtr& Accelmsg, const Vector3Stamped::ConstPtr& Orientationmsg )
{
  float roll = Orientationmsg->vector.y;
  float pitch = Orientationmsg->vector.x;
  float yaw = Orientationmsg->vector.z;

  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

  Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle ;	

  sensor_msgs::Imu msg;

  msg.header.frame_id = Gyromsg->header.frame_id;
  msg.header.stamp = Gyromsg->header.stamp;
  
  msg.orientation.x = q.x() ;
  msg.orientation.y = q.y() ;
  msg.orientation.z = q.z() ;
  msg.orientation.w = q.w() ;

  msg.angular_velocity.x = Gyromsg->vector.x;
  msg.angular_velocity.y = Gyromsg->vector.y;
  msg.angular_velocity.z = Gyromsg->vector.z;

  msg.linear_acceleration.x = Accelmsg->vector.x;
  msg.linear_acceleration.y = Accelmsg->vector.y;
  msg.linear_acceleration.z = Accelmsg->vector.z;

  imu_pub.publish(msg);

  Eigen::Vector3d GravityVector;

  GravityVector[0] = 0.0;
  GravityVector[1] = 0.0;
  GravityVector[2] = 9.81; 

  Eigen::Vector3d AccelerationVector, AccelerationVector_GravityCompensated;

  AccelerationVector[0] = msg.linear_acceleration.x;
  AccelerationVector[1] = msg.linear_acceleration.y;
  AccelerationVector[2] = msg.linear_acceleration.z;

  Eigen::Affine3d transformation_matrix = Eigen::Affine3d::Identity();

  transformation_matrix.rotate(q);

  AccelerationVector_GravityCompensated = (transformation_matrix.rotation()*AccelerationVector) - GravityVector;

  geometry_msgs::Vector3Stamped msg2;

  msg2.header.frame_id = Gyromsg->header.frame_id;
  msg2.header.stamp = Gyromsg->header.stamp;

  msg2.vector.x = AccelerationVector_GravityCompensated[0];
  msg2.vector.y = AccelerationVector_GravityCompensated[1];
  msg2.vector.z = AccelerationVector_GravityCompensated[2];

  accel_pub.publish(msg2);
   
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_imu_node");
  
  ros::NodeHandle n;

  imu_pub = n.advertise<sensor_msgs::Imu>("crazyflie/imu", 10);
  accel_pub = n.advertise<sensor_msgs::Imu>("crazyflie/accel/gravity_compensated", 10);

  message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyro_sub(n, "crazyflie/gyro", 1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> accel_sub(n, "crazyflie/accel", 1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> orientation_sub(n, "crazyflie/orientation", 1);
 
  typedef sync_policies::ApproximateTime< geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
  
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gyro_sub, accel_sub, orientation_sub );
  
  sync.registerCallback(boost::bind(&callback, _1,_2,_3)); 
  
  ros::spin();
   
  return 0;
}
