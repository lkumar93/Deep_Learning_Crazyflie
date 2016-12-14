//
// THIS IS AN IMPLEMENTATION OF 2D POSITION CALCULATION USING OPTICAL FLOW 
// TECHNIQUE BASED ON PX4's PX4FLOW/SNAPCAM AND KLT FEATURE TRACKER
//
// COPYRIGHT BELONGS TO THE AUTHOR OF THIS CODE AND PX4
//
// AUTHOR : LAKSHMAN KUMAR
// AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
// EMAIL : LKUMAR93@UMD.EDU
// LINKEDIN : WWW.LINKEDIN.COM/IN/LAKSHMANKUMAR1993
//
// THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THE MIT LICENSE
// THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF
// THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
// 
// BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
// BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
// CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
// CONDITIONS.
//


        //  lambda enabled: self._cf.param.set_value("flightmode.althold",
//enabled))


///////////////////////////////////////////
//
//	LIBRARIES
//
///////////////////////////////////////////

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <typeinfo>
#include <time.h>
#include <opencv2/highgui/highgui.hpp>
#include "trackFeatures.h"
#include <string>
#include <iostream>
#include <deep_learning_crazyflie/camera_parameters.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>

using namespace sensor_msgs;
using namespace std;
using namespace std_msgs;

#define NumberOfFeaturesToUse 100
#define GyroCompensationThreshold 0.1
#define PI 3.14159265358979323846
#define DEG2RAD ((PI)/(180.0))
#define PARAM_BOTTOM_FLOW_WEIGHT_NEW 0.3

class OpticalFlowComputer
{
  string image_topic;
  string imu_topic = "crazyflie/imu";
  string barometer_topic = "crazyflie/height/filtered"; 
  string image_prefix = "crazyflie/cameras/";
  string image_postfix = "/image" ;
  string topic_name;
  double Image_Timestamp_Prev = 0.0;
  double ImuPacket_Timestamp_Prev = 0.0;
  std::vector<cv::Point2f> features_current;
  std::vector<int> updateVector;
  cv::Mat_<cv::Point2f> out_features_previous;
  double focal_length_x;
  double focal_length_y;
  double principal_point_y;
  double principal_point_x;
  double distortion_params[5]; 
  cv::Mat_<float> cam_matrix;
  cv::Mat_<float> distortion;
  double position_x = 0.0;
  double position_y = 0.0;
  double position_x_Prev = 0.0;
  double position_y_Prev = 0.0;

  float velocity_x_lp = 0.0f;
  float velocity_y_lp = 0.0f;

  double gyro_pixel_x = 0.0;
  double gyro_pixel_y = 0.0;

  double height = 0.0;

  double ChangeInTime = 0.0;
int linear_acceleration_z_acc_count = 0;
double linear_acceleration_z_acc = 0.0;
 
  ros::Subscriber ImageSubscriber ; 
  ros::Subscriber ImuSubscriber ; 
  ros::Subscriber BarometerSubscriber ; 

  double InitialPressure = 0.0; // Average sealevel pressure in mbar

 double InitialAcceleration = 0.0;
  int PressureAveragingCount = 0;
  int PressureAveragingThreshold = 100;

double vz_est = 0.0;
double hz_est = 0.0;

ros::Publisher m_pubHeight;

  public :

  OpticalFlowComputer(string camera_position, int NumberOfFeatures,  double FL_x,double FL_y, double PP_x,double PP_y,double* DistArray)
	:cam_matrix(3,3), distortion(1,5),m_pubHeight()
  {
	ros::NodeHandle nh;

	updateVector.resize(NumberOfFeatures, 2);

	focal_length_x = FL_x;
	focal_length_y = FL_y;
	principal_point_x = PP_x;
	principal_point_y = PP_y;

	distortion << DistArray[0],DistArray[1],DistArray[2],DistArray[3],DistArray[4];	
	cam_matrix << FL_x, 0, PP_x, 0, FL_y, PP_y, 0, 0, 1 ;

	image_topic = image_prefix+camera_position+image_postfix;
	ImageSubscriber = nh.subscribe(image_topic , 1, &OpticalFlowComputer::CalcOpticalFlow, this);
	ImuSubscriber = nh.subscribe(imu_topic, 1, &OpticalFlowComputer::CalcAngularDisplacement, this);
	BarometerSubscriber = nh.subscribe(barometer_topic, 1, &OpticalFlowComputer::CalcHeight, this);
	m_pubHeight = nh.advertise<std_msgs::Float32>("crazyflie/heightfiltered",1);
	
  }


  void CalcAngularDisplacement(const sensor_msgs::ImuConstPtr& ImuPacket)
  {

     double ImuPacket_Timestamp = ImuPacket->header.stamp.toSec();
     double gyro_x = ImuPacket->angular_velocity.x;
     double gyro_y = ImuPacket->angular_velocity.y;
     double linear_acceleration_z = ImuPacket->linear_acceleration.z - InitialAcceleration;

     if (ImuPacket_Timestamp_Prev == 0.0)
     {	
	ImuPacket_Timestamp_Prev = ImuPacket_Timestamp ;
	InitialAcceleration = linear_acceleration_z;
	ROS_INFO("Gyros Initialized");
	return;
     } 

     double dt = ImuPacket_Timestamp - ImuPacket_Timestamp_Prev ;

     if (abs(gyro_x) > GyroCompensationThreshold)
     {
     	gyro_pixel_x += gyro_x*dt*focal_length_x;

     }
     else
     {
	gyro_pixel_x = 0;
      }

     if (abs(gyro_y) > GyroCompensationThreshold)
     {
	gyro_pixel_y += gyro_y*dt*focal_length_y;
     
     }
     else
     {
	gyro_pixel_y = 0;
      }


     if (abs(linear_acceleration_z) > 0.05)
     {
     	linear_acceleration_z_acc += linear_acceleration_z*dt;
        linear_acceleration_z_acc_count++;
        ChangeInTime += dt;

     }
    
  }



  void CalcHeight(const geometry_msgs::Vector3StampedConstPtr& PressurePacket)
  {

/*
	if(PressureAveragingCount < PressureAveragingThreshold )
	{
		InitialPressure += PressurePacket->vector.z;

		PressureAveragingCount++;

		if (PressureAveragingCount == PressureAveragingThreshold)
		{
			InitialPressure = InitialPressure/PressureAveragingThreshold;
			cout<<" Initial Pressure is " << InitialPressure;
			
		}
				
		return;
	}

	//float coef = 1.0f / 5.255f;
	//double Pressure = PressurePacket->data;
	//height = 44330.0f * (1.0f - (float)pow((Pressure/InitialPressure), coef));

        height = PressurePacket->vector.z - InitialPressure;

        if(linear_acceleration_z_acc_count > 0)
	{
		vz_est += (linear_acceleration_z_acc/linear_acceleration_z_acc_count)*ChangeInTime;
		hz_est +=  vz_est*ChangeInTime;


		float k_vz = -0.15;
		float k_h_est= -0.92;

		vz_est += k_vz*(hz_est -height);
		hz_est += k_h_est*(hz_est- height);

		std_msgs::Float32 msg;
		msg.data = hz_est;

		m_pubHeight.publish(msg);

	     	linear_acceleration_z_acc = 0.0;
		linear_acceleration_z_acc_count = 0;
		ChangeInTime = 0.0;
		

	
        }

 	//ROS_INFO("Height is %f ", height);

*/

	height = PressurePacket->vector.z;
   }


  void CalcOpticalFlow(const sensor_msgs::ImageConstPtr& Image )
  {
	cv_bridge::CvImageConstPtr Image_Ptr;

	double Image_Timestamp = Image->header.stamp.toSec();

	if (Image_Timestamp_Prev == 0.0)
	   {	
		Image_Timestamp_Prev = Image_Timestamp;
		ROS_INFO("Initialized Optical Flow");
		return;
	   } 

        try
	    {
	      Image_Ptr = cv_bridge::toCvShare(Image, sensor_msgs::image_encodings::MONO8);
	    }

        catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception in OpticalFlowComputer::CalcOpticalFlow() : %s", e.what());
	      return;
	    }

	 cv::Mat InputImage = Image_Ptr->image ;
  
	// cv::Mat InputImage = cv::Mat(60, 80, CV_8UC3, cv::Scalar(0, 0, 0)) ;

         //resize(Image_Ptr->image,InputImage,InputImage.size(),0,0);
	//cv::Mat InputImage = cv::Mat(OriginalImage.rows, OriginalImage.cols, CV_8UC3, cv::Scalar(0, 0, 0));
	// cvtColor(OriginalImage, InputImage, CV_BGR2GRAY);

	std::vector<cv::Point2f> useless;
	int meancount = 0;

	double pixel_flow_x_mean = 0.0;
	double pixel_flow_y_mean = 0.0;
	double pixel_flow_x_integral = 0.0;
	double pixel_flow_y_integral = 0.0;
	double pixel_flow_x_stddev = 0.0;
	double pixel_flow_y_stddev = 0.0;

	trackFeatures(InputImage, InputImage, features_current, useless, updateVector, 0);

	int npoints = updateVector.size();
	
	cv::Mat_<cv::Point2f> out_features_current(1, npoints);
	
	cv::undistortPoints(features_current, out_features_current, cam_matrix, distortion);

	// cv::undistortPoints returns normalized coordinates... -> convert
	for (int i = 0; i < npoints; i++) {

		out_features_current(i).x = out_features_current(i).x * focal_length_x +  principal_point_x;
		out_features_current(i).y = out_features_current(i).y * focal_length_y +  principal_point_y;

	}

	if (!out_features_current.empty() && !out_features_previous.empty()) {
		// compute the mean flow
		for (int i = 0; i < updateVector.size(); i++) {
			if (updateVector[i] == 1) {
				pixel_flow_x_mean += out_features_current(i).x - out_features_previous(i).x;
				pixel_flow_y_mean += out_features_current(i).y - out_features_previous(i).y;
				meancount++;
			}
		}

		if (meancount) {
			pixel_flow_x_mean /= meancount;
			pixel_flow_y_mean /= meancount;

			// compute the flow variance
			for (int i = 0; i < updateVector.size(); i++) {
				if (updateVector[i] == 1) {
					pixel_flow_x_stddev += (out_features_current(i).x - out_features_previous(i).x - pixel_flow_x_mean) *
							       (out_features_current(i).x - out_features_previous(i).x - pixel_flow_x_mean);
					pixel_flow_y_stddev += (out_features_current(i).y - out_features_previous(i).y - pixel_flow_y_mean) *
							       (out_features_current(i).y - out_features_previous(i).y - pixel_flow_y_mean);
				}
			}

			pixel_flow_x_stddev /= meancount;
			pixel_flow_y_stddev /= meancount;

			// convert to std deviation
			pixel_flow_x_stddev = sqrt(pixel_flow_x_stddev);
			pixel_flow_y_stddev = sqrt(pixel_flow_y_stddev);

			// re-compute the mean flow with only the 95% consenting features
			meancount = 0;

			for (int i = 0; i < updateVector.size(); i++) {
				if (updateVector[i] == 1) {
					double this_flow_x = out_features_current(i).x - out_features_previous(i).x;
					double this_flow_y = out_features_current(i).y - out_features_previous(i).y;

					if (abs(this_flow_x - pixel_flow_x_mean) < 2 * pixel_flow_x_stddev
					    && abs(this_flow_y - pixel_flow_y_mean) < 2 * pixel_flow_y_stddev) {
						pixel_flow_x_integral += out_features_current(i).x - out_features_previous(i).x;
						pixel_flow_y_integral += out_features_current(i).y - out_features_previous(i).y;
						meancount++;

					} else {
						updateVector[i] = 0;
					}
				}

			}

			if (meancount ){//&& height > 0.0) {
				pixel_flow_x_integral /= meancount;
				pixel_flow_y_integral /= meancount;

				double flow_quality = 255.0 * meancount / updateVector.size();
				double delta_time = Image_Timestamp - Image_Timestamp_Prev;

				double flow_x_ang = atan2(pixel_flow_x_integral,focal_length_x);
				double flow_y_ang = atan2(pixel_flow_y_integral,focal_length_y);

				//position_x = position_x + height*(flow_x_ang - (gyro_pixel_y/focal_length_y));
				//position_y = position_y + height*(flow_y_ang + (gyro_pixel_x/focal_length_x));

				double comp_flow_x = (pixel_flow_x_integral/focal_length_x + gyro_pixel_y/(focal_length_y*500)) ;
				double comp_flow_y = (pixel_flow_y_integral/focal_length_y - gyro_pixel_x/(focal_length_x*500));

				position_x = position_x + comp_flow_x;//* delta_time;
				position_y = position_y + comp_flow_y;// * delta_time;

			
				ROS_INFO("x value =  %f ,y value = %f  , height = %f, gyro = %f & %f ", position_x,position_y, height, (gyro_pixel_y/focal_length_y),(gyro_pixel_x/focal_length_x));

        			gyro_pixel_x = 0.0;
  				gyro_pixel_y = 0.0;
			}

		} else {

			velocity_x_lp = (1.0f - PARAM_BOTTOM_FLOW_WEIGHT_NEW) * velocity_x_lp;
			velocity_y_lp = (1.0f - PARAM_BOTTOM_FLOW_WEIGHT_NEW) * velocity_y_lp;
			ROS_ERROR("No valid measurements");
		}
	}

	for (int i = 0; i < updateVector.size(); i++) {
		if (updateVector[i] == 2) {
			updateVector[i] = 1;
		}

		if (updateVector[i] == 0) {
			updateVector[i] = 2;
		}
	}

	out_features_previous = out_features_current;
	Image_Timestamp_Prev = Image_Timestamp ;
	position_x_Prev = position_x;
	position_y_Prev = position_y;


  }
 
};

int main( int argc, char** argv )
{    
    //Initialize the Crazyflie Camera Publisher Node
    ros::init(argc, argv, "crazyflie_opticalflow_node");
   // ros::NodeHandle nh;
    OpticalFlowComputer BottomCameraOptFlow("bottom", NumberOfFeaturesToUse, FocalLength_X, FocalLength_Y, PrincipalPoint_X, PrincipalPoint_Y, Distortion);
    ros::spin();
    return 0;
}
