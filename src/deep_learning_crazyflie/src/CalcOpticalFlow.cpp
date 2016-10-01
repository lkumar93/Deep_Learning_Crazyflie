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

using namespace sensor_msgs;
using namespace std;

#define NumberOfFeaturesToUse 100
#define GyroCompensationThreshold 0.01

class OpticalFlowComputer
{
  string image_topic;
  string imu_topic = "crazyflie/imu";
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

  double gyro_pixel_x = 0.0;
  double gyro_pixel_y = 0.0;
  
  ros::Subscriber ImageSubscriber ; 
  ros::Subscriber ImuSubscriber ; 

  public :

  OpticalFlowComputer(string camera_position, int NumberOfFeatures,  double FL_x,double FL_y, double PP_x,double PP_y,double* DistArray)
	:cam_matrix(3,3), distortion(1,5)
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
	ImuSubscriber = nh.subscribe(imu_topic, 1, &OpticalFlowComputer::IntegrateGyros, this);
	
  }


  void IntegrateGyros(const sensor_msgs::ImuConstPtr& ImuPacket)
  {

     double ImuPacket_Timestamp = ImuPacket->header.stamp.toSec();
     double gyro_x = ImuPacket->angular_velocity.x;
     double gyro_y = ImuPacket->angular_velocity.y;

     if (ImuPacket_Timestamp_Prev == 0.0)
     {	
	ImuPacket_Timestamp_Prev = ImuPacket_Timestamp ;
	ROS_INFO("Gyros Initialized");
	return;
     } 

     double dt = ImuPacket_Timestamp - ImuPacket_Timestamp_Prev ;

     if (gyro_x > GyroCompensationThreshold)
     {
     	gyro_pixel_x += gyro_x*dt*focal_length_x;

     }

     if (gyro_y > GyroCompensationThreshold)
     {
	gyro_pixel_y += gyro_y*dt*focal_length_y;
     
     }

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

			if (meancount) {
				pixel_flow_x_integral /= meancount;
				pixel_flow_y_integral /= meancount;

				double flow_quality = 255.0 * meancount / updateVector.size();
				double delta_time = Image_Timestamp - Image_Timestamp_Prev;

				double flow_x_ang = atan2(pixel_flow_x_integral,focal_length_x);
				double flow_y_ang = atan2(pixel_flow_y_integral,focal_length_y);

				position_x = position_x + flow_x_ang + gyro_pixel_x;
				position_y = position_y + flow_y_ang + gyro_pixel_y;

				double rate_x = (position_x - position_x_Prev)/delta_time;
				double rate_y = (position_y - position_y_Prev)/delta_time;
				
				ROS_INFO("x value =  %f  & %f ,y value = %f & %f  , quality = %f ", position_x,rate_x,position_y,rate_y, flow_quality);
			}

		} else {
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
