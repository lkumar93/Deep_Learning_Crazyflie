//
// THIS CONTAINS THE IMPLEMENTATION FOR PUBLISHING ON-BOARD CAMERA IMAGES
// ON CRAZYFLIE USING THE CAMERA PUBLISHER CLASS
//
// COPYRIGHT BELONGS TO THE AUTHOR OF THIS CODE
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
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <deep_learning_crazyflie/bottom_camera_parameters.h>
#include <deep_learning_crazyflie/front_camera_parameters.h>
#include <opencv2/videostab.hpp>

using namespace cv;
using namespace std;

///////////////////////////////////////////
//
//	CLASSES
//
///////////////////////////////////////////
class CameraPublisher
{  

  //Declare all the necessary variables
  string camera_position;
  int camera_input_id ;
  VideoCapture cap;
  Mat InputImage;
  Mat UndistortedImage;
  Mat DeNoisedImage;
  image_transport::CameraPublisher camera_pub;
  string prefix = "crazyflie/cameras/";
  string postfix = "/image" ;
  string camera_topic_name ;
  sensor_msgs::CameraInfo* camera_info;
  sensor_msgs::CameraInfoPtr camera_info_ptr;
  cv::Mat_<float> camera_matrix;
  cv::Mat_<float> distortion_params;

  public:

  //Use the constructor to initialize variables
  CameraPublisher(string position, int id, image_transport::ImageTransport ImageTransporter, double FL_X ,double PP_X ,double FL_Y, double PP_Y, double* DistArray)
	:camera_matrix(3,3), distortion_params(1,5)
  {
	camera_position = position ;
	camera_input_id = id ;	
	camera_topic_name = prefix + camera_position + postfix ;
	camera_pub = ImageTransporter.advertiseCamera(camera_topic_name, 3);	
	camera_info = new sensor_msgs::CameraInfo ();
	camera_info_ptr = sensor_msgs::CameraInfoPtr (camera_info);
	
	camera_info_ptr->header.frame_id = camera_position+"_camera";
	camera_info->header.frame_id = camera_position+"_camera";
	camera_info->width = 640;
	camera_info->height = 480;

	camera_info->K.at(0) = FL_X;
	camera_info->K.at(2) = PP_X;
	camera_info->K.at(4) = FL_Y;
	camera_info->K.at(5) = PP_Y;
	camera_info->K.at(8) = 1;
	camera_info->P.at(0) = camera_info->K.at(0);
	camera_info->P.at(1) = 0;
	camera_info->P.at(2) = camera_info->K.at(2);
	camera_info->P.at(3) = 0;
	camera_info->P.at(4) = 0;
	camera_info->P.at(5) = camera_info->K.at(4);
	camera_info->P.at(6) = camera_info->K.at(5);
	camera_info->P.at(7) = 0;
	camera_info->P.at(8) = 0;
	camera_info->P.at(9) = 0;
	camera_info->P.at(10) = 1;
	camera_info->P.at(11) = 0;
	camera_info->distortion_model = "plumb_bob";

	// Make Rotation Matrix an identity matrix
	camera_info->R.at(0) = (double) 1;
	camera_info->R.at(1) = (double) 0;
	camera_info->R.at(2) = (double) 0;
	camera_info->R.at(3) = (double) 0;
	camera_info->R.at(4) = (double) 1;
	camera_info->R.at(5) = (double) 0;
	camera_info->R.at(6) = (double) 0;
	camera_info->R.at(7) = (double) 0;
	camera_info->R.at(8) = (double) 1;

	for (int i = 0; i < 5; i++)
	    camera_info->D.push_back (DistArray[i]);

	camera_matrix << FL_X, 0, PP_X, 0, FL_Y, PP_Y, 0, 0, 1 ;
	distortion_params << DistArray[0],DistArray[1],DistArray[2],DistArray[3],DistArray[4];

  }

  // Initialize the camera
  bool Initialize()
  {
  	cap.open(camera_input_id);

	cap.set(CV_CAP_PROP_FRAME_WIDTH , 480);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT , 640);

    	if( !cap.isOpened() )
	    {
		ROS_ERROR("Could not initialize camera with id : %d ",camera_input_id);
		return false;
	    }

	ROS_INFO("Camera with id : %d initialized",camera_input_id);
	
	return true;

  }

  // Publish the image from the camera to the corresponding topic
  void Publish()
  {

	ROS_INFO("Publishing image from camera with id : %d ",camera_input_id);
        cap >> InputImage;
        if( InputImage.empty() )
	{
		ROS_INFO("Empty image from camera with id : %d ",camera_input_id);
	}
	else
	{
	    sensor_msgs::ImagePtr msg;
	    //undistort(InputImage, UndistortedImage, camera_matrix, distortion_params); 
	    DeNoisedImage = InputImage;
	    //GaussianBlur(InputImage,InputImage, Size(5,5),0,0);
	    //medianBlur(InputImage,DeNoisedImage,5);
	    //fastNlMeansDenoisingColored(InputImage,DeNoisedImage,3,3,5,11);
	    msg  = cv_bridge::CvImage(std_msgs::Header(), "bgr8", DeNoisedImage).toImageMsg();
	    msg->header.stamp = ros::Time::now() ;
	    msg->header.frame_id = camera_position+"_camera";

	    camera_info->header.stamp = ros::Time::now();

	    camera_pub.publish(msg, camera_info_ptr) ;
	}
  }
  
};

///////////////////////////////////////////
//
//	MAIN FUNCTION
//
///////////////////////////////////////////

int main( int argc, char** argv )
{    
    //Initialize the Crazyflie Camera Publisher Node
    ros::init(argc, argv, "crazyflie_camera_node");
    ros::NodeHandle nh;

    int bottom_camera_id,front_camera_id;

    image_transport::ImageTransport it(nh);
 
    try
    {
        bottom_camera_id = atoi(argv[1]);
       front_camera_id = atoi(argv[2]);
    }

   catch (...)
   {
	cout << "Camera ID missing in Command Line";
	exit(0);
   }

     ROS_INFO(" Front Camera = %d Bottom Camera = %d ", front_camera_id, bottom_camera_id);

    //Create a publisher for the bottom facing camera
    CameraPublisher bottom_camera_pub("bottom", bottom_camera_id, it, FocalLength_X_Bottom, FocalLength_Y_Bottom, PrincipalPoint_X_Bottom, PrincipalPoint_Y_Bottom, Distortion_Bottom);

    //Create a publisher for the front facing camera
    CameraPublisher front_camera_pub("front", front_camera_id, it, FocalLength_X_Front, FocalLength_Y_Front, PrincipalPoint_X_Front, PrincipalPoint_Y_Front, Distortion_Front);

    //Initialize the cameras and check for errors
    bool Initialized = bottom_camera_pub.Initialize() && front_camera_pub.Initialize();    

    //If the camera has been initialized and everything is ok , publish the images from the cameras
    while(nh.ok() && Initialized)
    {
	bottom_camera_pub.Publish() ;
	front_camera_pub.Publish();
    }
     
    return 0;
}
