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
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>

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
  image_transport::Publisher camera_pub;
  string prefix = "crazyflie/cameras/";
  string postfix = "/image" ;
  string topic_name ;

  public:

  //Use the constructor to initialize variables
  CameraPublisher(string position, int id, image_transport::ImageTransport ImageTransporter)
  {
	camera_position = position ;
	camera_input_id = id ;	
	topic_name = prefix + camera_position +postfix ;
	camera_pub = ImageTransporter.advertise(topic_name, 1);		

  }

  // Initialize the camera
  bool Initialize()
  {
  	cap.open(camera_input_id);

    	if( !cap.isOpened() )
	    {
		cout << "Could not initialize camera with id : "<< camera_input_id << endl;
		return false;
	    }
	
	return true;

  }

  // Publish the image from the camera to the corresponding topic
  void Publish()
  {
        cap >> InputImage;

        if( InputImage.empty() )
	{
	    cout << "Empty image from camera with id : "<< camera_input_id << endl ;
	}
	else
	{
	    sensor_msgs::ImagePtr msg;
	    msg  = cv_bridge::CvImage(std_msgs::Header(), "bgr8", InputImage).toImageMsg();
	    msg.header.stamp = ros::Time::now() ;
	    camera_pub.publish(msg) ;
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
    image_transport::ImageTransport it(nh);
 
    //Create a publisher for the bottom facing camera
    CameraPublisher bottom_camera_pub("bottom",0,it);

    //Initialize the camera and check for errors
    bool Initialized = bottom_camera_pub.Initialize();    


    //If the camera has been initialized and everything is ok , publish the images from the camera
    while(nh.ok() && Initialized)
    {
	bottom_camera_pub.Publish() ;
    }
     
    return 0;
}
