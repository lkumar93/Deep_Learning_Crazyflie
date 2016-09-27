#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

class CameraPublisher
{  
  string camera_position;
  int camera_input_id ;
  VideoCapture cap;
  Mat InputImage;
  image_transport::Publisher camera_pub;
  string prefix = "crazyflie/camera/";
  string postfix = "/image" ;
  string topic_name ;

  public:

  CameraPublisher(string position, int id, ros::NodeHandle nh)
  {
	camera_position = position ;
	camera_input_id = id ;
	image_transport::ImageTransport ImageTransporter = image_transport::ImageTransport(nh);
	topic_name = prefix + camera_position +postfix ;
	camera_pub = ImageTransporter.advertise(topic_name, 1);	
	

  }

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
	    camera_pub.publish(msg) ;
	}
  }
  
};

int main( int argc, char** argv )
{

    ros::init(argc, argv, "crazyflie_image_publisher");
    ros::NodeHandle nh;
    
    CameraPublisher bottom_camera_pub("bottom",0,nh);

    bool Initialized = bottom_camera_pub.Initialize();    

    while(nh.ok() && Initialized)
    {
	bottom_camera_pub.Publish() ;
    }
     
    return 0;
}
