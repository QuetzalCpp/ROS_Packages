#ifndef IMAGE_VIEWER
#define IMAGE_VIEWER

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class Image_viewer
{

//private:
public:

	ros::NodeHandle nh_;
	
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber Image_;

	Mat Webcam_image;
	
	cv_bridge::CvImagePtr cv_ptr;
	

public:
	Image_viewer();
	~Image_viewer();
	
	void Image(const sensor_msgs::ImageConstPtr &msg);
};

#endif
