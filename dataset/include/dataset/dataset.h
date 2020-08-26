#ifndef DATASET
#define DATASET

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/Int8.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"

using namespace cv;
using namespace std;

class dataset
{

//private:
public:

	ros::NodeHandle nh_;
	ros::Subscriber CmdVel_;
	ros::Subscriber OvR_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber Image_;
	
	Mat image;
	cv_bridge::CvImagePtr cv_ptr;
	
	int override;
	int player;
	int datasetnum;
	int cont;
	bool Train;
	int cont_img;
	
	
	fstream file;
	fstream file_complete;
	std::string directory;
	std::string filename;
	std::string datasetname;
	std::string imagename;
	std::string imagenameOut;
	std::string imagenameOut2;
	
	std::string filenameOut;
	std::string filenameOut_vel;
	std::string filenameOut_img;
	std::string windowName;
	std::string Command;
	
	double roll;
	double pitch;
	double yaw;
	double altitude;
	
	
	
public:
	dataset();
	~dataset();
	
	void Image(const sensor_msgs::ImageConstPtr &msg);
	void CmdVel(const geometry_msgs::Twist& msg);
	void flags(const std_msgs::Int8 &flag);
	
};

#endif
