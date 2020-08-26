#ifndef CONTROL_FRONTAL
#define CONTROL_FRONTAL

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Gazebo
#include "sensor_msgs/Range.h"  //sonar
#include "sensor_msgs/Imu.h"  //odometry
#include "gazebo_msgs/ModelStates.h"  //Pose from gazebo

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include <cmath> 
#include <vector>

using namespace cv;
using namespace std;

class control_frontal
{

//private:
public:

	ros::NodeHandle nh_;
	
  	image_transport::ImageTransport it_;
  	image_transport::Subscriber front_image_;

	Mat front_image;
	
	cv_bridge::CvImagePtr cv_ptr_1;
	
	//GAZEBO POSE
	ros::Subscriber Gazebo_pose_; //subcriber
	geometry_msgs::Pose start_pose; 
	
	//ar drone altitude
	ros::Subscriber Altitude_;
	sensor_msgs::Range range;
	float altitude;
	
	//ar drone imu
	ros::Subscriber Odom_ARdrone_;
	sensor_msgs::Imu odom;
	
	float odom_ARdrone_x;
	float odom_ARdrone_y;
	float odom_ARdrone_z;
	float odom_ARdrone_w;
	
	double roll;
	double pitch;
	double yaw;
	
	std::vector<float> yaw_reference;
	double yaw_error;
	double max_grad;
	double u;
	
	std::vector<float> alt_ref;
	double altura_error;
	
	double kp_yaw;
	double kp_alt;
	
	double kp;
	
	double pose_xi;
	double pose_yi;
	double pose_x;
	double pose_y;
	
	std::vector<float> distance;
	double dist_error;
	double dist_x;
	double dist_y;
	double current_dist;
	
	double signal;
	double error;
	
	int state;
	int index;
	int model;
	
	//cmdvel 
	ros::Publisher pubCommandPilot1_;
	geometry_msgs::Twist commandPilot;
	
	ros::Subscriber OvR_;
	
	int override;

	
public:
	control_frontal();
	~control_frontal();
	
	void FrontImage(const sensor_msgs::ImageConstPtr &msg);
	void Gazebo_pose(const gazebo_msgs::ModelStates::ConstPtr &msg);
	void Ardrone_altitude(const sensor_msgs::Range::ConstPtr &range);
	void Ardrone_odom(const sensor_msgs::Imu::ConstPtr &odom);
	void flags(const std_msgs::Int8 &flag);
};

#endif
