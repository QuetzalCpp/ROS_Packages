/*********************************************************************** 
 * Derechos reservados                                                 *
 * Autores: Leticia Oyuki Rojas Perez, Jose Martinez Carranza          * 
 * Versión: 1.0                                                        *
 * Última actualización:  05/06/2018                                   *
 * Curso EIR 2019                                                      *  
 *                                                                     *
 * Ejemplo de control de frontal del vehículo Ar drone simulado        *
 * usando GAZEBO                                                       *     
 ***********************************************************************/
 
#include "control_frontal.h"

control_frontal::control_frontal():
	nh_("~"), it_(nh_)
{
	ROS_INFO("Init control_frontal");
	front_image_ = it_.subscribe("/camera/image_raw", 1, &control_frontal::FrontImage, this);
	Gazebo_pose_= nh_.subscribe("/gazebo/model_states", 10, &control_frontal::Gazebo_pose, this);
	Altitude_ = nh_.subscribe("/sonar_height", 10, &control_frontal::Ardrone_altitude, this);
	Odom_ARdrone_ = nh_.subscribe("/ardrone/imu", 10, &control_frontal::Ardrone_odom, this);
	
	pubCommandPilot1_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 3);
	OvR_ = nh_.subscribe("/keyboard/override", 1, &control_frontal::flags, this);
	
	yaw_error = 0.0;
	dist_error = 0.0;
	u = 0.0;
	current_dist = 0.0;
	
	//------------- REFERENCE ---------------------------
	
	distance.clear();
	yaw_reference.clear();
	alt_ref.clear();
	
	nh_.getParam("distance", distance);  
	nh_.getParam("yaw_reference", yaw_reference);  
	nh_.getParam("alt_ref", alt_ref);

	//----------- kp ------------------------------------
	nh_.getParam("kp_yaw", kp_yaw);
	nh_.getParam("kp_alt", kp_alt);
	
	nh_.getParam("kp", kp);
	nh_.getParam("model", model);
	
	state = 0;
	index = 0;
	override = 0;
}

control_frontal::~control_frontal()
{

}

void control_frontal::FrontImage(const sensor_msgs::ImageConstPtr &msg)
{	
	try
	{
		cv_ptr_1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	front_image = cv_ptr_1->image;
	
	ostringstream stCurYaw, stRefYaw, stErrorYaw,stsignalYaw, stCurState;
	stCurYaw << yaw;
	stRefYaw << yaw_reference[index];
	stErrorYaw << yaw_error;
	stsignalYaw << u;
	stCurState << state;
	
	ostringstream stRefAltitude, stAltitude;
	stRefAltitude << alt_ref[index]; 
	stAltitude << altitude;
	
	ostringstream stDist, stCurDist, stErrorDist, stSignal;
	stDist << distance[index]; 
	stCurDist << current_dist; 
	stErrorDist << dist_error; 
	stSignal << signal; 
	
	putText(front_image," ref Yaw: " + stRefYaw.str() + " " + " Cur Yaw: " + stCurYaw.str(), Point(5,10), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB( 255, 0, 40), 1, 1);
	putText(front_image," Error Yaw: " + stErrorYaw.str() + " " + " Signal Yaw: " + stsignalYaw.str(), Point(5,25), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB( 255, 0, 40), 1, 1);

	putText(front_image," ref Alt "+ stRefAltitude.str()  + " " + "Cur Alt: " + stAltitude.str(), Point( 5, 40), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB( 255, 255, 255), 1, 1);
	putText(front_image," ref Dist: "+ stDist.str() + " Cur Dist: "+ stCurDist.str(), Point( 5, 55), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB( 255, 255, 255), 1, 1);
	putText(front_image," Dist Error: "+ stErrorDist.str() + " Signal: " + stSignal.str(), Point( 5, 70), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB( 255, 255, 255), 1, 1);
			
	if(override == 0)
	{
		
		putText(front_image," CONTROLLER OFF ", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB( 255, 0, 0), 2, 2);
		state = 0;
		imshow("Ar drone front image", front_image);
		cv::waitKey(1);
		
	}else
	if( override == 1)
	{	
		putText(front_image," CONTROLLER ON ", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB( 0, 255, 0), 2, 2);	
		
		if(state == 0)
		{
			
			//-------------- YAW CONTROL ---------------- 
			max_grad = 180.0;
			yaw_error = yaw_reference[index] - yaw;
			//~ ROS_INFO_STREAM("yaw_reference = " << yaw_reference[index]);
			u = kp_yaw*yaw_error/max_grad;

			commandPilot.angular.z = u;
			
			//------------ ALTITUDE CONTROL ---.---------
		
			altura_error = alt_ref[index] - altitude;
			
			//~ ROS_INFO_STREAM("alt_ref = " << alt_ref[index]);
			//~ std::cout<< "Altura : " << altitude;
			
			commandPilot.linear.z = kp_alt * altura_error;	

			if(commandPilot.linear.z > 0.5)
				commandPilot.linear.z = 0.5;
		
			if(fabsf(altura_error) < 0.01 && fabsf(yaw_error) < 0.5)
			{
				state = 1;
				commandPilot.linear.x = 0;
				commandPilot.linear.y = 0;
				pose_xi = start_pose.position.x;
				pose_yi = start_pose.position.y;
				//~ ROS_INFO_STREAM("Gazebo pose: X = " << pose_xi << " Y = " << pose_yi);
			}	
			
			
		}else
		if(state == 1)
		{	
			
			dist_x = (pose_x - pose_xi);
			dist_y = (pose_y - pose_yi);
			
			current_dist = sqrt((dist_x*dist_x)+(dist_y*dist_y));
			dist_error = distance[index] - current_dist;
			
			//~ ROS_INFO_STREAM("distance = " << distance[index]);
			
			signal = kp * dist_error;
			
			commandPilot.linear.x = signal;
			
			if(fabsf(dist_error) < 0.01)
			{
				commandPilot.linear.x = 0;
				commandPilot.linear.y = 0;
				state = 0;
				index++;
				
				if(index >= yaw_reference.size())
				{
					index = 0;
					ROS_INFO_STREAM("index = " << index);
				}
			
			}
			
			
		}
	}
	
	
	imshow("Ar drone front image", front_image);
	waitKey(1);
	pubCommandPilot1_.publish(commandPilot);
	
}

void control_frontal::Gazebo_pose(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
	
	pose_x = start_pose.position.x = msg->pose[model].position.x;
	pose_y = start_pose.position.y = msg->pose[model].position.y;
	start_pose.position.z = msg->pose[model].position.z;
	start_pose.orientation.x = msg->pose[model].orientation.x;
	start_pose.orientation.y = msg->pose[model].orientation.y;
	start_pose.orientation.z = msg->pose[model].orientation.z;
	start_pose.orientation.w = msg->pose[model].orientation.w;
	
	//ROS_INFO_STREAM("Gazebo pose: X = " << start_pose.position.x << " Y = " << start_pose.position.y << " Z = " << start_pose.position.z );
	
}

void control_frontal::Ardrone_altitude(const sensor_msgs::Range::ConstPtr &range)
{
	altitude = range->range;
}

void control_frontal::Ardrone_odom(const sensor_msgs::Imu::ConstPtr &odom)
{	
	odom_ARdrone_x = odom->orientation.x;
	odom_ARdrone_y = odom->orientation.y;
	odom_ARdrone_z = odom->orientation.z;
	odom_ARdrone_w = odom->orientation.w;

	tf::Quaternion q1(odom_ARdrone_x,odom_ARdrone_y,odom_ARdrone_z,odom_ARdrone_w);
	tf::Matrix3x3 m(q1);
	
	m.getRPY(roll, pitch, yaw);

	roll = (roll * 180) / M_PI;
	pitch = (pitch * 180) / M_PI;
	yaw = (yaw * 180) / M_PI;
	
}

void control_frontal::flags (const std_msgs::Int8 &flag)
{
	if(flag.data == 6)
		override = 1;
	if(flag.data == 10)
		override = 0;
		
	//~ ROS_INFO_STREAM(" override : "<< override);	
}

