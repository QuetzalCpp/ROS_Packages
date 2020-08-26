/*********************************************************************** 
 * Derechos reservados                                                 *
 * Autores: Leticia Oyuki Rojas Perez                                  * 
 * Versión: 1.0                                                        *
 * Última actualización:  03/08/2018                                   *
 *                                                                     *
 * Dataset Creation for PoseNet                                        *
 * Used for speed estimation                                           *
 *                                                                     *
 ***********************************************************************/
 
#include "dataset.h"

dataset::dataset():
	nh_("~"), it_(nh_)
{
	ROS_INFO("Init ");
	Image_ = it_.subscribe("/image_raw", 1, &dataset::Image, this);
	CmdVel_ = nh_.subscribe("/cmd_vel", 1000, &dataset::CmdVel, this);
	OvR_ = nh_.subscribe("/override", 1, &dataset::flags, this);
		 
    nh_.getParam("player",player);   
    nh_.getParam("datasetnum",datasetnum);   
    nh_.getParam("cont",cont);   
    nh_.getParam("Train",Train);   
    
	override = 0;
	cont_img = 0;
	Command = "";
	
	windowName = "Player 1";
	
}

dataset::~dataset()
{

}

void dataset::Image(const sensor_msgs::ImageConstPtr &msg)
{	
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	image = cv_ptr->image;
	Mat img_copy = image;
	
	ostringstream stPlayer;
	stPlayer << player;
	
	ostringstream stDataset;
	stDataset << datasetnum;
	
	Point p1 =  Point(image.cols/4,0);
	Point p2 =  Point(image.cols/4,image.rows);
	Point p3 =  Point(3*image.cols/4,0);
	Point p4 =  Point(3*image.cols/4,image.rows);
	
	Point p5 =  Point(image.cols/4,image.rows/4);
	Point p6 =  Point(3*image.cols/4,image.rows/4);
	Point p7 =  Point(image.cols/4, 3*image.rows/4);
	Point p8 =  Point(3*image.cols/4, 3*image.rows/4);
	
	if (Train)
	{
		datasetname =  "train_" + stDataset.str();
		//~ cout<< "Train Dataset Number - " << datasetnum <<endl;
	}
	else
	{
		datasetname =  "test_" + stDataset.str();
		//~ cout<< "Test Dataset Number - " << datasetnum <<endl;
	}
					
	if(override == 1)
	{	
		cont++;
		
		if(cont % 5 == 0)
		{	
			cont_img++;
			
			ostringstream stcont;
			stcont << cont_img;
	
			if(roll == 0 && pitch == 0 && yaw == 0 && altitude == 0)
			{	
				Command = "Hovering";
			}
			else
			{
				if(roll > 0)
				{	
					Command = "left";
				}
				else if(roll < 0)
				{	
					Command = "right";
				}
				else if(pitch > 0)
				{	
					Command = "front";
				}
				else if(pitch < 0)
				{	
					Command = "back";
				}
				else if(yaw > 0)
				{	
					Command = "yaw-to-left";
				}
				else if(yaw < 0)
				{	
					Command = "yaw-to-right";
				}
				else if(altitude > 0)
				{	
					Command = "up";
				}
				else if(altitude < 0)
				{	
					Command = "down";
				}
			}
			
			fstream file;
			fstream fileimg;
			fstream file_complete;
			
			directory =  "path";
			
			imagename = datasetname + "_" + Command + "_img_" + stcont.str() + ".jpg";
			
			filenameOut_vel =  directory + datasetname +"/"+ datasetname + "_vel.txt";
			filenameOut_img =  directory + datasetname +"/"+ datasetname + "_img.txt";
			filenameOut =      directory + datasetname +"/"+ datasetname + "_complete.txt";	
			
			file.open (filenameOut_vel.c_str(), ofstream::app);
			file << setprecision(1) << roll << " " << setprecision(1) << pitch << " " << setprecision(1) << yaw << " " << setprecision(1) << altitude <<"\n";
			file.close();

			fileimg.open (filenameOut_img.c_str(), ofstream::app);
			fileimg << datasetname + "/"+ Command +"/"+ imagename <<"\n";
			fileimg.close();

			file_complete.open (filenameOut.c_str(), ofstream::app);
			file_complete << datasetname + "/"+ Command +"/"+ imagename << " " << setprecision(1) << roll << " " << setprecision(1) << pitch << " " << setprecision(1) << yaw << " " << setprecision(1) << altitude << " "<< Command <<"\n";
			file_complete.close();
			
			
			imagenameOut = directory + datasetname + "/"+ Command +"/"+ imagename;
			imagenameOut2 = directory + datasetname + "/all_images/"+ imagename;
			imwrite(imagenameOut,image);
			imwrite(imagenameOut2,image);
			
			cout<< "======================" <<endl;
			cout<< "Command: " << Command <<endl;
			cout<< "Path: " <<endl;
			//~ cout<< "	vel.txt: " << datasetOut_vel <<endl;
			//~ cout<< "	img.txt: " << datasetOut_img <<endl;
			//~ cout<< "complete.txt: " << datasetOut <<endl;
			cout<< "	Image Name: " << imagenameOut <<endl;
			cout<< ""<<endl;
		}
	}
	
	//~ cout<< "Not Recording Data - " << windowName <<endl;
	line( img_copy, p1, p2, Scalar(255,0,0),  2, 8 );
	line( img_copy, p3, p4, Scalar(255,0,0),  2, 8 );
	line( img_copy, p5, p6, Scalar(255,0,0),  2, 8 );
	line( img_copy, p7, p8, Scalar(255,0,0),  2, 8 );
	imshow(windowName, img_copy);
	waitKey(1);

}

void dataset::CmdVel(const geometry_msgs::Twist& msg)
{
	roll = msg.linear.y;
	pitch = msg.linear.x;
	yaw = msg.angular.z;
	altitude = msg.linear.z;
}
	
void dataset::flags (const std_msgs::Int8 &flag)
{
	if(flag.data == 6)
		override = 1;
	if(flag.data == 10)
		override = 2;
}

