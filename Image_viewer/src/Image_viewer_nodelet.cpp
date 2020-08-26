/*********************************************************************** 
 * Derechos reservados                                                 *
 * Autores: Leticia Oyuki Rojas Perez, Jose Martinez Carranza          * 
 * Versión: 1.0                                                        *
 * Última actualización:  05/06/2018                                   *
 * Curso IntRob 2019                                                      *  
 *                                                                     *
 * Ejemplo para visualizar la Imagen usando un susbcriptor             *
 *                                                                     *     
 ***********************************************************************/
 
#include "Image_viewer.h"

Image_viewer::Image_viewer():
	nh_("~"), it_(nh_)
{
	ROS_INFO("Init Webcam Viewer");
	//~ Image_ = it_.subscribe("/ardrone/bottom/image_raw", 1, &Image_viewer::Image, this);
	Image_ = it_.subscribe("/ardrone/front/image_raw", 1, &Image_viewer::Image, this);

}

Image_viewer::~Image_viewer()
{

}

void Image_viewer::Image(const sensor_msgs::ImageConstPtr &msg)
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
	
	Webcam_image = cv_ptr->image;
	
	imshow("Webcam-EIR 2019", Webcam_image);
	waitKey(1);
	
}


