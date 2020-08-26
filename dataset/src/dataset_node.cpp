/*********************************************************************** 
 * Derechos reservados                                                 *
 * Autores: Leticia Oyuki Rojas Perez                                  * 
 * Versión: 1.0                                                        *
 * Última actualización:  03/08/2018                                   *
 *                                                                     *
 * Dataset Creation                                                    *
 *                                                                     *
 ***********************************************************************/
 
#include "dataset.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dataset");
  dataset OBJECT;
  
  ros::spin();
  return 0;
}
