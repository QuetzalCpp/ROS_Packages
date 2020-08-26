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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Image_viewer");
  Image_viewer OBJECT;
  
  ros::spin();
  return 0;
}
