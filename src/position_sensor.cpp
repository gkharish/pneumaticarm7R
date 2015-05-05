/****************************************
 * Fichier position_sensor.cpp         *
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/


#include <iostream>
#include "position_sensor.hh"

//sense of capture
#define POSITIF  1
#define NEGATIF -1
#define NUL      0



/********************************************************************

 *                          CONSTRUCTOR                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *


 *                  off   : offset	                                *

 *		              pen	: pente					                            *

 *                                                                  *

 ********************************************************************/

position_sensor::position_sensor (double off,double pen)

{


  offset = off;
  pente = pen;
}


/********************************************************************

 *                Lecture de la position du capture                 *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de l'angle mesur� en degr�                 *

 *                                                                  *

 ********************************************************************/

double position_sensor::read_position (void)

{

  double angle=0;
  float v1=0.0;
  /* recup�ration de la tension */

  //v1 = pcard->adconv(chanNumber);
  //v1 = pcard -> read_sensors(*axis_num);
  std::cout << "Value read b cap_pos_lire:" << v1 << std::endl;
  /* calcul de l'angle */

  angle = v1*180/3.14;//((double)v * 360)/4095;

  return (angle);

}

double position_sensor::read_sensors_array (int ind)

{

  double angle=0;
  //  unsigned int v=0;
  double v1;
  /* recup�ration de la tension */

  //v1 = pcard->adconv(chanNumber);
  //std::cout << "position_sensor:read_sensors_array:" << ind << std::endl;
  v1 = pcard -> read_sensors(ind);
  //std::cout << "\n Value read by the IO board:" << v1 << std::endl;
  //printf("valeur lue au can %d \r",v);
  /* calcul de l'angle */

  angle = v1*18; // 1volt from potentiometers = 18 degrees //*180/3.14;//((double)v * 360)/4095;
  //std::cout << "angle cap_pos_read_sensor_array"<<ind<<": " << angle << std::endl;
  return (angle);

}


/********************************************************************

 *                   	     get_offset       		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de l'offset		                    *

 *                                                                  *

 ********************************************************************/

double position_sensor::get_offset(void)
{
  return(offset);
}
/********************************************************************

 *                   	     set_offset       		            *

 ********************************************************************

 *                                                                  *

 *   		                                                    *

 *          modifie la valeur de l'offset                           *

 *                                                                  *

 ********************************************************************/

void position_sensor::set_offset(double off)
{
  offset = off;
}


/********************************************************************

 *                   	     get_pente       		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de la pente		                    *

 *                                                                  *

 ********************************************************************/

 double position_sensor::get_pente(void)
 {
 	return(pente);
 }
/********************************************************************

 *                   	     set_pente       		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de la pente		                    *

 *                                                                  *

 ********************************************************************/

void position_sensor::set_pente(double apente)
 {
 	pente = apente;
 }
