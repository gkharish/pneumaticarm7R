/****************************************
 * Fichier captor_position.cpp         *
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/


#include <iostream>
#include "captor_position.hh"

//sens du captor
#define POSITIF  1
#define NEGATIF -1
#define NUL      0



/********************************************************************

 *                          CONSTRUCTEUR                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *


 *                off   : offset	                            *

 *		  pen	: pente					    *

 *                                                                  *

 ********************************************************************/

captor_position::captor_position (double off,double pen)

{


  offset = off;
  pente = pen;
}


/********************************************************************

 *                Lecture de la position du captor                 *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de l'angle mesur� en degr�                 *

 *                                                                  *

 ********************************************************************/

double captor_position::lire_position (void)

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

double captor_position::read_sensors_array (int ind)

{

  double angle=0;
  //  unsigned int v=0;
  double v1;
  /* recup�ration de la tension */

  //v1 = pcard->adconv(chanNumber);
  //std::cout << "captor_position:read_sensors_array:" << ind << std::endl;
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

double captor_position::get_offset(void)
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

void captor_position::set_offset(double off)
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

 double captor_position::get_pente(void)
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

void captor_position::set_pente(double apente)
 {
 	pente = apente;
 }
