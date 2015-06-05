/****************************************
 * Fichier position_sensor.cpp         *
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/


#include <iostream>
#include "position_sensor.hh"
#include <debug.hh>

// Sign of sensor
#define POSITIF  1
#define NEGATIF -1
#define NUL      0



/********************************************************************

 *                          CONSTRUCTOR                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *


 *                  off   : offset_	                                *

 *		              pen	: slope					                            *

 *                                                                  *

 ********************************************************************/

position_sensor::position_sensor (double off,double pen, int index_index)
{
  offset_ = off;
  slope_ = pen;
  index_ = index_index;
}


/********************************************************************

 *                Position sensor reading                          *

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

  //v1 = pioboards->adconv(chanIndex);
  //v1 = pioboards -> read_sensors(*axis_num);
  ODEBUGL("Value read b cap_pos_lire:" << v1,3);
  /* calcul de l'angle */

  angle = v1*180/3.14;//((double)v * 360)/4095;

  return (angle);

}

double position_sensor::read_sensors_array ()
{

  double angle=0;
  //  unsigned int v=0;
  double v1;
  /* Reading the potentionmeter value */

  ODEBUGL("position_sensor:read_sensors_array:" << index_,3);
  v1 = pioboards -> read_sensors(index_);
  ODEBUGL("\n Value read by the IO board:" << v1,3);
  /* Computing the angle */

  angle = v1*18; // 1volt from potentiometers = 18 degrees //*180/3.14;//((double)v * 360)/4095;
  ODEBUGL("angle cap_pos_read_sensor_array"<<index_<<": " << angle ,3);
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
  return(offset_);
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
  offset_ = off;
}


/********************************************************************

 *                   	     get_slope       		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de la slope_		                    *

 *                                                                  *

 ********************************************************************/

 double position_sensor::get_slope(void)
 {
 	return(slope_);
 }
/********************************************************************

 *                   	     set_slope       		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de la slope		                    *

 *                                                                  *

 ********************************************************************/

void position_sensor::set_slope(double aslope)
 {
 	slope_ = aslope;
 }

void position_sensor::set_index(int aindex)
{
  index_ = aindex;
}
