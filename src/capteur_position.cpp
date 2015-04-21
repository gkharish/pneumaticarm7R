/****************************************
 * Fichier capteur_position.cpp         *
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/


#include <iostream>
#include "capteur_position.h"

//sens du capteur
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

 capteur_position::capteur_position (double off,double pen)

 {


	offset = off;
	pente = pen;
 }


/********************************************************************

 *                Lecture de la position du capteur                 *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de l'angle mesur� en degr�                 *

 *                                                                  *

 ********************************************************************/

 double capteur_position::lire_position (void)

 {

	double angle=0;
	unsigned int v=0;
	float v1;
	/* recup�ration de la tension */

	//v1 = pcarte->adconv(chanNumber);
 //v1 = pcarte -> read_sensors(*axis_num);
 std::cout << "Value read by the IO board:" << v1 << std::endl;
	//printf("valeur lue au can %d \r",v);
	/* calcul de l'angle */

	angle = v1*180/3.14;//((double)v * 360)/4095;

	return (angle);

 }

 double capteur_position::read_sensors_array (int ind)

 {

	double angle=0;
	unsigned int v=0;
	double v1;
	/* recup�ration de la tension */

	//v1 = pcarte->adconv(chanNumber);
	//std::cout << "capteur_position:read_sensors_array:" << ind << std::endl;
  v1 = pcarte -> read_sensors(ind);
  //std::cout << "\n Value read by the IO board:" << v1 << std::endl;
	//printf("valeur lue au can %d \r",v);
	/* calcul de l'angle */

	angle = v1;//*180/3.14;//((double)v * 360)/4095;

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

 double capteur_position::get_offset(void)
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

 void capteur_position::set_offset(double off)
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

 double capteur_position::get_pente(void)
 {
 	return(pente);
 }
