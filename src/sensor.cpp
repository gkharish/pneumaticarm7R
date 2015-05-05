/****************************************
 * Fichier capture.cpp          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/
#include "sensor.hh"

/********************************************************************

 *                   	  set_association      		            *

 ********************************************************************

 *                                                                  *
 * PARAMETRES :  	                                            *


 *   - pcard : pointeur sur la card effectuant l'operation adconv *	                                            *



 *   - chan : numero de voie utilise par le sensor                 *



 *                                                                  *

 ********************************************************************/

void sensor::set_association(card *pcard,int chan)
{
  pcard = pcard;
  chanNumber = chan;
}

/********************************************************************

 *                   	     get_channel       		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                le numero de voie du sensor		            *

 *                                                                  *

 ********************************************************************/

int sensor::get_channel()
{
	return(chanNumber);
}



/********************************************************************

 *                   	     get_card        		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                un pointeur sur la card	                    *

 *                                                                  *

 ********************************************************************/

 card * sensor::get_card(void)
 {
 	return(pcard);
 }
