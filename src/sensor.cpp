/****************************************
 * Fichier sensor.cpp          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/
#include "sensor.hh"

/********************************************************************

 *                   	  set_association      		            *

 ********************************************************************

 *                                                                  *
 * PARAMETRES :  	                                            *


 *   - pioboards : pointeur sur la ioboards effectuant l'operation adconv *	                                            *



 *   - chan : numero de voie utilise par le sensor                 *



 *                                                                  *

 ********************************************************************/

void sensor::set_association(ioboards *pioboards,int chan)
{
  pioboards = pioboards;
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

 *                   	     get_ioboards        		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                un pointeur sur la ioboards	                    *

 *                                                                  *

 ********************************************************************/

ioboards * sensor::get_ioboards(void)
 {
 	return(pioboards);
 }
