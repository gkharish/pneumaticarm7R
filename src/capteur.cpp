/****************************************
 * Fichier capteur.cpp          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/
#include "capteur.h"

/********************************************************************

 *                   	  set_association      		            *

 ********************************************************************

 *                                                                  *
 * PARAMETRES :  	                                            *


 *   - pcarte : pointeur sur la carte effectuant l'operation adconv *	                                            *



 *   - chan : numero de voie utilise par le capteur                 *



 *                                                                  *

 ********************************************************************/

void capteur::set_association(carte *pcard,int chan)
{
  pcarte = pcard;
  chanNumber = chan;
}

/********************************************************************

 *                   	     get_channel       		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                le numero de voie du capteur		            *

 *                                                                  *

 ********************************************************************/

int capteur::get_channel()
{
	return(chanNumber);
}



/********************************************************************

 *                   	     get_carte        		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                un pointeur sur la carte	                    *

 *                                                                  *

 ********************************************************************/

 carte * capteur::get_carte(void)
 {
 	return(pcarte);
 }