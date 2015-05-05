/****************************************
 * Fichier captor.cpp          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/
#include "captor.hh"

/********************************************************************

 *                   	  set_association      		            *

 ********************************************************************

 *                                                                  *
 * PARAMETRES :  	                                            *


 *   - pcard : pointeur sur la card effectuant l'operation adconv *	                                            *



 *   - chan : numero de voie utilise par le captor                 *



 *                                                                  *

 ********************************************************************/

void captor::set_association(card *pcard,int chan)
{
  pcard = pcard;
  chanNumber = chan;
}

/********************************************************************

 *                   	     get_channel       		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                le numero de voie du captor		            *

 *                                                                  *

 ********************************************************************/

int captor::get_channel()
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

 card * captor::get_card(void)
 {
 	return(pcard);
 }
