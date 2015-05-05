/****************************************
 * Fichier actuator.cpp          	*
 * Mehdi SEFFAR										*
 * cree le 17/07/2002  						*
 * Modified by Ganes Kumar on 04/5/2015
****************************************/
#include "actuator.hh"
#include "math.h"


/********************************************************************

 *                          CONSTRUCTOR                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *                pcard : Pointer to the card type CIODAS64 				*


 *                chan : Number of port of entry in the card				*

 *                                                                  *

 *    RETURN :                                                    	*

 *                An object actuator                               	*

 *                                                                  *

 ********************************************************************/

Actuator::Actuator (int chan1,int chan2 ,card* card)

{

	pcard = card;

	muscle1 = chan1;

	muscle2 = chan2;
}


/********************************************************************

 *                          RECEVOIR_COMMANDE                       *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *      commande : valeur de la pression envoyer sur les muscles    *

 *                                                                  *


 *                                                                  *

 ********************************************************************/

void Actuator::recevoir_commande (double commande)

{
	double m1,m2;
	m1 = 2.5 + commande ;
	m2 = 2.5 - commande ;
	recevoir_commande_decouple(m1,m2);
}

/********************************************************************

 *                    RECEVOIR_COMMANDE_DECOUPLE                    *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *         val1 : valeur de la pression envoyer sur le muscle	1   *

 *         val2 : valeur de la pression envoyer sur le muscle	2   *

 *                                                                  *

 ********************************************************************/

void Actuator::recevoir_commande_decouple (double val1,double val2)

{

	double buffer;
	if (val1>=0 && val1 <= PRESSION_MAX)

	{


		buffer = (val1);// * RAPPORT;
		//pcard->daconv(muscle1,(int)ceil(buffer));
		//pcard->daconv(muscle1, buffer);
		pcard -> send_command_array(muscle1, buffer);


	}



	if (val2>=0 && val2 <= PRESSION_MAX)

	{



		buffer = (val2) ;//* RAPPORT;

		//pcard->daconv(muscle2,(int)ceil(buffer));
		//pcard->daconv(muscle2, buffer);
		pcard -> send_command_array(muscle2, buffer);

	}


}
