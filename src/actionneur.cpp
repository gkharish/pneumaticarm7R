/****************************************
 * Fichier actionneur.cpp          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/
#include "actionneur.h" 
#include "math.h"


/********************************************************************

 *                          CONSTRUCTEUR                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *                pcard : pointeur sur une carte du type CIODAS6402 *

 *                chan : numero de port d'entr�e sur la carte       *
 *                                                                  *

 *    RETOURNE :                                                    *

 *                un objet actionneur                               *

 *                                                                  *

 ********************************************************************/

actionneur::actionneur (int chan1,int chan2 ,carte* carte)

{

	pcarte = carte;

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

void actionneur::recevoir_commande (double commande)

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

void actionneur::recevoir_commande_decouple (double val1,double val2)

{

	double buffer;
	if (val1>=0 && val1 <= PRESSION_MAX)

	{


		buffer = (val1);// * RAPPORT;
		//pcarte->daconv(muscle1,(int)ceil(buffer));
		pcarte->daconv(muscle1, buffer);

		
	}

	
		
	if (val2>=0 && val2 <= PRESSION_MAX)

	{

    
	            
		buffer = (val2) ;//* RAPPORT;

		//pcarte->daconv(muscle2,(int)ceil(buffer));
		pcarte->daconv(muscle2, buffer);
	
	}

	
}
	