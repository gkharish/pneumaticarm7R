/****************************************
 * Fichier I_teleop.cpp         	*
 * Mehdi SEFFAR				*
 ****************************************/
 
#include "I_teleop.h" 


double I_teleop:: get_seuil() {
	return seuil;
}



/********************************************************************

 *                          CONSTRUCTEUR                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *                pcard : pointeur sur une card du type CIODAS6402 *

 *                + Numero du canal	                            *
 *                                                                  *

 ********************************************************************/

 
palonnier :: palonnier(card * card, int canal,double delta) {
     pcard = card;
     channel = canal;
     seuil= delta;
     UMAX = 5.0;
}


/********************************************************************

 *             Lecture de la position indiquee par le palonnier     *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	valeur mesur�e en volt	                    *

 *                                                                  *

 ********************************************************************/

 
double palonnier::lire_position (void) {
    double valeur;
    unsigned int val;
    //recuperation de la valeur
    //val = pcard->adconv(channel);
    //conversion en tension
    valeur = (val * UMAX) / 4095.0;
    return valeur;
}