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

 *                pioboards : pointeur sur une ioboards du type CIODAS6402 *

 *                + Numero du canal	                            *
 *                                                                  *

 ********************************************************************/

 
palonnier :: palonnier(ioboards * ioboards, int canal,double delta) {
     pioboards = ioboards;
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

 
double palonnier::read_position (void) {
    double valeur;
    unsigned int val;
    //recuperation de la valeur
    //val = pioboards->adconv(channel);
    //conversion en tension
    valeur = (val * UMAX) / 4095.0;
    return valeur;
}