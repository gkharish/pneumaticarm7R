/****************************************
 * Fichier Controleur_outil.cpp        	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/
 

#include "controleur_outil.h"


#define PRESSION_MAX_NUM 4095
#define PRESSION_MIN_NUM    0
/********************************************************************

 *                          Constructeur                            *

 ********************************************************************

 *                                                                  *

 *   PARAMETRES :                                                   *

 *               card  : pointeur sur une carte		            *

 *               v1    : voie 1 de la pince sur la carte de commande*
 *		 v2    : voie 2 de la pince sur la carte de commande*
 *                                                                  *

 ********************************************************************/
 
 
 controleur_outil::controleur_outil(carte * card,int v1,int v2) {
 	pcarte = card;
 	pince_ouverte = false;
 	voie_pince_1 = v1;
 	voie_pince_2 = v2;
 	
 }
 
/********************************************************************

 *                          Initisialiser                           *

 ********************************************************************

 *                                                                  *

 *      Initialisation de la pince en position fermee               *

 *               						    *

 ********************************************************************/
 
 
 void controleur_outil::initialiser () {
 	pcarte->daconv(voie_pince_1,PRESSION_MAX_NUM);
 	pcarte->daconv(voie_pince_2,PRESSION_MIN_NUM);
 	taskDelay(sysClkRateGet());
 	pcarte->daconv(voie_pince_1,PRESSION_MIN_NUM);
 }
 

/********************************************************************

 *                          Controler_pince                         *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *                ouvrir_pince  : consigne sur la pince             *

 *							            *
 *	Pour ouvrir la pince une impulsion sur la voie 2 suffit     *
 *	il n'est pas oblige qu'elle reste consamment a 1	    *
 *	donc 1e appel a controler_pince met a 1 la voie 2	    *
 *	et 2e appel remet a 0 la voie 2 toujours avec true en	    *
 *	parametre et voie 1 reste toujours a 0			    *
 *		  		        			    *
 ********************************************************************/
 
 
 void controleur_outil::controler_pince(bool ouvrir_pince) {
	
	
	//fin de l'impulsion d'ouverture 
 	if (etat_voie_2 == 1) {
 		pcarte->daconv(voie_pince_2,PRESSION_MIN_NUM);
 		etat_voie_2 = 0;
 	}
 	
 	//fin de l'impulsion de fermeture
 	if (etat_voie_1 == 1) {
 		pcarte->daconv(voie_pince_1,PRESSION_MIN_NUM);
 		etat_voie_1 = 0;
 	}
 	
 	//debut de l'impulsion d'ouverture
 	if (ouvrir_pince && !pince_ouverte) {
 		pcarte->daconv(voie_pince_2,PRESSION_MAX_NUM); 		
 		pince_ouverte = true;
 		etat_voie_2 = 1;
 	}
 	
 	//debut de l'impulsion de fermeture  
 	if (!ouvrir_pince && pince_ouverte) {
 		pcarte->daconv(voie_pince_1,PRESSION_MAX_NUM);
 		pince_ouverte = false;
 		etat_voie_1 = 1;
 		}
 }
 
 
 
/********************************************************************

 *                          get_etat_pince                         *

 ********************************************************************

 *                                                                  *

 *    RETOURNE : Etat de a pince                                    *

 *                						    *

 *		  		        			    *
 ********************************************************************/
 
 
 double controleur_outil::get_etat_pince()  {
	if (pince_ouverte) 
	 	return 1;
	else
	 	return 0;
 }

/********************************************************************

 *                          get_etat_voie_1                         *

 ********************************************************************

 *                                                                  *

 *    RETOURNE : etat de la voie 1 (1 ou 0)                         *

 *                						    *

 *		  		        			    *
 ********************************************************************/
 
 
 double controleur_outil::get_etat_voie_1 () {
 	return (etat_voie_1);
 }

/********************************************************************

 *                          get_etat_voie_2b                        *

 ********************************************************************

 *                                                                  *

 *    RETOURNE : Etat de la voie 2 (1 ou 0)                         *

 *                						    *

 *		  		        			    *
 ********************************************************************/
 
 
 double controleur_outil::get_etat_voie_2 () {
 	return (etat_voie_2);
 }
 	