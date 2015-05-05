/* joystick.cpp - corps de la classe classe joystick */

/* Erwan Guiochet - 2002 */



/*

Historique des modifications

----------------------------



*/


#include "joystick.h"

/********************************************************************

 *                          CONSTRUCTEUR                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *                pcard : pointeur sur une card du type CIODAS6402 *

 *                + Numero de voie des divers elements		    *
 *                                                                  *

 ********************************************************************/

 joystick::joystick (card * card,int X, int Y, int Z, int A,int B, int C,int D, int acc,double delta)

 {

  	pcard = card;

	voie_bouton_a = A;

	voie_bouton_b = B;
	voie_bouton_c = C;
	voie_bouton_d = D;
	voie_axeX = X;
	voie_axeY = Y;
	voie_vitesse = acc;
	voie_axeZ = Z;
	seuil = delta;
	UMAX = 5.0;
 }

/********************************************************************

 *          Lecture de la position sur l'axe X du joystick          *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	valeur mesur� en volt	                    *

 *                                                                  *

 ********************************************************************/

 double joystick::read_position_x (void)

 {

  	double valeur;
  	unsigned int val;
	/* recup�ration de la valeur*/

	//val = pcard->adconv(voie_axeX);
	//conversion en tension
	valeur = (val * UMAX) /4095.0;
	return (valeur);

 }

/********************************************************************

 *          Lecture de la position sur l'axe Y du joystick          *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	valeur mesur� en volt	                    *

 *                                                                  *

 ********************************************************************/

 double joystick::read_position_y (void)

 {

  	double valeur;
	unsigned int val;
	/* recup�ration de la valeur*/

	//val = pcard->adconv(voie_axeY);

	//conversion en tension
	valeur = (val * UMAX) /4095.0;
	return (valeur);

 }

/********************************************************************

 *          Lecture de la position sur l'axe Z du joystick          *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	valeur mesur� en volt	                    *

 *                                                                  *

 ********************************************************************/

 double joystick::read_position_z (void)

 {

  	double valeur;
	unsigned int val;
	/* recup�ration de la valeur*/

	//val = pcard->adconv(voie_axeZ);
	//conversion en tension
	valeur = (val * UMAX) /4095.0;
	return (valeur);

 }

/********************************************************************

 *          Lecture de la position vitesse du joystick              *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	valeur mesur� en volt	                    *

 *                                                                  *

 ********************************************************************/

 double joystick::read_position_vitesse (void)

 {

  	double valeur;
	unsigned int val;
	/* recup�ration de la valeur*/

	//val = pcard->adconv(voie_vitesse);

	//conversion en tension
	valeur = (val * UMAX) /4095;
	return (valeur);

 }

/********************************************************************

 *          Lecture de la position de la gachette du joystick       *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	vrai si la gachette est enclenchee          *

 *                                                                  *

 ********************************************************************/

 bool joystick::lire_bouton_A (void)

 {

	unsigned int val;
	/* recup�ration de la valeur*/

	//val = pcard->adconv(voie_bouton_a);

	return (val<200);

 }

/********************************************************************

 *          	  Lecture de la position du bouton B 		    *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	vrai si le bouton B est enclenche           *

 *                                                                  *

 ********************************************************************/

 bool joystick::lire_bouton_B (void)

 {

  	unsigned int val;
	/* recup�ration de la valeur*/

	//val = pcard->adconv(voie_bouton_b);

	return (val<200);

 }

/********************************************************************

 *          	  Lecture de la position du bouton C 		    *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	vrai si le bouton C est enclenche           *

 *                                                                  *

 ********************************************************************/

 bool joystick::lire_bouton_C (void)

 {

  	unsigned int val;
	/* recup�ration de la valeur*/

	//val = pcard->adconv(voie_bouton_c);

	return (val<200);
 }

/********************************************************************

 *          	  Lecture de la position du bouton D		    *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	vrai si le bouton D est enclenche           *

 *                                                                  *

 ********************************************************************/

 bool joystick::lire_bouton_D (void)

 {

  	unsigned int val;
	/* recup�ration de la valeur*/

	//val = pcard->adconv(voie_bouton_d);

	return (val<200);

 }

 
/********************************************************************

 *          	  Lecture  de la vitesse			    *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                	la vitesse du mouvement selon l'intervalle  *
 *			du curseur de vitesse dans lequel on se	    * 
 *			trouve     			            *

 *                                                                  *

 ********************************************************************/
 
 int joystick::get_vitesse() {
 	
 	if (read_position_vitesse() <= 3.55 && read_position_vitesse() > 3.30)
 		return 0;
 	else 
 		if (read_position_vitesse() <= 4.75 && read_position_vitesse() > 3.55)
 			return 1;
 		else
 			return 2;
 	
 }