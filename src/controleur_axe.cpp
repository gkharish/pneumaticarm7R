/****************************************
 * Fichier Controleur_axe.cpp          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/
#define P_ECHANT         10.0  //periode d echantillonnage en ms
#define P_ECHANT_S	 1/P_ECHANT
#define PRESSION_BASE     2.5
#define K_BOUCLE_OUVERTE (2.5/180)
#include "controleur_axe.h"
#include <math.h>
#include <stdio.h>
#include <iostream>
using namespace std;

/***********************************************************
 *			CONSTRUCTEUR			   *
 ***********************************************************/
 controleur_axe::controleur_axe (I_teleop* pjoy,actionneur *paction,int num,double angle_init,int s_cap,int s_pre,double p,double d)
 {
    boucle=FERMEE; //par defaut on est en boucle fermee
    numero = num;
    pactionneur = paction;
    pjoystick = pjoy;
    angle_repos = angle_init;
    angle_reel = angle_init;
    sens_capteur = s_cap;
    sens_pression = s_pre;
    angle_th = angle_init;
    angle_reel_prec = angle_init;
    angle_filtre_prec = angle_init;
    angle_filtre = angle_init;
    erreur = 0;
    saturation_avant = false;
    saturation_arriere = false;
    delta_repos = 0;
    P = p;
    D = d;
    commande = 0;
    for (int i = 0;i < 10;i++)
  	tab_erreur [i] = 0;
    
    
 }






/********************************************************************

 *                          set_boucle                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *                _boucle  : 0 pour boucle ouverte , 		    *

 *		  et 1 pour boucle FERMEE  			    *

 *                                                                  *

 ********************************************************************/

 void controleur_axe::set_boucle (int _boucle)

 {

	if (_boucle==OUVERTE) pcalculer_commande=&controleur_axe::calculer_commande_BO;
	if (_boucle==FERMEE) pcalculer_commande=&controleur_axe::calculer_commande_BF;
	else pcalculer_commande=&controleur_axe::calculer_commande_BF;
 }




/********************************************************************

 *                          set_capteur                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *                pcap  : pointeur sur un capteur		    *

 *		  rap	: rapport mecanique			    *

 *                                                                  *

 ********************************************************************/

 void controleur_axe::set_capteur (capteur_position* pcap)

 {
   //cout << "\n stecapteur" << endl;
   //printf("\n controleur_axe.setcapteur()1");
   
  	pcapteur = pcap;
  	//rapport=rap;
  	double var = pcapteur -> read_sensors_array(numero);//lire_position();
  	//cout << "\n controleur_axe.setcapteur()2";
  	double var1 = var - angle_repos;
  	double offset_capteur = fabs( var1); 
  	//printf("\n controleur_axe.setcapteur()3");
  	//cet offset est recalcule plus tard, inutile ?
 }

 
 
/********************************************************************

 *                Lecture de la position du capteur                 *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de l'angle mesur� en degr�                 *

 *                                                                  *

 ********************************************************************/

 double controleur_axe::lire_position (void)

 {

	double angle;

	/* recup�ration de la tension */
	//angle = pcapteur->lire_position();
	angle = pcapteur->read_sensors_array(numero);

 //std::cout << "Angle is :" << angle << std::endl;
	//Calcul de l'angle theorique en fonction de l'angle lu par le capteur
	
	//Calcul different selon le sens de rotation du capteur 
	
	if (sens_capteur == 1) 
	{
		
		if (angle_repos > offset_lu)
			angle =  angle + offset_capteur ;
		else
			angle =  angle - offset_capteur ;
	}
	else  
	{
		if (angle_repos > offset_lu)
			angle = 360 - ( angle + offset_capteur );
		else
			angle = 360 - ( angle - offset_capteur );	
	}
	//angle =  fmod( angle ,360);
	
	//Pour avoir un intervalle -180->180
	if (angle <= 360 && angle > 180)
		angle = - (360 - angle);
	
	//std::cout << "Angle controleur_axe:lireposition and rapport :" << offset_capteur<<","<<offset_lu<< ","<<angle_repos<<","<<sens_capteur <<angle << rapport << std::endl;
	return ( rapport*angle);

 }


/********************************************************************

 *                   	   get_capteur      		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                un pointeur sur le capteur	                    *

 *                                                                  *

 ********************************************************************/

 capteur_position * controleur_axe::get_capteur(void)
 {
 	return(pcapteur);
 }

/********************************************************************

 *                   	     get_rapport    		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur du rapport		                    *

 *                                                                  *

 ********************************************************************/

 double controleur_axe::get_rapport(void)
 {
 	return(rapport);
 }
 
/********************************************************************

 *                   	     get_delta    		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de delta		                    *

 *                                                                  *

 ********************************************************************/

 double controleur_axe::get_delta(void)
 {
 	return(delta_repos+commande);
 } 


/********************************************************************

 *                   	   get_angle_desire	    	            *

 ********************************************************************

 *     RETOURNE:                                                    *

 *    	  	 la valeur de l'angle theorique	        	    *

 *                                                                  *

 ********************************************************************/
 
 double controleur_axe::get_angle_desire(void) {
 	return angle_th;
 }

/********************************************************************

 *                   	   get_angle_filtre	    	            *

 ********************************************************************

 *     RETOURNE:                                                    *

 *    	  	 la valeur de l'angle mesure filtre       	    *

 *                                                                  *

 ********************************************************************/
 

double controleur_axe::get_angle_filtre (void) {
	return (angle_filtre);
}


/********************************************************************

 *                   	   get_angle_reel	    	            *

 ********************************************************************

 *     RETOURNE:                                                    *

 *    	  	 la valeur de l'angle mesure filtre       	    *

 *                                                                  *

 ********************************************************************/
 

double controleur_axe::get_angle_reel (void) {
	return (angle_reel);
}

double controleur_axe::get_commande(void) {
	return (commande);
}


/**********************************************************************

 *                   	   controler    		              *

 **********************************************************************

 *                                                                    *

 *    	capte la valeur d'un axe et la retranspose en pression        *

 *      PARAMETRES :                                                  *

 *			pos_joy : position du joystick		      *
 *			coef_vitesse : regle la vitesse du mouvement  *
 *			vitesse_angle : vitesse angulaire du mouvement*
 **********************************************************************/

 void controleur_axe::controler () 
 {	
 	
 	// on ne modifie la commande que si le seuil de prise en compte du 
 	//mouvement du joystick a ete depasse	
		
 	/*if (pos_joy > zero_joy+ pjoystick->get_seuil()) 
 	{
		//On n'actualise l'angle theorique que si l'on n'a pas
		//atteint la pression maximale 	
 		if (!saturation_avant)	
 			angle_th = angle_th + sens_pression * (coef_vitesse * vitesse_angle);		
	 }
	 else
 		if (pos_joy < zero_joy - pjoystick->get_seuil()) 
 		{
 			if (!saturation_arriere)
	 			angle_th = angle_th - sens_pression * (coef_vitesse * vitesse_angle);
 		}*/
 		
 	//cout << "inside controleur.controler()debug1" << endl;
	//On calcule la commande correspondant a l'angle theorique actuel 	
 	(this->*pcalculer_commande)();
	//cout << "inside controleur.controler()debug2" << endl;
	//On verifie que delta_repos ne depasse pas les limite
	//delta_repos = pression reglee lors de l'initialisation des muscles en position de repos
	//a tout instant le couple de muscle recoit PRESSION_BASE (+/-) delta_repos + commande
	
	if (((delta_repos + commande)<= PRESSION_BASE) && ((delta_repos + commande) >=-PRESSION_BASE)) 
	{
 		if (saturation_avant) saturation_avant = false;
 		if (saturation_arriere) saturation_arriere = false; 
	 	pactionneur->recevoir_commande(delta_repos+commande);
 	}
 	else 
 	{	
 		if ((delta_repos+commande) > PRESSION_BASE)
 			saturation_avant = true;
 				
 		else	
 			saturation_arriere = true;
 	}
 //cout << "inside controleur.controler()debug3" << delta_repos<<endl;
} 

/********************************************************************

 *                   	   initialisation_muscles    		    *

 ********************************************************************

 *                                                                  *

 *    	  	Gonflement des muscles de facon a        	    *

 *              faire arriver le bras en position de repos          *
 *								    *
 *	PARAMETRES:						    *
 *			delta_init : delta correspondant a la       *
 *				position de repos de l'axe          *
 *			vitesse_pression: incrementation du delta a *
 *				chaque pas                     
    *
 ********************************************************************/

 void controleur_axe::initialisation_muscles (double delta_init,double vitesse_pression)
 {	
 		
 		double i = 0.0,j = 0.0;
 		while (i < (PRESSION_BASE + delta_init ) || j < (PRESSION_BASE - delta_init )) 
 		{
    if (i >=PRESSION_BASE + delta_init) 
 				i = PRESSION_BASE + delta_init;
 			else 
 				i = i + 2 * vitesse_pression;	
 		 if (j >=PRESSION_BASE - delta_init)
 		  j = PRESSION_BASE - delta_init;
 		 else 
 		  j = j + 2 * vitesse_pression;
 			
 			pactionneur -> recevoir_commande_decouple(i,j);
 			//for (int k = 0;k< 400000;k++) {}
			taskDelay (sysClkRateGet ( ) / 32);

 			
 		}
 		delta_repos = delta_init;
 		
 	
 }
 
/********************************************************************

 *                   	   init_angles		    	            *

 ********************************************************************

 *                                                                  *

 *    	  	initialisation des offset		   	    *

 *                                                                  *

 ********************************************************************/

 void controleur_axe::init_angles () 
 {
 	offset_lu = pcapteur-> get_offset();
 	offset_capteur = fabs(offset_lu - angle_repos);
 }

/********************************************************************

 *                   	   initialisation_carte    	            *

 ********************************************************************

 *                                                                  *

 *    	  	Mise a zero des pressions des muscles   	    *

 *                                                                  *

 ********************************************************************/

 void controleur_axe::initialisation_carte ()
 {	
     double i = 0;
	   	pactionneur->recevoir_commande_decouple(i,i);
 } 
 
 

/********************************************************************

 *                   	   degonfle		    	            *

 ********************************************************************

 *                                                                  *

 *    	  	permet de degonfler un actionneur	   	    *

 *                                                                  *

 *	PARAMETRE:					            *
 *		   vitesse_pression : incrementation du delta a     *
 *		   		      chaque pas		    *
 ********************************************************************/

 
void controleur_axe::degonfle (double vitesse_pression)
{
double i,j;
i = PRESSION_BASE + (delta_repos + commande);
j = PRESSION_BASE - (delta_repos + commande);

while(i>0||j>0)
 {
  i=i - 2 * vitesse_pression;
  j=j - 2 * vitesse_pression;
  if (i < 0)
 	i=0;
  if (j < 0)
 	j=0;
  pactionneur->recevoir_commande_decouple(i,j);
  taskDelay (sysClkRateGet ( ) / 32);
//for (int t=0;t<100000;t++){}
 }	
}


/********************************************************************

 *                   	   calculer_commande	    	            *

 ********************************************************************

 *                                                                  *

 *    	  	calcule la commande a envoyer a l'actionneur en     *

 *              en foncton de l'erreur mesuree			    *
 *                                                                  *

 ********************************************************************/


void controleur_axe::calculer_commande_BF () {
  //Lecture de l'angle reel mesure par la capteur 
 angle_reel = (this ->lire_position());
 //std::cout << "\n angle reel inside calcler_commande_BF :" << angle_reel << endl;
  //on filtre l'angle mesure pour eviter les oscillations
  angle_filtre = (P_ECHANT_S *(angle_reel + angle_reel_prec) - angle_filtre_prec * (P_ECHANT_S - 2 * TAU)) / (P_ECHANT_S + 2* TAU);
  //std::cout << "\n angle filtre inside calcler_commande_BF :" << angle_filtre<< endl;
  //Calcul de l'erreur
  erreur = angle_th - angle_filtre;
  //std::cout << "\n erreur inside calcler_commande_BF :" << erreur << endl;
  //Calcul de la derivee de l'erreur	
  derivee_erreur = (erreur - tab_erreur[9]) / (10 * P_ECHANT);
  //std::cout << "\n derivee_erreur_er inside calcler_commande_BF :" << derivee_erreur << endl;
  //Calcul de la commande
  commande  = sens_pression * (P * erreur  +D * derivee_erreur);
  //std::cout << "\n commande inside calcler_commande_BF :" << commande << endl;
  //Actualisation du tableau d'erreurs
  for (int i = 1; i < 10;i++)
  	tab_erreur [i] = tab_erreur[i-1];
  tab_erreur [0] = erreur;
  angle_filtre_prec = angle_filtre;
  angle_reel_prec = angle_reel;
}

/********************************************************************

 *                   	   calculer_commande_BO	    	            *

 ********************************************************************

 *                                                                  *

 *    	  	calcule la commande a envoyer a l'actionneur en     *

 *              	BOUCLE OUVERTE				    *
 *                                                                  *

 ********************************************************************/


void controleur_axe::calculer_commande_BO () {
  //Lecture de l'angle reel mesure par la capteur 
  angle_reel = (this ->lire_position());

  //on filtre l'angle mesure pour eviter les oscillations
  angle_filtre = (P_ECHANT_S *(angle_reel + angle_reel_prec) - angle_filtre_prec * (P_ECHANT_S - 2 * TAU)) / (P_ECHANT_S + 2* TAU);

  
  //Calcul de la commande
  commande  = sens_pression * K_BOUCLE_OUVERTE*angle_th;
  
  angle_filtre_prec = angle_filtre;
  angle_reel_prec = angle_reel;
}
