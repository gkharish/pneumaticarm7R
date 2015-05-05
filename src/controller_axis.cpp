/****************************************
 * Fichier controller_axis.cpp          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/
#define P_ECHANT         20.0  //periode d echantillonnage en ms
#define P_ECHANT_S	 1/P_ECHANT
#define PRESSION_BASE     2.5
#define K_BOUCLE_OUVERTE (2.5/180)
#include "controller_axis.hh"
#include <math.h>
#include <stdio.h>
#include <iostream>
using namespace std;

/***********************************************************
 *			CONSTRUCTOR			   *
 ***********************************************************/
controller_axis::controller_axis (I_teleop* pjoy,
                                  Actuator *paction,
                                  int num,
                                  double angle_init,
                                  double angle_min_bound,
                                  double angle_max_bound,
                                  int s_cap,
                                  int s_pre,
                                  double p,
                                  double d)

{
  controller_axis_data aControllerAxisData;
  aControllerAxisData.numero = num;
  aControllerAxisData.pactuator = paction;
  aControllerAxisData.pjoystick = pjoy;
  aControllerAxisData.angle_repos = angle_init;
  aControllerAxisData.angle_reel = angle_init;
  aControllerAxisData.sens_capteur = s_cap;
  aControllerAxisData.sens_pression = s_pre;

  aControllerAxisData.angle_th = angle_init;
  aControllerAxisData.angle_min_bound = angle_min_bound;
  aControllerAxisData.angle_max_bound = angle_max_bound;
  aControllerAxisData.angle_reel_prec = angle_init;
  aControllerAxisData.angle_filtre_prec = angle_init;
  aControllerAxisData.angle_filtre = angle_init;
  aControllerAxisData.error = 0;
  aControllerAxisData.forward_saturation = false;
  aControllerAxisData.backward_saturation = false;
  aControllerAxisData.delta_repos = 0;
  aControllerAxisData.P = p;
  aControllerAxisData.D = d;

  init_controller_axis(aControllerAxisData);
}

void controller_axis::init_controller_axis (controller_axis_data & aControllerAxisData)
{
  ControllerAxisData_ = aControllerAxisData;
  commande = 0;
  for (int i = 0;i < 10;i++)
    tab_erreur [i] = 0;
}






/********************************************************************

 *                          set_loop                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETERS :                                                  *

 *                loop  : 0 for an open loop                        *

 *		  and 1 for a closed loop  			    *

 *                                                                  *

 ********************************************************************/

void controller_axis::set_loop (int aloop)
{
  if (aloop==OUVERTE) pcalculer_commande=&controller_axis::calculer_commande_BO;
  if (aloop==FERMEE)  pcalculer_commande=&controller_axis::calculer_commande_BF;
  if (aloop==PRESCMD) pcalculer_commande=&controller_axis::pressure_commande;
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

void controller_axis::set_capteur (capteur_position* pcap)
{
  pcapteur = pcap;
  //rapport=rap;
  double var = pcapteur -> read_sensors_array(numero);//lire_position();
  cout << "\n controller_axis.setcapteur read sensors array: " << var << endl;
  //  	double var1 = var - angle_repos;
  //double offset_capteur = fabs( var1);
  //printf("\n controller_axis.setcapteur()3");
  //cet offset est recalcule plus tard, inutile ?
}

void controller_axis::set_userpressure(double pres)
{
  user_pressure = pres;
  //cout << "\n set userpressure: " << user_pressure << endl;
}

/********************************************************************

 *                Lecture de la position du capteur                 *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                valeur de l'angle mesur� en degr�                 *

 *                                                                  *

 ********************************************************************/

double controller_axis::lire_position (void)

{

  double angle;

  /* recup�ration de la tension */
  //angle = pcapteur->lire_position();
  angle = pcapteur->read_sensors_array(numero);

  //std::cout << " read sensor array Angle is :" << angle << std::endl;
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
  angle =  fmod( angle ,360);

  //Pour avoir un intervalle -180->180
  if (angle <= 360 && angle > 180)
    angle = - (360 - angle);

  /*std::cout << "Angle controller_axis:lireposition and rapport : " << offset_capteur<<", "
    <<offset_lu<< ", "
    <<angle_repos<<", "
    <<sens_capteur <<", "
    <<angle <<", "
    << rapport <<","<< std::endl;*/
  //cout << "rapport" << rapport << endl;
  return ( angle);

}


/********************************************************************

 *                   	   get_capteur      		            *

 ********************************************************************

 *                                                                  *

 *    RETOURNE :                                                    *

 *                un pointeur sur le capteur	                    *

 *                                                                  *

 ********************************************************************/

capteur_position * controller_axis::get_capteur(void)
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

double controller_axis::get_rapport(void)
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

double controller_axis::get_delta(void)
{
  return(ControllerAxisData_.delta_repos+commande);
}


/********************************************************************

 *                   	   get_angle_desire	    	            *

 ********************************************************************

 *     RETOURNE:                                                    *

 *    	  	 la valeur de l'angle theorique	        	    *

 *                                                                  *

 ********************************************************************/

double controller_axis::get_angle_desire(void) {
  return angle_th;
}

/********************************************************************

 *                   	   get_angle_filtre	    	            *

 ********************************************************************

 *     RETOURNE:                                                    *

 *    	  	 la valeur de l'angle mesure filtre       	    *

 *                                                                  *

 ********************************************************************/


double controller_axis::get_angle_filtre (void) {
  return (ControllerAxisData_.angle_filtre);
}


/********************************************************************

 *                   	   get_angle_reel	    	            *

 ********************************************************************

 *     RETOURNE:                                                    *

 *    	  	 la valeur de l'angle mesure filtre       	    *

 *                                                                  *

 ********************************************************************/


double controller_axis::get_angle_reel (void)
{
  return (angle_reel);
}

double controller_axis::get_angle_lire (void) // get te angle s read by controller from DAQ
{
  double angle_lire;
  angle_lire = (this ->lire_position());
  return (angle_lire);
}

double controller_axis::get_commande(void)
{
  return (commande);
}

/**********************************************************************

 *                   	   Refernce Generator    		              *

 **********************************************************************/
void controller_axis::get_reference_angle(double coef_vitesse, double vitesse_angle)
{
  if (!ControllerAxisData_.forward_saturation)
    {
      angle_th = angle_th + sens_pression * (coef_vitesse * vitesse_angle);
      //cout << "!saturation_avant" << endl;
    }
  //return(angle_th);
  //cout << "\n Angle_the: "<< angle_th << endl;

}
/**********************************************************************

 *                   	   controller    		              *

 **********************************************************************

 *                                                                    *

 *    	capte la valeur d'un axe et la retranspose en pression        *

 *      PARAMETRES :                                                  *

 *			pos_joy : position du joystick		      *
 *			coef_vitesse : regle la vitesse du mouvement  *
 *			vitesse_angle : vitesse angulaire du mouvement*
 **********************************************************************/


void controller_axis::controller ()
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

  //angle_th = 60;
  // SECURITY CHECK
  //double  angle_boundary = (this ->lire_position());
  double  angle_boundary = pcapteur->read_sensors_array(numero);
  cout << "\n Angle Boundary : "<< angle_boundary << endl;
  //On calcule la commande correspondant a l'angle theorique actuel
  if(angle_boundary < angle_max && angle_boundary > angle_min)
    {
      (this->*pcalculer_commande)();
    }
  else
    {
      pactuator->recevoir_commande(ControllerAxisData_.delta_repos);
    }
  //cout << "inside controleur.controler()debug2" << endl;
  //On verifie que delta_repos ne depasse pas les limite
  //delta_repos = pression reglee lors de l'initialisation des muscles en position de repos
  //a tout instant le couple de muscle recoit PRESSION_BASE (+/-) delta_repos + commande

  if (((ControllerAxisData_.delta_repos + commande)<= PRESSION_BASE) && ((ControllerAxisData_.delta_repos + commande) >=-PRESSION_BASE))
    {
      if (ControllerAxisData_.forward_saturation) ControllerAxisData_.forward_saturation = false;
      if (ControllerAxisData_.backward_saturation) ControllerAxisData_.backward_saturation = false;
      pactuator->recevoir_commande(ControllerAxisData_.delta_repos+commande);
    }
  else
    {
      if ((ControllerAxisData_.delta_repos+commande) > PRESSION_BASE)
	ControllerAxisData_.forward_saturation = true;

      else
	ControllerAxisData_.backward_saturation = true;
    }
  //cout << "inside controleur.controler()debug3" << ControllerAxisData_.delta_repos<<endl;
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

void controller_axis::initialisation_muscles (double delta_init,double vitesse_pression)
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

      pactuator -> recevoir_commande_decouple(i,j);
      //for (int k = 0;k< 400000;k++) {}
      // cout << "sysclkrateget value :" << sysClkRateGet ( ) << endl;
      taskDelay (sysClkRateGet ( ) / 32);


    }
  ControllerAxisData_.delta_repos = delta_init;


}

/********************************************************************

 *                   	   init_angles		    	            *

 ********************************************************************

 *                                                                  *

 *    	  	initialisation des offset		   	    *

 *                                                                  *

 ********************************************************************/

void controller_axis::init_angles ()
{
  offset_lu = pcapteur -> get_offset();
  offset_capteur = fabs(offset_lu - angle_repos);
}

/********************************************************************

 *                   	   initialisation_carte    	            *

 ********************************************************************

 *                                                                  *

 *    	  	Mise a zero des pressions des muscles   	    *

 *                                                                  *

 ********************************************************************/

void controller_axis::initialisation_carte ()
{
  double i = 0;
  pactuator->recevoir_commande_decouple(i,i);
}



/********************************************************************

 *                   	   Deflation		    	            *

 ********************************************************************

 *                                                                  *

 *    	  	Allows deflating of an Actuator   	    *

 *                                                                  *

 *	PARAMETRE:					            *
 *		   vitesse_pression : incrementation du delta a     *
 *		   		      chaque pas		    *
 ********************************************************************/


void controller_axis::degonfle (double vitesse_pression)
{
  double i,j;
  i = PRESSION_BASE + (ControllerAxisData_.delta_repos + commande);
  j = PRESSION_BASE - (ControllerAxisData_.delta_repos + commande);

  while(i>0||j>0)
    {
      i=i - vitesse_pression;//i - 2 * vitesse_pression;
      j=j - vitesse_pression;//j - 2 * vitesse_pression;
      if (i < 0)
 	i=0;
      if (j < 0)
 	j=0;
      pactuator->recevoir_commande_decouple(i,j);
      taskDelay (sysClkRateGet () / 32);
      //for (int t=0;t<100000;t++){}
    }
}


/********************************************************************

 *                   	   Calculating control command	    	            *

 ********************************************************************

 *                                                                  *

 *    	  	Calculates the command to send to the actuator     *

 *              And  function of measuring the error			    *
 *                                                                  *

 ********************************************************************/


void controller_axis::calculer_commande_BF ()
{
  //Lecture de l'angle reel mesure par la capteur
  angle_reel = (this ->lire_position());
  //std::cout << "\n angle reel inside calcler_commande_BF :" << angle_reel << endl;
  //on filtre l'angle mesure pour eviter les oscillations
  ControllerAxisData_.angle_filtre = (P_ECHANT_S *(ControllerAxisData_.angle_reel + ControllerAxisData_.angle_reel_prec) 
                                      - ControllerAxisData_.angle_filtre_prec * (P_ECHANT_S - 2 * TAU)) / (P_ECHANT_S + 2* TAU);
  //std::cout << "\n angle filtre inside calcler_commande_BF :" << angle_filtre<< endl;
  //Calcul de l'erreur
  ControllerAxisData_.error = ControllerAxisData_.angle_th - ControllerAxisData_.angle_filtre;

  std::cout << "\n erreur inside calcler_commande_BF :" << ControllerAxisData_.error << endl;
  //Calcul de la derivee de l'erreur
  derivee_erreur = (ControllerAxisData_.error - tab_erreur[9]) / (10 * P_ECHANT);
  //std::cout << "\n derivee_erreur_er inside calcler_commande_BF :" << derivee_erreur << endl;
  //Calcul de la commande
  commande  = sens_pression * (ControllerAxisData_.P * ControllerAxisData_.error
                               + ControllerAxisData_.D * derivee_erreur); //numero
  std::cout << "\n commande inside calcler_commande_BF :" << commande << endl;
  //Actualisation du tableau d'erreurs
  for (int i = 1; i < 10;i++)
    tab_erreur [i] = tab_erreur[i-1];
  tab_erreur [0] = ControllerAxisData_.error;
  ControllerAxisData_.angle_filtre_prec = ControllerAxisData_.angle_filtre;
  ControllerAxisData_.angle_reel_prec = angle_reel;
}

/********************************************************************

 *    		Calculating the control command in Open loop	  	        *

 ********************************************************************

 *                                                                  *

 *    	  	Calculating the command to be sent to the actuator    	*

 *              				OPEN LOOP				    												*
 *                                                                  *

 ********************************************************************/


void controller_axis::calculer_commande_BO ()
{
  //Lecture de l'angle reel mesure par la capteur
  angle_reel = (this ->lire_position());
  //cout << "\n angle read in openloop: " << angle_reel << endl;
  //on filtre l'angle mesure pour eviter les oscillations
  ControllerAxisData_.angle_filtre = (P_ECHANT_S *
                                      (ControllerAxisData_.angle_reel + ControllerAxisData_.angle_reel_prec) - 
                                      ControllerAxisData_.angle_filtre_prec * (P_ECHANT_S - 2 * TAU)) / (P_ECHANT_S + 2* TAU);
  
  //Calcul de la commande
  commande  = sens_pression * K_BOUCLE_OUVERTE*angle_th;

  ControllerAxisData_.angle_filtre_prec = ControllerAxisData_.angle_filtre;
  ControllerAxisData_.angle_reel_prec = angle_reel;
}

void controller_axis::pressure_commande ()
{


  //scanf("%f", user_pressure);
  commande = user_pressure;
  //cout << "\n pressure commande: " << commande << endl;

}