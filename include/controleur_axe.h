/****************************************
 * Fichier Controleur_axe.h         	*
 * Jeremie Guiochet			*
 * cree le 17/07/2002 par Mehdi SEFFAR	*
 ****************************************/
 
/*** MODIFICATIONS ***
20/12/2002 : Ajout du pointeur de methode pour choisir Boucle Fermee ou boucle Ouverte


*/ 
 
#ifndef CONTROLEUR_AXE
#define CONTROLEUR_AXE

#define OUVERTE 0
#define FERMEE 1

#define FREQ_COUPURE 2.0  //frequence de coupure du filtre en Hz
#define TAU   1/FREQ_COUPURE //periode de coupure

#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)

/********************************************************************

 *                          CLASSE controle_axe                     *

 ********************************************************************

 *                                                                  *


 *     Cette classe fournit toutes les operations servant 	    *
 *     a controler le mouvement de rotation de certain axes	    *

 *                                                                  *

 ********************************************************************/

#include "actionneur.h"
#include "controleur_outil.h"
#include "joystick.h"
#include "capteur_position.h"
#include "I_teleop.h"
#include <stdlib.h>
#include <stdio.h>
using namespace std;

class controleur_axe

{

	private :

		int numero;			//numero de l'axe

  		actionneur * pactionneur;	//actionneur associe
  		
        I_teleop * pjoystick;     //joystick associe

        double zero_joy;		//position initiale du joystick

        capteur_position *pcapteur;   //capteur de position associe

        int sens_capteur;  //sens de rotation de l'axe par rapport au sens du capteur

        int sens_pression;  //sens de rotation de l'axe par rapport a la variation de la pression	

        double angle_repos,angle_reel;
		
        double delta_repos; //regle lors de l'initialisation des muscles
        		            //reste constant pendant la phase de controle			
		double offset_capteur;  //difference entre la valeur initiale lue
             				// par le capteur et l'angle au repos theorique
		double offset_lu; //valeur lue par le capteur a t =0
		double rapport;
		bool saturation_avant,saturation_arriere;
		int boucle; // 0 pour boucle OUVERTE et 1 pour boucle FERMEE
             	
             	//Donnees calcul commande  PID      
        double P;  			//coefficient du proportionnel
	  	double D;  			//coefficient de la derivee
  		double derivee_erreur;
  		double tab_erreur [10];		//sauvegarde des 10 dernieres erreurs
  		double commande;
  		double erreur, angle_th; 	//angle theorique
  		double angle_reel_prec,angle_filtre,angle_filtre_prec; //infos filtre
        void calculer_commande_BF(void);   //calcul de la commande en Boucle fermee
        void calculer_commande_BO(void);   //calcul de la commande en Boucle Ouverte
        void (controleur_axe::*pcalculer_commande)(); //Pointeur de methode qui servira pour 
        						  //pointer sur calculer_commande en BO ou BF
        	
	public :

		//constructeurs
		controleur_axe (){};
		controleur_axe (I_teleop*,actionneur *,int,double,int,int,double,double);
		
		//Fonction d'association du capteur au controleur d'axe
		void set_capteur (capteur_position *,double);
		
		//Lecture de l'angle
		double lire_position(void);
		
		//initialisation elctronique de la carte de commande
		void initialisation_carte();
		
		//Initialisation des muscles en position de repos
		void initialisation_muscles(double,double);
		
		//Changer de style de boucle fermee ou ouverte
		void set_boucle(int);
				
		//Controle de l'axe
		void controler();
		
		//Degonflement des muscles
		void degonfle (double);
		
		/** recuperation des attributs **/
		void init_angles (void);
		capteur_position * get_capteur(void);
		double get_rapport(void);
		double get_delta(void);
		double get_angle_desire();
		double get_angle_filtre();
		double get_angle_reel();
		double get_commande();
		
		
		
};


#endif
