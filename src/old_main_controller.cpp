/************************************************************
*	       PROGRAMME DE TEST DE LA REALISATION	    *
*		     DU CONTROLEUR DE ROBOT	            *
************************************************************
* Cree par : Erwan GUIOCHET/ Mehdi Seffar / JEREMIE GUIOCHET*
* Date     : 02/07/2002 				    *
* Derniere modification de M.Seffar: 20/09/2002
* REPRIS PAR J. GUIOCHET le 15/12/2002			    *
* VERSION  	    
************************************************************/

/*MODIFICATIONS 
17/12/2002: AJOUT DES RAPPORTS MECANIQUES POUR LES CAPTEURS DE POSITION
20/12/2002: AJOUT Du choix boucle ouverte boucle fermee
*/



/***** DEFINITION DE L'ADRESSAGE DES CARTES *****/
/*         BASE_REG_CIODAC16 : CIO-DAC16-I 	*/
/*         BASE_REG_CIODAS64 : CIO-DAS6402/16   */
/************************************************/
#define BASE_REG_CIODAC16	0x200
#define BASE_REG_CIODAS64	0x250

/***** DEFINITION DES INFOS CONCERNANT LA GACHETTE *****/
#define VOIE_PINCE_1        14   // sorties de la carte de commande 
#define VOIE_PINCE_2        15   // reliees a la pince

#define PRESSION_MAX_NUM 4095  // pression maximale numerique
#define PRESSION_MIN_NUM    0  // pression minimale numerique


//infos d'initialisations des pressions correspondant a la position initiale du bras
#define DELTA_INIT_AXE_1 -1.4
#define DELTA_INIT_AXE_2 -2.5
#define DELTA_INIT_AXE_3  0.0
#define DELTA_INIT_AXE_4  1.8 
#define DELTA_INIT_AXE_5  0.2
#define DELTA_INIT_AXE_6 -0.6
#define DELTA_INIT_AXE_7 -0.2 

//Ports de sorties des actionneurs sur la carte de commande (CIODAC16)
#define VOIE_1_1  2
#define VOIE_1_2  3
#define VOIE_2_1  0
#define VOIE_2_2  1
#define VOIE_3_1  5
#define VOIE_3_2  4
#define VOIE_4_1  7
#define VOIE_4_2  6
#define VOIE_5_1 12
#define VOIE_5_2 13
#define VOIE_6_1  9
#define VOIE_6_2  8
#define VOIE_7_1 10
#define VOIE_7_2 11

//Ports d'entree des joysticks et du palonnier sur la carte d'acquisition (CIODAS64)
#define VOIE_X_1          1
#define VOIE_Y_1	  0
#define VOIE_Z_1          3
#define VOIE_VITESSE_1    2
#define VOIE_X_2	  7
#define VOIE_Y_2	  6
#define VOIE_Z_2	  4
#define VOIE_BOUTTON_A_2  5
#define VOIE_PALONNIER   23 
#define VOIE_INUTILISEE  -1

//seuil de prise en compte des mouvements du joystick et du palonnier
#define SEUIL_JOY  0.65
#define SEUIL_PAL  0.25

//Informations concernant la vitesse du mouvement
#define COEF_LENT        0.5    //coefficients de rapidite du 
#define COEF_MOYEN       1 	// mouvement
#define COEF_RAPIDE      2

#define VITESSE_LENTE    0
#define VITESSE_MOY      1
#define VITESSE_RAPIDE   2 

#define PRESSION_BASE     2.5
#define DEMI_TRAJET     180.0
#define P_ECHANT         10.0  //periode d echantillonnage en ms
#define P_ECHANT_S	 1/P_ECHANT
#define DUREE_GONFLEMENT  4.0     //duree pour effectuer 1/2 trajet (180 deg) en secondes
#define VITESSE_PRESSION (P_ECHANT*PRESSION_BASE)/(DUREE_GONFLEMENT*1000)
#define VITESSE_ANGLE   ( P_ECHANT * DEMI_TRAJET) / (DUREE_GONFLEMENT*1000)

//Infos concernant les Message Queues
#define NB_MAX_MSG      1
#define LONG_MAX_MSG    100

//angles theoriques au repos des axes
#define ANGLE_REPOS_1    0.0
#define ANGLE_REPOS_2    0.0
#define ANGLE_REPOS_3    0.0
#define ANGLE_REPOS_4    0.0
#define ANGLE_REPOS_5    0.0
#define ANGLE_REPOS_6    0.0
#define ANGLE_REPOS_7    0.0

// Rapports mecaniques entre rotation des axes des capteurs et rotation reelle des articulations
// Determines par une mesure de la tension avec l articulation a 90 degres K=90/(Vmes*360/5)
#define RAP_MECA_CAP_1    1.5
#define RAP_MECA_CAP_2    1
#define RAP_MECA_CAP_3    1
#define RAP_MECA_CAP_4    1
#define RAP_MECA_CAP_5    2
#define RAP_MECA_CAP_6    1
#define RAP_MECA_CAP_7    1


//sens de rotation des capteurs par rapport au sens theorique
#define SENS_CAPTEUR_1   1
#define SENS_CAPTEUR_2   0
#define SENS_CAPTEUR_3   0
#define SENS_CAPTEUR_4   0
#define SENS_CAPTEUR_5   0
#define SENS_CAPTEUR_6   1
#define SENS_CAPTEUR_7   0

#define SENS_PRESSION_1   1
#define SENS_PRESSION_2   1
#define SENS_PRESSION_3   1
#define SENS_PRESSION_4  -1
#define SENS_PRESSION_5  -1
#define SENS_PRESSION_6  -1
#define SENS_PRESSION_7   1

//nombre de points maximal pour la sauvegarde des rseultats
#define NB_POINTS  20

#define BOUCLE_OUVERTE  0
#define BOUCLE_FERMEE  1

//Infos PID
#define P_AXE_1 (0.0139) *1.5 // ancien : 1.0
#define P_AXE_2 (0.0139) *1.75
#define P_AXE_3 (0.0139) *1.0 // ancien : 1.25
#define P_AXE_4 (0.0139) *1.25 //ancien: 1.25
#define P_AXE_5 (0.0139) *1.5
#define P_AXE_6 (0.0139) *1.75
#define P_AXE_7 (0.0139) 

//CONSTANTE POUR LA BOUCLE OUVERTE
#define K_BOUCLE_OUVERTE (2.5/180)

#define D_AXE_1 0 //1.5
#define D_AXE_2 0  // 2
#define D_AXE_3 0 //0.75
#define D_AXE_4 0 //1.5
#define D_AXE_5 0 //2
#define D_AXE_6 0 //1.5
#define D_AXE_7 0 //1



/****** INCLUDES ******/
//#include "taskLib.h"
//#include "wdLib.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)
//#include "kernelLib.h"
//#include "msgQLib.h"

#include "carte.h"
#include "CIODAC16.h"
#include "CIODAS64.h"
#include "actionneur.h"
#include "I_teleop.h"
#include "joystick.h"
#include "capteur.h"
#include "capteur_position.h"
#include "controleur_axe.h"
#include "controleur_outil.h"
#include "fichier.h"
#include "modele.h"

/***** VARIABLES GLOBALES *****/

//Controleurs d'axes
controleur_axe controleur1,controleur2,controleur3,
		controleur4,controleur5,controleur6,controleur7;

//Controleur d'outil
controleur_outil controleur_pince;

//cartes
CIODAC16 *ciodac16;
CIODAS64 *ciodas64;

//actionneurs
actionneur a1,a2,a3,a4,a5,a6,a7;

//joysticks
I_teleop * joy1,*joy2,* ppalonnier;

//tableau de capteurs
capteur_position cap[7];

//tableau de mesures d'angles
double angle[7];
double erreur[7];

//watchdog
WDOG_ID tempo;

//Messages Queues
MSG_Q_ID msgq1,msgq2,msgq3,msgq4,msgq5,msgq6,msgq7,msgq_pince,msgqfin;

//taches
long int main1,tgo,
    tache_arret,tache_controle_mvt,tache_controle_outil,
    tache_init_1,tache_init_2,tache_init_3,
    tache_init_4,tache_init_5,tache_init_6,tache_init_7,
    tache_joy,
    tache_muscle_1,tache_muscle_2,tache_muscle_3,
    tache_muscle_4,tache_muscle_5,tache_muscle_6,tache_muscle_7,
    tache_fin_1,tache_fin_2,tache_fin_3,
    tache_fin_4,tache_fin_5,tache_fin_6,tache_fin_7;


//modele
modele mon_modele;


/* divers */
double temps = 0.0;
int saisie = 0,increm = 0;
char debut;
bool fin = false,tele_op = false;
bool sortie = false;
char * buffer_joy[7];





/********************************************************
 *							*
 *	trait_musclei ()	i = 1..7		*
 *							*
 *	Routine appelee par les taches 	                *
 *	d'initialisation, de controle et de finalisation*
 *      des axes.					*
 *      Trois  executions differentes correspondant aux *
 *	trois etapes.			                *
 ********************************************************/

void init_muscle_i (controleur_axe *controleur_i,MSG_Q_ID *msgq_i,double * delta, double * vitesse) {
	controleur_i->initialisation_muscles(*delta,*vitesse);
	msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}

void reset_muscle_i (controleur_axe *controleur_i,MSG_Q_ID *msgq_i,double * delta, double * vitesse) {
		controleur_i->degonfle(*vitesse);
		//signale a la tache principale la fin du degonflement des muscles
		msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}	


void trait_muscle1 (double * delta, double * vitesse) {
	//Variables locales
	double vit = *vitesse;
	char * buf = "ok";
	char * buffer = new char[2 * sizeof(double) + 2];
	double pos_joy,coef;
	if (!fin) {
		if (!tele_op) {
			double del = *delta;
			controleur1.initialisation_muscles(del,vit);
			
			//signale a la tache principale la fin de l'initialisation
			msgQSend(msgq1,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
		}
		else {
			while (!sortie) {
				//Reception de tache_joy des informations de controle 
				msgQReceive(msgq1,buffer,2 * sizeof(double) + 2,WAIT_FOREVER);
				
				//verification s'il s'agit d'un message de donnees ou d'un signal de fin
				if ((strncmp(buffer,"fin",3))) {
				
					//extraction des deux reels de la chaine de caracteres  
					sscanf(buffer,"%lf_%lf",&pos_joy,&coef);
					controleur1.controler (pos_joy,coef,VITESSE_ANGLE);
					controleur1.get_delta();
					
					//Enregistrement des donnees dans des tableaux
					//lecture de l'angle actuel de l'axe
					angle[0] = controleur1.lire_position();
				}
			
				else
					taskDelete(tache_muscle_1);
			}
		}
	}
	else  { 
		controleur1.degonfle(vit);
		//signale a la tache principale la fin du degonflement des muscles
		msgQSend(msgqfin,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
	}
}


void trait_muscle2 (double * delta, double * vitesse) {
	double vit = *vitesse;
	char * buf = "ok";
	char * buffer = new char[2 * sizeof(double) + 2];
	double pos_joy,coef;
	if (!fin) {
		if (!tele_op) {
			double del = *delta;
			controleur2.initialisation_muscles(del,vit);
			msgQSend(msgq2,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
		}
		else {
			while (!sortie) {
				msgQReceive(msgq2,buffer,2 * sizeof(double) + 2,WAIT_FOREVER);
				if ((strncmp(buffer,"fin",3))) {
					sscanf(buffer,"%lf_%lf",&pos_joy,&coef);
					controleur2.controler (pos_joy,coef,VITESSE_ANGLE);
					angle[1] = controleur2.lire_position();
				}
				
				else
					taskDelete(tache_muscle_2);
			}
		}
	}
	else  { 
		controleur2.degonfle(vit);
		msgQSend(msgqfin,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
	}
}

void trait_muscle3 (double * delta, double * vitesse) {
	double vit = *vitesse;
	char * buf = "ok";
	char * buffer = new char[2 * sizeof(double) + 2];
	double pos_joy,coef;
	if (!fin) {
		if (!tele_op) {
			double del = *delta;
			controleur3.initialisation_muscles(del,vit);
			msgQSend(msgq3,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
		}
		else {
			while (!sortie) {
				msgQReceive(msgq3,buffer,2 * sizeof(double) + 2,WAIT_FOREVER);
				if ((strncmp(buffer,"fin",3))) {
					sscanf(buffer,"%lf_%lf",&pos_joy,&coef);
					controleur3.controler (pos_joy,coef,VITESSE_ANGLE);
					angle[2] = controleur3.lire_position();

				}
				else
					taskDelete(tache_muscle_3);
			}
		}
	}
	else  { 
		controleur3.degonfle(vit);
		msgQSend(msgqfin,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
	}
}
void trait_muscle4 (double * delta, double * vitesse) {
	double vit = *vitesse;
	char * buffer = new char[2 * sizeof(double) + 2];
	char * buf = "ok";
	double pos_joy,coef;
	if (!fin) {
		if (!tele_op) {
			double del = *delta;
			controleur4.initialisation_muscles(del,vit);
			msgQSend(msgq4,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
		}
		else {
			while (!sortie) {
				msgQReceive(msgq4,buffer,2 * sizeof(double) + 2,WAIT_FOREVER);
				if ((strncmp(buffer,"fin",3))) {
					sscanf(buffer,"%lf_%lf",&pos_joy,&coef);
					controleur4.controler (pos_joy,coef,VITESSE_ANGLE);
					angle[3] = controleur4.lire_position();
				}
				else 
					taskDelete(tache_muscle_4);
				
			}
		}
	}
	else  { 
		controleur4.degonfle(vit);
		msgQSend(msgqfin,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
	}
}
void trait_muscle5 (double * delta, double * vitesse) {
	double vit = *vitesse;
	char * buffer = new char[2 * sizeof(double) + 2];
	char * buf = "ok";
	double pos_joy,coef;
	if (!fin) {
		if (!tele_op) {
			double del = *delta;
			controleur5.initialisation_muscles(del,vit);
			msgQSend(msgq5,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
		}
		else {
			while (!sortie) {
				msgQReceive(msgq5,buffer,2 * sizeof(double) + 2,WAIT_FOREVER);
				if ((strncmp(buffer,"fin",3))) {
					sscanf(buffer,"%lf_%lf",&pos_joy,&coef);
					controleur5.controler (pos_joy,coef,VITESSE_ANGLE);
					angle[4] = controleur5.lire_position();
				}
				else
					taskDelete(tache_muscle_5);
			}
		}
	}
	else  { 
		controleur5.degonfle(vit);
		msgQSend(msgqfin,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
	}
}
void trait_muscle6 (double * delta, double * vitesse) {
	double vit = *vitesse;
	char * buf = "ok";
	char * buffer = new char[2 * sizeof(double) + 2];
	double pos_joy,coef;
	if (!fin) {
		if (!tele_op) {
			double del = *delta;
			controleur6.initialisation_muscles(del,vit);
			msgQSend(msgq6,buffer,2,WAIT_FOREVER,MSG_PRI_NORMAL);
		}
		else {
			while (!sortie) {
				msgQReceive(msgq6,buffer,2 * sizeof(double) + 2,WAIT_FOREVER);
				if ((strncmp(buffer,"fin",3))) {
					sscanf(buffer,"%lf_%lf",&pos_joy,&coef);
					controleur6.controler (pos_joy,coef,VITESSE_ANGLE);
					angle[5] = controleur6.lire_position();
				}
				else 
					taskDelete(tache_muscle_6);
			}
		}
	}
	else  { 
		controleur6.degonfle(vit);
		msgQSend(msgqfin,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
	}
}

void trait_muscle7 (double * delta, double * vitesse) {
	double vit = *vitesse;
	char * buf= "ok";
	char * buffer = new char[2 * sizeof(double) + 2];
	double pos_joy,coef;
	if (!fin) {
		if (!tele_op) {
			double del = *delta;
			controleur7.initialisation_muscles(del,vit);
			msgQSend(msgq7,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
		}
		else {
			while (!sortie) {
				msgQReceive(msgq7,buffer,2 * sizeof(double) + 2,WAIT_FOREVER);
				if ((strncmp(buffer,"fin",3))) {
					sscanf(buffer,"%lf_%lf",&pos_joy,&coef);
					controleur7.controler (pos_joy,coef,VITESSE_ANGLE);
					angle[6] = controleur7.lire_position();
				}
				else 
					taskDelete(tache_muscle_7);
			}
		}		
	}
	else  { 
		controleur7.degonfle(vit);
		msgQSend(msgqfin,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
	}
}


/********************************************************
 *							*
 *	init ()						*
 *							*
 *	permet l'initialisation de tous les objets	*
 *							*
 ********************************************************/
 
void init(){
     //Construction des cartes
     ciodac16 = new CIODAC16();
     ciodas64 = new CIODAS64();

     // initialisation de la ciodas64
     ciodas64->initialisation();

     for (int i = 0; i<7;i++) {
         //construction des capteurs
         cap[i] = capteur_position(0,0);

         //Association des capteurs aux ports de la carte d'acquisition
         cap[i].set_association(ciodas64,i+16);
     }
	printf("\n jusqu'ici tout va bien 1");
     //Actionneurs
     a1 = actionneur(VOIE_1_1,VOIE_1_2,ciodac16);
     printf("\n init()debug1 \n");
     a2 = actionneur(VOIE_2_1,VOIE_2_2,ciodac16);
     a3 = actionneur(VOIE_3_1,VOIE_3_2,ciodac16);
     a4 = actionneur(VOIE_4_1,VOIE_4_2,ciodac16);
     a5 = actionneur(VOIE_5_1,VOIE_5_2,ciodac16);
     a6 = actionneur(VOIE_6_1,VOIE_6_2,ciodac16);
     a7 = actionneur(VOIE_7_1,VOIE_7_2,ciodac16);
	printf("\n init()debug2 \n");
     //construction des joysticks
     // -1 : pas de connection
     joy1 = new joystick(ciodas64,VOIE_X_1,VOIE_Y_1,VOIE_Z_1,
     			VOIE_INUTILISEE,VOIE_INUTILISEE,VOIE_INUTILISEE,
     			VOIE_INUTILISEE,VOIE_VITESSE_1,SEUIL_JOY);
     joy2 = new joystick(ciodas64,VOIE_X_2,VOIE_Y_2,VOIE_Z_2,
     			VOIE_BOUTTON_A_2,VOIE_INUTILISEE,VOIE_INUTILISEE,
     			VOIE_INUTILISEE,VOIE_INUTILISEE,SEUIL_JOY);
    printf("\n init()debug1 \n"); 			
     ppalonnier = new palonnier(ciodas64,VOIE_PALONNIER,SEUIL_PAL);
    printf("\n init()debug1 \n"); 
     for (int i = 0;i < 7;i++)
     	buffer_joy[i] = new char [2 * sizeof(double) + 2];
    printf("\n init()debug1 \n"); 	
     //Controleur de la pince
     controleur_pince = controleur_outil(ciodac16,VOIE_PINCE_1,VOIE_PINCE_2);
     printf("\n jusqu'ici tout va bien 2");     
     //controleurs d'axe
     controleur1 = controleur_axe(joy1,&a1,1,ANGLE_REPOS_1,SENS_CAPTEUR_1,SENS_PRESSION_1,P_AXE_1,D_AXE_1);
     controleur2 = controleur_axe(joy1,&a2,2,ANGLE_REPOS_2,SENS_CAPTEUR_2,SENS_PRESSION_2,P_AXE_2,D_AXE_2);
     controleur3 = controleur_axe(joy1,&a3,3,ANGLE_REPOS_3,SENS_CAPTEUR_3,SENS_PRESSION_3,P_AXE_3,D_AXE_3);
     controleur4 = controleur_axe(joy2,&a4,4,ANGLE_REPOS_4,SENS_CAPTEUR_4,SENS_PRESSION_4,P_AXE_4,D_AXE_4);
     controleur5 = controleur_axe(ppalonnier,&a5,5,ANGLE_REPOS_5,SENS_CAPTEUR_5,SENS_PRESSION_5,P_AXE_5,D_AXE_5);
     controleur6 = controleur_axe(joy2,&a6,6,ANGLE_REPOS_6,SENS_CAPTEUR_6,SENS_PRESSION_6,P_AXE_6,D_AXE_6);
     controleur7 = controleur_axe(joy2,&a7,7,ANGLE_REPOS_7,SENS_CAPTEUR_7,SENS_PRESSION_7,P_AXE_7,D_AXE_7);
     
     
     //Initialisation des controleurs 
     controleur1.initialisation_carte();
     controleur2.initialisation_carte();
     controleur3.initialisation_carte();
     controleur4.initialisation_carte();
     controleur5.initialisation_carte();
     controleur6.initialisation_carte();
     controleur7.initialisation_carte();

     //creation du watchdog
     tempo = wdCreate();
     printf("\n jusqu'ici tout va bien 3");
     //Creation des Messages Queues (sept : un par axe)
     //Utilite : echange d'information entre taches et aussi synchronisation
     msgq1 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
     msgq2 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
     msgq3 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
     msgq4 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
     msgq5 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
     msgq6 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
     msgq7 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
     msgq_pince = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
     msgqfin = 	msgQCreate(7,LONG_MAX_MSG,MSG_Q_FIFO);

    //Association des controleur aux capteurs correspondants	     
     controleur1.set_capteur(cap+4,RAP_MECA_CAP_1);
     controleur2.set_capteur(cap+2,RAP_MECA_CAP_2);
     controleur3.set_capteur(cap+6,RAP_MECA_CAP_3);
     controleur4.set_capteur(cap,RAP_MECA_CAP_4);
     controleur5.set_capteur(cap+1,RAP_MECA_CAP_5);
     controleur6.set_capteur(cap+3,RAP_MECA_CAP_6);
     controleur7.set_capteur(cap+5,RAP_MECA_CAP_7);
     printf("\n jusqu'ici tout va bien 4");
     //Construction du modele
     mon_modele = modele(&controleur1,&controleur2,&controleur3,&controleur4,
     			 &controleur5,&controleur6,&controleur7,
     			 &controleur_pince,
     			 (palonnier *)ppalonnier,(joystick *)joy1,(joystick *)joy2);
printf("\n jusqu'ici tout va bien 5");
     			 
}

/********************************************************
 *							*
 *	 init_capteurs()				*
 *							*
 *	initialisations des offsets des capteurs	*
 *							*
 *							*
 ********************************************************/

void init_capteurs () 
{
	for (int i = 0;i < 7;i++)
	{
		cap[i].set_offset(cap[i].lire_position());
	}
	controleur1.init_angles();
	controleur2.init_angles();
	controleur3.init_angles();
	controleur4.init_angles();
	controleur5.init_angles();
	controleur6.init_angles();
	controleur7.init_angles();
}
	



/********************************************************
 *							*
 *	lire_joy()					*
 *							*
 *	Routine appelee par la tache tache_joy  :	*
 *	recupere les donnees des joysticks		*
 *	et transmission aux differentes taches de 	*
 *	controle des axes				*
 *							*
 ********************************************************/


void lire_joy (void) 
{
	
	//Variables locales
	double pos1,pos2,pos3,pos4,pos5,pos6,pos7,coef_vitesse;
 	char * buffer_fin = "fin";
 	
	taskSuspend(tache_joy);

 	while(!sortie) 
 	{
 		
 		//Lecture de la vitesse
 		if (((joystick * )joy1)-> get_vitesse() == VITESSE_LENTE)
				coef_vitesse = COEF_LENT;
		else 
			if (((joystick * )joy1)->get_vitesse() == VITESSE_MOY)
				coef_vitesse = COEF_MOYEN;
			else 
				coef_vitesse = COEF_RAPIDE;

		//Lecture des differentes voies des joysticks,
		//conversion des donnees en chaines de caracteres
		//et transmission aux taches de controle des axes 
	 	
	 	pos1 = ((joystick * )joy1)->lire_position_y();
 		sprintf(buffer_joy[0],"%lf_%lf",pos1,coef_vitesse);
 		msgQSend(msgq1,buffer_joy[0],2 * sizeof(double) + 2,WAIT_FOREVER,MSG_PRI_NORMAL);
        	
        	pos2 = ((joystick * )joy1)->lire_position_x();
 		sprintf(buffer_joy[1],"%lf_%lf",pos2,coef_vitesse);
 		msgQSend(msgq2,buffer_joy[1],2 * sizeof(double) + 2,WAIT_FOREVER,MSG_PRI_NORMAL);	     
 	
 		pos3 =((joystick * )joy1)->lire_position_z();
 		sprintf(buffer_joy[2],"%lf_%lf",pos3,coef_vitesse);
 		msgQSend(msgq3,buffer_joy[2],2 * sizeof(double) + 2,WAIT_FOREVER,MSG_PRI_NORMAL); 		     
 	
 		pos4 = ((joystick * )joy2)->lire_position_x();
 		sprintf(buffer_joy[3],"%lf_%lf",pos4,coef_vitesse);
 		msgQSend(msgq4,buffer_joy[3],2 * sizeof(double) + 2,WAIT_FOREVER,MSG_PRI_NORMAL);		     
	
		pos5 = ((palonnier *)ppalonnier)->lire_position();   
 		sprintf(buffer_joy[4],"%lf_%lf",pos5,coef_vitesse);
 		msgQSend(msgq5,buffer_joy[4],2 * sizeof(double) + 2,WAIT_FOREVER,MSG_PRI_NORMAL); 		    
	
		pos6 = ((joystick * )joy2)->lire_position_y();
 		sprintf(buffer_joy[5],"%lf_%lf",pos6,coef_vitesse);
 		msgQSend(msgq6,buffer_joy[5],2 * sizeof(double) + 2,WAIT_FOREVER,MSG_PRI_NORMAL); 		     
	
		pos7 = ((joystick * )joy2)->lire_position_z();
 		sprintf(buffer_joy[6],"%lf_%lf",pos7,coef_vitesse);
 		msgQSend(msgq7,buffer_joy[6],2 * sizeof(double) + 2,WAIT_FOREVER,MSG_PRI_NORMAL); 	
		
		if(((joystick * )joy2)->lire_bouton_A()) {
			msgQSend(msgq_pince,"ouv",4,NO_WAIT,MSG_PRI_NORMAL);
		}
		 
		else  {  
			msgQSend(msgq_pince,"fer",4,NO_WAIT,MSG_PRI_NORMAL); 

		}		

 		taskSuspend(tache_joy);	     
 	}
 	//Une fois la phase de controle terminee ,emission d'un signal de fin aux differentes 
 	//taches de controle des axes et a la tache de controle de l'outil
 	
 	msgQSend(msgq1,buffer_fin,4,WAIT_FOREVER,MSG_PRI_NORMAL);
 	msgQSend(msgq2,buffer_fin,4,WAIT_FOREVER,MSG_PRI_NORMAL);
 	msgQSend(msgq3,buffer_fin,4,WAIT_FOREVER,MSG_PRI_NORMAL);
 	msgQSend(msgq4,buffer_fin,4,WAIT_FOREVER,MSG_PRI_NORMAL);
 	msgQSend(msgq5,buffer_fin,4,WAIT_FOREVER,MSG_PRI_NORMAL);
 	msgQSend(msgq6,buffer_fin,4,WAIT_FOREVER,MSG_PRI_NORMAL);
 	msgQSend(msgq7,buffer_fin,4,WAIT_FOREVER,MSG_PRI_NORMAL);
	msgQSend(msgq_pince,buffer_fin,4,WAIT_FOREVER,MSG_PRI_NORMAL);
 	
 	//Reprise de la tache de controle des mouvements
 	taskResume(tache_controle_mvt);
 	
}	
	      

		

/********************************************************
 *							*
 *	 gonfler()					*
 *							*
 *	fonction de					*
 *	gonflement simultane des muscles		*
 *							*
 ********************************************************/

void gonfler(void) {
 	
 	//variables locales
 	    
     	double  * vit,*d1,*d2,*d3,*d4,*d5,*d6,*d7;
     	vit = new double (VITESSE_PRESSION);
     	char * buffer;
     	buffer = new char [10];
     	
     	d1 = new double (DELTA_INIT_AXE_1);
     	d2 = new double (DELTA_INIT_AXE_2);
     	d3 = new double (DELTA_INIT_AXE_3);
     	d4 = new double (DELTA_INIT_AXE_4);
     	d5 = new double (DELTA_INIT_AXE_5);
     	d6 = new double (DELTA_INIT_AXE_6);
     	d7 = new double (DELTA_INIT_AXE_7);
     	
     	
     	//Lancement en parallele des taches d'initialisation des muscles
     							
tache_init_1= taskSpawn ("init_muscle_1",81,0,22000,(FUNCPTR)init_muscle_i,
					(long int)&controleur1, (long int)&msgq1, (long int)d1, (long int)vit,0,0,0,0,0,0);
tache_init_2= taskSpawn ("init_muscle_2",81,0,22000,(FUNCPTR)init_muscle_i,
					(long int)&controleur2, (long int)&msgq2,(long int)d1,(long int)vit,0,0,0,0,0,0);
tache_init_3= taskSpawn ("init_muscle_3",81,0,22000,(FUNCPTR)init_muscle_i,
					(long int)&controleur3, (long int)&msgq3,(long int)d1,(long int)vit,0,0,0,0,0,0);
tache_init_4= taskSpawn ("init_muscle_4",81,0,22000,(FUNCPTR)init_muscle_i,
					(long int)&controleur4, (long int)&msgq4,(long int)d1,(long int)vit,0,0,0,0,0,0);
tache_init_5= taskSpawn ("init_muscle_5",81,0,22000,(FUNCPTR)init_muscle_i,
					(long int)&controleur5, (long int)&msgq5,(long int)d1,(long int)vit,0,0,0,0,0,0);
tache_init_6= taskSpawn ("init_muscle_6",81,0,22000,(FUNCPTR)init_muscle_i,
					(long int)&controleur6, (long int)&msgq6,(long int)d1,(long int)vit,0,0,0,0,0,0);
tache_init_7= taskSpawn ("init_muscle_7",81,0,22000,(FUNCPTR)init_muscle_i,
					(long int)&controleur7, (long int)&msgq7,(long int)d1,(long int)vit,0,0,0,0,0,0);
	
	/*tache_init_1= taskSpawn ("init_muscle_1",81,0,22000,(FUNCPTR)trait_muscle1,
    				(int)d1,(int)vit,0,0,0,0,0,0,0,0);
	tache_init_2= taskSpawn ("init_muscle_2",81,0,22000,(FUNCPTR)trait_muscle2,
     				(int)d2,(int)vit,0,0,0,0,0,0,0,0);
        tache_init_3= taskSpawn ("init_muscle_3",81,0,22000,(FUNCPTR)trait_muscle3,
     				(int)d3,(int)vit,0,0,0,0,0,0,0,0);							
     	tache_init_4= taskSpawn ("init_muscle_4",81,0,22000,(FUNCPTR)trait_muscle4,
     				(int)d4,(int)vit,0,0,0,0,0,0,0,0);	
        tache_init_5= taskSpawn ("init_muscle_5",81,0,22000,(FUNCPTR)trait_muscle5,
      	        		(int)d5,(int)vit,0,0,0,0,0,0,0,0);				
     	tache_init_6= taskSpawn ("init_muscle_6",81,0,22000,(FUNCPTR)trait_muscle6,
     				(int)d6,(int)vit,0,0,0,0,0,0,0,0);
        tache_init_7= taskSpawn ("init_muscle_7",81,0,22000,(FUNCPTR)trait_muscle7,
                        	(int)d7,(int)vit,0,0,0,0,0,0,0,0);*/
     
     
	//Reception des signaux de fin des differentes taches d'initialisation des muscles
	//Reception bloquante, deblocage quand les 7 taches auront fini     
     	msgQReceive(msgq7,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgq6,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgq5,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgq4,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgq3,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgq2,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgq1,buffer,2,WAIT_FOREVER);
 	
 	//mise a vrai de la condition d'entree en phase de teleoperation
     	tele_op = true;

 	
}

/********************************************************
 *							*
 *	degonfler ()					*
 *							*
 *	fonction de					*
 *	degonflement simultane des muscles		*
 *							*
 ********************************************************/

void degonfler(void) {
	
	//Variables locales
	double  * vit;
     	vit = new double (VITESSE_PRESSION);
     	char * buffer;
     	buffer = new char [10];
     	
     	//Lancement en parallele des taches de degonflement des muscles
     	
	tache_fin_1= taskSpawn ("deg_muscle_1",101,0,22000,(FUNCPTR)reset_muscle_i,
     				(long int)&controleur1,(long int)&msgqfin,0,(long int)vit,0,0,0,0,0,0);
 	tache_fin_2= taskSpawn ("deg_muscle_2",101,0,22000,(FUNCPTR)reset_muscle_i,
     				(long int)&controleur2,(long int)&msgqfin,0,(long int)vit,0,0,0,0,0,0);
  	tache_fin_3= taskSpawn ("deg_muscle_3",101,0,22000,(FUNCPTR)reset_muscle_i,
     				(long int)&controleur3,(long int)&msgqfin,0,(long int)vit,0,0,0,0,0,0);
 	tache_fin_4= taskSpawn ("deg_muscle_4",101,0,22000,(FUNCPTR)reset_muscle_i,
     				(long int)&controleur4,(long int)&msgqfin,0,(long int)vit,0,0,0,0,0,0);
 	tache_fin_5= taskSpawn ("deg_muscle_5",101,0,22000,(FUNCPTR)reset_muscle_i,
     				(long int)&controleur5,(long int)&msgqfin,0,(long int)vit,0,0,0,0,0,0);
 	tache_fin_6= taskSpawn ("deg_muscle_6",101,0,22000,(FUNCPTR)reset_muscle_i,
     				(long int)&controleur6,(long int)&msgqfin,0,(long int)vit,0,0,0,0,0,0);
 	tache_fin_7= taskSpawn ("deg_muscle_7",101,0,22000,(FUNCPTR)reset_muscle_i,
     				(long int)&controleur7,(long int)&msgqfin,0,(long int)vit,0,0,0,0,0,0);
    	     	

	/*tache_fin_1= taskSpawn ("deg_muscle_1",101,0,22000,(FUNCPTR)trait_muscle1,
     				0,(int)vit,0,0,0,0,0,0,0,0);
     	tache_fin_2= taskSpawn ("deg_muscle_2",101,0,22000,(FUNCPTR)trait_muscle2,
     				0,(int)vit,0,0,0,0,0,0,0,0);
     	tache_fin_3= taskSpawn ("deg_muscle_3",101,0,22000,(FUNCPTR)trait_muscle3,
     				0,(int)vit,0,0,0,0,0,0,0,0);							
        tache_fin_4= taskSpawn ("deg_muscle_4",101,0,22000,(FUNCPTR)trait_muscle4,
     				0,(int)vit,0,0,0,0,0,0,0,0);	
        tache_fin_5= taskSpawn ("deg_muscle_5",101,0,22000,(FUNCPTR)trait_muscle5,
       				0,(int)vit,0,0,0,0,0,0,0,0);				
     	tache_fin_6= taskSpawn ("deg_muscle_6",101,0,22000,(FUNCPTR)trait_muscle6,
     				0,(int)vit,0,0,0,0,0,0,0,0);
        tache_fin_7= taskSpawn ("deg_muscle_7",101,0,22000,(FUNCPTR)trait_muscle7,
        			0,(int)vit,0,0,0,0,0,0,0,0);*/
        
        
        //Reception des signaux de fin des differentes taches de degonflement des muscles
	//Reception bloquante, deblocage quand les 7 taches auront fini     
        msgQReceive(msgqfin,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgqfin,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgqfin,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgqfin,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgqfin,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgqfin,buffer,2,WAIT_FOREVER);
     	msgQReceive(msgqfin,buffer,2,WAIT_FOREVER);
        
}

/********************************************************************

 *          	  Controler_pince	()                          *

 *
								    *
 * permet de controler la pince a partir de la gachette du          *

 * joystick 2                                                       *

 *                                                                  *

 ********************************************************************/

 void controler_pince (void)
 {	
	//Variables locales 
 	char * buffer_prec = NULL;
 	char * buffer = NULL;
 	bool impulsion = false;
 	int increm_impulsion = 0;
 	buffer = new char [4];
 	buffer_prec = new char [4];
 	strcpy(buffer_prec,"fer");
	
	while(!sortie) {
	
		//Receptions des infos joystick
		msgQReceive(msgq_pince,buffer,4,WAIT_FOREVER);
		
 		if (!impulsion){
 		
 			//si message == fermer et  que pince etait ouverte :
 			//alors ouvrir pince
			if (!(strcmp(buffer,"fer")) && strcmp(buffer,buffer_prec) ) {
				impulsion = true;
				increm_impulsion = increm;
				controleur_pince.controler_pince(false);
			
			}
		
			//Si message == ouvrir et pince etait fermee	:
			//alors fermer pince
			if(!(strcmp(buffer,"ouv")) && strcmp(buffer,buffer_prec)) {
				impulsion = true;
				increm_impulsion = increm;
				controleur_pince.controler_pince(true);
			}		
		
		
			//Mise en memoire de l'etat precedent de la pince
			strcpy(buffer_prec,buffer);
		}
		
		//Si on est dans l,impulsion
		else     
			//si l'impulsion a dure 75 centiemes 
 			if (increm == (increm_impulsion + (750/P_ECHANT))) {
 				if (!strcmp(buffer,"fer")) 
					controleur_pince.controler_pince(false);
				
				if (!strcmp(buffer,"ouv")) 
					controleur_pince.controler_pince(true);
				
				impulsion = false;
			}
		
		//Si reception du message de fin alors destruction de la tache 
		if (!(strcmp(buffer,"fin"))) {
			taskDelete(tache_controle_outil);
		}
	}
}	
	
/********************************************************
 *							*
 *	attente ()					*
 *							*
 *methode associee a la tache d'attente d'appuie clavier*
 *							*
 ********************************************************/
void attente(){
        //Attente de saisie d'un caractere par l'utilisateur
	scanf("%d",&saisie);
	
	//Mise  a vrai de la condition de sortie
	sortie = true;
	fin = true;
	
	//On suspend la tache courante  pour permettre la terminaison de tache_joy moins prioritaire
	//taskSuspend(tache_arret);
	
	//On annule puis detruit le watchdog une fois la saisie clavier faite
	wdCancel(tempo);
	wdDelete(tempo);
	
	//Suppression des taches controle_mvt et lecture joystick
	taskDelete(tache_controle_mvt);
	taskDelete(tache_joy);
	
	//On poursuit l'execution de la tache principale
	taskResume(main1);
}

/********************************************************
 *							*
 *	traitement()					*
 *							*
 *Routine appelee par la tache au timeout du watchdog   *
 *Reprise de la tache de controle			*
 ********************************************************/

void traitement ()
{  
   taskResume(tache_controle_mvt);   
}


/********************************************************
 *							*
 *	controler ()					*
 *							*
 *methode associee a la tache de controle des axes du   *
 *robot							*
 ********************************************************/

void controler ()
{	
	
	//Tache de scrutation des joysticks + prioritaire car on doit d'abord lire
	//les donnees joysticks puis controler les axes
        tache_joy = taskSpawn("lect_joy",94,0,22000,(FUNCPTR)lire_joy,0,0,0,0,0,0,0,0,0,0);
        printf("\n jusqu'ici tout va bien 13");
        //Suspension de tache_joy
        //taskSuspend(tache_joy);
            		
        //Lancement en parallele des taches de controle des axes
        tache_muscle_1 = taskSpawn("t_muscle_1",94,0,22000,(FUNCPTR)trait_muscle1,(int)0,(int)0,0,0,0,0,0,0,0,0);
	printf("\n tache_muscle_1: %d",tache_muscle_1);
		
	printf("\n jusqu'ici tout va bien 14");
	tache_muscle_2 = taskSpawn("t_muscle_2",94,0,22000,(FUNCPTR)trait_muscle2,0,0,0,0,0,0,0,0,0,0);
	printf("\n jusqu'ici tout va bien 1");
	tache_muscle_3 = taskSpawn("t_muscle_3",94,0,22000,(FUNCPTR)trait_muscle3,0,0,0,0,0,0,0,0,0,0);
	tache_muscle_4 = taskSpawn("t_muscle_4",94,0,22000,(FUNCPTR)trait_muscle4,0,0,0,0,0,0,0,0,0,0);
	tache_muscle_5 = taskSpawn("t_muscle_5",94,0,22000,(FUNCPTR)trait_muscle5,0,0,0,0,0,0,0,0,0,0);
	tache_muscle_6 = taskSpawn("t_muscle_6",94,0,22000,(FUNCPTR)trait_muscle6,0,0,0,0,0,0,0,0,0,0);
	tache_muscle_7 = taskSpawn("t_muscle_7",94,0,22000,(FUNCPTR)trait_muscle7,0,0,0,0,0,0,0,0,0,0);
	
	/**** ON LANCE LE WD ET ON SUSPEND LA TACHE  ****/
	WDOG_ID tempo = wdCreate();
	wdStart(tempo, (sysClkRateGet() * (int)P_ECHANT)/1000, 
		(wind_timer_t)taskResume((long)tache_controle_mvt), 0);	
	taskSuspend(tache_controle_mvt);
	
	
	while (!sortie) {

		//Rappel du watchdog  de facon a avoir un traitement periodique
		//time out = sysclkrateget * P_ECHANT (en ticks de CPU) correspond a 1 temps 
		//d'echantillonage de P_ECHANT (en secondes)
		
		wdStart(tempo,(sysClkRateGet() * (int)P_ECHANT) / 1000,(wind_timer_t)traitement,0);
		
		//Suspension de la tache en cours pour permettre a la tache tache_joy de s'executer		
		//taskSuspend(tache_controle_mvt);
			
 		//reprise de la tache joystick
   		taskResume(tache_joy);
   		
		if (!sortie) {
			//Mise a jour du modele
			//if ((increm % 2) == 0) 
				//mon_modele.mettre_a_jour(temps);
				
			//Affichages des mesures faites par les differents capteurs		
			/*for (int i = 0; i<7;i++) {
				if (angle[i] >= -0.001) 
   					printf(" %.2lf   ", angle[i]);
   				else
   					printf("%.2lf   ",  angle[i]);
   				
			}
			printf ("\r");*/
			increm++; //incrementation du nombre d'iterations
			temps = increm * P_ECHANT;			
		}
		//le traitement est fini, la tache se suspend
		taskSuspend(tache_controle_mvt);	
	}
	
	/*** ON SORT DE LA BOUCLE WHILE, LE CONTROLE EST FINI ******/
	
	//Suspension de la tache controle mouvement pour permettre a la tache lecture joystick de terminer
	taskSuspend(tache_controle_mvt);
	//Reprise de la tache arret apres terminaison de la tache de controle des mouvements
	taskResume(tache_arret);
}


/********************************************************
 *							*
 *	controler_robot ()				*
 *							*
 *routine appelee par la tache principale main qui lance*
 *les deux taches de controle et d'attente clavier	*
 ********************************************************/
void controler_robot()
{	
	
	// tache_arret : pour les evenements clavier
	tache_arret=taskSpawn("tache_arret",90,0,22000,(FUNCPTR)attente,0,0,0,0,0,0,0,0,0,0);
	
	printf(" Pour arreter appuyez sur une touche puis validez\n\n");
	printf( "\n\n capt1   capt2   capt3   capt4   capt5   capt6   capt7\n");
	printf("\n jusqu'ici tout va bien 10");
	
	// tache_controle_mvt : controle des differnts axes du robot et acquisition des mesures
	tache_controle_mvt=taskSpawn("t_controle_mvt",95,0,22000,(FUNCPTR)controler,0,0,0,0,0,0,0,0,0,0);
     	printf("\n jusqu'ici tout va bien 11");
	
	//tache_controle_outil : gestion de l'ouverture et de la fermeture de la pince
	tache_controle_outil = taskSpawn("t_controle_outil",95,0,22000,(FUNCPTR)controler_pince,0,0,0,0,0,0,0,0,0,0);
	
	/*REMARQUE: tache arret + prioritaire que tache_controle_mvt pour permettre une sortie du 
	  programme des qu'une saisie clavier a ete faite */
	printf("\n jusqu'ici tout va bien 12");
	
	//On suspend la tache courante pour que les taches de controle ,moins prioriaires, s'executent
	taskSuspend (main1);
	printf("\n jusqu'ici tout va bien 13");
	//suppression de la tache arret
	taskDelete(tache_arret);
	printf("\n jusqu'ici tout va bien 14");
	//Remise a faux de variables booleennes pour une autre execution consecutive	
	sortie = false;
	
	//Mise a vrai de la condition de degonflement des muscles
	fin = true;	
}



/********************************************************
 *							*
 *	principale () = tache principale		*
 *							*
 *routine appelee par debut() apres 
 *l initialisation					*
 ********************************************************/
void principale (void) {
	//variables locales
	bool bonne_saisie = false,ok1 = false,ok2 = false,ok3 =false;
	char * fich = new char [40];
	char * commencer = new char [1];
	char * cont2 = new char [1]; 
	char * cont1 = new char [1];
	char * tmp = new char [1];
	bool boucle=FERMEE;
	
	
	//int i = 0;
		
	printf("\n ..... initialisation electronique des muscles...");
	
	//Appel a la fonction de gonflement des muscles
	//gonfler();
	
	printf(" \n METTEZ LA PRESSION puis \n");
 	while (!ok2) 
 	{
 		printf("\nAppuyez sur c pour continuer ou q pour quitter, puis validez : ");
 		scanf("%s",cont1);
 		if (!strcmp(cont1,"c"))
 		{
 			ok2 = true;
 			printf("\n ..... initialisation des parametres ...\n");
        		init_capteurs();
        		printf("\n ..... debug1...\n");
        		controleur_pince.initialiser();
        		printf("\n ..... debug2...\n");
        		
        		while (!ok3) 	{
        				printf("\nTaper o pour commander en BOUCLE OUVERTE, ou f pour commander en BOUCLE FERMEE, puis validez : ");
        				scanf("%s",tmp);
        				if (strcmp(tmp,"o")==0) {boucle=BOUCLE_OUVERTE;ok3=true;}        				    
        				else {
        					if (strcmp(tmp,"f")==0) {boucle=BOUCLE_FERMEE;ok3=true;}
        				        else ok3=false;
        				      }
        				}
        		//Controleurs en Boucle ouverte ou fermee
			     controleur1.set_boucle(boucle);
			     controleur2.set_boucle(boucle);
			     controleur3.set_boucle(boucle);
			     controleur4.set_boucle(boucle);
			     controleur5.set_boucle(boucle);
			     controleur6.set_boucle(boucle);
			     controleur7.set_boucle(boucle);
			     
        		while(!bonne_saisie) {	
        			printf("\n Appuyez sur c pour commencer ou appuyer sur q pour quitter, puis validez :\n");
 				scanf("%s",commencer);
        			if (!strcmp (commencer,"c")) {

        				bonne_saisie = true;
        				//controle des mouvements du robot
        				
			
					//controler_robot();
     
   				
        				//sauvegarde des mesures
					printf ("\n Entrez le nom du fichier (sans l'extension `.mat`) \n");
					
					scanf("%s",fich);
					//mon_modele.save_matlab(fich);
					
					printf("\n ENLEVEZ LA PRESSION puis \n");
					while(!ok1) {
						
						printf(" appuyez sur c pour continuer\n");
 						scanf("%s",cont2);
 						
 						if (!strcmp(cont2,"c")) {
 							ok1 = true;
 						}
 						else {
 								
 							printf ("\n Mauvaise saisie      !!!\n");
 						}
 					}
				}
				else {
					if (!strcmp(commencer,"a")) {
						bonne_saisie = true;
						fin = true;	 
					}
					else {
							printf("\n Mauvaise saisie   	!!!\n");
					}
				}	
			}
		}
		else {
			if (!strcmp(cont1,"a")) {
				ok2 = true;
				fin = true;
			}
			else
				printf("\n Mauvaise saisie     !!!\n");
		}
	}
	
	
	// terminaison

	printf("\n .... remise a zero electronique ...\n");
	
	//Appel a la fonction de degonflement des muscles
	degonfler();
	
	//Remise a faux de variables booleennes 	
	fin = false;
	tele_op = false;
	sortie = false;
	
        //Destruction des controleurs d'axe	
    controleur1.~controleur_axe();
    controleur2.~controleur_axe();
    controleur3.~controleur_axe();
    controleur4.~controleur_axe();
    controleur5.~controleur_axe();
    controleur6.~controleur_axe();
    controleur7.~controleur_axe();
     	
     a1.~actionneur();
     a2.~actionneur();
     a3.~actionneur();
     a4.~actionneur();
     a5.~actionneur();
     a6.~actionneur();
     a7.~actionneur();
     	
     controleur_pince.~controleur_outil();
     	
     joy1->~I_teleop();
     joy2->~I_teleop();
     ppalonnier->~I_teleop();
	
	ciodac16->~CIODAC16();
	ciodas64->~CIODAS64();
	
	for (int i = 0; i < 7;i++)
		cap[i].~capteur_position();

printf("\n    ====== FIN DU PROGRAMME ======    \n\n");        

}
	
/********************************************************
 *							*
 *	depart ()					*
 *							*
 *	tache init du programme				*
 *							*
 ********************************************************/
void depart(void){
	
	//reglage de la periode du round robin
	//kernelTimeSlice(25);
	
	//Appel a la fonction d'initialisation des objets et des variables
	init();
	
	printf("\n\n\n\n\n\n\n");
	printf("	*****************************************************************\n");
	printf("	*								*\n");
	printf("	*		        PROGRAMME DE				*\n");
	printf("	*               TELEOPERATION DU BRAS 7 AXES			*\n");
	printf("	*								*\n");
	printf("	*****************************************************************\n");
	printf("\n\n\n");
	printf("Tapez une touche puis validez pour initialiser les muscles, ou q pour quitter\n");
        scanf("%c",&debut);
        if (debut != 'a') {
        	//lancement de la tache pricipale
		main1 = taskSpawn("t_main",110,0,22000,(FUNCPTR)principale,0,0,0,0,0,0,0,0,0,0);
	}
	else
		printf("\n\n    ====== FIN ======    \n\n");
	
}



/********************************************************
 *							*
 *	go ()						*
 *							*
 *	lancement du programme sous forme de tache	*
 *							*
 ********************************************************/

void go(void){
tgo = taskSpawn("tgo",70,0,22000,(FUNCPTR)depart,0,0,0,0,0,0,0,0,0,0);
}