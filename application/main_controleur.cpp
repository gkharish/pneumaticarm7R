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


#define DBG_INFO  std::endl << __FILE__ << "\n" << __LINE__ 
#define ODEBUG(x) std::cerr << x << DBG_INFO << std::endl

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
#include <iostream>
#include <limits>
#include "stdlib.h"
#include "string.h"
#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)
//#include "kernelLib.h"
//#include "msgQLib.h"

//#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>

#include <time.h>
//#include <string.h>
#include <sstream>
#include <math.h>

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
#include "clientudp3.h"

using namespace std;
/***** VARIABLES GLOBALES *****/
bool boucle=FERMEE;
double control_command;
RT_TASK principal_task;
/*struct axisparam 
{
  long int li_delta;
  long int li_speed;
  control_axe * controleur_i;
  long int tache_muscle_i;
  int ang_num;
}*/
//Controleurs d'axes
controleur_axe controleur1,controleur2,controleur3,
						   controleur4,controleur5,controleur6,controleur7;
						   
//axisparam axisparam1, axisparam2, axisparam3, axisparam4, axisparam5, axisparam6, axisparam7;

//Controleur d'outil
controleur_outil controleur_pince;

ClientUDP *clientUDP;

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
double angle_read;
//watchdog
WDOG_ID tempo;

//Messages Queues
MSG_Q_ID msgq1,msgq2,msgq3,msgq4,msgq5,msgq6,msgq7,msgq_pince,msgqfin;

//taches
long int  main1,tgo,
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

void init_muscle_i (controleur_axe *controleur_i, double * delta, double * vitesse) 
{
  controleur_i -> initialisation_muscles(*delta,*vitesse);
  //msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}

void reset_muscle_i (controleur_axe *controleur_i,  double * vitesse)
{
  controleur_i -> degonfle(*vitesse);
  //signale a la tache principale la fin du degonflement des muscles
  //msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}	

void trait_muscle_i (controleur_axe *controleur_i, double * delta, double * vitesse) 
{
  double vit = *vitesse;
  const char * buf = std::string("ok").c_str();
  char * buffer = new char[2 * sizeof(double) + 2];
  double pos_joy,coef;
  if (!fin) 
  {
    double del = *delta;
    controleur_i -> controler();//initialisation_muscles(del,vit);
    /*if (!tele_op) 
    {
      double del = *delta;
      controleur_i -> initialisation_muscles(del,vit);
      //msgQSend(msgq2,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
    }*/
   
  }
  else  
  { 
    controleur_i -> degonfle(vit);
  }
}

/********************************************************
 *							*
 *	init ()						*
 *							*
 *	permet l'initialisation de tous les objets	*
 *							*
 ********************************************************/
 
void init()
{
  //Construction des cartes
  clientUDP = new ClientUDP();
  clientUDP -> client_start();
  
  ciodac16 = new CIODAC16();
  ciodac16->get_client(clientUDP);
  
  ciodas64 = new CIODAS64();
  ciodas64->get_client(clientUDP);

  // initialisation de la ciodas64
  //ciodas64 -> client_start();//initialisation();
  
  for (int i = 0; i<7;i++) 
  {
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
  /*joy1 = new joystick(ciodas64,VOIE_X_1,VOIE_Y_1,VOIE_Z_1,
		      VOIE_INUTILISEE,VOIE_INUTILISEE,VOIE_INUTILISEE,
		      VOIE_INUTILISEE,VOIE_VITESSE_1,SEUIL_JOY);
  joy2 = new joystick(ciodas64,VOIE_X_2,VOIE_Y_2,VOIE_Z_2,
		      VOIE_BOUTTON_A_2,VOIE_INUTILISEE,VOIE_INUTILISEE,
		      VOIE_INUTILISEE,VOIE_INUTILISEE,SEUIL_JOY);*/
  printf("\n init()debug3 \n");
		      
  //ppalonnier = new palonnier(ciodas64,VOIE_PALONNIER,SEUIL_PAL);
  printf("\n init()debug4 \n");  
  for (int i = 0;i < 7;i++)
  {
    buffer_joy[i] = new char [2 * sizeof(double) + 2];
  }  
  printf("\n init()debug5 \n");	
  //Controleur de la pince
  //controleur_pince = controleur_outil(ciodac16,VOIE_PINCE_1,VOIE_PINCE_2);
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
  //tempo = wdCreate();
  printf("\n jusqu'ici tout va bien 3");
  //Creation des Messages Queues (sept : un par axe)
  //Utilite : echange d'information entre taches et aussi synchronisation
 /* msgq1 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
  msgq2 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
  msgq3 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
  msgq4 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
  msgq5 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
  msgq6 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
  msgq7 = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
  msgq_pince = msgQCreate(NB_MAX_MSG,LONG_MAX_MSG,MSG_Q_FIFO);
  msgqfin = 	msgQCreate(7,LONG_MAX_MSG,MSG_Q_FIFO);*/

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
 /* mon_modele = modele(&controleur1,&controleur2,&controleur3,&controleur4,
		      &controleur5,&controleur6,&controleur7,
		      &controleur_pince,
		      (palonnier *)ppalonnier,(joystick *)joy1,(joystick *)joy2);
  printf("\n jusqu'ici tout va bien 5");*/
     			 
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
  //printf("\n inside init_capteurs()1 \n");
  for (int i = 0;i < 7;i++)
  {
    cap[i].set_offset(cap[i].lire_position());
  }
  //printf("\n inside init_capteurs()2 \n");
  controleur1.init_angles();
  //printf("\n inside init_capteurs()3 \n");
  controleur2.init_angles();
  controleur3.init_angles();
  controleur4.init_angles();
  controleur5.init_angles();
  controleur6.init_angles();
  controleur7.init_angles();
  printf("\n Done init_capteurs() \n");
}

/********************************************************
 *							*
 *	 gonfler()					*
 *							*
 *	fonction de					*
 *	gonflement simultane des muscles		*
 *							*
 ********************************************************/

void gonfler(void) 
{
 	
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
  init_muscle_i(&controleur1, d1, vit); 
  //init_muscle_i(&controleur2, d1, vit);
     							

  //tele_op = true;

 	
}

/********************************************************
 *							*
 *	degonfler ()					*
 *							*
 *	fonction de					*
 *	degonflement simultane des muscles		*
 *							*
 ********************************************************/

void degonfler(void) 
{
	
  //Variables locales
  double  * vit;
  vit = new double (VITESSE_PRESSION);
  char * buffer;
  buffer = new char [10];
     	
  //Lancement en parallele des taches de degonflement des muscles
  reset_muscle_i(&controleur1, vit);	

        
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
	
  
  //printf("\n jusqu'ici tout va bien 13 controler");
  
  double  * vit;
  vit = new double (VITESSE_PRESSION);          		
  //Lancement en parallele des taches de controle des axes
  
  
  controleur1.set_boucle(boucle);
  control_command = controleur1.get_commande(); 
  angle_read = controleur1.get_angle_reel();
  trait_muscle_i(&controleur1, &control_command, vit);
  std:: cout << "\n Angle read position " << angle_read << endl;
 
  //printf("\n jusqu'ici tout va bien 2 control\n");
  
	
  printf("\n jusqu'ici tout va bien control\n");	
  
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
  //tache_arret=taskSpawn("tache_arret",90,0,22000,(FUNCPTR)attente,0,0,0,0,0,0,0,0,0,0);
	
  //printf(" To stop press a button and confirm Pour arreter appuyez sur une touche puis validez\n\n");
  //printf( "\n\n capt1   capt2   capt3   capt4   capt5   capt6   capt7\n");
  //printf("\n jusqu'ici tout va bien 10 controler_robot");
	
  // tache_controle_mvt : controle des differnts axes du robot et acquisition des mesures
  //tache_controle_mvt=taskSpawn("t_controle_mvt",95,0,22000,(FUNCPTR)controler,0,0,0,0,0,0,0,0,0,0);
  controler();
  printf("\n jusqu'ici tout va bien controler_robot");
	
  //tache_controle_outil : gestion de l'ouverture et de la fermeture de la pince
  //tache_controle_outil = taskSpawn("t_controle_outil",95,0,22000,(FUNCPTR)controler_pince,0,0,0,0,0,0,0,0,0,0);
	
  
  //printf("\n jusqu'ici tout va bien 12 controler_robot");
	
  
  //printf("\n jusqu'ici tout va bien 13 controler_robot");
  
  //printf("\n jusqu'ici tout va bien 14 controler_robot");
  //Remise a faux de variables booleennes pour une autre execution consecutive	
  	
}


//udppacket_control send_packet;
/********************************************************
 *							*
 *	principale () = tache principale		*
 *							*
 *routine appelee par debut() apres 
 *l initialisation					*
 ********************************************************/
void principale (void* ) 
{
  //variables locales
  bool bonne_saisie = false,ok1 = false,ok2 = false,ok3 =false;
  char * fich = new char [40];
  char * commencer = new char [1];
  char * cont2 = new char [1]; 
  char * cont1 = new char [1];
  char * tmp = new char [1];
  
  /* variables used in the principal program */
  int whileloop_counter = 0, error_counter = 0, loop = 0;
  int timeofsimulation_s = 10; /* time in seconds*/
  int FLAG = 1;
  
  RTIME   now, previous, TASK_PERIOD = 1000000;
  double t, time_start_loop, present_time; 
  //ciodac16 -> client_start();
  //udppacket_control send_packet;
	
  //int i = 0;
  rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));	
  printf("\n ..... initialisation of Electronics of muscles  .....");
	
  //Appel a la fonction de gonflement des muscles
  gonfler();
	
	
	printf("\n Type o for OPEN LOOP control, or f for CLOSED LOOP conttrol and confirm (ok3, tmp) : ");
	std::cin >> tmp; //scanf("%s",tmp);
	     
	std::cin.clear(); std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n'); 	
	if (strcmp(tmp,"o")==0) {boucle=BOUCLE_OUVERTE;ok3=true;}        				    
	else 
	{
	  if (strcmp(tmp,"f")==0) {boucle=BOUCLE_FERMEE;ok3=true;}
		else ok3=false;
	}
	
  printf(" \n APPLY THE PRESSURE   \n");
  
  now = rt_timer_read();
  time_start_loop  = round(now/1.0e9);
  init_capteurs();
  while (FLAG) 
  {
    rt_task_wait_period(NULL);
    
    
    //controleur_pince.initialiser();
    
    now = rt_timer_read();
    present_time  = round(now/1.0e9);
    t = present_time - time_start_loop;
	    
	  controler_robot();
   
    cout << "\n the time past is : " << t;
    if(t >= timeofsimulation_s)
    {
      FLAG = 0;
      //printf("\n Task completion: \n\n\n");
      cout << "\n END of Loop";

    }
  } //while (ok2) finish
	
	
  // terminaison

  cout << "outside while loop ok2" << endl ;
	
  //Appel a la fonction de degonflement des muscles
  degonfler();
	
  //Remise a faux de variables booleennes 	
  fin = false;
  tele_op = false;
  sortie = false;
	
  //ciodac16 -> ~ClientUDP();
	
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

  printf("\n    ====== PROGRAM FINISHED ======    \n\n");        
  printf("\n .... ELectronics is reset ...\n");
} //principale finish

        
/*** SiGNAL catch  **/
         
void catch_signal(int sig)
{
        
}
     
	
/********************************************************
 *							*
 *	depart ()					*
 *							*
 *	tache init du programme				*
 *							*
 ********************************************************/
int main(void)
{
  int n;
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);
  
  mlockall(MCL_CURRENT|MCL_FUTURE);
  
  // Round robin period
  //kernelTimeSlice(25);
	
  // Initializing objects and variables 
  init();

  printf("\n");
  printf("	*****************************************************************\n");
  printf("	*								*\n");
  printf("	*		        TELEOPERATION SOFTWARE  		*\n");
  printf("	*                         for the 7 DOFs ARM			*\n");
  printf("	*			  (depart function)         		*\n");
  printf("	*****************************************************************\n");
  printf("\n\n\n");
  

  n = rt_task_create(&principal_task, "principal_function", 0, 99, 0);
  if (n!=0)
  {
    cout << "Failed @ RT Create" << n <<endl;
  }
  else cout << "END of RT Create" << endl;
  
  n = rt_task_start(&principal_task, &principale, NULL);
  if (n!=0)
  {
    cout << "Failed of RT STart" <<n<< endl;
  }
  else cout << "END of RT Start" << endl;
  
  pause();

  cout << "END of Pause" << endl;

  n = rt_task_delete(&principal_task);
  if(n!=0)cout << "Failed of RT Task delete" << endl;
  else cout << "END of RT taslk delete";
  
  
  
}


