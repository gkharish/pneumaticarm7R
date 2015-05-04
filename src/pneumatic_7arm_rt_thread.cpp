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
#ifndef NDEBUG
#define ODEBUG(x) std::cerr << x << std::endl
#else
#define ODEBUG(x)
#endif

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
#define DELTA_INIT_AXE_1 -1.4//-1.4
#define DELTA_INIT_AXE_2 -2.5 //-2.5
#define DELTA_INIT_AXE_3  0.0
#define DELTA_INIT_AXE_4  1.8
#define DELTA_INIT_AXE_5  0.2
#define DELTA_INIT_AXE_6 -0.6
#define DELTA_INIT_AXE_7 -0.2

//Ports de sorties des actionneurs sur la carte de commande (CIODAC16)
#define VOIE_1_1  0	//2
#define VOIE_1_2  1 //3
#define VOIE_2_1  2	//0
#define VOIE_2_2  3	//1
#define VOIE_3_1  5
#define VOIE_3_2  4
#define VOIE_4_1  7
#define VOIE_4_2  6
#define VOIE_5_1 10	//12
#define VOIE_5_2 	9	//13
#define VOIE_6_1 11	//9
#define VOIE_6_2 12	//8
#define VOIE_7_1 13	//10
#define VOIE_7_2 14	//11

//Ports d'entree des joysticks et du palonnier sur la carte d'acquisition (CIODAS64)
#define VOIE_X_1          1
#define VOIE_Y_1	  0
#define VOIE_Z_1          3
#define VOIE_VITESSE_1    2
#define VOIE_X_2	  7
#define VOIE_Y_2	  6I
#define VOIE_Z_2	  45
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
#define DUREE_GONFLEMENT  8.0     //duree pour effectuer 1/2 trajet (180 deg) en secondes
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

// Angles maximum and minimum reached by joints
#define ANGLE_MIN_1    	-12.0
#define ANGLE_MAX_1    90.0

#define ANGLE_MIN_2    -15.0
#define ANGLE_MAX_2    90.0

#define ANGLE_MIN_3   -90.0
#define ANGLE_MAX_3    50.0

#define ANGLE_MIN_4    -5.0
#define ANGLE_MAX_4   130.0

#define ANGLE_MIN_5   -90.0
#define ANGLE_MAX_5    90.0

#define ANGLE_MIN_6   -30.0
#define ANGLE_MAX_6    30.0

#define ANGLE_MIN_7   -30.0
#define ANGLE_MAX_7    30.0


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
//#define BOUCLE_PRESCMD 2
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
#include <stdio.h>
#include <stdlib.h>
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
#include <fstream>
#include <istream>
#include <math.h>


#include "carte.h"
//#include "CIODAC16.h"
//#include "CIODAS64.h"
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
#include "test_config.h"

#include <pneumatic_7arm_rt_thread.hh>
using namespace std;
using namespace Eigen;
/********************************************************
 *							*
 *	Pneumatic7ARmRtThread Constructorxs	*
 *							*
 *	Creating the objects.	*
 *							*
 ********************************************************/

Pneumatic7ArmRtThread::Pneumatic7ArmRtThread():
  BOUCLE_PRESCMD(2),
  DEFAULT_FLAG(0),
  CALIBRATION_FLAG(1),
  CONTROL_MODE_NOPRES_FLAG(1),
  CONTROL_MODE_PRES_FLAG(0),
  INFLATING_FLAG(0),
  PRES_INDIVIDUAL_FLAG(0),
  recving_Data_(15),
  CTRL_FLAG(7),
  pressure_command_array_(7),
  sensors_array_(7),
  temps_(0.0),
  saisie_(0),
  increm_(0),
  fin_(false),
  tele_op_(false),
  sortie_(false)
{
  // Creating UDP Client
  clientUDP = new ClientUDP();

  // Creating IO board managers.
  ciodac16_ = new CIODAC16();
  ciodas64_ = new CIODAS64();

  // Creating position sensors.
  for (int i = 0; i<7;i++)
    {
      //construction des capteurs
      cap[i].set_offset(0);
      cap[i].set_pente(0);

    }

}

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

void Pneumatic7ArmRtThread::
init_muscle_i (controleur_axe *controleur_i, double * delta, double * vitesse)
{
  controleur_i -> initialisation_muscles(*delta,*vitesse);
  //msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}

void Pneumatic7ArmRtThread::
reset_muscle_i (controleur_axe *controleur_i,  double * vitesse)
{
  controleur_i -> degonfle(*vitesse);
  //signale a la tache principale la fin du degonflement des muscles
  //msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}

void Pneumatic7ArmRtThread::
trait_muscle_i (controleur_axe *controleur_i,
                double * ,//delta,
                double * vitesse)
{
  double vit = *vitesse;
  //const char * buf = std::string("ok").c_str();
  //char * buffer = new char[2 * sizeof(double) + 2];
  //double pos_joy,coef;
  if (!fin_)
    {
      //double del = *delta;
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

/*** SiGNAL catch  **/

void catch_signal(int
                  //sig
)
{

}

/********************************************************
 *							*
 *	init () -> Initializing						*
 *							*
 *	Creating and initializing all the objects.	*
 *							*
 ********************************************************/


void Pneumatic7ArmRtThread::Initializing()
{


  // Capture signals.
  string line;
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);
  //int man_pres;
  //double user_pressure;
  mlockall(MCL_CURRENT|MCL_FUTURE);

  // Initialize control flags.
  test1 = new test();
  test1 -> test_config();
  DEFAULT_FLAG= test1 -> get_DEFAULT_FLAG();
  CALIBRATION_FLAG = test1 -> get_CALIBRATION_FLAG();
  CONTROL_MODE_NOPRES_FLAG = test1 -> get_CONTROL_MODE_NOPRES_FLAG();
  CONTROL_MODE_PRES_FLAG = test1 -> get_CONTROL_MODE_PRES_FLAG();
  INFLATING_FLAG = test1 -> get_INFLATING_FLAG();
  PRES_INDIVIDUAL_FLAG = test1 -> get_PRES_INDIVIDUAL_FLAG();
  for(int lpctr =0; lpctr < 7; lpctr++)
    {
      CTRL_FLAG(lpctr) = test1 -> get_CTRL_FLAG(lpctr);
    }
  for(int lppres =0; lppres < 7; lppres++)
    {
      pressure_command_array_(lppres) =
        test1 -> get_pressure_command_array(lppres);
    }


  cout<< "Control flag after testconfig: "<<CTRL_FLAG(0) << endl;
  ODEBUG("Pressure array after testconfig: ");
  for(unsigned int i=0;i<7;i++)
    cout << pressure_command_array_(i) <<",";
  cout << endl;



  // Starting new connection with the NI module.
  clientUDP -> client_start();

  // Initializing ciodac16 and ciodac64 with the new UPD connection
  ciodac16_->get_client(clientUDP);
  ciodas64_->get_client(clientUDP);
  ciodas64_ -> openlogudpdata();

  for (int i = 0; i<7;i++)
    {
      // Linking sensors with the data IO boards
      cap[i].set_association(ciodas64_,i+16);
    }

  //printf("\n jusqu'ici tout va bien 1");
  //Actionneurs
  a1 = actionneur(VOIE_1_1,VOIE_1_2,ciodac16_);
  //printf("\n init()debug1 \n");
  a2 = actionneur(VOIE_2_1,VOIE_2_2,ciodac16_);
  a3 = actionneur(VOIE_3_1,VOIE_3_2,ciodac16_);
  a4 = actionneur(VOIE_4_1,VOIE_4_2,ciodac16_);
  a5 = actionneur(VOIE_5_1,VOIE_5_2,ciodac16_);
  a6 = actionneur(VOIE_6_1,VOIE_6_2,ciodac16_);
  a7 = actionneur(VOIE_7_1,VOIE_7_2,ciodac16_);

  //controleurs d'axe
  controleur1 = controleur_axe(joy1,&a1,1,
			       ANGLE_REPOS_1,ANGLE_MIN_1, ANGLE_MAX_1,SENS_CAPTEUR_1,SENS_PRESSION_1,P_AXE_1,D_AXE_1);
  controleur2 = controleur_axe(joy1,&a2,2,ANGLE_REPOS_2,ANGLE_MIN_2, ANGLE_MAX_2,SENS_CAPTEUR_2,SENS_PRESSION_2,P_AXE_2,D_AXE_2);
  controleur3 = controleur_axe(joy1,&a3,3,ANGLE_REPOS_3,ANGLE_MIN_3, ANGLE_MAX_3,SENS_CAPTEUR_3,SENS_PRESSION_3,P_AXE_3,D_AXE_3);
  controleur4 = controleur_axe(joy2,&a4,4,ANGLE_REPOS_4,ANGLE_MIN_4, ANGLE_MAX_4,SENS_CAPTEUR_4,SENS_PRESSION_4,P_AXE_4,D_AXE_4);
  controleur5 = controleur_axe(ppalonnier,&a5,5,ANGLE_REPOS_5,ANGLE_MIN_5, ANGLE_MAX_5,SENS_CAPTEUR_5,SENS_PRESSION_5,P_AXE_5,D_AXE_5);
  controleur6 = controleur_axe(joy2,&a6,6,ANGLE_REPOS_6,ANGLE_MIN_6, ANGLE_MAX_6,SENS_CAPTEUR_6,SENS_PRESSION_6,P_AXE_6,D_AXE_6);
  controleur7 = controleur_axe(joy2,&a7,7,ANGLE_REPOS_7,ANGLE_MIN_7, ANGLE_MAX_7,SENS_CAPTEUR_7,SENS_PRESSION_7,P_AXE_7,D_AXE_7);

  //printf("\n init()debug7 \n");
  //Initialisation des controleurs
  controleur1.initialisation_carte();
  controleur2.initialisation_carte();
  controleur3.initialisation_carte();
  controleur4.initialisation_carte();
  controleur5.initialisation_carte();
  controleur6.initialisation_carte();
  controleur7.initialisation_carte();
  //printf("\n init()debug8 \n");
  ciodac16_ -> daconv(1, '0'); //start the NI module to send data
  //printf("\n init()debug8.1 \n");

  ciodas64_ -> adconv(1);
  //printf("\n init()debug8.2\n");
  //cout << "/n init() recv data:adconv:" << recving_Data << endl;
  //char header = '1';

  ciodac16_ -> daconv(1, '1');

  //printf("\n init()debug8.3\n");



  //Association des controleur aux capteurs correspondants
  ciodas64_ -> adconv(1);
  //cout << "/n init() recv data:adconv:" << recving_Data << endl;
  //printf("\n init()debug9 \n");
  controleur1.set_capteur(cap+4);
  //printf("\n init()debug10 \n");
  controleur2.set_capteur(cap+2);
  //printf("\n init()debug11 \n");
  controleur3.set_capteur(cap+6);
  //printf("\n init()debug12 \n");
  controleur4.set_capteur(cap);
  controleur5.set_capteur(cap+1);
  controleur6.set_capteur(cap+3);
  controleur7.set_capteur(cap+5);
  //printf("\n init()debug13 \n");
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

void Pneumatic7ArmRtThread::InitializeSensors ()
{
  //printf("\n inside init_capteurs()1 \n");
  char header = '1';

  ciodac16_ -> daconv(1, header);
  ciodas64_ -> adconv(1);

  for (int i = 1;i < 8;i++)
    {
      cap[i-1].set_offset( cap[i-1].read_sensors_array(i) );
    }
  //  typing is difficult
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

  ciodac16_ -> daconv(1, header);
}

/********************************************************
 *							*
 *	 Inflating()					*
 *                                                      *
 * Simultaneously inflating all the muscles.            *
 *							*
 ********************************************************/

void Pneumatic7ArmRtThread::
Inflating(void)
{
  double  * vit,*d1,*d2,*d3,*d4,*d5,*d6,*d7;
  vit = new double (VITESSE_PRESSION);

  d1 = new double (DELTA_INIT_AXE_1);
  d2 = new double (DELTA_INIT_AXE_2);
  d3 = new double (DELTA_INIT_AXE_3);
  d4 = new double (DELTA_INIT_AXE_4);
  d5 = new double (DELTA_INIT_AXE_5);
  d6 = new double (DELTA_INIT_AXE_6);
  d7 = new double (DELTA_INIT_AXE_7);


  //Lancement en parallele des taches d'initialisation des muscles
  pneumatic_muscle.init_muscle_i(&controleur1, d1, vit);
  init_muscle_i(&controleur1, d1, vit);
  init_muscle_i(&controleur2, d2, vit);
  init_muscle_i(&controleur3, d3, vit);
  init_muscle_i(&controleur4, d4, vit);
  init_muscle_i(&controleur5, d5, vit);
  init_muscle_i(&controleur6, d6, vit);
  init_muscle_i(&controleur7, d7, vit);
  //init_muscle_i(&controleur2, d1, vit);

  char header = '1';

  ciodac16_ -> daconv(1, header);
  ciodas64_ -> adconv(1);
  //tele_op = true;


}

/********************************************************
 *							*
 *	Deflating ()					*
 *							*
 * Deflating all the muscles.                           *
 *							*
 ********************************************************/

void Pneumatic7ArmRtThread::
Deflating(void)
{

  //Variables locales
  double  * vit;
  vit = new double (VITESSE_PRESSION);

  //Lancement en parallele des taches de degonflement des muscles
  reset_muscle_i(&controleur1, vit);
  reset_muscle_i(&controleur2, vit);
  reset_muscle_i(&controleur3, vit);
  reset_muscle_i(&controleur4, vit);
  reset_muscle_i(&controleur5, vit);
  reset_muscle_i(&controleur6, vit);
  reset_muscle_i(&controleur7, vit);
  char header = '1';

  ciodac16_ -> daconv(1, header);
  ciodas64_ -> adconv(1);

}

/********************************************************
 *							*
 *	Calibration ()					*
 *							*
 *
 ********************************************************/
void Pneumatic7ArmRtThread::
Calibration()
{

  sensors_array_(0) = controleur1.get_angle_lire();
  sensors_array_(1) = controleur2.get_angle_lire();
  sensors_array_(2) = controleur3.get_angle_lire();
  sensors_array_(3) = controleur4.get_angle_lire();
  sensors_array_(4) = controleur5.get_angle_lire();
  sensors_array_(5) = controleur6.get_angle_lire();
  sensors_array_(6) = controleur7.get_angle_lire();
  //cout << "\n received sensors data : " << endl;
  for(int loop_sensors_array_ = 0; loop_sensors_array_ <7; loop_sensors_array_++)
    {
      cout << sensors_array_(loop_sensors_array_) << endl;

    }
  for(unsigned int i=0;i<7;i++)
    sensorlog_ << sensors_array_(i) << "\t";
  sensorlog_ << endl;
}

void Pneumatic7ArmRtThread::
ReferenceGenerator()
{
  controleur1.get_reference_angle(COEF_LENT, VITESSE_ANGLE);
  controleur2.get_reference_angle(COEF_LENT, VITESSE_ANGLE);
  controleur3.get_reference_angle(COEF_LENT, VITESSE_ANGLE);
  controleur4.get_reference_angle(COEF_LENT, VITESSE_ANGLE);
  controleur5.get_reference_angle(COEF_LENT, VITESSE_ANGLE);
  controleur6.get_reference_angle(COEF_LENT, VITESSE_ANGLE);
  controleur7.get_reference_angle(COEF_LENT, VITESSE_ANGLE);
}
/********************************************************
 *							*
 *	controler ()					*
 *							*
 *methode associee a la tache de controle des axes du   *
 *robot							*
 ********************************************************/

void Pneumatic7ArmRtThread::
Controler()
{


  //printf("\n jusqu'ici tout va bien 13 controler");

  double  * vit;
  vit = new double (VITESSE_PRESSION);
  //Lancement en parallele des taches de controle des axes

  ciodas64_ -> adconv(1);
  Calibration();
  ciodas64_ -> logudpdata();
  ReferenceGenerator();
  /*Add here all 7 axis control*/
  if(CTRL_FLAG(0)==1)
    {
      controleur1.set_loop(loop_);
      //cout <<  "\n loop_" << loop_ << endl;
      if(loop_ == 2)
	{
	  controleur1.set_userpressure(pressure_command_array_(0));
	  //cout << "inside loop_  = 2" << endl;
	}
      control_command_ = controleur1.get_commande();
      angle_read = controleur1.get_angle_reel();
      //cout << "controlcommand1: " << control_command_ << endl;
      //cout << "\n angle read CTRL_FLAG 1 :" << angle_read << endl;
      trait_muscle_i(&controleur1, &control_command_, vit);
    }
  if(CTRL_FLAG(1)==1)
    {
      controleur2.set_loop(loop_);
      if(loop_ == 2)
	{
	  controleur2.set_userpressure(pressure_command_array_(1));
	}

      control_command_ = controleur2.get_commande();
      angle_read = controleur2.get_angle_reel();
      trait_muscle_i(&controleur2, &control_command_, vit);
    }
  if(CTRL_FLAG(2)==1)
    {
      controleur3.set_loop(loop_);
      if(loop_ == 2)
	{
	  controleur3.set_userpressure(pressure_command_array_(2));
	}

      control_command_ = controleur3.get_commande();
      //angle_read = controleur3.get_angle_reel();
      trait_muscle_i(&controleur3, &control_command_, vit);
    }
  if(CTRL_FLAG(3)==1)
    {
      controleur4.set_loop(loop_);
      if(loop_ == 2)
	{
	  controleur4.set_userpressure(pressure_command_array_(3));
	}
      control_command_ = controleur4.get_commande();
      angle_read = controleur4.get_angle_reel();
      cout << "controlcommand4: " << control_command_ << endl;
      trait_muscle_i(&controleur4, &control_command_, vit);
    }
  if(CTRL_FLAG(4)==1)
    {
      controleur5.set_loop(loop_);
      if(loop_ == 2)
	{
	  controleur5.set_userpressure(pressure_command_array_(4));
	}
      control_command_ = controleur5.get_commande();
      angle_read = controleur5.get_angle_reel();
      trait_muscle_i(&controleur5, &control_command_, vit);
    }
  if(CTRL_FLAG(5)==1)
    {
      controleur6.set_loop(loop_);
      if(loop_ == 2)
	{
	  controleur6.set_userpressure(pressure_command_array_(5));
	}
      control_command_ = controleur6.get_commande();
      angle_read = controleur6.get_angle_reel();
      trait_muscle_i(&controleur6, &control_command_, vit);
    }
  if(CTRL_FLAG(6)==1)
    {
      controleur7.set_loop(loop_);
      if(loop_ == 2)
	{
	  controleur7.set_userpressure(pressure_command_array_(6));
	}
      control_command_ = controleur7.get_commande();
      angle_read = controleur7.get_angle_reel();
      trait_muscle_i(&controleur7, &control_command_, vit);
    }

  //std:: cout << "\n Angle read position " << angle_read << endl;

  ciodac16_ -> daconv(1, '1');


  //printf("\n jusqu'ici tout va bien 2 control\n");


  //printf("\n jusqu'ici tout va bien control\n");

}


/********************************************************
 *							*
 *	controler_robot ()				*
 *							*
 *routine appelee par la tache principale main qui lance*
 *les deux taches de controle et d'attente clavier	*
 ********************************************************/
void Pneumatic7ArmRtThread::
RobotControler()
{
  Controler();
}

void principale(void *arg)
{
  Pneumatic7ArmRtThread * aPneumaticArm = static_cast<Pneumatic7ArmRtThread *> (arg);
  if (aPneumaticArm!=0)
    aPneumaticArm->PrincipalTask();
}

//udppacket_control send_packet;
/********************************************************
 *							*
 *	principale () = tache principale		*
 *							*
 *routine appelee par debut() apres
 *l initialisation					*
 ********************************************************/
void Pneumatic7ArmRtThread::PrincipalTask ()
{
  //variables locales
  //bool bonne_saisie = false,ok1 = false,ok2 = false,ok3 =false;
  bool ok3 =false;
  //char * fich = new char [40];
  //char * commencer = new char [1];
  //char * cont2 = new char [1];
  //char * cont1 = new char [1];
  char * tmp = new char [1];
  //double user_pressure;

  /* variables used in the principal program */
  //int whileloop_counter = 0, error_counter = 0, loop = 0;
  int timeofsimulation_s = 30; /* time in seconds*/
  int FLAG = 1;

  RTIME   now, previous=0,  time_diff, TASK_PERIOD = 1.0e8;//1000000; ..present,
  double t, time_start_loop, present_time;
  //ciodac16_ -> client_start();
  //udppacket_control send_packet;

  sensorlog_.open("sensorlog.txt");
  //int i = 0;
  rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
  if(CALIBRATION_FLAG == 1)
    {
      cout << "\n CALIBRATION MODE ON ...... \n" << endl;
    }
  if(INFLATING_FLAG == 1)
    {
      printf("\n ..... INFLATING THE MUSCLES   .....");

      //Appel a la fonction de gonflement des muscles
      Inflating();

      sleep(5);
      printf("\n ..... INFLATING should be completed  .....");
    }
  if(PRES_INDIVIDUAL_FLAG == 1)
    {
      int index_pres_indiv;
      double pres_pres_indiv;
      char depres;
      cout << "\n Select the muscle number: "<< endl;
      std::cin >> index_pres_indiv; //scanf("%s",tmp);
      cout << "\n Enter the pressure value: " << endl;
      std::cin >> pres_pres_indiv;
      std::cin.clear(); std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
      ciodac16_ -> pressure_inidividualmuscle(index_pres_indiv, pres_pres_indiv);
      printf("\n Type Any letter to depressurize : ");
      std::cin >> depres; //scanf("%s",tmp);
      std::cin.clear(); std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
      Deflating();
      timeofsimulation_s = 2; /* time in seconds*/
    }

  if(CONTROL_MODE_PRES_FLAG == 1 || CONTROL_MODE_NOPRES_FLAG == 1)
    {
      printf("\n ..... CONTROL MODE Begins   .....");

      printf("\n Type o for OPEN LOOP control, or f for CLOSED LOOP conttrol, or p for open loop pressure command and confirm (ok3, tmp) : ");
      std::cin >> tmp; //scanf("%s",tmp);

      std::cin.clear(); std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
      if (strcmp(tmp,"o")==0) {loop_=BOUCLE_OUVERTE;ok3=true;}
      if (strcmp(tmp,"f")==0) {loop_=BOUCLE_FERMEE;ok3=true;}
      if (strcmp(tmp,"p")==0)  {loop_=BOUCLE_PRESCMD; cout << "loop_ pressure is set :"  << loop_;ok3=true;}
    }


  //printf(" \n APPLY THE PRESSURE   \n");

  now = rt_timer_read();
  time_start_loop  = round((double)now/1.0e9);

  InitializeSensors();

  while (FLAG)
    {
      rt_task_wait_period(NULL);


      //controleur_pince.initialiser();

      now = rt_timer_read();
      present_time  = round((double)now/1.0e9);
      t = present_time - time_start_loop;
      time_diff = now - previous;
      cout << "\n time difference :" << (double)time_diff/(double)1.0e6 << endl;
      if(CALIBRATION_FLAG == 1)
	{
	  ciodas64_ -> adconv(1);
	  Calibration();
	}

      if(CONTROL_MODE_NOPRES_FLAG == 1 || CONTROL_MODE_PRES_FLAG == 1)
	{
	  RobotControler();
	}
      if(INFLATING_FLAG == 1 && CONTROL_MODE_PRES_FLAG == 0 && CONTROL_MODE_NOPRES_FLAG == 0)
	{
	  cout << "\n You have Inflated the muscles!" << endl;
	}

      previous = now;
      //cout << "\n the time past is : " << t;
      if(t >= timeofsimulation_s)
	{
	  FLAG = 0;
	  //printf("\n Task completion: \n\n\n");
	  cout << "\n END of Loop";

	}
    } //while (ok2) finish
  sensorlog_.close();

  // terminaison

  cout << "outside while loop ok2" << endl ;
  printf("\n ..... DEFLATING THE MUSCLES   .....");

  Deflating();
  sleep(5);
  printf("\n ..... DEFLATING should be completed  .....");
  //Appel a la fonction de degonflement des muscles

  //Remise a faux de variables booleennes
  fin_ = false;
  tele_op_ = false;
  sortie_ = false;

  //ciodac16_ -> ~ClientUDP();

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

  ciodac16_->~CIODAC16();
  ciodas64_->~CIODAS64();


  for (int i = 0; i < 7;i++)
    cap[i].~capteur_position();

  printf("\n    ====== PROGRAM FINISHED ======    \n\n");
  printf("\n .... ELectronics is reset ...\n");
} //principale finish

void Pneumatic7ArmRtThread::StartingRealTimeThread()
{
  int n;
  printf("\n");
  printf("	*****************************************************************\n");
  printf("	*								                      *\n");
  printf("	*		        CONTORL SOFTWARE  		  *\n");
  printf("	*               for the 7 DOFs ARM			                      *\n");
  printf("	*			                                                      		*\n");
  printf("	*****************************************************************\n");
  printf("\n\n\n");
  for(unsigned int i=0;i<7;i++)
    cout << pressure_command_array_(i) <<",";
  cout << endl;

  n = rt_task_create(&principal_task_, "principal_function", 0, 99, 0);
  if (n!=0)
    {
      cout << "Failed @ RT Create" << n <<endl;
    }
  else cout << "END of RT Create" << endl;

  n = rt_task_start(&principal_task_, &principale, this);
  if (n!=0)
    {
      cout << "Failed of RT STart" <<n<< endl;
    }
  else cout << "END of RT Start" << endl;

  pause();

  cout << "END of Pause" << endl;

  n = rt_task_delete(&principal_task_);
  if(n!=0)cout << "Failed of RT Task delete" << endl;
  else cout << "END of RT taslk delete";

}


/********************************************************
 *							*
 *	depart ()					*
 *							*
 *	tache init du programme				*
 *							*
 ****************************************************** */

int main(void)
{


}
