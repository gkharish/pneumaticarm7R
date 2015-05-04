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
#define DEBUG_LEVEL 0
#define ODEBUG(x) std::cerr << x << std::endl
#define ODEBUGL(x,y) if (y<DEBUG_LEVEL) std::cerr << x << std::endl;
#else
#define ODEBUG(x)
#define ODEBUGL(x,y);
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
#include "controller_axis.hh"
#include "controller_tool.hh"
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
  joy1_(0), joy2_(0),ppalonnier_(0),
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
      sensors_[i].set_offset(0);
      sensors_[i].set_pente(0);

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
init_muscle_i (controller_axis *controleur_i, double * delta, double * vitesse)
{
  controleur_i -> initialisation_muscles(*delta,*vitesse);
  //msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}

void Pneumatic7ArmRtThread::
reset_muscle_i (controller_axis *controleur_i,  double * vitesse)
{
  controleur_i -> degonfle(*vitesse);
  //signale a la tache principale la fin du degonflement des muscles
  //msgQSend(*msgq_i,"ok",2,WAIT_FOREVER,MSG_PRI_NORMAL);
}

void Pneumatic7ArmRtThread::
trait_muscle_i (controller_axis *controleur_i,
                double * delta,
                double * vitesse)
{
  if ((delta==0) || (vitesse==0))
    return;
  //const char * buf = std::string("ok").c_str();
  //char * buffer = new char[2 * sizeof(double) + 2];
  //double pos_joy,coef;
  if (!fin_)
    {
      //double del = *delta;
      controleur_i -> initialisation_muscles(*delta,*vitesse);
      /*if (!tele_op)
	{
	double del = *delta;
	controleur_i -> initialisation_muscles(del,vit);
	//msgQSend(msgq2,buf,2,WAIT_FOREVER,MSG_PRI_NORMAL);
	}*/

    }
  else
    {
      controleur_i -> degonfle(*vitesse);
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

void Pneumatic7ArmRtThread::InitActuators()
{
  int channel1[7] = {VOIE_1_1, VOIE_2_1,  VOIE_3_1, VOIE_4_1, VOIE_5_1, VOIE_6_1, VOIE_7_1};
  int channel2[7] = {VOIE_1_2, VOIE_2_2,  VOIE_3_2, VOIE_4_2, VOIE_5_2, VOIE_6_2, VOIE_7_2};
  
  for (unsigned int i=0;i<7;i++)
    actuators_[i] = actionneur(channel1[i],channel2[i],ciodac16_);

}

void Pneumatic7ArmRtThread::InitControllers()
{
  double rest_angles[7] = { ANGLE_REPOS_1, ANGLE_REPOS_2, ANGLE_REPOS_3, ANGLE_REPOS_4, ANGLE_REPOS_5, ANGLE_REPOS_6, ANGLE_REPOS_7};
  double min_angles[7] = { ANGLE_MIN_1, ANGLE_MIN_2, ANGLE_MIN_3, ANGLE_MIN_4, ANGLE_MIN_5, ANGLE_MIN_6, ANGLE_MIN_7};
  double max_angles[7] = { ANGLE_MAX_1, ANGLE_MAX_2, ANGLE_MAX_3, ANGLE_MAX_4, ANGLE_MAX_5, ANGLE_MAX_6, ANGLE_MAX_7};
  int sensor_directions[7] = {SENS_CAPTEUR_1,SENS_CAPTEUR_2,SENS_CAPTEUR_3,SENS_CAPTEUR_4,SENS_CAPTEUR_5,SENS_CAPTEUR_6,SENS_CAPTEUR_7};
  int pressure_directions[7] = {SENS_PRESSION_1,SENS_PRESSION_2,SENS_PRESSION_3,SENS_PRESSION_4,SENS_PRESSION_5,SENS_PRESSION_6,SENS_PRESSION_7};
  double p_gains[7] = {P_AXE_1,P_AXE_2,P_AXE_3,P_AXE_4,P_AXE_5,P_AXE_6,P_AXE_7};
  double d_gains[7] = {D_AXE_1,D_AXE_2,D_AXE_3,D_AXE_4,D_AXE_5,D_AXE_6,D_AXE_7};
  I_teleop * joys[7] = {joy1_, joy1_, joy1_, joy2_, ppalonnier_,joy2_,joy2_};

  for(unsigned int i=0;i<7;i++)
    {
      // Bind actuators and controllers
      controllers_[i]=controller_axis(joys[i],& actuators_[i], i,
                                      rest_angles[i], min_angles[i],max_angles[i],
                                      sensor_directions[i], pressure_directions[i], p_gains[i],d_gains[i]);
      // Initialize electronic boards
      controllers_[i].initialisation_carte();
    }

  ODEBUGL("\n init()debug8 \n",3);
  //start the NI module to send data
  ciodac16_ -> daconv(1, '0'); 
  ODEBUGL("\n init()debug8.1 \n",3);

  ciodas64_ -> adconv(1);
  ODEBUGL("\n init()debug8.2\n",3);
  ODEBUGL("/n init() recv data:adconv:" ,4);

  ciodac16_ -> daconv(1, '1');
  ODEBUGL("\n init()debug8.3\n",3);

  // Bind controllers and sensors 
  ciodas64_ -> adconv(1);
  ODEBUGL("/n init() recv data:adconv:" ,4);
  ODEBUGL("\n init()debug9 \n",3);
  controllers_[0].set_capteur(sensors_+4);
  ODEBUGL("\n init()debug10 \n",3);
  controllers_[1].set_capteur(sensors_+2);
  ODEBUGL("\n init()debug11 \n",3);
  controllers_[2].set_capteur(sensors_+6);
  ODEBUGL("\n init()debug12 \n",3);
  controllers_[3].set_capteur(sensors_);
  controllers_[4].set_capteur(sensors_+1);
  controllers_[5].set_capteur(sensors_+3);
  controllers_[6].set_capteur(sensors_+5);
}

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
      sensors_[i].set_association(ciodas64_,i+16);
    }

  // Initialize actuators
  InitActuators();

  // Init axis controllers.
  InitControllers();

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
  ODEBUGL("\n inside init_capteurs()1 \n",3);
  char header = '1';

  ciodac16_ -> daconv(1, header);
  ciodas64_ -> adconv(1);

  for (int i = 1;i < 8;i++)
    sensors_[i-1].set_offset( sensors_[i-1].read_sensors_array(i) );

  ODEBUGL("\n inside init_capteurs()2 \n",3);
  for(unsigned int i=0;i<7;i++)
    controllers_[i].init_angles();

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
  double  vit=VITESSE_PRESSION;
  double d[7] = {DELTA_INIT_AXE_1,
                 DELTA_INIT_AXE_2,
                 DELTA_INIT_AXE_3,
                 DELTA_INIT_AXE_4,
                 DELTA_INIT_AXE_5,
                 DELTA_INIT_AXE_6,
                 DELTA_INIT_AXE_7};

  // Initialize muscles
  for(unsigned int motorID=0;motorID<7;motorID++)
    init_muscle_i(&controllers_[motorID], d+motorID, &vit);

  char header = '1';

  ciodac16_ -> daconv(1, header);
  ciodas64_ -> adconv(1);
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
  double  vit = VITESSE_PRESSION;

  // Lancement en parallele des taches de degonflement des muscles
  for(unsigned int motorID=0;motorID<7;motorID++)
    reset_muscle_i(&controllers_[motorID], &vit);

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
  for(unsigned int i=0;i<7;i++)
    sensors_array_(i) = controllers_[i].get_angle_lire();

  //cout << "\n received sensors data : " << endl;
  for(int loop_sensors_array_ = 0; loop_sensors_array_ <7; loop_sensors_array_++)
    cout << sensors_array_(loop_sensors_array_) << endl;
  
  for(unsigned int i=0;i<7;i++)
    sensorlog_ << sensors_array_(i) << "\t";
  sensorlog_ << endl;
}

void Pneumatic7ArmRtThread::
ReferenceGenerator()
{
  for(unsigned int i=0;i<7;i++)
    controllers_[i].get_reference_angle(COEF_LENT, VITESSE_ANGLE);
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


  ODEBUGL("\n jusqu'ici tout va bien 13 controler",3);

  double  vit = VITESSE_PRESSION;
  //Lancement en parallele des taches de controle des axes

  ciodas64_ -> adconv(1);
  Calibration();
  ciodas64_ -> logudpdata();
  ReferenceGenerator();

  /* Add here all 7 axis control*/
  for(unsigned int motorID=0;motorID<7;motorID++)
    {
      if(CTRL_FLAG(motorID)==1)
        {
          controllers_[motorID].set_loop(loop_);
          ODEBUGL("\n loop_" << loop_ ,3);
          if(loop_ == 2)
            {
              controllers_[motorID].set_userpressure(pressure_command_array_(motorID));
              ODEBUGL("inside loop_  = 2",3);
            }
          control_command_ = controllers_[motorID].get_commande();
          angle_read = controllers_[motorID].get_angle_reel();
          ODEBUGL("controlcommand1: " << control_command_ ,3);
          ODEBUGL("\n angle read CTRL_FLAG 1 :" << angle_read ,3);
          trait_muscle_i(&controllers_[motorID], &control_command_, &vit);
        }
    }

  ODEBUGL("\n Angle read position " << angle_read,3);

  ciodac16_ -> daconv(1, '1');

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
  char tmp ;
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
      if (strcmp(&tmp,"o")==0) {loop_=BOUCLE_OUVERTE;ok3=true;}
      if (strcmp(&tmp,"f")==0) {loop_=BOUCLE_FERMEE;ok3=true;}
      if (strcmp(&tmp,"p")==0)  {loop_=BOUCLE_PRESCMD; cout << "loop_ pressure is set :"  << loop_;ok3=true;}
    }


  //printf(" \n APPLY THE PRESSURE   \n");

  now = rt_timer_read();
  time_start_loop  = round((double)now/1.0e9);

  InitializeSensors();

  while (FLAG)
    {
      rt_task_wait_period(NULL);


      //controller_gripper.initialiser();

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

  // Calling controllers destructors
  for(unsigned int i=0;i<7;i++)
    controllers_[i].~controller_axis();
  
  // Calling actuators destructors.
  for(unsigned int i=0;i<7;i++)
    actuators_[i].~actionneur();

  controller_gripper_.~controller_tool();

  joy1_->~I_teleop();
  joy2_->~I_teleop();
  ppalonnier_->~I_teleop();

  ciodac16_->~CIODAC16();
  ciodas64_->~CIODAS64();


  for (int i = 0; i < 7;i++)
    sensors_[i].~capteur_position();

  printf("\n    ====== PROGRAM FINISHED ======    \n\n");
  printf("\n .... Electronics is reset ...\n");
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
