/************************************************************
 *	       PROGRAMME DE TEST DE LA REALISATION	    *
 *		     DU CONTROLEUR DE ROBOT	            *
 ************************************************************
 * Cree par : Erwan GUIOCHET/ Mehdi Seffar / JEREMIE GUIOCHET*
 * Date     : 02/07/2002 				    *
 * Derniere modification de M.Seffar: 20/09/2002            *
 * REPRIS PAR J. GUIOCHET le 15/12/2002			    *
 * VERSION                                                  *
 * Modified by G. Kumar and O. Stasse in 2015               *
 ************************************************************/

/*MODIFICATIONS
  17/12/2002: AJOUT DES RAPPORTS MECANIQUES POUR LES sensorS DE POSITION
  20/12/2002: AJOUT Du choix boucle ouverte boucle fermee.
*/


#include <debug.hh>

/***** DEFINITION DE L'ADRESSAGE DES ioboardsS *****/
/*         BASE_REG_CIODAC16 : CIO-DAC16-I 	*/
/*         BASE_REG_CIODAS64 : CIO-DAS6402/16   */
/************************************************/
#define BASE_REG_CIODAC16	0x200
#define BASE_REG_CIODAS64	0x250

/***** DEFINITION DES INFOS CONCERNANT LA GACHETTE *****/
#define VOIE_PINCE_1        14   // sorties de la ioboards de commande
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

//Outputports  of actuators of output ioboards (CIODAC16)
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

//Ports d'entree des joysticks et du palonnier sur la ioboards d'acquisition (CIODAS64)
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


// Rapports mecaniques entre rotation des axes des sensors et rotation reelle des articulations
// Determines par une mesure de la tension avec l articulation a 90 degres K=90/(Vmes*360/5)
#define RAP_MECA_CAP_1    1.5
#define RAP_MECA_CAP_2    1
#define RAP_MECA_CAP_3    1
#define RAP_MECA_CAP_4    1
#define RAP_MECA_CAP_5    2
#define RAP_MECA_CAP_6    1
#define RAP_MECA_CAP_7    1


//sens de rotation des sensors par rapport au sens theorique
#define SENS_sensor_1   1
#define SENS_sensor_2   0
#define SENS_sensor_3   0
#define SENS_sensor_4   0
#define SENS_sensor_5   0
#define SENS_sensor_6   1
#define SENS_sensor_7   0

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



/****** SYSTEM INCLUDES ******/
#include <iostream>
#include <limits>
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <istream>
#include <math.h>


/****** REAL TIME INCLUDES *****/
#include <vxworks/vxworks.h>

#define MODULE_LICENSE(x)
#include <native/task.h>
#include <native/timer.h>


/***** FRAMEWORK INCLUDES ******/
#include "ioboards.hh"
#include "actuator.hh"
#include "I_teleop.h"
#include "joystick.h"
#include "sensor.hh"
#include "position_sensor.hh"
#include "controller_axis.hh"
#include "controller_tool.hh"
#include "fichier.h"
#include "modele.h"
#include "clientudp3.hh"
#include "test_config.hh"
#include "shared_memory.hh"

#include <pneumatic_7arm_rt_thread.hh>

#define LOG_FILENAME "/var/log/pneumatic_arm.shm"

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
  CONTROL_MODE_NOPRES_FLAG(0),
  CONTROL_MODE_PRES_FLAG(0),
  INFLATING_FLAG(0),
  PRES_INDIVIDUAL_FLAG(0),
  timeofsimulation(10),
  controllers_(7),
  recving_Data_(15),
  CTRL_FLAG(7),
  pressure_command_array_(7),
  sensors_array_(7),
  actuators_(7),
  joy1_(0), joy2_(0),ppalonnier_(0),
  shmaddr_(0),
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

      //construction des sensors
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
}

void Pneumatic7ArmRtThread::
reset_muscle_i (controller_axis *controleur_i,  double * vitesse)
{
  controleur_i -> degonfle(*vitesse);
}

void Pneumatic7ArmRtThread::
trait_muscle_i (controller_axis *controleur_i,
                double * delta,
                double * vitesse)
{
  if ((delta==0) || (vitesse==0))
    return;
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
    actuators_[i] = new Actuator(channel1[i],channel2[i],ciodac16_);

}

void Pneumatic7ArmRtThread::InitControllers()
{
  double rest_angles[7] = { ANGLE_REPOS_1, ANGLE_REPOS_2, ANGLE_REPOS_3, ANGLE_REPOS_4, ANGLE_REPOS_5, ANGLE_REPOS_6, ANGLE_REPOS_7};
  double min_angles[7] = { ANGLE_MIN_1, ANGLE_MIN_2, ANGLE_MIN_3, ANGLE_MIN_4, ANGLE_MIN_5, ANGLE_MIN_6, ANGLE_MIN_7};
  double max_angles[7] = { ANGLE_MAX_1, ANGLE_MAX_2, ANGLE_MAX_3, ANGLE_MAX_4, ANGLE_MAX_5, ANGLE_MAX_6, ANGLE_MAX_7};
  int sensor_directions[7] = {SENS_sensor_1,SENS_sensor_2,SENS_sensor_3,SENS_sensor_4,SENS_sensor_5,SENS_sensor_6,SENS_sensor_7};
  int pressure_directions[7] = {SENS_PRESSION_1,SENS_PRESSION_2,SENS_PRESSION_3,SENS_PRESSION_4,SENS_PRESSION_5,SENS_PRESSION_6,SENS_PRESSION_7};
  double p_gains[7] = {P_AXE_1,P_AXE_2,P_AXE_3,P_AXE_4,P_AXE_5,P_AXE_6,P_AXE_7};
  double d_gains[7] = {D_AXE_1,D_AXE_2,D_AXE_3,D_AXE_4,D_AXE_5,D_AXE_6,D_AXE_7};
  I_teleop * joys[7] = {joy1_, joy1_, joy1_, joy2_, ppalonnier_,joy2_,joy2_};

  for(unsigned int i=0;i<7;i++)
    {
      // Bind actuators and controllers
      controllers_[i]=new controller_axis(joys[i],actuators_[i], i,
					  rest_angles[i], min_angles[i],max_angles[i],
					  sensor_directions[i], pressure_directions[i], p_gains[i],d_gains[i]);
      // Initialize electronic boards
      controllers_[i]->initialisation_ioboards();
    }


  //start the NI module to send data
  ciodac16_ -> daconv(1, '0');
  ciodas64_ -> adconv(1);
  ciodac16_ -> daconv(1, '1');

  // Bind controllers and sensors
  ciodas64_ -> adconv(1);
  controllers_[0]->set_sensor(sensors_+4);
  controllers_[1]->set_sensor(sensors_+2);
  controllers_[2]->set_sensor(sensors_+6);
  controllers_[3]->set_sensor(sensors_);
  controllers_[4]->set_sensor(sensors_+1);
  controllers_[5]->set_sensor(sensors_+3);
  controllers_[6]->set_sensor(sensors_+5);
}

void Pneumatic7ArmRtThread::Initializing()
{


  // Capture signals.
  string line;
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);
  //int man_pres;
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
  timeofsimulation=test1 -> get_timeofsimulation();
  for(int lpctr =0; lpctr < 7; lpctr++)
    {
      CTRL_FLAG(lpctr) = test1 -> get_CTRL_FLAG(lpctr);
    }
  for(int lppres =0; lppres < 7; lppres++)
    {
      pressure_command_array_(lppres) =
        test1 -> get_pressure_command_array(lppres);
    }


  ODEBUGL("Control flag after testconfig: "<<CTRL_FLAG(0),3);
  ODEBUGL("Pressure array after testconfig: ",1);
#ifndef NDEBUG  
#if DEBUG_LEVEL > 2
  for(unsigned int i=0;i<7;i++)
    cout << pressure_command_array_(i) <<",";
  cout << endl;
#endif
#endif 

  // Starting new connection with the NI module.
  clientUDP -> client_start();
  ODEBUGL("After client start",4);

  // Initializing ciodac16 and ciodac64 with the new UPD connection
  ciodac16_->get_client(clientUDP);
  ciodas64_->get_client(clientUDP);
  ciodas64_ -> openlogudpdata();
  ODEBUGL("After IODAC initialization",4);
  
  for (int i = 0; i<7;i++)
    {
      // Linking sensors with the data IO boards
      sensors_[i].set_association(ciodas64_,i+16);
    }

  // Initialize actuators
  InitActuators();
  ODEBUGL("After Actuator initialization",4);

  // Init axis controllers.
  InitControllers();
  ODEBUGL("After controllers initialization",4);

  // Init shared memory
  CreateSharedMemory();
}

/********************************************************
 *							*
 *	 init_sensors()				*
 *							*
 *	initialisations des offsets des sensors	*
 *							*
 *							*
 ********************************************************/

void Pneumatic7ArmRtThread::InitializeSensors ()
{
  ODEBUGL("\n inside init_sensors()1 \n",3);
  char header = '1';

  ciodac16_ -> daconv(1, header);
  ciodas64_ -> adconv(1);

  for (int i = 1;i < 8;i++)
    sensors_[i-1].set_offset( sensors_[i-1].read_sensors_array(i) );

  ODEBUGL("\n inside init_sensors()2 \n",3);
  for(unsigned int i=0;i<7;i++)
    controllers_[i]->init_angles();

  //printf("\n Done init_sensors() \n");

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
    init_muscle_i(controllers_[motorID], d+motorID, &vit);

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
    reset_muscle_i(controllers_[motorID], &vit);

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
    sensors_array_(i) = controllers_[i]->get_angle_lire();

  ODEBUG("\n received sensors data : ");
#ifndef NDEBUG 
#if DEBUG_LEVEL<1
  for(int loop_sensors_array_ = 0; loop_sensors_array_ <7; loop_sensors_array_++)
    cout << sensors_array_(loop_sensors_array_) << endl;
#endif /* DEBUG_LEVEL <1 */
#endif

  for(unsigned int i=0;i<7;i++)
    sensorlog_ << sensors_array_(i) << "\t";
  sensorlog_ << endl;
}

void Pneumatic7ArmRtThread::
ReferenceGenerator()
{
  for(unsigned int i=0;i<7;i++)
    controllers_[i]->get_reference_angle(COEF_LENT, VITESSE_ANGLE);
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
          controllers_[motorID]->set_loop(loop_);
          ODEBUGL("\n loop_" << loop_ ,3);
          if(loop_ == 2)
            {
              controllers_[motorID]->set_userpressure(pressure_command_array_(motorID));
              ODEBUGL("inside loop_  = 2",3);
            }
          control_command_ = controllers_[motorID]->get_commande();
          angle_read = controllers_[motorID]->get_angle_reel();
          ODEBUGL("controlcommand1: " << control_command_ ,3);
          ODEBUGL("\n angle read CTRL_FLAG 1 :" << angle_read ,3);
          trait_muscle_i(controllers_[motorID], &control_command_, &vit);
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
 ********************************************************/
void Pneumatic7ArmRtThread::PrincipalTask ()
{

  /* variables used in the principal program */
  int FLAG = 1;

  RTIME   now, previous=0,  time_diff, TASK_PERIOD = 1.0e8;//1000000; ..present,
  double t, time_start_loop, present_time;
  //ciodac16_ -> client_start();
  //udppacket_control send_packet;

  sensorlog_.open("sensorlog.txt");
  //int i = 0;
  rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
  UpdateSharedMemory();

  if(CALIBRATION_FLAG == 1)
    {
      ODEBUG("\n CALIBRATION MODE ON ...... \n");
    }
  if(INFLATING_FLAG == 1)
    {
      ODEBUG("\n ..... INFLATING THE MUSCLES   .....");
      
      //Appel a la fonction de gonflement des muscles
      Inflating();

      sleep(5);
      ODEBUG("\n ..... INFLATING should be completed  .....");
    }
  if(PRES_INDIVIDUAL_FLAG == 1)
    {
#ifndef NDEBUG
      int index_pres_indiv;
      double pres_pres_indiv;
      char depres;
      cout << "\n Select the muscle number: "<< endl;
      std::cin >> index_pres_indiv; //scanf("%s",tmp);
      cout << "\n Enter the pressure value: " << endl;
      std::cin >> pres_pres_indiv;
      std::cin.clear(); std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
      ciodac16_ -> pressure_inidividualmuscle(index_pres_indiv-1, pres_pres_indiv);
      printf("\n Type Any letter to depressurize : ");
      std::cin >> depres; //scanf("%s",tmp);
      std::cin.clear(); std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
      Deflating();
      timeofsimulation= 2; /* time in seconds*/
#endif
    }

  if(CONTROL_MODE_PRES_FLAG == 1 || CONTROL_MODE_NOPRES_FLAG == 1)
    {
#ifndef NDEBUG
      char tmp;
      printf("\n ..... CONTROL MODE Begins   .....");
/*
      printf("\n Type o for OPEN LOOP control, or f for CLOSED LOOP conttrol, or p for open loop pressure command and confirm (ok3, tmp) : ");
      std::cin >> tmp; //scanf("%s",tmp);

      std::cin.clear(); std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
      if (strcmp(&tmp,"o")==0) {loop_=BOUCLE_OUVERTE;}
      if (strcmp(&tmp,"f")==0) {loop_=BOUCLE_FERMEE;}
      if (strcmp(&tmp,"p")==0)  {loop_=BOUCLE_PRESCMD; cout << "loop_ pressure is set :"  << loop_;}
      */
#endif
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
      ODEBUGL("\n time difference :" << (double)time_diff/(double)1.0e6, 3);
      ReadStatus();
      ApplyPressure(); 

      if(FINITE_STATE_ == 3)
	{
	  ciodas64_ -> adconv(1);
	  Calibration();
	}


      if(FINITE_STATE_==1)
	{
          ApplyPressure(); 
	}
      if(FINITE_STATE_==2)
	{
	  ODEBUG("\n You have Inflated the muscles!");
	}

      previous = now;
      if(t >= timeofsimulation)
	{
	  FLAG = 0;
	  ODEBUG("\n END of Loop");

	}
    } //while (ok2) finish
  sensorlog_.close();

  // terminaison

  ODEBUG("outside while loop ok2");
  ODEBUG("\n ..... DEFLATING THE MUSCLES   .....");

  Deflating();
  sleep(5);
  ODEBUG("\n ..... DEFLATING should be completed  .....");
  //Appel a la fonction de degonflement des muscles

  //Remise a faux de variables booleennes
  fin_ = false;
  tele_op_ = false;
  sortie_ = false;

  //ciodac16_ -> ~ClientUDP();

  // Calling controllers destructors
  for(unsigned int i=0;i<7;i++)
    controllers_[i]->~controller_axis();

  // Calling actuators destructors.
  for(unsigned int i=0;i<7;i++)
    actuators_[i]->~Actuator();

  controller_gripper_.~controller_tool();

  joy1_->~I_teleop();
  joy2_->~I_teleop();
  ppalonnier_->~I_teleop();

  ciodac16_->~CIODAC16();
  ciodas64_->~CIODAS64();


  for (int i = 0; i < 7;i++)
    sensors_[i].~position_sensor();
  
  ODEBUG("\n    ====== PROGRAM FINISHED ======    \n\n");
  ODEBUG("\n .... Electronics is reset ...\n");
} //principale finish

void Pneumatic7ArmRtThread::StartingRealTimeThread()
{
  int n;
#ifndef NDEBUG
  printf("\n");
  printf("	*****************************************************************\n");
  printf("	*				    	                        *\n");
  printf("	*		        CONTROL SOFTWARE  		        *\n");
  printf("	*                       for the 7 DOFs ARM		        *\n");
  printf("	*			                                        *\n");
  printf("	*****************************************************************\n");
  printf("\n\n\n");
  for(unsigned int i=0;i<7;i++)
    cout << pressure_command_array_(i) <<",";
  cout << endl;
#endif

  n = rt_task_create(&principal_task_, "principal_function", 0, 99, 0);
  if (n!=0)
    {
      std::cerr << "Failed @ RT Create" << n <<endl;
    }
  else 
    { ODEBUGL("END of RT Create",3); }

  n = rt_task_start(&principal_task_, &principale, this);
  if (n!=0)
    {
      std::cerr<< "Failed of RT STart" <<n<< endl;
    }
  else 
    { ODEBUGL("END of RT Start",3); }

  pause();

  ODEBUGL("END of Pause",3);

  n = rt_task_delete(&principal_task_);
  
  if (n!=0)
    std::cerr << "Failed of RT Task delete" << endl;
  else 
    { ODEBUG("END of RT taslk delete"); }
  
  CloseSharedMemory();
}

/*****************************************
 * Create Shared Memory                  *
 *****************************************
 * The shared memory is linked with the  *
 * file /var/log/pneumatic_arm.shm       *
 * 
 *****************************************/
void Pneumatic7ArmRtThread::CreateSharedMemory()
{
  // Update and/or create the file.
  ofstream aof;
  aof.open(SHM_LOG_FILENAME,
           std::ofstream::out | std::ofstream::app);
  struct timeval current_time;
  gettimeofday(&current_time,0);
    
  aof << current_time.tv_sec << "." << current_time.tv_usec << std::endl;
  aof.close();
  
  // Attached the shared memory to a memory segment.
  shmaddr_ = CreateSharedMemoryForPneumaticArm(true);
}

void Pneumatic7ArmRtThread::UpdateSharedMemory()
{

  // Write desired pressure
  for(unsigned int i=0,j=0;i<14;i+=2,j++)
    {
      double m1,m2;
      m1 = shmaddr_[i];
      m2 = shmaddr_[i+1];

      actuators_[j]->receive_command_decouple(m1,m2);
    }
}

void Pneumatic7ArmRtThread::ApplyPressure()
{
  // Write desired pressure
  for(unsigned int i=0,j=0;i<14;i+=2,j++)
    {
      double m1,m2;
      m1 = shmaddr_[i];
      m2 = shmaddr_[i+1];

      actuators_[j]->receive_command_decouple(m1,m2);
      ODEBUGL("Muscle " << i << " = " << m1 << " , " <<m2, 3 );
    } 
  ciodac16_ -> daconv(1, '1');

 }


void Pneumatic7ArmRtThread::ReadStatus()
{
  // Read position
  ciodas64_ -> adconv(1);
  ciodas64_ -> logudpdata();

 for(unsigned int i=16;i<23;i++)
    {
      shmaddr_[i] = controllers_[i-16]->get_angle_lire();
      ODEBUGL("shmaddr_["<<i<<"]="<< shmaddr_[i],3);
    }
  FINITE_STATE_= (int) shmaddr_[23];
  ODEBUGL("FINITE_STATE_" << FINITE_STATE_, 0);
}
void Pneumatic7ArmRtThread::CloseSharedMemory()
{
  shmdt(shmaddr_);

  // Update the file.
  ofstream aof;
  aof.open(SHM_LOG_FILENAME,
           std::ofstream::out | std::ofstream::app);
  struct timeval current_time;
  gettimeofday(&current_time,0);
    
  aof << " - " << current_time.tv_sec << "." << current_time.tv_usec << std::endl;
  aof.close();

}
