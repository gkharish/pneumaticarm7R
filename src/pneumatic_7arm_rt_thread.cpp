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


/***** DEFINITION DES INFOS CONCERNANT LA GACHETTE *****/
#define VOIE_PINCE_1        14   // sorties de la ioboards de commande
#define VOIE_PINCE_2        15   // reliees a la pince

#define PRESSION_MAX_NUM 4095  // pression maximale numerique
#define PRESSION_MIN_NUM    0  // pression minimale numerique


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
  recving_Data_(15),
  CTRL_FLAG(7),
  pressure_command_array_(7),
  sensors_array_(7),
  actuators_(7),
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
  //unsigned int mapfromIO2chain[7] = {4,2,6,0,1,3,5};
    unsigned int mapfromIO2chain[7] = {0,1,2,3,4,5,6};

  for (int i = 0; i<7;i++)
    {
      // Set offset and slope
      sensors_[i].set_offset(0);
      sensors_[i].set_slope(0);
      sensors_[i].set_index(mapfromIO2chain[i]);
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

void Pneumatic7ArmRtThread::InitIOboards()
{
  //start the NI module to send data
  ciodac16_ -> daconv(1, '0');
 // Set the actuator command to zero.
  for (unsigned int i=0;i<7;i++)
    {
      double cmd=0.0;
      actuators_[i]->receive_command_decouple(cmd,cmd);
    }


  ciodas64_ -> adconv(1);
  if(ciodas64_->CheckBoundaryLimit() == false)
         ciodac16_ -> daconv(1, '1');
  else
      cout << "Boundary Limit reached, IOboards not initialized" << endl;

  // Bind controllers and sensors
  ciodas64_ -> adconv(1);
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

  // Init IO boards.
  InitIOboards();
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
    sensors_[i-1].set_offset( sensors_[i-1].read_sensors_array() );

  ODEBUG("Done init_sensors() ");

  ciodac16_ -> daconv(1, header);
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
  bool LOGFLAG = 0;

  RTIME   now, previous=0,  time_diff, TASK_PERIOD = 100e6;//1000000; ..present,
  double t, time_start_loop, present_time;
  double SubSampling_period = 100e6;
  double data_array[40][23];
  int subsampling_itr;
  //ciodac16_ -> client_start();
  //udppacket_control send_packet;

  sensorlog_.open("sensorlog.txt");
  log_pneumaticthread_data_.open("pneumaticthread_data.txt");
  //int i = 0;
  rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));


  now = rt_timer_read();
  time_start_loop  = round((double)now/1.0e9);

  InitializeSensors();
  
  unsigned long long nb_it;
  while (1)
    {
      rt_task_wait_period(NULL);


      //controller_gripper.initialiser();

      now = rt_timer_read();
      present_time  = round((double)now/1.0e9);
      t = present_time - time_start_loop;
      time_diff = now - previous;
      if (nb_it%50==0)
	ODEBUGF("\n time difference :" << (double)time_diff/(double)1.0e6);

      nb_it++;

      // Receiving information from NIC module
      ciodas64_ -> adconv(1);
      //ciodas64_ -> logudpdata();
      if(ciodas64_->CheckBoundaryLimit() == true)
            ciodac16_->SetBoundaryError(true);

      UpdateSharedMemory();

      // Sending desired pressure
     // ciodac16_ -> daconv(1, '1');
       
      previous = now;
      if(LOGFLAG == 1)
      {
	  FLAG = 0;
	  ///ODEBUG("\n END of Loop");
          for (unsigned int i=0; i <23; i++)
          {
             data_array[subsampling_itr][i] = shmaddr_[i];
          }


          if ( subsampling_itr  >= ( (int) SubSampling_period/TASK_PERIOD) -1)
          {
              for (unsigned int j = 0; j <= subsampling_itr; j++)
              {
                
                  for (unsigned int i = 0; i < 16; i++)
                        log_pneumaticthread_data_ << data_array[j][i] << " ";
                  
                  log_pneumaticthread_data_ << "\t";
                
                  for (unsigned int i = 16; i <  23; i++)
                        log_pneumaticthread_data_ << data_array[j][i] << " ";
                 
                  log_pneumaticthread_data_ << "\n";
              }
              subsampling_itr= 0;
          }
          subsampling_itr++;
      }
      if (FINITE_STATE_ == 5)
            InitIOboards();
    } //while (ok2) finish
  sensorlog_.close();
  log_pneumaticthread_data_.close();

  // terminaison

  ODEBUG("outside while loop ok2");
  ODEBUG("\n ..... DEFLATING THE MUSCLES   .....");
  InitIOboards();
  sleep(5);
  ODEBUG("\n ..... DEFLATING should be completed  .....");
  //Appel a la fonction de degonflement des muscles

  //Remise a faux de variables booleennes
  fin_ = false;
  tele_op_ = false;
  sortie_ = false;

  //ciodac16_ -> ~ClientUDP();

  // Calling actuators destructors.
  for(unsigned int i=0;i<7;i++)
    actuators_[i]->~Actuator();

  controller_gripper_.~controller_tool();

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
  ApplyPressure();
  ReadStatus();
}

void Pneumatic7ArmRtThread::ApplyPressure()
{
  // Write desired pressure
  for(unsigned int muscle_j=0;muscle_j<14;muscle_j++)
    {
      double pressure =  shmaddr_[muscle_j];

      ciodac16_->send_command_array(muscle_j, pressure);
      ODEBUGL("Muscle " << muscle_j << " = " << pressure, 3 );
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
      shmaddr_[i] = sensors_[i-16].read_sensors_array();
      ODEBUGL("shmaddr_["<<i<<"]="<< shmaddr_[i],3);
    }
  FINITE_STATE_= (int) shmaddr_[23];
 // ODEBUGL("FINITE_STATE_" << FINITE_STATE_, 0);
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
