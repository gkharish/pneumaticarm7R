/*!
 * Main thread to control the pnematic 7DOFS arm at INSA
 * Created from : Erwan GUIOCHET/ Mehdi Seffar / JEREMIE GUIOCHET
 * Date     : 02/07/2002 				    
 * Modif. M.Seffar    : 20/09/2002
 * Modif. J. Guiochet : 15/12/2002			    
 * Modif. G. Kumar    : 28/04/2015
 * Modif. O. Stasse   : 28/04/2015
 * 
 * LAAS, CNRS 2015 
 *
 */

#ifndef PNEUMATIC_7ARM_RT_THREAD_HH
#define PNEUMATIC_7ARM_RT_THREAD_HH

/* Includes */
#include <iostream>
#include <limits>
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>

#include <time.h>
#include <sstream>
#include <fstream>
#include <istream>
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
#include "test_config.h"


class Pneumatic7ArmRtThread: public
{
int boucle_;
double control_command_;
RT_TASK principal_task_;

  /* Internal states */
int BOUCLE_PRESCMD = 2;
bool DEFAULT_FLAG=0;
bool CALIBERATION_FLAG=1;
bool CONTROL_MODE_NOPRES_FLAG=1;
bool CONTROL_MODE_PRES_FLAG=0;
bool INFLATING_FLAG=0;
bool PRES_INDIVIDUAL_FLAG=0;

// Axis controlers
controleur_axe controleur1,controleur2,controleur3,
  controleur4,controleur5,controleur6,controleur7;

// Gripper controler
controleur_outil controleur_pince;

// Network connection.
ClientUDP *clientUDP;

test *test1;
// IO Boards
CIODAC16 *ciodac16;
CIODAS64 *ciodas64;

// Informations to be send to the NIC module
VectorXd recving_Data(15);
VectorXd CTRL_FLAG(7);
VectorXd pressure_command_array(7);
VectorXd sensors_array(7);
ofstream sensorlog_;

int num_joints_;
// Actuators
actionneur a1,a2,a3,a4,a5,a6,a7;

// Joysticks
I_teleop * joy1,*joy2,* ppalonnier;

// tableau de capteurs
capteur_position cap[7];

// Angles measurements
double angle[7];
double erreur[7];
double angle_read;
//watchdog
WDOG_ID tempo;


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


/* divers */
double temps = 0.0;
int saisie = 0,increm = 0;
char debut;
bool fin = false,tele_op = false;
bool sortie = false;
char * buffer_joy[7];

Pneumatic7ArmRtThread();

/**! Main method to apply the control. */
void principale();

/** ! Starting the main real time thread. */
void StartingRealTimeThread()

};
#endif /* _PNEMUATIC_7ARM_RT_THREAD_HH_ */
