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
#include <vector>

#include <native/task.h>
#include <native/timer.h>

#include <time.h>
#include <sstream>
#include <fstream>
#include <istream>
#include <math.h>

#include "ioboards.hh"
#include "CIODAC16.hh"
#include "CIODAS64.hh"
#include "actuator.hh"
#include "I_teleop.h"
#include "joystick.h"
#include "sensor.hh"
#include "position_sensor.hh"
#include "controller_axis.hh"
#include "controller_tool.hh"
#include "fichier.h"
#include "modele.h"
#include "muscle.hh"
#include "clientudp3.hh"
#include "test_config.hh"
#include <Eigen/Eigen>
using namespace Eigen;
class Pneumatic7ArmRtThread : public ClientUDP, public ioboards
{
  int loop_;
  double control_command_;
  RT_TASK principal_task_;

  /* Internal states */
  int BOUCLE_PRESCMD ;
  bool DEFAULT_FLAG;
  bool CALIBRATION_FLAG;
  bool CONTROL_MODE_NOPRES_FLAG;
  bool CONTROL_MODE_PRES_FLAG;
  bool INFLATING_FLAG;
  bool PRES_INDIVIDUAL_FLAG;
  int FINITE_STATE_;
  int timeofsimulation;

  /// \brief Muscle class object
  Muscle pneumatic_muscle;
  /// \brief Gripper controler
  controller_tool controller_gripper_;

  /// \brief  Network connection.
  ClientUDP *clientUDP;

  test *test1;
  /*@{ IO Boards */
  CIODAC16 *ciodac16_;
  CIODAS64 *ciodas64_;
  /*@} */

  /// \brief Informations to be send to the NIC module
  VectorXd recving_Data_;
  VectorXd CTRL_FLAG;
  VectorXd pressure_command_array_;
  VectorXd sensors_array_;
  ofstream sensorlog_;

  int num_joints_;

  /// \brief Actuators
  std::vector<Actuator *> actuators_;

  /// \brief Array of sensors
  position_sensor sensors_[7];

  /// \brief Angles measurements
  double angle[7];
  double erreur[7];
  double angle_read;
    
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

  /// \brief Reference to the shared memory */
  int shmid_;
  /// \brief Address to the shared memory */
  double * shmaddr_;
  

  /** \brief Miscelleanous */
  bool fin_ ,tele_op_ ;
  bool sortie_;
  char * buffer_joy_[7];

  /** ! Initialize actuators */
  void InitActuators();

  /** ! Init IO boards */
  void InitIOboards();

  /** ! Initialize sensors */
  void InitializeSensors(); // fka init_sensors

  /** @{ Shared memory related methods*/
  /// \brief Create shared memory. 
  void CreateSharedMemory();

  /// \brief Update shared memory
  void UpdateSharedMemory();

  /// \Read the potentiometers and Finite state
  void ReadStatus();

  ///\Apply the desired pressure to the muscles
  void ApplyPressure();

  /// \brief Free the shared memroy
  void CloseSharedMemory();

  /** @} */
public:
  Pneumatic7ArmRtThread();

  /**! Main method to apply the control. */
  void PrincipalTask();

  /**! Main initialization */
  void Initializing();

  /** ! Starting the main real time thread. */
  void StartingRealTimeThread();

  /**! Initialize muscle */
  void init_muscle_i (controller_axis *controleur_i, double * delta, double * vitesse);

  void reset_muscle_i(controller_axis *controleur_i, double *vitesse);

  void trait_muscle_i (controller_axis *controleur_i, double * delta, double * vitesse);
  

};
#endif /* _PNEMUATIC_7ARM_RT_THREAD_HH_ */
