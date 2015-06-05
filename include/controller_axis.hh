/****************************************
 * Fichier controller_axis.hh         	*
 * Jeremie Guiochet			*
 * cree le 17/07/2002 par Mehdi SEFFAR	*
 ****************************************/

/*** MODIFICATIONS ***
     20/12/2002 : Ajout du pointeur de methode pour choisir Boucle Fermee ou boucle Ouverte


*/

#ifndef CONTROLLER_AXIS
#define CONTROLLER_AXIS

#define OUVERTE 0
#define FERMEE 1
#define PRESCMD 2

#define FREQ_COUPURE 2.0  //frequence de coupure du filtre en Hz
#define TAU   1/FREQ_COUPURE //periode de coupure

#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)

/********************************************************************

 *                          CLASS controlle_axis                     *

 ********************************************************************

 *                                                                  *


 *     This class provides all of the operations used               *

 *        to check the rotational movement of some axes	            *

 *                                                                  *

 ********************************************************************/

#include "actuator.hh"
#include "controller_tool.hh"
#include "joystick.h"
#include "position_sensor.hh"

#include "I_teleop.h"
#include <stdlib.h>
#include <stdio.h>
//using Eigen;
using namespace std;

typedef struct controller_axis_data_s
{
  // Reference to the joystick
  I_teleop* pjoystick;
  // Actuator reference.
  Actuator *pactuator;
  // Identifier of the actuator
  int numero;
  // Rest value of the angle
  double angle_repos;
  // Reel value of the angle
  double angle_reel,angle_reel_prec;
  // Maximinal and minimal boundaries for the joint.
  double angle_min_bound;
  double angle_max_bound;
  // Direction of the sensor
  int sens_sensor;

  // Direction of the pressure
  int sens_pression;
  // PD gains
  double P;
  double D;
  double angle_th;

  /*@{ Filter related data*/
  /*! Filtered angle */
  double angle_filtre;
  /*! Previous filtered angle */
  double angle_filtre_prec; //infos filtre
  /*@}*/

  /*! Error */
  double error; // erreur

  /*@{ Saturations */
  /*! Forward saturation */
  bool forward_saturation; //saturation_avant;

  /*! Backward saturation */
  bool backward_saturation; //saturation_arriere;

  /*!  Rule during muscle initialization */
  double delta_repos;

} controller_axis_data;

class controller_axis
{

 private :
  controller_axis_data ControllerAxisData_;
  double zero_joy;		//position initiale du joystick
  position_sensor *psensor;   //sensor de position associe
  //double angle_repos,angle_reel, angle_max, angle_min;
  //reste constant pendant la phase de controle
  double offset_sensor;  //difference entre la valeur initiale lue
  // par le sensor et l'angle au repos theorique
  double offset_lu; //valeur lue par le sensor a t =0

  double rapport;
  int boucle; // 0 pour boucle OUVERTE et 1 pour boucle FERMEE
  double user_pressure;
  //Donnees calcul commande  PID

  double derivee_erreur;
  double tab_erreur [10];		//sauvegarde des 10 dernieres erreurs
  double commande;
  void calculer_commande_BF(void);   //calcul de la commande en Boucle fermee
  void calculer_commande_BO(void);   //calcul de la commande en Boucle Ouverte
  void pressure_commande (void);         //calcul de la commande en Boucle prescmd
  void (controller_axis::*pcalculer_commande)(); //Pointeur de methode qui servira pour
  //pointer sur calculer_commande en BO ou BF

 public :

  // Constructors
  controller_axis (){};

  /*! Constructor
    \param pjoy: Pointer towards the joystick structure (not supported anymore).
    \param lactuator: Pointer towards the actuator to controlled.
    \param num: Identifier of the joint
    \param angle_init: Rest position of the joint.
    \param amgle_min_bound: Lower limit of the joint.
    \param angle_max_bound: Upper limit of the joint.
    \param s_cap: Direction of the sensor.
    \param s_pre: Direction of the pressure.
    \param P: Proportionnal control gain.
    \param D: Derivative control gain.
   */
  controller_axis (I_teleop* pjoy,
                   Actuator *lactuator,
                   int num,
                   double angle_init,
                   double angle_min_bound,
                   double angle_max_bound,
                   int s_cap,
                   int s_pre,
                   double p,
                   double d);

  void init_controller_axis (controller_axis_data & aControllerAxeData);

  //Associating the sensor with corresponding joint axis
  void set_sensor (position_sensor*);

  //reading the potentiometer
  double read_position(void);


  //Initializing the muscles in resting position (zero-configuration)
  void initialisation_muscles(double,double);

  //Changer de style de boucle fermee ou ouverte
  void set_loop(int);
  void set_userpressure(double pres);
  //Controller of Jointsl'axe
  void controller();

  //Deflating of muscles
  void degonfle (double);

  /** Geting the attributes **/
  void init_angles (void);
  position_sensor * get_sensor(void);
  double get_rapport(void);
  double get_delta(void);
  double get_angle_desire();
  double get_angle_filtre();
  double get_angle_reel();
  double get_angle_lire (void);
  double get_commande();
  void get_reference_angle(double , double);
};


#endif
