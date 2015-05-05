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

 *                          CLASSE controle_axe                     *

 ********************************************************************

 *                                                                  *


 *     Cette classe fournit toutes les operations servant 	    *
 *     a controller le mouvement de rotation de certain axes	    *

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
  int sens_captor;
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
  int numero;			//numero de l'axe
  Actuator * pactuator;	//actuator associe
  I_teleop * pjoystick;     //joystick associe
  double zero_joy;		//position initiale du joystick
  captor_position *pcaptor;   //captor de position associe
  int sens_captor;  //sens de rotation de l'axe par rapport au sens du captor
  int sens_pression;  //sens de rotation de l'axe par rapport a la variation de la pression
  double angle_repos,angle_reel, angle_max, angle_min;
  //reste constant pendant la phase de controle
  double offset_captor;  //difference entre la valeur initiale lue
  // par le captor et l'angle au repos theorique
  double offset_lu; //valeur lue par le captor a t =0
  double rapport;
  int boucle; // 0 pour boucle OUVERTE et 1 pour boucle FERMEE
  double user_pressure;
  //Donnees calcul commande  PID

  double derivee_erreur;
  double tab_erreur [10];		//sauvegarde des 10 dernieres erreurs
  double commande;
  double angle_th; 	//angle theorique
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

  //Fonction d'association du captor au controller d'axe
  void set_captor (captor_position*);

  //Lecture de l'angle
  double lire_position(void);

  //initialisation elctronique de la card de commande
  void initialisation_card();

  //Initialisation des muscles en position de repos
  void initialisation_muscles(double,double);

  //Changer de style de boucle fermee ou ouverte
  void set_loop(int);
  void set_userpressure(double pres);
  //Controle de l'axe
  void controller();

  //Degonflement des muscles
  void degonfle (double);

  /** recuperation des attributs **/
  void init_angles (void);
  captor_position * get_captor(void);
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
