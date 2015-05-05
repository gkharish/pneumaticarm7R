#ifndef MUSCLE_HH
#define MUSCLE_HH

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
#include "actuator.hh"
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
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class Muscle
{
protected:
  /*! Boolean for the end. */
  bool end_;  // fin
public:
  void init_muscle_i (controller_axis *controleur_i, double * delta, double * vitesse);

  void reset_muscle_i(controller_axis *controleur_i, double *vitesse);

  void trait_muscle_i (controller_axis *controleur_i, double * delta, double * vitesse);
};
#endif /* MUSCLE_HH */
