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
#include "clientudp3.hh"
#include "test_config.hh"
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class Muscle
{
protected:
  /*! Boolean for the end. */
  bool end_;  // fin
  /*Muscle modelling parameters*/
  /*vector<double> po_;
  vector<double> alphao_;
  vector<double> epsilono_;
  vector<double> k_;
  vector<double> ro_;
  vector<double> R_;
  vector<double> fv_;*/
public:
  void init_muscle_i (controller_axis *controleur_i, double * delta, double * vitesse);

  void reset_muscle_i(controller_axis *controleur_i, double *vitesse);

  void trait_muscle_i (controller_axis *controleur_i, double * delta, double * vitesse);
  double muscle_force_i (controller_axis *controleur_i);
};
#endif /* MUSCLE_HH */
