 /* Fichier ioboards.hh               	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

/***** CLASSE ioboards : INTERFACE pour l'utilisation d'autre modele de ioboards ****/
#ifndef CARD_HH
#define CARD_HH
#include <Eigen/Core>
#include <math.h>
using namespace Eigen;
class ioboards
{
public :
  virtual void daconv(int ,char ){}
  virtual void send_command_array(int, double){}
  virtual void initialisation(){}
  //virtual float adconv(int ){ return 0;}
  virtual void adconv(int ){ }
  virtual unsigned char dread(){ return 0;}
  virtual double read_sensors(int ){return 0;}
};

#endif
