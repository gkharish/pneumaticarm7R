/****************************************
 * Fichier Carte.h               	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

/***** CLASSE CARTE : INTERFACE pour l'utilisation d'autre modele de carte ****/
#ifndef CARTE_H
#define CARTE_H
#include <Eigen/Core>
#include <math.h>
using namespace Eigen;
class carte 
{
public :
  virtual void daconv(int ,double ){}
  virtual void initialisation(){}
  //virtual float adconv(int ){ return 0;}
  virtual VectorXd adconv(int ){ }
  virtual unsigned char dread(){ return 0;}
  virtual double read_sensors(int* ){return 0;}
};

#endif
