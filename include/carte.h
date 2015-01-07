/****************************************
 * Fichier Carte.h               	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

/***** CLASSE CARTE : INTERFACE pour l'utilisation d'autre modele de carte ****/
#ifndef CARTE_H
#define CARTE_H

class carte {
public :
  virtual void daconv(int ,unsigned int ){}
  virtual void initialisation(){}
  virtual unsigned int adconv(int ){}
  virtual unsigned char dread(){}
};

#endif