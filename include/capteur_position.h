/****************************************
 * Fichier capteur_position.h          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#ifndef CAPTEUR_POSITION
#define CAPTEUR_POSITION


#include <capteur.h>
/********************************************************************
 *                   CLASSE capteur_position                        *
 ********************************************************************
 *                                                                  *
 *     Cette classe fournit toutes les operations servant 	    *
 *        a mesurer la position angulaire des axes		    *
 *                                                                  *
 ********************************************************************/


class capteur_position: public capteur
{
 private :
  double offset,pente;

 public :
  //constructeurs
  capteur_position (){}
  capteur_position (double,double);
		
  //Lecture de la position de l'axe
  double lire_position(void);
		double read_sensors_array(int*);
  /* recup attr. */
  carte * get_carte (void);
  double get_rapport(void);
  double get_offset();
  double get_pente();
		
  /* modif attr. */
  void set_offset(double);
};

#endif
