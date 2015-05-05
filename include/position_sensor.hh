/****************************************
 * Fichier captor_position.hh          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#ifndef CAPTOR_POSITION
#define CAPTOR_POSITION


#include <captor.hh>
/********************************************************************
 *                   CLASS captor_position                        *
 ********************************************************************
 *                                                                  *
 *       This class provides all operations serving
          to measure the angular position of the axes		            *
 *                                                                  *
 ********************************************************************/


class captor_position: public captor
{
 private :
  double offset,pente;

 public :
  //constructors
  captor_position (){}
  captor_position (double,double);

  //Lecture de la position de l'axe
  double lire_position(void);
	double read_sensors_array(int);
  /* recup attr. */
  card * get_card (void);
  double get_rapport(void);
  double get_offset();
  double get_pente();

  /* modif attr. */
  void set_offset(double);
  void set_pente(double);
};

#endif
