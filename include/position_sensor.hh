/****************************************
 * Fichier position_sensor.hh          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#ifndef CAPTOR_POSITION
#define CAPTOR_POSITION


#include <sensor.hh>
/********************************************************************
 *                   CLASS position_sensor                        *
 ********************************************************************
 *                                                                  *
 *       This class provides all operations serving
          to measure the angular position of the axes		            *
 *                                                                  *
 ********************************************************************/


class position_sensor: public sensor
{
 private :
  double offset,pente;

 public :
  //constructors
  position_sensor (){}
  position_sensor (double,double);

  //read the position of the joints
  double read_position(void);
	double read_sensors_array(int);
  /* get the attributes. */
  card * get_card (void);
  double get_rapport(void);
  double get_offset();
  double get_pente();

  /* set the  attributes. */
  void set_offset(double);
  void set_pente(double);
};

#endif
