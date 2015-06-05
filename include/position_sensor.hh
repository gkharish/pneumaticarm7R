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
  double offset_,slope_;
  // Index in the sensor array.
  int index_; 

 public :
  //constructors
  position_sensor (){}
  position_sensor (double,double, int);

  //read the position of the joints
  double read_position(void);
  double read_sensors_array();

  /* recup attr. */
  ioboards * get_ioboards (void);

  double get_rapport(void);
  double get_offset();
  double get_slope();

  /* set the  attributes. */
  void set_offset(double);
  void set_slope(double);
  void set_index(int);
};

#endif
