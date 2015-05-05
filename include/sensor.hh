/****************************************
 * Fichier sensor.hh       	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#ifndef CAPTOR_HH
#define CAPTOR_HH

#include <ioboards.hh>
/******* CLASSE  - GENERALISATION DES S *****/

class sensor
{
protected :
  ioboards * pioboards;  //pointeur sur la ioboards d'acquisition
  int chanNumber;  //voie correspondante sur la ioboards d'acquisition
  //int *axis_number;
public :

  //Association d'un ioboards et d'un numero de voie au
  void set_association(ioboards *,int);

  //Accesseurs
  int get_channel();
  //void set_axisnum(int *index)
  ioboards * get_ioboards();

};

#endif
