/****************************************
 * Fichier captor.hh       	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#ifndef CAPTOR_HH
#define CAPTOR_HH

#include <card.hh>
 /******* CLASSE  - GENERALISATION DES S *****/

class captor
{
	protected :
  card * pcard;  //pointeur sur la card d'acquisition
 	int chanNumber;  //voie correspondante sur la card d'acquisition
 	//int *axis_number;
 	public :

	//Association d'un card et d'un numero de voie au
	void set_association(card *,int);

	//Accesseurs
	int get_channel();
	//void set_axisnum(int *index)
	card * get_card();
};

#endif
