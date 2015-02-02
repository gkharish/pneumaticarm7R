/****************************************
 * Fichier capteur.h            	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#ifndef CAPTEUR
#define CAPTEUR
 
#include <carte.h>
 /******* CLASSE CAPTEUR - GENERALISATION DES CAPTEURS *****/

class capteur
{
	protected :
  	carte * pcarte;  //pointeur sur la carte d'acquisition
 	int chanNumber;  //voie correspondante sur la carte d'acquisition
 	//int *axis_number;
 	public :
 
	//Association d'un carte et d'un numero de voie au capteur
	void set_association(carte *,int);
 
	//Accesseurs
	int get_channel();
	//void set_axisnum(int *index)
	carte * get_carte();
};

#endif
