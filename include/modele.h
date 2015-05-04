/****************************************
 * Fichier modele.h             	*
 * Mehdi SEFFAR				*
 * cree le 29/08/2002  			*
 ****************************************/
 
#ifndef MODELE
#define MODELE

/********************************************************************

 *                          classe modele                           *

 ********************************************************************

 *                                                                  *

 *    Classe gerant la mise a jour et la sauvegarde des             *

 *     informations						    *
 *                						    *

 *		  		        			    *
 ********************************************************************/
#include "controller_axis.hh"
#include "controller_tool.hh"
#include "joystick.h"
#include "fichier.h"
#include "I_teleop.h"

 class modele {
 
 	private:
 	//Pointeurs sur les classes utilisees
 	controller_axis *caxe1,*caxe2,*caxe3,*caxe4,*caxe5,*caxe6,*caxe7;
 	controller_tool * coutil;
 	joystick * joy1, * joy2;
 	palonnier * pal;
 	fichier * mon_fichier;
 	
 	//tableaux de stockage des resultats
 	double *tab_temps;
	double** tab_pression;
	double * tab_consigne_pince;
	double *tab_etat_pince;
	double ** tab_angles;
	double ** tab_angle_des;
	double ** tab_angle_filtre;
	
	int numero_pt;
	
	public :
	
	//int numero_pt;
	//Constructeurs
	
	modele () {}
	modele (controller_axis *,controller_axis *,controller_axis *,
		controller_axis *,controller_axis *,controller_axis *,controller_axis *,
		controller_tool *,palonnier *,joystick *,joystick *) ;
	
	//Mise a jour		
	void mettre_a_jour(double);
	
	//sauvegarde
	void save_matlab(char *);
	
};

#endif
