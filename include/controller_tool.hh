/****************************************
 * Fichier controller_tool.h          	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/ 

#ifndef CONTROLLER_TOOL
#define CONTROLLER_TOOL

#include "carte.h"
#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)
/********************************************************
 *							*
 * classe controller_tool:				*
 ********************************************************
 * cette classe offre les attributs et methodes	        *
 * qui permettent de controler la pince du robot	*
 *							*
 ********************************************************/
 class controller_tool {
 	
 	private : 
 	bool pince_ouverte;   		// vraie si la pince est ouverte
 	carte * pcarte;        		// carte de commande associee
 	int voie_pince_1,voie_pince_2;  // voies correspondantes sur la carte de commande
 	int etat_voie_1,etat_voie_2;	// etat des voies (0 ou 1)
 	
 	public :
 	
 	//Constructeurs
 	controller_tool () {};
 	controller_tool(carte *,int,int);
 	
 	//Controle de la pince
 	void controler_pince(bool);
 	
	//Initialisation de la pince en position fermee
 	void initialiser (void);
 	
 	//Accesseurs
 	double get_etat_pince();
 	double get_etat_voie_1();
 	double get_etat_voie_2();
 };
 
 
#endif
