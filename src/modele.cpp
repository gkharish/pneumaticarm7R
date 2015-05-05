/****************************************
 * Fichier modele.h             	*
 * Mehdi SEFFAR				*
 * cree le 29/08/2002  			*
 ****************************************/

#include "modele.h"
#define NB_POINTS 20
/********************************************************************

 *                          constructeur                            *

 ********************************************************************

 *                                                                  *

 *    initialisations des objets et allocation memoire aux pointeurs*

 *                						    *

 *		  		        			    *
 ********************************************************************/
 
modele :: modele (controller_axis * ca1,
                  controller_axis *ca2,
                  controller_axis *ca3,
                  controller_axis *ca4,
                  controller_axis *ca5,
                  controller_axis *ca6,
                  controller_axis *ca7,
                  controller_tool * co,palonnier * p,joystick *j1,joystick *j2) {
	printf("\n jusqu'ici tout va bien 6");
	caxe1 = ca1;caxe2 = ca2;caxe3 = ca3;caxe4 = ca4;caxe5 = ca5;caxe6 = ca6;caxe7 = ca7;
	coutil = co;
	pal = p;
	joy1 =j1;joy2 = j2;
	//mon_fichier = new fichier("descards:/export/home/tornado/jeremie/matlab/");
	//mon_fichier = new fichier("c:/");
	numero_pt =0;
	printf("\n jusqu'ici tout va bien 7");
	
	tab_pression = new double* [7];
	for (int j = 0;j < 7;j++)
     		{tab_pression[j] = new double  [NB_POINTS];}
     
    //tab_pression = new double [7][20]; 
    tab_consigne_pince = new double [NB_POINTS];
    tab_etat_pince = new double  [NB_POINTS];
    tab_temps = new double [NB_POINTS];
     
    tab_angles = new double* [7];
     for (int i = 0;i < 7;i++)
     		tab_angles[i] = new double  [NB_POINTS];
     	
	printf("\n jusqu'ici tout va bien 8");
	    	tab_angle_des = new double* [7];
     	for (int i = 0;i < 7;i++)
     		tab_angle_des[i] = new double  [NB_POINTS];
     	
     	tab_angle_filtre = new double* [7];
     	for (int i = 0;i < 7;i++)
     		tab_angle_filtre[i] = new double  [NB_POINTS];
     		
printf("\n jusqu'ici tout va bien 9");
	}

/********************************************************************

 *                          mettre_a_jour                           *

 ********************************************************************

 *                                                                  *

 *    Mise a jour des informations dans les tableauxde reels        *

 *    PARAMETRE : le temps            						    *

 *		  		        			    *
 ********************************************************************/
 

void modele :: mettre_a_jour (double tps) {


	//temps
	tab_temps[numero_pt] =  tps ;
	
	//Pince
	tab_consigne_pince[numero_pt] = joy2 -> lire_bouton_A();
	tab_etat_pince[numero_pt] = coutil -> get_etat_pince();
	
	
	//Consigne pression
	tab_pression[0][numero_pt] = caxe1 -> get_delta();
	tab_pression[1][numero_pt] = caxe2 -> get_delta();
	tab_pression[2][numero_pt] = caxe3 -> get_delta();
	tab_pression[3][numero_pt] = caxe4 -> get_delta();
	tab_pression[4][numero_pt] = caxe5 -> get_delta();
	tab_pression[5][numero_pt] = caxe6 -> get_delta();
	tab_pression[6][numero_pt] = caxe7 -> get_delta();
	
	
	//Angles reels mesures
	tab_angles [0][numero_pt] = (float) caxe1 -> get_angle_reel();
	tab_angles [1][numero_pt] = (float) caxe2 -> get_angle_reel();
	tab_angles [2][numero_pt] = (float) caxe3 -> get_angle_reel();
	tab_angles [3][numero_pt] = (float) caxe4 -> get_angle_reel();
	tab_angles [4][numero_pt] = (float) caxe5 -> get_angle_reel();
	tab_angles [5][numero_pt] = (float) caxe6 -> get_angle_reel();
	tab_angles [6][numero_pt] = (float) caxe7 -> get_angle_reel();
	
	//Joystick
	tab_angle_filtre[0][numero_pt] = (float) caxe1 -> get_angle_filtre();
	tab_angle_filtre[1][numero_pt] = (float) caxe2 -> get_angle_filtre();
	tab_angle_filtre[2][numero_pt] = (float) caxe3 -> get_angle_filtre();
	tab_angle_filtre[3][numero_pt] = (float) caxe4 -> get_angle_filtre();
	tab_angle_filtre[4][numero_pt] = (float) caxe5 -> get_angle_filtre();
	tab_angle_filtre[5][numero_pt] = (float) caxe6 -> get_angle_filtre();
	tab_angle_filtre[6][numero_pt] = (float) caxe7 -> get_angle_filtre();
	
	
	//angles theoriques calcules
	tab_angle_des [0][numero_pt] = (float) caxe1 -> get_angle_desire();
	tab_angle_des [1][numero_pt] = (float) caxe2 -> get_angle_desire();
	tab_angle_des [2][numero_pt] = (float) caxe3 -> get_angle_desire();
	tab_angle_des [3][numero_pt] = (float) caxe4 -> get_angle_desire();
	tab_angle_des [4][numero_pt] = (float) caxe5 -> get_angle_desire();
	tab_angle_des [5][numero_pt] = (float) caxe6 -> get_angle_desire();
	tab_angle_des [6][numero_pt] = (float) caxe7 -> get_angle_desire();
	

	numero_pt ++;
	//Si on arrive a la limite du stockage on remet numero_pt a 0
	if (numero_pt==NB_POINTS) numero_pt=0; 
}

/********************************************************************

 *                          save_matlab                             *

 ********************************************************************

 *                                                                  *

 *    Enregistrement des tableaux sous format matlab                *

 *    PARAMETRE : le nom du fichier      			    *

 *		  		        			    *
 ********************************************************************/
 

void modele :: save_matlab(char * nom_fichier) {
	
	mon_fichier->sauvegarde_matlab(nom_fichier,tab_temps,
				      tab_pression,
				      tab_consigne_pince,tab_etat_pince,
				      tab_angles,tab_angle_des,tab_angle_filtre,
				      numero_pt);
	
	
	//Liberation de l'espace memoire utilise
	for (int i = 0;i < 7;i++)
		delete [] tab_pression[i];
	delete [] tab_pression;
	
	delete [] tab_consigne_pince;
	delete [] tab_etat_pince;
	
	for (int i = 0;i < 7;i++)
		delete []tab_angles[i];
	delete [] tab_angles;
	
	for (int i = 0;i < 7;i++)
		delete []tab_angle_des[i];
	delete [] tab_angle_des;
	
	delete [] tab_temps;
	
	for (int i = 0;i < 7;i++)
		delete [] tab_angle_filtre[i];
	delete [] tab_angle_filtre;
}
	
	
	
	
	
