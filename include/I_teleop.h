/****************************************
 * Fichier I_teleop.h           	*
 * Mehdi SEFFAR				*
 ****************************************/

#ifndef I_TELEOP
#define I_TELEOP

#include "card.hh" 

/********************************************************************

 *                          Classe I_teleop                         *

 ********************************************************************

 *                                                                  *

 *    Classe definissant un moyen de teleoperation                  *

 *                						    *

 *		  		        			    *
 ********************************************************************/

class I_teleop
{
   protected :
   	 card * pcard;
// pointeur sur card d'acquisition
   	 double UMAX ;   // tension d'alimentation
	 double seuil;   // seuil de prise en compte d'un mouvement
   public :
   	double get_seuil();
};

/********************************************************************

 *                          Classe palonnier                        *

 ********************************************************************

 *                                                                  *

 *    Classe heritant de I_teleop		                    *

 *     definit les operations necessaire a la gestion du palonnier   *
 *    tels que la lecture de la position			    *

 *		  		        			    *
 ********************************************************************/


class palonnier :public I_teleop
{
    private:
    	int channel; //voie correspondante sur la card d'acquisition

    public :
    	//constructeurs
    	palonnier () {}
    	palonnier (card *,int,double);

    	//lecture de la position
    	double lire_position(void);

};

#endif
