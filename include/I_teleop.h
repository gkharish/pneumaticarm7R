/****************************************
 * Fichier I_teleop.h           	*
 * Mehdi SEFFAR				*
 ****************************************/

#ifndef I_TELEOP
#define I_TELEOP

#include "ioboards.hh" 

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
   	 ioboards * pioboards;
// pointeur sur ioboards d'acquisition
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
    	int channel; //voie correspondante sur la ioboards d'acquisition

    public :
    	//constructeurs
    	palonnier () {}
    	palonnier (ioboards *,int,double);

    	//lecture de la position
    	double read_position(void);

};

#endif
