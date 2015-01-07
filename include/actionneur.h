/****************************************
 * Fichier actionneur.h         	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 ****************************************/

#ifndef ACTIONNEUR
#define ACTIONNEUR

#include "carte.h"
/********************************************************************

 *                        	DEFINES            	            *

 ********************************************************************/
#define PRESSION_MAX (5.0)             /* pression maximal excerc�e sur un muscle */

#define RAPPORT (4095.0/PRESSION_MAX)  /* quantification de la commande 12 bits */

#define PRESSION_DE_BASE (2.5);

/********************************************************************

 *                        CLASSE actionneur                         *

 ********************************************************************

 *                                                                  *


 *     Cette classe fournit toutes les operations servant 	    *
 * a controler un muscle, chaque actionneur correspond a un muscle  *

 *                                                                  *

 ********************************************************************/


class actionneur

{

	private :

        	int muscle1,muscle2; // num�ro de voix

        	carte * pcarte;
//carte cible
    	public :

    		//Constructeurs
    		actionneur (){}
         	actionneur (int,int, carte *);
         
        //==========================================================	
        //=== Fonctions de commande de l'actionneur en pression ====
        //========================================================== 	
         	
         	//Envoi de la meme pression sur les deux muscles
         	void recevoir_commande (double);

         	
         	//Envoi de pressions differentes sur les deux muscle
         	void recevoir_commande_decouple (double,double);

}
;

#endif
