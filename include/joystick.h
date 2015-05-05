/* joystick.h - entete de la classe joystick */

/* Erwan Guiochet - 2002 */



#ifndef JOYSTICK
#define JOYSTICK



/********************************************************
 *							*
 * 							*
 *			       				*
 *			_________			*
 *		       /	 \\  D 			*
 *		       \          \\			*
 *		       \\   (C)   |			*
 *		        \\	  |			*
 *		       A \\  (B)  |			*
 *			  \\	  |			*
 *			   |	  |			*
 *			   |	  |			*
 ********************************************************
 *		   BOUTONS DU JOYSTICK 			*
 *		Microsoft Side Windeur Pro		*
 ********************************************************
 *							*
 *	A - Gachette					*
 *	B - cote bas					*
 *	C - cote haut					*
 *	D - Bouton haut et direction numerique		*
 *							*
 ********************************************************/
 
#include "I_teleop.h" 
class joystick
: public I_teleop
{

	private :

        	int
voie_axeX,voie_axeY,voie_axeZ,	   //numeros de voie	
        	voie_vitesse,voie_bouton_a,voie_bouton_b,  //sur la card d'acquisition
        	voie_bouton_c,voie_bouton_d; 		
	public :

		//constructeurs
		joystick (){}
		joystick(card *,int,int,int,int,int,int,int,int,double); 
		 
		/* Permet de lire toutes les entr�es d'un joystick */

		double read_position_x(void);

		double read_position_y(void);
		double read_position_z(void);
		double read_position_vitesse(void);
		bool lire_bouton_A(void);
		bool lire_bouton_B(void);
		bool lire_bouton_C(void);
		bool lire_bouton_D(void);

		int  get_vitesse();
};

#endif