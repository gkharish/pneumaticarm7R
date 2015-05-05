/****************************************
 * Fichier actuator.hh         	*
 * Mehdi SEFFAR				*
 * cree le 17/07/2002  			*
 * Modified by Ganesh kumar on 04/05/2015 *
 ****************************************/

#ifndef ACTUATOR
#define ACTUATOR

#include "ioboards.hh"
/********************************************************************

 *                        	DEFINES            	            *

 ********************************************************************/
#define PRESSION_MAX (5.0)             /* Maximum pressue applied on the muscle */

#define RAPPORT (4095.0/PRESSION_MAX)  /* quantification of the control commande 12 bits */

#define PRESSION_DE_BASE (2.5);

/********************************************************************

 *                        CLASSE Actuator                         *

 ********************************************************************

 *                                                                  *


 *     This class provides all operational services *
  * A controller to  a muscle, each actuator corresponds to a muscle  *

*                                                                  *

********************************************************************/


class Actuator

{

private :
  int muscle1,muscle2; // num�ro de voix
  ioboards * pioboards;

public :
  //Constructors
  Actuator (){}
  Actuator (int,int, ioboards *);

  //==========================================================
  //===  Functions of commanding the actuators in presure ====
  //==========================================================

  // Sending the pressure to the two muscles. // Envoi de la meme pression sur les deux muscles
  void receive_command (double);
  // sending the pressure differences (Delta pressure ) to the two muscles.// Envoi de pressions differentes sur les deux muscle
  void receive_command_decouple (double,double);

};

#endif
