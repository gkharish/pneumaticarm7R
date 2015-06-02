/****************************************
 * Fichier actuator.cpp          	*
 * Mehdi SEFFAR										*
 * cree le 17/07/2002  						*
 * Modified by Ganes Kumar on 04/5/2015
 ****************************************/
#include "actuator.hh"
#include "math.h"


/********************************************************************

 *                          CONSTRUCTOR                            *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *                pioboards_ : Pointer to the ioboards type CIODAS64 				*


 *                chan : Number of port of entry in the ioboards				*

 *                                                                  *

 *    RETURN :                                                    	*

 *                An object actuator                               	*

 *                                                                  *

 ********************************************************************/

Actuator::Actuator (int chan1,int chan2 ,ioboards* ioboards)

{

  pioboards_ = ioboards;

  muscle1 = chan1;

  muscle2 = chan2;
}


/********************************************************************

 *                          RECEVOIR_COMMANDE                       *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *      commande : valeur de la pression envoyer sur les muscles    *

 *                                                                  *


 *                                                                  *

 ********************************************************************/

void Actuator::receive_command (double commande)

{
  double m1,m2;
  m1 = 2.5 + commande ;
  m2 = 2.5 - commande ;
  receive_command_decouple(m1,m2);
}

/********************************************************************

 *                    RECEVOIR_COMMANDE_DECOUPLE                    *

 ********************************************************************

 *                                                                  *

 *    PARAMETRES :                                                  *

 *         val1 : valeur de la pression envoyer sur le muscle	1   *

 *         val2 : valeur de la pression envoyer sur le muscle	2   *

 *                                                                  *

 ********************************************************************/

void Actuator::receive_command_decouple (double val1,double val2)

{

  double buffer;
  if (val1>=0 && val1 <= PRESSION_MAX)
    {
      buffer = (val1);// * Conversion from pressure to voltage;
      pioboards_->send_command_array(muscle1, buffer);
    }

  if (val2>=0 && val2 <= PRESSION_MAX)
    {
      buffer = (val2) ;//* Conversion from pressure to voltage;
      pioboards_->send_command_array(muscle2, buffer);
    }


}
