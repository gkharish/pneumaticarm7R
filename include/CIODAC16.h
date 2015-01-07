/* CIODAC16.h - entetes de la classe carteSortie */
/* refCarte : CIO-DAC16-I de Computer boards*/
/* Erwan Guiochet - 2002 */

#ifndef CIODAC
#define CIODAC

#include "carte.h"
/************************************************************************
 *									*
 *				DEFINES					*
 *									*
 ************************************************************************/

#define BASE_0 (BASE_REG_CIODAC16) 	/* 	       D/A LSB 	        */
#define BASE_1 (BASE_REG_CIODAC16+1)	/* D/A MSB & channel addresse 	*/

/************************************************************************
 *  Calcul de la valeur de sortie 					*
 *									*
 *	soit X(99v9999999) en mA la valeur que l'on veut :		*
 * 	CODE = ((X - 4)/16) x 4095					*
 *	si CODE = 0 alors X = 4						*
 *	si CODE = 4095 alors X = 20					*
 * le code est en 12 bit, les 4 autres sont pour le choix de la sortie  *
 *									*
 ************************************************************************/

/************************************************************************
 *									*
 *				INCLUDES				*
 *									*
 ************************************************************************/
//#include "sysLib.h"
#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)
/************************************************************************
 *									*
 *			CLASSE CIODAC16					*
 *									*
 ************************************************************************/
class CIODAC16 : public carte
{	 
	public :
		CIODAC16(){}  // constructeur
		virtual void daconv(int chan ,unsigned int valeur);
};

#endif