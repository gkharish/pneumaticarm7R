/* CIODAC16.cpp - corps de la classe carteSortie */
/* refCarte : CIO-DAC16-I de Computer boards*/
/* Erwan Guiochet - 2002 */

/*
Historique des modifications
----------------------------

*/
#include "CIODAC16.h"

/************************************************************************
 *									*
 *		CONVERSION NUMERIQUE -> ANALOGIQUE			*
 *									*
 ************************************************************************
 *									*
 *		Methode :						*
 *									*
 *	1 - Decoupage des quatres bits de poids fort de la valeur	*
 *	2 - Creation du mot de 8 bits contenant l'adresse(msb) et (1)	*
 *	3 - Ecriture de 8 bits de poids faible de la valeur dans BASE_0	*
 *	4 - Ecriture du mot de 8 bits dans BASE_1			*
 *									*
 ************************************************************************/
void CIODAC16::daconv(int chan ,unsigned int valeur)
{
	unsigned char buffer;
	// prise des 8 bits de poids fort de valeur
	// filtrage pour n'avoir que les 4 bits de poids faible
	buffer = (valeur>>8) & 0x000f;
	// on récupére les 4 bits de poids faible de chan (contenant l'info) 
	// que l'on décale de 4 bits pour l'additionner avec buffer
	buffer = ((chan<<4) & 0x00f0) | buffer ;
	// envoie des données
	//sysOutByte (BASE_0,(unsigned char) valeur);
	//sysOutByte (BASE_1, buffer);
}
