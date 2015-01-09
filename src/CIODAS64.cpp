/* CIODAS64.cpp - corps de la classe CIODAS64 */
/* refCarte : CIO-DAS6402/12 */
/* Erwan Guiochet - 2002 */

/*


Historique des modifications
----------------------------
Ajout des definitions des modes
Ajout de la fonction dread qui permet de lire les entrees numeriques
*/

#include "CIODAS64.h"
#include "clientudp3.h"
/****************************************************************************************
 * 											*
 * 			CONVERSION ANALOGIQUE -> NUMERIQUE				*
 * 				A PARTIR D'UNE ADRESSE					*
 * 											*
 ****************************************************************************************
 * 											*
 * 				ETAPE DE LA CONVERSION					*
 * 											*
 * 			1 - Ecriture des adresses pour l'aquisition			*
 * 			2 - Lecture des donnees (16 bits)				*
 * 			3 - Reecriture pour la conversion				*
 * 			4 - Delai d'attente pour la conversion				*
 * 			5 - Recuperation de la valeur lue				*
 * 			6 - Decalage de la valeur lue :					* 
 * 											*
 *		les quatres bits de poids faible ne serve pas avec la carte 12 bits	*
 *			il faut donc decaler tous les bits de 4 positions (>> 4)		*
 *
 ****************************************************************************************/
unsigned int CIODAS64::adconv(int chan)
{
	//unsigned int mot;	// mot de 12 bits
	
	/* select channel */	
	//mot = (chan + 2) & 0xff;
	//mot = (mot << 8);
	//mot = (mot | chan )& 0xffff;
	/*Ecriture des adresses*/
	//sysOutWord (CHAN_LIMITS,mot); 
		
	/* start convertion */
	/*Lecture du mot*/
	//mot = sysInWord(AD_DATA_REG);
	
	/*Re-Ecriture du mot */
	//sysOutWord (AD_DATA_REG, mot);
	//sysDelay();
	//sysDelay();
	
	/*Recuperation du mot*/
	//mot = sysInWord (AD_DATA_REG);

	//return (mot>>4);
	
	unsigned int val = 1;
	return(val);
}

/*Permet une initialisation de la carte
 ****************************************************************************************
 * 	******* MODE ENHANCED *********							*
 * 											*
 *	Single Ended									*
 *	Entree UNIPOLAIRE 5V								*
 *	Frequence du Pacer 10Mhz							*
 *	----- PAS DE DMA en mode Enhanced -----						*
 * 	pas d'interruption								*
 *	Burst mode OFF									*
 *	Post mode OFF									*
 *	Mode pre-trigger OFF								*
 * 	Conversion en software								*
 *	Sortie Numerique : +-5V								*
 *	fonction Trigger/Gate desactivee : 						*
 * 			toutes les entrees numeriques servent a l'aquisition		*
 *											*
 ****************************************************************************************
 *											*
 *		64 Entrees Analogiques, 8 entrees - sorties Numeriques			*
 *											*
 ****************************************************************************************/
void CIODAS64::initialisation ()
{
   	/******** Configuration du mode d'utilisation de la carte *******/

	// unipolar range, Single ended mode, ENHANCED MODE
	//sysOutByte (COMP_CNTRL,ENHANCED_MODE);
	//sysDelay();
	//sysOutByte (COMP_CNTRL,(unsigned char)(ENHANCED_MODE|SE_MODE|UNIPOLAR_RANGE|IN_5V));
   	   	
	/** configuration du registre des status ****/
	// passage en mode EXTEND
	//sysOutByte (STATUS_REG,EXTEND);
	// reglage de la cadence
	//sysOutByte (STATUS_REG,(unsigned char)(EXTEND|MHZ_10));
	// lib�ration des Msb ( 0 sur EXTEND)
	//sysOutByte (STATUS_REG,0x00);
	/******** Fin de configuration du registre des status *****/
	
	/******** Configuration du registre d'interruption et de control des Pacers */
	// Configuration sans interruption, sans bruste mode, avec une conversion A/D en sofware
	//sysOutByte (PACER_CNTRL,NO_INTERRUPT);
	/******** Configuration du registre de control des triggers *****/
	// Reglage sur du +- 5V sur les deux sorties, Mode pre-trigger OFF
	//sysOutByte (TRIG_CNTRL,OUT_5V);
}

/****************************************************************************************
 *											*
 *			LECTURE SUR LES ENTREES NUMERIQUES				*
 *			 ( a vide les entrees sont a '1')				*
 *											*
 ****************************************************************************************/
unsigned char CIODAS64::dread ()
{
	//return (sysInByte (DIGITAL_REG));
}



	