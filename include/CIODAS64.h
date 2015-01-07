/* CIODAS64.h - entetes de la classe CIODAS64 */
/* refCarte : CIO-DAS6402/12 */
/* Erwan Guiochet - 2002 */


#ifndef CIODAS
#define CIODAS

/****************************************************************************************/
/*					DEFINES			     			*/
/****************************************************************************************/           

/****************************************************************************************/
/*				ADRESSES DES REGISTRES					*/
/****************************************************************************************/			  	
#define AD_DATA_REG	(BASE_REG_CIODAS64)    	/* Registre des donnees et de conversion*/
#define CHAN_LIMITS	(BASE_REG_CIODAS64+2)  	/* Registre de reglage des adresses 	*/
#define DIGITAL_REG	(BASE_REG_CIODAS64+3) 	/* Registre des donnees Numerique	*/
#define DAC_OUT_0       (BASE_REG_CIODAS64+4)  	/* Sortie Numerique DAC1	 	*/
#define DAC_OUT_1       (BASE_REG_CIODAS64+6)  	/* Sortie Numerique DAC2 		*/
#define STATUS_REG      (BASE_REG_CIODAS64+8)  	/* Registre des status                 	*/
#define PACER_CNTRL     (BASE_REG_CIODAS64+9)  	/* Control des Interruptions & Pacer  	*/
#define TRIG_CNTRL	(BASE_REG_CIODAS64+10) 	/* Registre de configuration du trigger	*/
#define COMP_CNTRL     	(BASE_REG_CIODAS64+11) 	/* Registre de configuration de la carte*/
													  	
#define COMPTEUR_0  	(BASE_REG_CIODAS64+12) 	/* Registre du Compteur 0 	    	*/
#define COMPTEUR_1  	(BASE_REG_CIODAS64+13) 	/* Registre du compteur 1 	    	*/
#define COMPTEUR_2	(BASE_REG_CIODAS64+14) 	/* Registre du compteur 2	    	*/
#define COUNTER_CONTROL (BASE_REG_CIODAS64+15) 	/* Registre de control des compteurs  	*/

/*______________________________________________________________________________________*/
/****************************************************************************************/
/*				DETAILS DES REGISTRES				  	*/
/****************************************************************************************/	


/****************************************************************************************/
/*				REGISTRE DES STATUS					*/
/****************************************************************************************/
#define MHZ_1			0x00	  	/* Reglage de clock pacer � 1 Mhz  	*/
#define MHZ_10			0x80 	  	/* Reglage � 10 Mhz		  	*/   
#define POST_MODE 		0x40	  	/* Post-mode			  	*/
#define EXTEND			0x10		/* Bit d'autorisation de configuration	*/
#define CLRALL    		0x07	  	/* efface toutes les interruptions 	*/
#define CLRINT			0x01	  	/* efface les interruptions 	  	*/
#define CLRXTR			0x02	  	/* efface les Triggers Externe 		*/
#define CLRXIN			0x04	  	/* efface les interruptions externe	*/


/****************************************************************************************/
/*		REGISTRE DE CONTROL DES INTERRUPTIONS ET DU PACER			*/
/****************************************************************************************/
#define NO_INTERRUPT		0x00	  	/* Pas d'interruption			*/   
#define BURSTE_MODE		0x40	  	/* mode Burste ON			*/
#define INTE			0x80	  	/* Interruptions analogiques autorisees	*/
#define XINTE       		0x08	  	/* Les Entrees Externes sont autorisees	*/
/***********************************************/
/** 	  niveau d'interruption 	      **/
/***********************************************/
#define NONE_LEVEL		0x00	  	/* pas de niveau d'Interruption		*/
#define LEVEL_11		0x10		/* interrupt level = 11 in mode Eh.	*/
#define LEVEL_2			0x20		/* interrupt level = 2 in modes C/E	*/
#define LEVEL_3			0x30		/* interrupt level = 3 in modes C/E	*/
#define LEVEL_10		0x40		/* interrupt level = 4 in mode Eh. 	*/
#define LEVEL_5			0x50		/* interrupt level = 5 in modes C/E	*/
#define LEVEL_15  		0x60		/* interrupt level = 15 in mode Eh.	*/
#define LEVEL_6 		0x60		/* interrupt level = 6 in mode Comp	*/
#define LEVEL_7 		0x70		/* interrupt level = 7 in modes E/C	*/
/***********************************************/
/** 		mode de conversion	      **/
/***********************************************/
#define SOFT_CONVERT		0x00		/* conversion en Software 		*/
#define XFALL			0x01		/* External Pacer Falling Edge		*/
#define XRIS			0x02		/* External Pacer Rising Edge		*/
#define INT_PACER		0x02		/* Internal Pacer			*/
/***********************************************/
/** 	specifique au mode COMPATIBLE 	      **/
/***********************************************/
#define DIN_IN			0x01		/* Enable DIN0 input to gate the pacer	*/
#define COUNT_CNTRL		0x02		/* 100 khz en entreesur le compteur 0	*/


/****************************************************************************************/
/*		REGISTRE DE CONTROL DU TRIGGER ET REGLAGE DU NUMERIC		  	*/
/****************************************************************************************/
#define OUT_5V			0x00 		/* Reglage � +- 5v les 2 sorties   	*/   
#define OUT_10V_DAC0 		0x10		/* Reglage � +- 10v la sortie DAC0 	*/
#define OUT_0_5V_DAC0   	0x20		/* Reglage de 0 � 5v la sortie DAC0	*/
#define OUT_0_10V_DAC0  	0x30		/* Reglage de 0 � 5v la sortie DAC0	*/
 
#define OUT_10V_DAC1 		0x40		/* Reglage � +- 10v la sortie DAC1	*/
#define OUT_0_5V_DAC1  		0x80		/* Reglage de 0 � 5v la sortie DAC1	*/
#define OUT_0_10V_DAC1  	0xC0		/* Reglage de 0 � 5v la sortie DAC1	*/

#define TG_EN 			0x01		/* Active la fonction Trigger/Gate 	*/
#define TG_SEL			0x03		/* Trigger/Gate select bit		*/
#define TG_POL			0x07		/* Trigger/Gate polarity bit		*/
#define PRETRIG			0x09		/* Stop pacing a certain number of conversion*/

/****************************************************************************************/
/*			REGISTRE DE CONFIGURATION DE LA CARTE				*/
/****************************************************************************************/
#define DMA_1			0x00		/* R�glage sue le DMA 1			*/
#define DMA_3			0x80		/* Reglage sur le DMA 3			*/   
#define UNIPOLAR_RANGE  	0x40		/* MODE UNIPOLAIRE			*/
#define BIPOLAR_RANGE   	0x00		/* MODE BIPOLAIRE			*/
#define SE_MODE    		0x20		/* mode Single-ended 		 	*/
#define DIFF_MODE		0x00		/* mode differential 		 	*/
#define ENHANCED_MODE  		0x10	  	/* mode Enhanced 			*/
#define COMPATIBLE_MODE 	0x00		/* mode compatible			*/
#define IN_10V			0x00		/* Entr�e de 0 � 10 V ou +-10V	  	*/
#define IN_5V			0x01		/* Entr�e de 0 � 5 V ou +- 5V	  	*/		 
#define IN_2_5V			0x02		/* Entr�e de 0 � 2.5 V ou +-2.5V	*/
#define IN_1_25V		0x03		/* Entr�e de 0 � 1.25 V ou +-1.25V 	*/

/****************************************************************************************
 *   			    						      		*
 *					INCLUDE				      		*
 *   			    						      		*
 ****************************************************************************************/
//#include "sysLib.h"
#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)
/****************************************************************************************
 *   			    						      		*
 *   			    	   CLASSE CIODAS64			      		*
 *   			    						      		*
 ****************************************************************************************/
#include "carte.h"

class CIODAS64 : public carte
{
	public :
		CIODAS64(){}
		virtual void initialisation ();
		virtual unsigned int adconv(int chan);
	 	virtual unsigned char dread ();
	
};

#endif