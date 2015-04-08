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
CIODAS64::CIODAS64()
{

}

CIODAS64::~CIODAS64()
{

}

void CIODAS64::adconv(int chan)
{
	//cout << "\n ciodas64:adconv()debug1 " ;
	unsigned int val;
	float val1;
	recv_buffer[1] = 't';
	//cout << "\n cioas64:adconv()debug1.2 " ;

  memset(recv_buffer,0,BUFLEN);
	client_obj->client_recv(recv_buffer, BUFLEN);

	recv_packet_DAQ = (udppacket_DAQ *)recv_buffer;

	int lind=0;
	printf("Sensor raw packet \n");
	{
	  unsigned char auc=recv_buffer[lind++];
	  printf("0x%02x ",auc);
  }

	{
	  printf("( 0x%02x%02x%02x%02x , %d) ",
		  (unsigned char)recv_buffer[lind++],
			(unsigned char)recv_buffer[lind++],
			(unsigned char)recv_buffer[lind++],
			(unsigned char)recv_buffer[lind++],
			(*recv_packet_DAQ).label);
  }
	for(int lp =0; lp < 4; lp++  )
	{
		printf("\t0x%02x%02x%02x%02x ",
		  (unsigned char)recv_buffer[lind++],
			(unsigned char)recv_buffer[lind++],
			(unsigned char)recv_buffer[lind++],
			(unsigned char)recv_buffer[lind++]);
	}
  printf("\n");

    if(recv_buffer[0] == 'a')
    {
    	recv_packet_DAQ = (udppacket_DAQ *)recv_buffer;
    	cout << "\n Sensor's data: " << endl;//<<(*recv_packet_DAQ);
    	printf(" %x %x %x %x %x %x %x %x \n ",
				(*recv_packet_DAQ).label, (*recv_packet_DAQ).data[0],
				(*recv_packet_DAQ).data[1], (*recv_packet_DAQ).data[2],
				(*recv_packet_DAQ).data[3], (*recv_packet_DAQ).data[4],
				(*recv_packet_DAQ).data[5], (*recv_packet_DAQ).data[6]);
    	//cout << (*recv_packet_DAQ).data[0];
    	//struct udppacket_DAQ daq = recv_packet_DAQ;
    	//std::cout << "\n  CIODAC16 message send is unsigned int control: " << *daq << std::endl;
    	/*recv_data(0) = (*recv_packet_DAQ).data[0];
		recv_data(1) = (*recv_packet_DAQ).data[1];
    	val1 = (*recv_packet_DAQ).data[0];
    	val = (unsigned int ) val1;*/
   	}


    else if	(recv_buffer[0] == 'b')
    {
		recv_packet_COUNTER = (udppacket_COUNTER *)recv_buffer;
		cout << "\n Encoder's data in loop b: " << endl;
    	//recv_data(0) = 0;
    }

    else if(recv_buffer[0] == 'c')
	{
		recv_packet_error = (udppacket_error *)recv_buffer;
		cout << "\n Error's data in loop c: " << endl;
		//recv_data(0) = 0;
	}


    //return(recv_data);
}

double CIODAS64::read_sensors(int axis_num)
{
	//cout << "\n cioads64:read_sensors()0 ";
	double val;
	float val1;
	int index = axis_num;
	//cout << "\n cioads64:read_sensors()1 " << index;
	//recv_packet_DAQ = (udppacket_DAQ *)recv_buffer;
    val1 = (*recv_packet_DAQ).data[index -1];
    val = (double ) val1;
    //cout << "\n cioads64:read_sensors()2 " << val1;

    float val2 = rand() % 10;
		//cout << "\n random  value in read_sensors: " << val2/10 << endl;
    return(val2/10);
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


void CIODAS64::get_client(ClientUDP* parent_client)
{
	client_obj = parent_client;
}
