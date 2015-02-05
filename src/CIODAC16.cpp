/* CIODAC16.cpp - corps de la classe carteSortie */
/* refCarte : CIO-DAC16-I de Computer boards*/
/* Erwan Guiochet - 2002 */

/*
Historique des modifications
----------------------------

*/
#include "CIODAC16.h"
#include <string.h>


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
CIODAC16::CIODAC16()
{
	
//	ClientUDP::~ClientUDP();
}

CIODAC16::~CIODAC16()
{
	
//	ClientUDP::~ClientUDP();
}

void CIODAC16::daconv(int chan , char header)
{
	char* buffer_send;
	//ClientUDP *client;
	send_packet.CLIENT_HEADER = header;//'0';
	//value(14) = 0; // control value for the tool
	send_array[14] = 0;
	for(int loop =0; loop < 15; loop++)
	{
		send_packet.control_cmd[loop] = send_array[loop];
	}
    ///4095;//value;
    //client_start();
                
    buffer_send = (char*)&send_packet;
	
	
	client_obj->client_send(buffer_send, sizeof(send_packet));
	struct udppacket_control *asp_control = &send_packet;
    std::cout << "\n  CIODAC16 message send is unsigned int control: " << *asp_control << std::endl;
	 
}
/*double get_send_array()
{
	return(send_array);
}*/

void CIODAC16:: send_command_array(int index, double control_value)
{
	//VectorXd 
	
	//send_packet.control_cmd[index] = control_value;
	send_array[index] = control_value;
}

void CIODAC16::get_client(ClientUDP* parent_client)
{
	client_obj = parent_client;
}
