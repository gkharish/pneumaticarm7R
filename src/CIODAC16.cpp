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
	//'0';
	//value(14) = 0; // control value for the tool
	if(header == '1')
	{	
		//std::cout << "CIOD16:daconv:header=1: error1" <<std::endl;
		send_packet.CLIENT_HEADER = '1';
		send_array[14] = 0;
		send_array[15] = 0;
		for(int loop =0; loop < 16; loop++)
		{
			send_packet.control_cmd[loop] = send_array[loop];
		}
        //cout << "CIOD16:daconv:header=1: error2"  <<std::endl;       
    	buffer_send = (char*)&send_packet;
    	//cout << "CIOD16:daconv:header=1: error3 " <<sizeof(send_packet) << std::endl ;
		client_obj->client_send(buffer_send, sizeof(send_packet));
		//cout << "CIOD16:daconv:header=1: error4" ;
		struct udppacket_control *asp_control = &send_packet;
    	std::cout << "\n  CIODAC16 message: CONTROL_CMD (unsigned int): " << *asp_control << std::endl;
	}
	
	else if(header == '0')
	{
		send_packet_init.CLIENT_HEADER = '0';
		send_packet_init.ADC = 0x7;
		send_packet_init.counters = 0x0;
		send_packet_init.errors = 0x0;
		send_packet_init.sampling_period = 100;
		buffer_send = (char*)&send_packet_init;
		
		client_obj->client_send(buffer_send, sizeof(send_packet_init));
		struct udppacket_init *asp_control1 = &send_packet_init;
    	std::cout << "\n  CIODAC16 message: To initialize NI-module init_packet sent: " << *asp_control1 << std::endl;
	
	}
	
	else if(header == '2')
	{
		send_packet_countersreset.CLIENT_HEADER = header;
		send_packet_countersreset.data = true;
		buffer_send = (char*)&send_packet_countersreset;
		client_obj->client_send(buffer_send, sizeof(send_packet_countersreset));
		
	}
	else if(header == '3')
	{
		send_packet_digitaloutputcontrol.CLIENT_HEADER = header;
		send_packet_countersreset.data = true;
		buffer_send = (char*)&send_packet_digitaloutputcontrol;
		client_obj -> client_send(buffer_send, sizeof(send_packet_digitaloutputcontrol));
		
	}
	 
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
