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

	/*test_cmdarray[0]= 0x31;
	for(int lp =1; lp <33; lp++)
	{
		test_cmdarray[lp]=0x80;
		//test_cmdarray[lp+1]=0x80;

	}*/

	if(header == '1')
	{
		//std::cout << "CIOD16:daconv:header=1: error1" <<std::endl;

		send_packet.CLIENT_HEADER[0] = '1';
		send_packet.CLIENT_HEADER[1] = '1';

		send_array[0] = 0;
		send_array[1] = 0;
		send_array[2] = 0;
		send_array[3] = 0;
		send_array[14] = 0;
		send_array[15] = 0;
		for(int loop =0; loop < 16; loop++)
		{
			send_packet.control_cmd[loop] = (unsigned short)(13107.0*send_array[loop]);
		}
		buffer_send = (char*)&send_packet;
		client_obj->client_send(buffer_send, sizeof(send_packet));
		struct udppacket_control *asp_control = &send_packet;
    std::cout << "\n  CIODAC16 message: CONTROL_CMD (unsigned int): " << *asp_control << std::endl;
		//std::cout << "\n  CIODAC16 message: CONTROL_CMD (float): " << *asp_control/13107 << std::endl;
	}

	else if(header == '0')
	{
		send_packet_init.CLIENT_HEADER[0] = '0';
		send_packet_init.CLIENT_HEADER[1] = '0';

		send_packet_init.ADC = 0x7;
		send_packet_init.counters = 0x0;
		send_packet_init.errors = 0x0;
		send_packet_init.sampling_period = 100;

		buffer_send = (char*)&send_packet_init;

		client_obj -> client_send(buffer_send, sizeof(send_packet_init));
		//struct udppacket_init *asp_control1 = &send_packet_init;
    //std::cout << "\n  CIODAC16 message: To initialize NI-module init_packet sent: " << *asp_control1 << std::endl;
		printf("command raw packet \n");
		for(int lp =0; lp < sizeof(send_packet_init); lp++  )
		{
			unsigned char auc=buffer_send[lp];
			printf("0x%02x ",auc);
			if (lp%4==3)
				printf("\t");
		}
		printf("\n");
	}

	else if(header == '2')
	{
		send_packet_countersreset.CLIENT_HEADER[0] = '2';
		send_packet_countersreset.CLIENT_HEADER[1] = '2';

		send_packet_countersreset.data = true;
		buffer_send = (char*)&send_packet_countersreset;
		client_obj->client_send(buffer_send, sizeof(send_packet_countersreset));

	}
	else if(header == '3')
	{
		send_packet_digitaloutputcontrol.CLIENT_HEADER[0] = header;
		send_packet_digitaloutputcontrol.CLIENT_HEADER[1] = header;
		send_packet_countersreset.data = true;
		buffer_send = (char*)&send_packet_digitaloutputcontrol;
		client_obj -> client_send(buffer_send, sizeof(send_packet_digitaloutputcontrol));

	}

}
/*double get_send_array()
{
	return(send_array);
}*/
void CIODAC16::pressure_inidividualmuscle(int index, double pres)
{

			send_packet.CLIENT_HEADER[0] = '1';
			send_packet.CLIENT_HEADER[1] = '1';
			send_array[0] = 0;
			send_array[1] = 0.0;
			send_array[2] = 0.0;
			send_array[3] = 0.0;
			send_array[4] = 0;
			send_array[5] = 0;
			send_array[6] = 0;
			send_array[7] = 0;
			send_array[8] = 0;
			send_array[9] = 0;
			send_array[10] = 0;
			send_array[11] = 0;
			send_array[12] = 0;
			send_array[13] = 0;
			send_array[14] = 0;
			send_array[15] = 0;
			//*buffer_send = send_packet.CLIENT_HEADER;
			//buffer_send++;

			for(int loop =0; loop < 16; loop++)
			{
				send_packet.control_cmd[loop] = (unsigned short)(13107.0*send_array[loop]);

			}
			send_packet.control_cmd[index] = (unsigned short)(13107.0*pres);

			buffer_send = (char*)&send_packet;
			client_obj->client_send(buffer_send, sizeof(send_packet));
			struct udppacket_control *asp_control = &send_packet;
	    std::cout << "\n  CIODAC16 message: CONTROL_CMD to inidividual muscles : " << *asp_control << std::endl;

}
void CIODAC16::send_command_array(int index, double control_value)
{
	//VectorXd

	//send_packet.control_cmd[index] = control_value;
	send_array[index] = control_value;
}

void CIODAC16::get_client(ClientUDP* parent_client)
{
	client_obj = parent_client;
}
