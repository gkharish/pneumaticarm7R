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
	char test_cmdarray[33];
	/*test_cmdarray[0]= 0x31;
	for(int lp =1; lp <33; lp++)
	{
		test_cmdarray[lp]=0x80;
		//test_cmdarray[lp+1]=0x80;

	}*/
	cout << "\n size of send_packetcbeforeif" <<sizeof(send_packet)<< endl;
	cout << "\n size of send_packet_init"<<sizeof(send_packet_init)<< endl;
	if(header == '1')
	{
		//std::cout << "CIOD16:daconv:header=1: error1" <<std::endl;
		cout << "\n size of send_packetcinsideif"<<sizeof(send_packet)<< endl;
		send_packet.CLIENT_HEADER = '1';
		send_array[14] = 0;
		send_array[15] = 0;
		//*buffer_send = send_packet.CLIENT_HEADER;
		//buffer_send++;
		cout << "\n size of send_packetc1"<<sizeof(send_packet)<< endl;
		for(int loop =0; loop < 16; loop++)
		{
			send_packet.control_cmd[loop] = (unsigned short)13107.0*send_array[loop];
			cout<<"\n sendpacket[loop]: " << send_packet.control_cmd[loop] << endl;
			//*(buffer_send + loop*2) =(char)13107.0*send_array[loop];
			cout << "\n size of send_packetc"<<loop<<"\t"<<sizeof(send_packet)<< endl;
		}
        //cout << "CIOD16:daconv:header=1: error2"  <<std::endl;
			//*buffer_send = (char)send_packet.CLIENT_HEADER;
			//buffer_send++;
			//buffer_send = (char*)&send_packet.control_cmd;
		cout << "\n size of send_packetc"<<sizeof(send_packet)<< endl;
		buffer_send = (char*)&send_packet;
		cout << "\n size of buffer_sendc"<<sizeof(buffer_send)<< endl;
    	//cout << "CIOD16:daconv:header=1: error3 " <<sizeof(send_packet) << std::endl ;
			//buffer_send = test_cmdarray;
			/*printf("test comdarray :");
			for(int lp =0; lp <sizeof(send_packet); lp++)
			{
				printf("%2x %x  %d", test_cmdarray[lp], *(buffer_send+lp), lp );
				printf("--");
			}
			printf("\n");*/
		//cout << "Buffer_send array :" << *buffer_send << endl;
		client_obj->client_send(buffer_send, sizeof(send_packet));
		//client_obj->client_send(buffer_send, 33);
		//cout << "CIOD16:daconv:header=1: error4" ;
		//struct udppacket_control *asp_control = &send_packet;
    //std::cout << "\n  CIODAC16 message: CONTROL_CMD (unsigned int): " << *asp_control << std::endl;
	}

	else if(header == '0')
	{
		send_packet_init.CLIENT_HEADER = '0';
		send_packet_init.ADC = 0x7;
		send_packet_init.counters = 0x0;
		send_packet_init.errors = 0x0;
		send_packet_init.sampling_period = 100;
		cout << "\n size of sendpacket_init"<<sizeof(send_packet_init)<< endl;
		buffer_send = (char*)&send_packet_init;
		cout << "\n size of buffer_send_init"<<sizeof(buffer_send)<< endl;
		client_obj->client_send(buffer_send, sizeof(send_packet_init));
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
