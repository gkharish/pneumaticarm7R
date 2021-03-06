/* CIODAC16.cpp - corps de la classe ioboardsSortie
   refioboards : CIO-DAC16-I de Computer boards
   Erwan Guiochet - 2002
*/

/*
  Historique des modifications
  ----------------------------
*/

#include <string>

#include <debug.hh>

#include <CIODAC16.hh>


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
    boundary_error_ = false;
}

CIODAC16::~CIODAC16()
{
}
void CIODAC16::SetBoundaryError(bool idx)
{
    boundary_error_ = idx;
}
void CIODAC16::daconv(int  , char header)
{
  char* buffer_send;

  if(header == '1')
    {
      //ODEBUGL("CIOD16:daconv:header=1: error1",4);
     if(boundary_error_ == false)
       {
         send_packet.CLIENT_HEADER[0] = '1';
         send_packet.CLIENT_HEADER[1] = '1';

         /*      send_array[0] = 0;
         send_array[1] = 0;
         send_array[2] = 0;
         send_array[3] = 0; */
         //send_array[14] = 0;
         //send_array[15] = 0;
         for(int loop =0; loop < 16; loop++)
            {
	      send_packet.control_cmd[loop] = (unsigned short)(13107.0*send_array[loop]);
            }
        buffer_send = (char*)&send_packet;
       }

      else
      {
        for(int loop =0; loop < 16; loop++)
	{
	  send_packet.control_cmd[loop] = 0.0;
	}
        buffer_send = (char*)&send_packet;
        cout<< "BOUNDARY ERROR ------- /_\!!!"  << endl;
      }
     client_obj->client_send(buffer_send, sizeof(send_packet));
     struct udppacket_control *asp_control = &send_packet;

#ifndef NDEBUG
#if DEBUG_LEVEL > 3
      //ODEBUGL("\n  CIODAC16 message: CONTROL_CMD (unsigned int): " << *asp_control,2);
#endif
#endif

    }

  else if(header == '0')
    {
      send_packet_init.CLIENT_HEADER[0] = '0';
      send_packet_init.CLIENT_HEADER[1] = '0';

      send_packet_init.ADC = 0x9;
      send_packet_init.counters = 0x1;
      send_packet_init.errors = 0x0;
      send_packet_init.sampling_period = 20;

      buffer_send = (char*)&send_packet_init;

      client_obj -> client_send(buffer_send, sizeof(send_packet_init));
#ifndef NDEBUG
#if DEBUG_LEVEL > 3

      struct udppacket_init *asp_control1 = &send_packet_init;
      //ODEBUGL("\n  CIODAC16 message: To initialize NI-module init_packet sent: " << *asp_control1,3);
#endif
#endif
      //ODEBUGL("command raw packet \n",3);
#ifndef NDEBUG
#if DEBUG_LEVEL > 2
      for(unsigned int lp =0; lp < sizeof(send_packet_init); lp++  )
	{
	  unsigned char auc=buffer_send[lp];
	    printf("\t");
	}
      printf("\n");
#endif
#endif
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
  for(unsigned int i =0; i <16; i++)
      send_array[i]=0.0;
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
  //ODEBUGL("\n  CIODAC16 message: CONTROL_CMD to inidividual muscles : " << *asp_control,3);

#ifndef NDEBUG
#if DEBUG_LEVEL > 3
  #warning "Compile here"
      for(unsigned int lp =0; lp < sizeof(send_packet); lp++  )
	{
	  unsigned char auc=buffer_send[lp];
	  printf("0x%02x ",auc);
	  if (lp%4==3)
	    printf(" ");
	}
      printf("\n");
#endif
#endif

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
