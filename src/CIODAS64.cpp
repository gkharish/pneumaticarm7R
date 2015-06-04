/* CIODAS64.cpp - corps de la classe CIODAS64 */
/* refioboards : CIO-DAS6402/12 */
/* Erwan Guiochet - 2002 */

/*


  Historique des modifications
  ----------------------------
  Ajout des definitions des modes
  Ajout de la fonction dread qui permet de lire les entrees numeriques
*/

#include "CIODAS64.hh"
#include <debug.hh>

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
 *		les quatres bits de poids faible ne serve pas avec la ioboards 12 bits	*
 *			il faut donc decaler tous les bits de 4 positions (>> 4)		*
 *
 ****************************************************************************************/
CIODAS64::CIODAS64()
{
  memset(recv_buffer,0,BUFLEN);
}

CIODAS64::~CIODAS64()
{

}

void CIODAS64::adconv(int )
{
  //cout << "\n ciodas64:adconv()debug1 " ;
  //	unsigned int val;
  //	float val1;

  //cout << "\n cioas64:adconv()debug1.2 " ;

  client_obj->client_recv(recv_buffer, BUFLEN);

  recv_packet_DAQ = (udppacket_DAQ *)recv_buffer;

  /*int lind=0;
  //printf("Sensor raw packet \n");
  {
  unsigned char auc=recv_buffer[lind];
  printf("0x%02x ",auc);
  lind+=4;
  }

  {
  unsigned int TheTrueLabel;
  char * cTheTrueLabel= (char *)&TheTrueLabel;

  cTheTrueLabel[0]=recv_buffer[4];
  cTheTrueLabel[1]=recv_buffer[5];
  cTheTrueLabel[2]=recv_buffer[6];
  cTheTrueLabel[3]=recv_buffer[7];

  printf("( 0x%02x%02x%02x%02x , %d, %d) ",
  (unsigned char)recv_buffer[lind++],
  (unsigned char)recv_buffer[lind++],
  (unsigned char)recv_buffer[lind++],
  (unsigned char)recv_buffer[lind++],
  (*recv_packet_DAQ).label,
  TheTrueLabel);
  }
  for(int lp =0; lp < 7; lp++  )
  {
  float TheTrueFloat;
  char * cTheTrueFloat= (char *)&TheTrueFloat;

  cTheTrueFloat[0]=recv_buffer[lind];
  cTheTrueFloat[1]=recv_buffer[lind+1];
  cTheTrueFloat[2]=recv_buffer[lind+2];
  cTheTrueFloat[3]=recv_buffer[lind+3];


  //printf("\t( 0x%02x%02x%02x%02x, %f )",
  printf("\t%f",
  (unsigned char)recv_buffer[lind++],
  (unsigned char)recv_buffer[lind++],
  (unsigned char)recv_buffer[lind++],
  (unsigned char)recv_buffer[lind++],
  TheTrueFloat);
  }
  printf("\n");*/

  if(recv_buffer[0] == 'a'&&recv_buffer[1] == 'a'&&recv_buffer[2] == 'a'&&recv_buffer[3] == 'a')
    {
      recv_packet_DAQ = (udppacket_DAQ *)recv_buffer ;
      
      ODEBUGL("\n Sensor's data: ",3);//<<(*recv_packet_DAQ);
#ifndef NDEBUG
#if DEBUG_LEVEL > 3

      printf("%x %u %f %f %f %f %f %f %f \n ",
	     (*recv_packet_DAQ).SERVER_HEADER[0],
	     ( (*recv_packet_DAQ).label), (*recv_packet_DAQ).data[0],
	     (*recv_packet_DAQ).data[1], (*recv_packet_DAQ).data[2],
	     (*recv_packet_DAQ).data[3], (*recv_packet_DAQ).data[4],
	     (*recv_packet_DAQ).data[5], (*recv_packet_DAQ).data[6]);
#endif
#endif
      //cout << (*recv_packet_DAQ).data[0];
      //struct udppacket_DAQ daq = recv_packet_DAQ;
      //std::cout << "\n  CIODAC16 message send is unsigned int control: " << *daq << std::endl;
      /*recv_data(0) = (*recv_packet_DAQ).data[0];
	recv_data(1) = (*recv_packet_DAQ).data[1];
    	val1 = (*recv_packet_DAQ).data[0];
    	val = (unsigned int ) val1;*/
    }


  else if	(recv_buffer[0] == 'b'&&recv_buffer[1] == 'b'&&recv_buffer[2] == 'b'&&recv_buffer[3] == 'b')
    {
      recv_packet_COUNTER = (udppacket_COUNTER *)recv_buffer;
      cout << "\n Encoder's data in loop b: " << endl;
      //recv_data(0) = 0;
    }

  else if(recv_buffer[0] == 'c'&&recv_buffer[1] == 'c'&&recv_buffer[2] == 'c'&&recv_buffer[3] == 'c')
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
  float val1;
  int index = axis_num;
  //cout << "\n cioads64:read_sensors()1 " << index;
  //recv_packet_DAQ = (udppacket_DAQ *)recv_buffer;
  val1 = (*recv_packet_DAQ).data[6- index];
  ODEBUGL(axis_num << " - cioads64:read_sensors() " << (double)val1 << " ",3);

  //float val2 = rand() % 10;
  //cout << "\n random  value in read_sensors: " << val2/10 << endl;
  return((double)val1);
}
void CIODAS64::openlogudpdata()
{
  udprecvlog.open("udprecv_datalog.txt");
}
void CIODAS64::logudpdata()
{

  udprecvlog << (*recv_packet_DAQ).data[0] << "\t" << (*recv_packet_DAQ).data[1] << "\t"
	     << (*recv_packet_DAQ).data[2] << "\t" << (*recv_packet_DAQ).data[3] << "\t"
	     << (*recv_packet_DAQ).data[4] << "\t" << (*recv_packet_DAQ).data[5] << "\t"
	     << (*recv_packet_DAQ).data[6] << "\n" << endl;
}
/*Permet une initialisation de la ioboards
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
  /******** Configuration du mode d'utilisation de la ioboards *******/

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
  return 0;
}


void CIODAS64::get_client(ClientUDP* parent_client)
{
  client_obj = parent_client;
}
