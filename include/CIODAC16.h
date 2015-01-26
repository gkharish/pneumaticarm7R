/* CIODAC16.h - entetes de la classe carteSortie */
/* refCarte : CIO-DAC16-I de Computer boards*/
/* Erwan Guiochet - 2002 */

#ifndef CIODAC
#define CIODAC


/************************************************************************
 *									*
 *				DEFINES					*
 *									*
 ************************************************************************/

#define BASE_0 (BASE_REG_CIODAC16) 	/* 	       D/A LSB 	        */
#define BASE_1 (BASE_REG_CIODAC16+1)	/* D/A MSB & channel addresse 	*/

/************************************************************************
 *  Calcul de la valeur de sortie 					*
 *									*
 *	soit X(99v9999999) en mA la valeur que l'on veut :		*
 * 	CODE = ((X - 4)/16) x 4095					*
 *	si CODE = 0 alors X = 4						*
 *	si CODE = 4095 alors X = 20					*
 * le code est en 12 bit, les 4 autres sont pour le choix de la sortie  *
 *									*
 ************************************************************************/

/************************************************************************
 *									*
 *				INCLUDES				*
 *									*
 ************************************************************************/
#include "carte.h"
#include "clientudp3.h"
#include "port.h"
//#include "sysLib.h"
#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)
#include <stdlib.h>
#include <stdio.h>
#define BUFLEN 2048
#include <fstream>
#include <sstream>

using namespace std;

/************************************************************************
 *									*
 *			CLASSE CIODAC16					*
 *									*
 ************************************************************************/

struct udppacket_control                    // clientheader = '0';
{
    char CLIENT_HEADER;
    //double control_cmd[3];
    unsigned int control_cmd;
}client_packet_control;

std::ostream& operator<<(std::ostream& os, const struct udppacket_control & obj)
{
    // write obj to stream
     os << " " << obj.CLIENT_HEADER 
	<< " " << obj.control_cmd
	;
    return os; 
} 

class CIODAC16 : public carte//, public  ClientUDP
{	 
	public :
		CIODAC16();//:  ClientUDP()
		//{}  // constructeur
		virtual ~CIODAC16();
		virtual void daconv(int chan ,unsigned int valeur);
		char*  buffer_send;
		udppacket_control send_packet;
		
		ClientUDP* client_obj;
		void get_client(ClientUDP* parent_client);
};

#endif