/* CIODAC16.hh - Entities of the CLASS of Output ioboards CIODAC16 */
/* refioboards : CIO-DAC16-I de Computer boards*/
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
#include "ioboards.hh"
#include "clientudp3.hh"
#include "port.hh"
//#include "sysLib.h"
#include <vxworks/vxworks.h>
#define MODULE_LICENSE(x)
#include <stdlib.h>
#include <stdio.h>
#define BUFLEN 2048
#include <fstream>
#include <sstream>
#include <Eigen/Core>
#include <math.h>
using namespace std;
using namespace Eigen;
/************************************************************************
 *									*
 *			CLASSE CIODAC16					*
 *									*
 ************************************************************************/

typedef struct udppacket_init                    // clientheader = '0';
{
  char CLIENT_HEADER[2];
  //double control_cmd;
  unsigned short ADC;
  unsigned short counters;
  unsigned short errors;
  unsigned short sampling_period;
  friend std::ostream& operator<<(std::ostream& os, const struct udppacket_init & obj)
  {
    // write obj to stream
    os << " " << obj.CLIENT_HEADER
       << " " << obj.ADC
       << " " << obj.counters
       << " " << obj.errors
       << " " << obj.sampling_period;

    return os;
  }

} client_packet_init;

typedef struct udppacket_control                    // clientheader = '0';
{
  char CLIENT_HEADER[2];
  //double control_cmd[3];
  //unsigned int control_cmd[16];
  unsigned short control_cmd[16];

  friend std::ostream& operator<<(std::ostream& os, const udppacket_control & obj)
  {
    // write obj to stream
    os << " " << obj.CLIENT_HEADER
       << " " << obj.control_cmd[0]
       << " " << obj.control_cmd[1]
       << " " << obj.control_cmd[2]
       << " " << obj.control_cmd[3]
       << " " << obj.control_cmd[4]
       << " " << obj.control_cmd[5]
       << " " << obj.control_cmd[6]
       << " " << obj.control_cmd[7]
       << " " << obj.control_cmd[8]
       << " " << obj.control_cmd[9]
       << " " << obj.control_cmd[10]
       << " " << obj.control_cmd[11]
       << " " << obj.control_cmd[12]
       << " " << obj.control_cmd[13]
       << " " << obj.control_cmd[14]
       << " " << obj.control_cmd[15]

      ;
    return os;
  }

} client_packet_control;

typedef struct udppacket_countersreset              // clientheader = '1';
{
  char CLIENT_HEADER[2];
  unsigned short data;
} client_packet_countersreset;


class CIODAC16 : public ioboards//, public  ClientUDP
{
 public :
  CIODAC16();//:  ClientUDP()
  //{}  // constructeur
  virtual ~CIODAC16();
  virtual void daconv(int chan , char header);
  void pressure_inidividualmuscle(int, double);
  void send_command_array(int, double);
  char*  buffer_send;
  udppacket_control send_packet;
  udppacket_countersreset send_packet_countersreset, send_packet_digitaloutputcontrol;
  udppacket_init send_packet_init;
  bool boundary_error_;
  //VectorXd send_array;
  double send_array[16];
  double get_send_array();
  ClientUDP* client_obj;
  void get_client(ClientUDP* parent_client);
  void SetBoundaryError(bool);
};

#endif
