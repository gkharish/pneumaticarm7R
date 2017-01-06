#include"clientudp3.hh"
#include <debug.hh>

ClientUDP::ClientUDP()
{
  //server =  char *server_address
}

ClientUDP::~ClientUDP()
{
  close(fd_recv_);
  close(fd_send_);
}

bool ClientUDP::client_start()
{
  server = "192.168.101.2" ; //"127.0.0.1"; // 169.254.7.183 // 192.168.101.2 //127.0.0.0
  if ((fd_recv_=socket(AF_INET, SOCK_DGRAM, 0))==-1)
    {
      perror("socket created failed");
      return false;
    }


  /* bind it to all local addresses and pick any port number */

  memset((char *)&myaddr_recv, 0, sizeof(myaddr_recv));
  myaddr_recv.sin_family = AF_INET;
  myaddr_recv.sin_addr.s_addr = inet_addr("192.168.101.1"); //htonl(INADDR_ANY); // //INADDR_ANY or 0
  myaddr_recv.sin_port = htons(SERVICE_PORT_recv); //hotns(0) for local server and htons(SERVICE_PORT) in case of NI server

  if (bind(fd_recv_, (struct sockaddr *)&myaddr_recv, sizeof(myaddr_recv)) < 0)
    {
      perror("bind failed");
      return false;
    }

  memset((char *) &remaddr_send, 0, sizeof(remaddr_send));
  remaddr_send.sin_family = AF_INET;
  remaddr_send.sin_port = htons(SERVICE_PORT_recv);
  if (inet_aton(server, &remaddr_send.sin_addr)==0)
    {
      fprintf(stderr, "inet_aton() failed\n");
      exit(1);
    }

  /* now define remaddr, the address to whom we want to send messages */
  /* For convenience, the host address is expressed as a numeric IP address */
  /* that we will convert to a binary format via inet_aton */

  // now opening another port to receive from NI module
   if ( ( fd_send_ = socket(AF_INET, SOCK_DGRAM, 0) ) < 0)
     {
       perror("cannot create socket\n");
       return 0;
     }

   memset((char *)&myaddr_send, 0, sizeof(myaddr_send));
   myaddr_send.sin_family = AF_INET;
   myaddr_send.sin_addr.s_addr = inet_addr("192.168.101.1");//htonl(INADDR_ANY);
   myaddr_send.sin_port = htons(SERVICE_PORT_send);
   if (bind(fd_send_, (struct sockaddr *)&myaddr_send, sizeof(myaddr_send)) < 0)
     {
       perror("bind failed");
       return 0;
     }
   memset((char *) &remaddr_recv, 0, sizeof(remaddr_recv));
   remaddr_recv.sin_family = AF_INET;
   remaddr_recv.sin_addr.s_addr = inet_addr("192.168.101.2");
   remaddr_recv.sin_port = htons(SERVICE_PORT_send);
   if (inet_aton(server, &remaddr_recv.sin_addr)==0)
     {
       fprintf(stderr, "inet_aton() failed\n");
       exit(1);
     }

   slen = sizeof(remaddr_recv);
   addrlen = sizeof(remaddr_send);

   return true;
 }



 bool ClientUDP::client_send(char* buf, int size)
 {

   //ODEBUGL("size:" << size,3);
   //ODEBUGL("fd_send_: " << fd_send_,3);
   if (sendto(fd_send_, buf, size, 0, (struct sockaddr *)&remaddr_recv, slen)==-1)
     {
       perror("sendto");
       std::cerr << "Problem when sending to " << std::endl;
       return false;
     }
   //ODEBUGL("I did send data !",3);
  return true;
}


bool ClientUDP::client_recv(char* buf, int size)
{

  recvlen = recvfrom(fd_recv_, buf, size, 0, (struct sockaddr *)&remaddr_send, &addrlen);
  if (recvlen >= 0)
    {
      buf[recvlen] = 0;	/* expect a printable string - terminate it */

    }
  //ODEBUGL("Received " << recvlen << "bytes",3);


  return true;
}
