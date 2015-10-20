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


   return true;
 }



 bool ClientUDP::client_send(char* buf, int size)
 {

   ODEBUGL("size:" << size,3);
   ODEBUGL("I did send data !",3);
  return true;
}


bool ClientUDP::client_recv(char* buf, int size)
{

  ODEBUGL("Received " << recvlen << "bytes",3);


  return true;
}
