#ifndef CLIENTUDP3_H
#define CLIENTUDP3_H


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "port.h"

#include <iostream>
#define BUFLEN 2048



using namespace std;

class ClientUDP
{
    public: 
            //ClientUDP();
            ClientUDP();
            virtual ~ClientUDP();
                
                
            bool client_start();
            bool client_send(char* buf, int size);
            bool client_recv(char* buf, int size);
                
                
                  
    private:
                
            int n;          // Socket 
        
            
            int fd_recv_, fd_send_, i;
            //char buf[BUFLEN];	/* message buffer */
            socklen_t slen, addrlen;
            int recvlen;		/* # bytes in acknowledgement message */
            char* server;	/* change this to use a different server */
            struct sockaddr_in myaddr_send, myaddr_recv, remaddr_send, remaddr_recv;

};

#endif

