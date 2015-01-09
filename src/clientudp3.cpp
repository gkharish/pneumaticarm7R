#include"clientudp3.h"
//#include "controlatclient.h"



ClientUDP::ClientUDP()
{
    //server =  char *server_address
}

ClientUDP::~ClientUDP()
{
    close(fd);
}
    
bool ClientUDP::client_start()
{
    server = "127.0.0.1"; // 169.254.7.183 // 192.168.101.2 //127.0.0.0
    if ((fd=socket(AF_INET, SOCK_DGRAM, 0))==-1)
    {
        perror("socket created failed");
        return 0;
    }
    

    /* bind it to all local addresses and pick any port number */

    memset((char *)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY); //INADDR_ANY or 0
    myaddr.sin_port = htons(SERVICE_PORT);
    
    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) 
    {
        perror("bind failed");
        return 0;
    }       
    
    /* now define remaddr, the address to whom we want to send messages */
    /* For convenience, the host address is expressed as a numeric IP address */
    /* that we will convert to a binary format via inet_aton */
    
    memset((char *) &remaddr, 0, sizeof(remaddr));
    remaddr.sin_family = AF_INET;
    remaddr.sin_port = htons(SERVICE_PORT);
    if (inet_aton(server, &remaddr.sin_addr)==0) 
    {
    	fprintf(stderr, "inet_aton() failed\n");
    	exit(1);
    }
    
    slen = sizeof(remaddr);
    
}
    
    

bool ClientUDP::client_send(char* buf, int size)
{
    
    	
	if (sendto(fd, buf, size, 0, (struct sockaddr *)&remaddr, slen)==-1) 
	{
		perror("sendto");
		exit(1);
	}
    
}
    
   
bool ClientUDP::client_recv(char* buf, int size)
{
    	
    recvlen = recvfrom(fd, buf, size, 0, (struct sockaddr *)&remaddr, &slen);
    if (recvlen >= 0)
    {
        buf[recvlen] = 0;	/* expect a printable string - terminate it */
        
    }
            
    
    return true;
}
    
    
    
    
