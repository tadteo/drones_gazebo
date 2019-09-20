#include <stdio.h> 
#include <errno.h>
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include "message.h"  



int client_init(char * host, int port, struct sockaddr_in *server)
{   
	int sockfd; 

	// Creating socket file descriptor 
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
		perror("socket creation failed"); 
		exit(EXIT_FAILURE); 
	} 

	//memset(&servaddr, 0, sizeof(servaddr)); 

	// Filling server information 
	server->sin_family = AF_INET; 
	server->sin_port = htons(port); 
	server->sin_addr.s_addr = inet_addr(host); 

	return sockfd;
}

int client_send(int fd, struct sockaddr_in *server, Message *m)
{
	return sendto(fd, m, MESSAGE_SIZE, 
			MSG_CONFIRM, (const struct sockaddr *) server,  
			sizeof(*server)); 

}

