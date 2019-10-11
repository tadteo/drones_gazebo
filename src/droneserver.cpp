// Server side implementation of UDP client-server model 
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <stdint.h>
#include "message.h"
#include <sys/time.h>

int server_init(int port)
{

	struct sockaddr_in servaddr; 
	int sockfd; 
	// Creating socket file descriptor 
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
		perror("socket creation failed"); 
		exit(EXIT_FAILURE); 
	} 
	struct timeval read_timeout;
	read_timeout.tv_sec = 0;
	read_timeout.tv_usec = 10;
	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

	memset(&servaddr, 0, sizeof(servaddr)); 

	// Filling server information 
	servaddr.sin_family    = AF_INET; // IPv4 
	servaddr.sin_addr.s_addr = INADDR_ANY; 
	servaddr.sin_port = htons(port); 
	// Bind the socket with the server address 
	if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
				sizeof(servaddr)) < 0 ) 
	{ 
		perror("bind failed"); 
		exit(EXIT_FAILURE); 
	} 
	return sockfd;  
}


Message* server_receive(int socketfd)
{
	int n; 	
	Message *buffer=(Message *)malloc(MESSAGE_SIZE); 

	n = recvfrom(socketfd, buffer, MESSAGE_SIZE,  0, NULL, NULL);
	if (n== MESSAGE_SIZE)
	{

		return buffer;		
	}
	else
	{
		free(buffer);
		return NULL;
	}
}

