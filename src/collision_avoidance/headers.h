#ifndef SER_CLI_H
#define SER_CLI_H
#include "message.h"
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <stdint.h>
int server_init(int port);
Message* server_receive(int socketfd);

int client_init(char * host, int port, struct sockaddr_in *server);
int client_send(int fd, struct sockaddr_in *server, Message *m);

#endif
