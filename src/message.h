#ifndef MESSAGE_H
#define MESSAGE_H

#include <stdint.h>
#define MESSAGE_SIZE 56


typedef struct message
{
	uint16_t src;
	uint16_t id;
	double x,y,z;
	double vx,vy,vz;
} Message;

#endif
