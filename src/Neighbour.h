#ifndef NEIGHBOUR_H
#define NEIGHBOUR_H

#include <stdint.h>

typedef struct neighbour
{
    uint16_t id_;
    double x,y,z;
    double vx,vy,vz;

} Neighbour;

#endif