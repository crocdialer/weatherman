#ifndef _NODE_TYPES_H
#define _NODE_TYPES_H

#include <stdint.h>

#define STRUCT_TYPE_WEATHERMAN 0x77

typedef struct weather_t
{
    uint8_t stype = STRUCT_TYPE_WEATHERMAN;
    uint8_t battery = 0;
} weather_t;

#endif
