#ifndef _NODE_TYPES_H
#define _NODE_TYPES_H

#include <stdint.h>

#define STRUCT_TYPE_WEATHERMAN 0x77

typedef struct weather_t
{
    uint8_t stype = STRUCT_TYPE_WEATHERMAN;

    uint8_t battery = 0;

    //! temperature in range [-100C .. 100C]
    uint16_t temperature = 0;

    //! pressure in range [0hPa .. 2000hPa]
    uint16_t pressure = 0;

    // relative humidity in range [0..1]
    uint8_t humidity = 0;

} weather_t;

#endif
