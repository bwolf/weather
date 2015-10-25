// pressure.c -- pressure calculations

#include "config.h"

#include <stdint.h>
#include <math.h>

float pressure_calculate_altitude(int32_t p)
{
    // TODO most of this is constant
    const float p0 = 101325; // Pressure at sea level in Pa
    return (float) 44330 * (1 - pow(((float) p/p0), 0.190295));
}

// NN is normal null
uint32_t pressure_calculate_pressure_nn(int32_t p)
{
    // Calculation according to Bosch data sheet
    // TODO most of this is constant
    return (uint32_t) (((float) p) / pow((float)1 - ((float) ALTITUDE_SENSOR_LOCATION / (float) 44330), (float) 5.255));
}

// NN is normal null, truncate a value like 102464 meaning 1024,64 to 10246 fitting into an uint16_t.
uint16_t pressure_calculate_pressure_nn16(int32_t p)
{
    // Calculation according to Bosch data sheet, value/10 rounded
    // TODO most of this is constant
    return (uint16_t) roundf((((float) p) / pow((float)1 - ((float) ALTITUDE_SENSOR_LOCATION / (float) 44330), (float) 5.255)) / 10.f);
}

// EOF
