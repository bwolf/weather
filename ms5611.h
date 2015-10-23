#ifndef MS5611_H
#define MS5611_H

typedef struct ms5611 {
    int16_t  temperature; // Temperature in decicelsius
    uint16_t pressure;    // Pressure in millis
} ms5611_t;

void ms5611_read(ms5611_t *ms5611);

#endif // MS5611_H
