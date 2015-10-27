#ifndef MS5611_H
#define MS5611_H

typedef struct ms5611_coeff {
    uint16_t c[8];
} ms5611_coeff_t;

typedef struct ms5611 {
    int16_t  temperature; // Temperature in decicelsius
    uint16_t pressure;    // Pressure in millis
} ms5611_t;

void   ms5611_init(void);
void   ms5611_read_calibration_coefficients(ms5611_coeff_t *coeff);
int8_t ms5611_read_data(ms5611_t *ms5611, ms5611_coeff_t *coeff);

#endif // MS5611_H
