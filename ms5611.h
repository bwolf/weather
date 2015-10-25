#ifndef MS5611_H
#define MS5611_H

typedef struct ms5611_coeff {
    uint16_t c[8];
} ms5611_coeff_t;

typedef struct ms5611 {
    int16_t  temperature; // Temperature in decicelsius
    uint16_t pressure;    // Pressure in millis
} ms5611_t;

enum ms5611_oversampling {
    MS5611_OVERSAMPLING_256  = 0x00,
    MS5611_OVERSAMPLING_512  = 0x02,
    MS5611_OVERSAMPLING_1024 = 0x04,
    MS5611_OVERSAMPLING_2048 = 0x06,
    MS5611_OVERSAMPLING_4096 = 0x08
};

void ms5611_init(void);
void ms5611_read_calibration_coefficients(ms5611_coeff_t *coeff);
void ms5611_read_data(ms5611_t *ms5611, ms5611_coeff_t *coeff, enum ms5611_oversampling oversampling);

#endif // MS5611_H
