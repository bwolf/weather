// bmp085.h - Bosch BMP085 Sensor

#ifndef BMP085_H
#define BMP085_H

// Define this to get error messages via uart
#undef BMP085_WITH_UART_ERROR_MSGS

// Wait 4.5ms
#define BMP085_OVERSAMPLING_ULTRA_LOW_POWER       0
// Wait 7.5ms
#define BMP085_OVERSAMPLING_STANDARD              1
// Wait 13.5ms
#define BMP085_OVERSAMPLING_HIGH_RESOLUTION       2
// Wait 25.5ms
#define BMP085_OVERSAMPLING_ULTRA_HIGH_RESOLUTION 3

typedef struct bmp085 {
    // Calibration coefficients from BMP085 eeprom
    int16_t  AC1;
    int16_t  AC2;
    int16_t  AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t  B1;
    int16_t  B2;
    int16_t  MB;
    int16_t  MC;
    int16_t  MD;
} bmp085_t;

void bmp085_init(bmp085_t *b085);
void bmp085_read_calibration_coefficients(bmp085_t *b085);

uint16_t bmp085_read_ut(void);
uint32_t bmp085_read_up(uint8_t oss);

int16_t bmp085_calculate_temperature(uint16_t ut, int32_t *B5, const bmp085_t * const b085);
int32_t bmp085_calculate_true_pressure(uint32_t up, const int32_t * const B5, const bmp085_t * const b085, uint8_t oss);
float bmp085_calculate_altitude(int32_t p);
uint32_t bmp085_calculate_pressure_nn(int32_t p, uint16_t altitude);

typedef struct bmp085_results {
    int16_t decicelsius; // Temperature in deci degrees C
    int32_t pressure;    // True pressure
} bmp085_results_t;

// All in one functionality
void bmp085_read(bmp085_results_t *res, const bmp085_t * const bmp085);

// BMP085_H
#endif
