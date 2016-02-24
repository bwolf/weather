/* Copyright 2016 Marcus Geiger. All rights reserved. Use of this
 * source code is governed by a Apache 2.0 license that can be found
 * in the LICENSE file.
 *
 * bmp085.h - Bosch BMP085 Sensor
 *
 * Commentary
 *
 * Define BMP085_WITH_UART_ERROR_MSGS in config.h to get error messages via uart.
 */

#ifndef BMP085_H
#define BMP085_H

typedef struct bmp085 {
    int16_t decicelsius;  // Temperature in deci degrees C
    uint16_t pressure_nn; // True pressure at altitude NN
} bmp085_t;

#ifndef BMP085_DATA_TYPE_ONLY

typedef struct bmp085_coeff {
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
} bmp085_coeff_t;

void bmp085_init(bmp085_coeff_t *b085);
void bmp085_read_calibration_coefficients(bmp085_coeff_t *b085);

uint16_t bmp085_read_ut(void);
uint32_t bmp085_read_up(void);

int16_t bmp085_calculate_temperature(uint16_t ut, int32_t *B5, const bmp085_coeff_t * const b085);
int32_t bmp085_calculate_true_pressure(uint32_t up, const int32_t * const B5, const bmp085_coeff_t * const b085);
float bmp085_calculate_altitude(int32_t p);

uint32_t bmp085_calculate_pressure_nn(int32_t p);
uint16_t bmp085_calculate_pressure_nn16(int32_t p);

// All in one functionality
void bmp085_read_data(bmp085_t *res, const bmp085_coeff_t * const bmp085);

#endif // BMP085_DATA_TYPE_ONLY

#endif // BMP085_H
