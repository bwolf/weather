/* Copyright 2016 Marcus Geiger. All rights reserved. Use of this
 * source code is governed by a Apache 2.0 license that can be found
 * in the LICENSE file.
 *
 * bmp085.c - Bosch BMP085
 *
 * Example calibration coefficients from eeprom.
 *
 *  AC1 7106
 *  AC2 -1261
 *  AC3 -14633
 *  AC4 34391
 *  AC5 25021
 *  AC6 17113
 *  B1 5498
 *  B2 69
 *  MB -32768
 *  MC -11075
 *  MD 2432
 */

#include "config.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "uart.h"
#include "uart_addons.h"
#include "twim.h"

#include "pressure.h"

#include "bmp085.h"


#ifndef BMP085_OVERSAMPLING_VALUE
# error "Missing definition BMP085_OVERSAMPLING_VALUE."
#endif


// Define this in config.h to get error messages via uart
// #define BMP085_WITH_UART_ERROR_MSGS

#define BMP085_ADDRESS (0x77 << 1)

#undef _AC1
#undef _AC2
#undef _AC3
#undef _AC4
#undef _AC5
#undef _AC6
#undef _B1
#undef _B2
#undef _MB
#undef _MC
#undef _MD

#define _AC1 (coeff->AC1)
#define _AC2 (coeff->AC2)
#define _AC3 (coeff->AC3)
#define _AC4 (coeff->AC4)
#define _AC5 (coeff->AC5)
#define _AC6 (coeff->AC6)
#define _B1 (coeff->B1)
#define _B2 (coeff->B2)
#define _MB (coeff->MB)
#define _MC (coeff->MC)
#define _MD (coeff->MD)


// Init struct with sane values. Otherwise gcc complains about maybe
// unitialized members.
void bmp085_init(bmp085_coeff_t *coeff)
{
    _AC1 = 0;
    _AC2 = 0;
    _AC3 = 0;
    _AC4 = 0;
    _AC5 = 0;
    _AC6 = 0;
    _B1 = 0;
    _B2 = 0;
    _MB = 0;
    _MC = 0;
    _MD = 0;
}

// Reads BMP085 register value as uint16_t
// Returns register value or 0 on error
static int16_t bmp085_read_i16(uint8_t reg)
{
    uint8_t msb, lsb;

    if (twi_start(BMP085_ADDRESS + TWI_WRITE)) goto error;
    if (twi_write(reg)) goto error;
    if (twi_repeated_start(BMP085_ADDRESS + TWI_READ)) goto error;
    msb = twi_read_ack();
    lsb = twi_read_nack();
    twi_stop();

    return (int16_t) msb << 8 | lsb;

error:
    uart_putsln_P("Error bmp085_read_u16");
    return 0;
}

// Read calibration coefficients from BMP085 eeprom
void bmp085_read_calibration_coefficients(bmp085_coeff_t *coeff)
{
#undef  BMP085_GET_CP
#define BMP085_GET_CP(prnfn, name, reg_addr)                            \
    coeff->name = bmp085_read_i16(reg_addr);                            \
    if (coeff->name == 0)                                               \
        goto error

    BMP085_GET_CP(uart_puti16, AC1, 0xAA);
    BMP085_GET_CP(uart_puti16, AC2, 0xAC);
    BMP085_GET_CP(uart_puti16, AC3, 0xAE);
    BMP085_GET_CP(uart_putu16, AC4, 0xB0);
    BMP085_GET_CP(uart_putu16, AC5, 0xB2);
    BMP085_GET_CP(uart_putu16, AC6, 0xB4);
    BMP085_GET_CP(uart_puti16, B1, 0xB6);
    BMP085_GET_CP(uart_puti16, B2, 0xB8);
    BMP085_GET_CP(uart_puti16, MB, 0xBA);
    BMP085_GET_CP(uart_puti16, MC, 0xBC);
    BMP085_GET_CP(uart_puti16, MD, 0xBE);

    // Get rid of local macro
#undef BMP085_GET_CP
    return;

error:
    uart_putsln_P("Error bmp085_read_calibration_coefficients");
    return;
}

// Read uncompensated temperature value.
// Example value 26536
uint16_t bmp085_read_ut(void)
{
    if (twi_start(BMP085_ADDRESS + TWI_WRITE)) goto error;
    if (twi_write(0xF4)) goto error;
    if (twi_write(0x2E)) goto error;
    twi_stop();

    // Wait at least 4.5ms
    _delay_ms(5);

    return bmp085_read_i16(0xF6);

error:
    uart_putsln_P("Error bmp085_read_ut");
    return 0;
}

// Read uncompensated pressure value.
// Param: oss - oversampling setting of BMP085
// Example value 39680
uint32_t bmp085_read_up(void)
{
    uint8_t msb, lsb, xlsb;
    int8_t delayms;

    if (twi_start(BMP085_ADDRESS + TWI_WRITE)) goto error;
    if (twi_write(0xF4)) goto error;
    if (twi_write(0x34 + (BMP085_OVERSAMPLING_VALUE << 6))) goto error;
    twi_stop();

    // Delay according to chosen oversampling setting
    delayms = 2 + (3 << BMP085_OVERSAMPLING_VALUE);
    do {
        _delay_us(1000);
    } while (--delayms > 0);

    // Read 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
    if (twi_start(BMP085_ADDRESS + TWI_WRITE)) goto error;
    if (twi_write(0xF6)) goto error;
    if (twi_repeated_start(BMP085_ADDRESS + TWI_READ)) goto error;
    msb = twi_read_ack();
    // uart_puts_P("up msb "); uart_putu8(msb); uart_crlf();
    lsb = twi_read_ack();
    // uart_puts_P("up lsb "); uart_putu8(lsb); uart_crlf();
    xlsb = twi_read_nack();
    // uart_puts_P("up xlsb "); uart_putu8(xlsb); uart_crlf();
    twi_stop();

    return (((uint32_t) msb << 16) | ((uint32_t) lsb << 8) | (uint32_t) xlsb) >> (8 - BMP085_OVERSAMPLING_VALUE);

error:
    uart_putsln_P("Error bmp085_read_up");
    return 0;
}

// Calculate temperature in deci degress C
// Param: ut - uncompensated temperature from BMP085
// Param: bmp085 - structure with calibration coefficients
// Return: temperature in deci Celsius
int16_t bmp085_calculate_temperature(uint16_t ut, int32_t *B5, const bmp085_coeff_t * const coeff)
{
    int32_t x1 = (((int32_t) ut - (int32_t) _AC6) * (int32_t) _AC5) >> 15;
    int32_t x2 = ((int32_t) _MC << 11) / (x1 + _MD);
    *B5 = x1 + x2; // Value is reused in pressure calculation.
    int16_t decicelsius = ((*B5 + 8) >> 4);

    return decicelsius;
}

// Calculate true pressure
// Param: up - uncompensated pressure from BMP085
// Param: b085 - BMP085 structure with calibration coefficients
// Return: pressure in pascal (P)
int32_t bmp085_calculate_true_pressure(uint32_t up, const int32_t * const B5,
                                       const bmp085_coeff_t * const coeff)
{
    int32_t x1, x2, x3, b3, b6, p;
    uint32_t b4, b7;

    b6 = *B5 - 4000;

    // Calculate B3
    x1 = (_B2 * (b6 * b6) >> 12) >> 11;
    x2 = (_AC2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t) _AC1) * 4 + x3) << BMP085_OVERSAMPLING_VALUE) + 2) >> 2;

    // Calculate B4
    x1 = (_AC3 * b6) >> 13;
    x2 = (_B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (_AC4 * (uint32_t) (x3 + 32768)) >> 15;

    b7 = ((uint32_t) (up - b3) * (50000 >> BMP085_OVERSAMPLING_VALUE));
    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    return p;
}

void bmp085_read_data(bmp085_t *res, const bmp085_coeff_t * const coeff)
{
    // Read uncompensated temperature value
    uint16_t ut = bmp085_read_ut();

    // Read uncompensated pressure value
    uint32_t up = bmp085_read_up();

#ifdef BMP085_PRINT_RAW_VALUES
    uart_crlf();
    uart_puts_P("BMP085 raw (coeff/ut/up) ");
    uart_puti16(_AC1); uart_space();
    uart_puti16(_AC2); uart_space();
    uart_puti16(_AC3); uart_space();
    uart_putu16(_AC4); uart_space();
    uart_putu16(_AC5); uart_space();
    uart_putu16(_AC6); uart_space();
    uart_puti16(_B1); uart_space();
    uart_puti16(_B2); uart_space();
    uart_puti16(_MB); uart_space();
    uart_puti16(_MC); uart_space();
    uart_puti16(_MD); uart_space();
    uart_putu16(ut); uart_space();
    uart_putu32(up);
    uart_crlf();
#endif

    // Calculate temperature in deci degress C
    int32_t B5;
    int16_t decicelsius = bmp085_calculate_temperature(ut, &B5, coeff);
    res->decicelsius = decicelsius;

    // Calculate true pressure
    int32_t p = bmp085_calculate_true_pressure(up, &B5, coeff);

    // Calculate pressure NN
    res->pressure_nn = pressure_to_nn16(p);
}

// EOF
