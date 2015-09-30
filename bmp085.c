/* bmp085.c - Bosch BMP085
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
#include <util/delay.h>

#include "uart.h"
#include "uart_addons.h"
#include "twim.h"

#include "bmp085.h"



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

#define _AC1 (b085->AC1)
#define _AC2 (b085->AC2)
#define _AC3 (b085->AC3)
#define _AC4 (b085->AC4)
#define _AC5 (b085->AC5)
#define _AC6 (b085->AC6)
#define _B1 (b085->B1)
#define _B2 (b085->B2)
#define _MB (b085->MB)
#define _MC (b085->MC)
#define _MD (b085->MD)



// Init struct with sane values. Otherwise gcc complains about maybe
// unitialized members.
void bmp085_init(bmp085_t *b085)
{
    b085->AC1 = 0;
    b085->AC2 = 0;
    b085->AC3 = 0;
    b085->AC4 = 0;
    b085->AC5 = 0;
    b085->AC6 = 0;
    b085->B1 = 0;
    b085->B2 = 0;
    b085->MB = 0;
    b085->MC = 0;
    b085->MD = 0;
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
#ifdef BMP085_WITH_UART_ERROR_MSGS
    uart_putsln_P("Error bmp085_read_u16");
#endif
    return 0;
}

// Read calibration coefficients from BMP085 eeprom
void bmp085_read_calibration_coefficients(bmp085_t *b085)
{
#undef  BMP085_GET_CP
#define BMP085_GET_CP(prnfn, name, reg_addr)                            \
    b085->name = bmp085_read_i16(reg_addr);                             \
    if (b085->name == 0)                                                \
        goto error
    // uart_puts_P(#name " ");
    // prnfn(b085->name);
    // uart_crlf()

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
#ifdef BMP085_WITH_UART_ERROR_MSGS
    uart_putsln_P("Error bmp085_read_calibration_coefficients");
#endif
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
#ifdef BMP085_WITH_UART_ERROR_MSGS
    uart_putsln_P("Error bmp085_read_ut");
#endif
    return 0;
}

// Read uncompensated pressure value.
// Param: oss - oversampling setting of BMP085
// Example value 39680
uint32_t bmp085_read_up(uint8_t oss)
{
    uint8_t msb, lsb, xlsb;
    int8_t delayms;

    if (twi_start(BMP085_ADDRESS + TWI_WRITE)) goto error;
    if (twi_write(0xF4)) goto error;
    if (twi_write(0x34 + (oss << 6))) goto error;
    twi_stop();

    // Delay according to chosen oversampling setting
    delayms = 2 + (3 << oss);
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

    return (((uint32_t) msb << 16) | ((uint32_t) lsb << 8) | (uint32_t) xlsb) >> (8 - oss);

error:
#ifdef BMP085_WITH_UART_ERROR_MSGS
    uart_putsln_P("Error bmp085_read_up");
#endif
    return 0;
}

// Calculate temperature in deci degress C
// Param: ut - uncompensated temperature from BMP085
// Param: bmp085 - structure with calibration coefficients
// Return: temperature in deci Celsius
int16_t bmp085_calculate_temperature(uint16_t ut, int32_t *B5, const bmp085_t * const b085)
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
int32_t bmp085_calculate_true_pressure(uint32_t up, const int32_t * const B5, const bmp085_t * const b085, uint8_t oss)
{
    int32_t x1, x2, x3, b3, b6, p;
    uint32_t b4, b7;

    b6 = *B5 - 4000;

    // Calculate B3
    x1 = (_B2 * (b6 * b6) >> 12) >> 11;
    x2 = (_AC2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t) _AC1) * 4 + x3) << oss) + 2) >> 2;

    // Calculate B4
    x1 = (_AC3 * b6) >> 13;
    x2 = (_B1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (_AC4 * (uint32_t) (x3 + 32768)) >> 15;

    b7 = ((uint32_t) (up - b3) * (50000 >> oss));
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

float bmp085_calculate_altitude(int32_t p)
{
    const float p0 = 101325; // Pressure at sea level in Pa
    return (float) 44330 * (1 - pow(((float) p/p0), 0.190295));
}

// NN is normal null
uint32_t bmp085_calculate_pressure_nn(int32_t p, uint16_t altitude)
{
    return (uint32_t) (((float) p) / pow((float)1 - ((float)altitude / (float) 44330), (float) 5.255));
}

void bmp085_read(bmp085_results_t *res, const bmp085_t * const bmp085)
{
    // Read uncompensated temperature value
    uint16_t ut = bmp085_read_ut();

    // Read uncompensated pressure value
    uint32_t up = bmp085_read_up(BMP085_OSS_VALUE);

    // Calculate temperature in deci degress C
    int32_t B5;
    int16_t decicelsius = bmp085_calculate_temperature(ut, &B5, bmp085);
    res->decicelsius = decicelsius;

    // Calculate true pressure
    int32_t p = bmp085_calculate_true_pressure(up, &B5, bmp085, BMP085_OSS_VALUE);
    res->pressure = p;
}

// EOF
