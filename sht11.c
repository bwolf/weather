/* Copyright 2016 Marcus Geiger. All rights reserved. Use of this
 * source code is governed by a Apache 2.0 license that can be found
 * in the LICENSE file.
 *
 * --- Sensirion SHT7x/SHT11 sensor reading.
 *
 * The sensor is driven with its default 14-bit (temp) and 12-bit
 * (humidity) readout mode. All temperature values are in degrees
 * celsius.
 *
 * An external pull-up resistor is required on the SDA pin.
 */

#include "config.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "uart.h"
#include "uart_addons.h"

#include "sht11.h"

// Requried in config.h for voltage compensation
// #include "sht11_config.h"
// #define SHT11_VOLTAGE_COMPENSATION_D1 SHT11_VOLTAGE_COMPENSATION_D1_5V

// Required in config.h according actual wiring
//
// #define SHT11_DDR     DDRC
// #define SHT11_DDR_SCL DDC0
// #define SHT11_DDR_SDA DDC1
// #define SHT11_PORT    PORTC
// #define SHT11_SCL     PC0
// #define SHT11_SDA     PC1
// #define SHT11_PIN     PINC
// #define SHT11_PIN_SDA PINC1

#ifdef SHT11_CONVERT_ALT_COMPENSATED
# ifndef SHT11_VOLTAGE_COMPENSATION_D1
#  error "Missing definition for voltage compensation"
# endif
#endif

#if !(SHT11_DDR_SCL == SHT11_SCL && SHT11_DDR_SDA == SHT11_SDA && SHT11_SDA == SHT11_PIN_SDA)
# error "Malformed DDR/PORT/PIN configuration"
#endif


// Commands

#define SHT11_CMD_TEMP  0x03
#define SHT11_CMD_HUMID 0x05
#define SHT11_CMD_WSTAT 0x06
#define SHT11_CMD_RSTAT 0x07
#define SHT11_CMD_RESET 0x1E


// I/O port utilities

static void delay(void) { _delay_us(2); }

// SCL is defined as output
static void scl_hi(void) { SHT11_PORT |= (1 << SHT11_SCL); }
static void scl_lo(void) { SHT11_PORT &= ~(1 << SHT11_SCL); }

// SDA is either input or output
// - logical 1 if configured as input (external pull-up to VDD)
// - logical 0 if configured as output and set to low
static void sda_hi(void) { SHT11_DDR &= ~(1 << SHT11_DDR_SDA); }
static void sda_lo(void) { SHT11_DDR |= (1 << SHT11_DDR_SDA); SHT11_PORT &= ~(1 << SHT11_SDA); }

static void scl_pulse(void) { scl_hi(); delay(); scl_lo(); }
static uint8_t sda_val(void) { return SHT11_PIN & (1 << SHT11_PIN_SDA); }


// CRC

static uint8_t crc_value;

static void crc8(uint8_t b)
{
    for (uint8_t i = 0; i < 8; ++i) {
        if ((crc_value ^ b) & 0x80) {
            crc_value <<= 1;
            crc_value ^= 0x31;
        } else {
            crc_value <<= 1;
        }
        b <<= 1;
    }
}


// Sensor communication and data handling

static uint8_t send(uint16_t b)
{
    crc8(b);

    // Data
    for (uint8_t i = 0; i < 8; ++i) {
        if (b & 0x80)
            sda_hi();
        else
            sda_lo();
        b <<= 1;
        delay();
        scl_pulse();
    }

    // Acknowledge
    sda_hi();
    delay();
    uint8_t ack = sda_val();
    scl_pulse();
    return ack;
}

static uint8_t recv_data(void)
{
    // Data
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        // Data is transmitted MSB first
        b <<= 1;
        if (sda_val())
            b |= 1;
        scl_pulse();
        delay();
    }

    // lo acknowledge
    sda_lo();
    delay();
    scl_pulse();
    sda_hi();
    delay();

    crc8(b);
    return b;
}

static uint8_t recv_crc(void)
{
    // data
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        b >>= 1; // CRC is transmitted LSB first
        if (sda_val())
            b |= 0x80;
        scl_pulse();
        delay();
    }

    // hi acknowledge
    sda_hi();
    delay();
    scl_pulse();
    delay();

    return b;
}

static void start(void)
{
    SHT11_DDR |= (1 << SHT11_DDR_SCL); // SCK output low
    SHT11_PORT &= ~(1 << SHT11_SCL);
    SHT11_DDR &= ~(1 << SHT11_DDR_SDA); // SDA input high

    // Reset communication
    for (uint8_t i = 0; i < 10; ++i) {
        scl_pulse();
        delay();
    }

    // Send "start" sequence
    scl_hi(); delay();
    sda_lo(); delay();
    scl_lo(); delay();
    scl_hi(); delay();
    sda_hi(); delay();
    scl_lo(); delay();
}


// Core logic

uint8_t sht11_temp_start(void)
{
    crc_value = 0;
    start();
    return send(SHT11_CMD_TEMP) == 0;
}

uint8_t sht11_humid_start(void)
{
    crc_value = 0;
    start();
    return send(SHT11_CMD_HUMID) == 0;
}

uint8_t sht11_ready(void)
{
    return sda_val() == 0;
}

static int16_t get_data(void)
{
    if (!sht11_ready())
        return SHT11_UNAVAIL;
    int16_t v = recv_data() << 8; v |= recv_data();
    uint8_t crc = recv_crc();
    if (crc != crc_value)
        return SHT11_CRC_FAIL;
    return v;
}


// Temperature

int16_t sht11_temp_get_raw(void)
{
    int16_t v = get_data();
    return v;
}

#if defined(SHT11_CONVERT_SENSIRION)

// Convert raw temperature readout to actual degrees according to Sensirion datasheet.
float sht11_temp_convert_sensirion(int16_t raw_temperature)
{
    const float d1 = SHT11_VOLTAGE_COMPENSATION_D1;
    const float d2 = 0.01f; // 14-bit readout
    return (d1 + d2 * (float) raw_temperature);
}

#elif defined(SHT11_CONVERT_SIMPLE)

// Simple conversion using a fixed offset
int16_t sht11_temp_convert_simple(int16_t raw_temp)
{
    raw_temp -= 4000;
    return raw_temp;
}

#elif defined(SHT11_CONVERT_ALT_COMPENSATED)

// Alternative conversion with voltage compensation
int16_t sht11_temp_convert_alt_compensated(uint16_t raw_temp)
{
    return (int16_t) (raw_temp + (((int16_t) SHT11_VOLTAGE_COMPENSATION_D1) * 100));
}

#endif


// Humidity

int16_t sht11_humid_get_raw(void)
{
    int16_t v = get_data();
    return v;
}

#if defined(SHT11_CONVERT_SENSIRION)

// Convert raw humidity readout to actual pascals according to Sensirion datasheet.
float sht11_humid_convert_sensirion(float Tcels, int16_t SOrh)
{
    // Relative Humidity
    //
    // For compensating non-linearity of the humidity sensor – see
    // Figure 13 – and for obtaining the full accuracy of the sensor
    // it is recommended to convert the humidity readout (SORH) with
    // the following formula
    const float c1 = -2.0468f; // for 12-bit sensor readout
    const float c2 = 0.0367f;  // for 12-bit sensor readout
    const float c3 = -1.5955E-6f;

    const float RHlinear = c1 + c2 * SOrh + c3 * (SOrh * SOrh);

    // Temperature compensation of Humidity Signal
    //
    // For temperatures significantly different from 25°C (~77°F) the
    // humidity signal requires temperature compensation. The
    // temperature correction corresponds roughly to 0.12%RH/°C @
    // 50%RH. Coefficients for the temperature compensation are given
    // in Table 8.
    const float t1 = 0.01f;    // for 12-bit sensor readout
    const float t2 = 0.00008f; // for 12-bit sensor readout

    const float RHtrue = (Tcels - 25.f) * (t1 + t2 * SOrh) + RHlinear;

    return RHtrue;
}

#elif defined(SHT11_CONVERT_SIMPLE)

int16_t sht11_humid_convert_simple(int16_t raw_humid)
{
    // inspired by Werner Hoch
    const int32_t C1 = (int32_t)(-4.0 * 100);
    const int32_t C2 = (int32_t)(0.0405 * 100 * (1L<<28));
    const int32_t C3 = (int32_t)(-2.8e-6 * 100 * (1L<<30));
    return (int16_t)((((((C3 * raw_humid) >> 2) + C2) >> 11) * raw_humid + (1L<<16)) >> 17) + C1;
}

#elif defined(SHT11_CONVERT_ALT_COMPENSATED)

uint16_t sht11_humid_convert_alt_compensated(uint16_t const converted_temp, uint16_t const raw_humid)
{
    uint32_t humid;

    // first step: relative humidity

    // constants from datasheet (values for V4 sensors), shifted and adapted
    const uint32_t c1 = (uint32_t) ((2.0468 * 100 * (1L << 18)) + 0.5);
    const uint32_t c2 = (uint32_t) ((0.0367 * 100 * (1L << 18)) + 0.5);
    const uint32_t c3 = (uint32_t) ((1.5955e-6 * 100 * (1L << 27)) + 0.5);

    // uncompensated = c2*raw - (c3 * raw * raw) - c1
    humid = (c2 * (uint32_t) raw_humid)
        - (((((uint32_t) raw_humid * (uint32_t) raw_humid) >> 7) * c3) >> 2)
        - c1;

    // humid >> 18 would now be the uncompensated humidity in RH% * 100

    // second step: temperature compensation

    // constants from datasheet, shifted and adapted
    const int32_t t1 = (int32_t) ((0.01 * 100 * (1L << 18)) + 0.5);
    const int32_t t2 = (int32_t) ((0.00008 * 100 * (1L << 18)) + 0.5);

    // humid += (temp-25) * (t1 + t2*raw_temp)
    humid += (int32_t) ((converted_temp / 100) - 25) * (t1 + (t2 * (int32_t) raw_humid));

    // humid >> 18 now is the real humidity in RH% * 100

    // shift it back and round to RH% * 10
    // (the sensor doesn't really provide enough accuracy to justify more resolution)
    return (uint16_t) (((humid >> 18) + 5) / 10);
}

#endif


// All in one conversion according to Sensirion sample code

#ifdef SHT11_CONVERT_SENSIRION_ALL_IN_ONE_FLOATS

void sth11_convert_sensirion_all_in_one(float *p_humidity, float *p_temperature)
{
    const float C1 = -2.0468;       // for 12 Bit RH
    const float C2 = +0.0367;       // for 12 Bit RH
    const float C3 = -0.0000015955; // for 12 Bit RH
    const float T1 = +0.01;         // for 12 Bit RH
    const float T2 = +0.00008;      // for 12 Bit RH

    const float d1 = SHT11_VOLTAGE_COMPENSATION_D1;
    const float d2 = +0.01;   // for 14 Bit temperature

    float rh = *p_humidity;   // rh:      Humidity [Ticks] 12 Bit
    float t = *p_temperature; // t:       Temperature [Ticks] 14 Bit
    float rh_lin;             // rh_lin:  Humidity linear
    float rh_true;            // rh_true: Temperature compensated humidity
    float t_C;                // t_C:     Temperature [°C]

    t_C = t * d2 + d1;                    // calc. temperature[°C]from 14 bit temp.ticks
    rh_lin = C3*rh*rh + C2*rh + C1;       // calc. humidity from ticks to [%RH]
    rh_true = (t_C-25)*(T1+T2*rh)+rh_lin; // calc. temperature compensated humidity [%RH]

    if (rh_true > 99)  rh_true = 100; // cut if the value is outside of
    if (rh_true < 0.1) rh_true = 0.1; // the physical possible range

    *p_temperature = t_C;
    *p_humidity = rh_true;
}

#endif


// Initialize

void sht11_init(void)
{
    start();
    send(SHT11_CMD_RESET);
    _delay_ms(11); // After power-up the sensor needs 11ms to get out of sleep state
}

// Power down sensor
void sht11_down(void)
{
    // Both pins as output
    SHT11_DDR |= (1 << SHT11_DDR_SCL);
    SHT11_DDR |= (1 << SHT11_DDR_SDA);

    // Both pins to GND
    SHT11_PORT &= ~(1 << SHT11_SCL);
    SHT11_PORT &= ~(1 << SHT11_SDA);
}


// High level interface

#define READ_MAX_ATTEMPTS 10

uint8_t sht11_read_data(sht11_t *sht11)
{
    uint8_t n;
    int16_t temp = 0;  // raw temperature
    int16_t humid = 0; // raw humidity

    // Read raw temperature, outcome: temp.
    if ((n = sht11_temp_start())) {
        for (n = 0; n < READ_MAX_ATTEMPTS; n++) {
            _delay_ms(100); // Give sensor more time [1]
            if (sht11_ready()) {
                temp = sht11_temp_get_raw();
                if (sht11_valid(temp)) {
                    break;
                } else {
                    return SHT11_READ_ERROR_INVALID;
                }
            }
        }
    } else {
        return SHT11_READ_ERROR_TEMP_START;
    }

    _delay_ms(5); // Delay for next reading

    // Read raw humidity, outcome: humid.
    if ((n = sht11_humid_start())) {
        for (n = 0; n < READ_MAX_ATTEMPTS; n++) {
            _delay_ms(50);
            if (sht11_ready()) { // Give sensor more time [2]
                humid = sht11_humid_get_raw();
                if (sht11_valid(humid)) {
                    break;
                } else {
                    return SHT11_READ_ERROR_INVALID;
                }
            }
        }
    } else {
        return SHT11_READ_ERROR_HUMID_START;
    }

    // NOTE: [1],[2] The initial temperature reading takes longer
    // compared to reading the humidity after the temperature has been
    // read.

#ifdef SHT11_PRINT_RAW_VALUES
    uart_crlf();
    uart_puts_P("SHT11 raw (temp/humid) ");
    uart_puti16(temp); uart_space();
    uart_puti16(humid);
    uart_crlf();
#endif

    // Conversion
#if defined(SHT11_CONVERT_SENSIRION)
    const float Tcels = sht11_temp_convert_sensirion(temp);
    sht11->temp = Tcels * 100.f;
    sht11->rh_true = sht11_humid_convert_sensirion(Tcels, humid) * 100.f;
#elif defined(SHT11_CONVERT_SIMPLE)
    sht11->temp = sht11_temp_convert_simple(temp);
    sht11->rh_true = sht11_humid_convert_simple(humid);
#elif defined(SHT11_CONVERT_ALT_COMPENSATED)
    sht11->temp = sht11_temp_convert_alt_compensated(temp);
    sht11->rh_true = sht11_humid_convert_alt_compensated(temp, humid);
#elif defined(SHT11_CONVERT_SENSIRION_ALL_IN_ONE_FLOATS)
    float cels = temp;
    float humi = humid;
    sth11_convert_sensirion_all_in_one(&humi, &cels);
    sht11->temp = cels * 100.f;
    sht11->rh_true = humi * 100.f;
#endif

#ifdef SHT11_WITH_RAW_SENSOR_VALUES
# warning "Compiling with raw SHT11 sensor values. Use only for debugging."
    sht11->raw_temp = temp;
    sht11->raw_humi = humid;
#endif

    return 0;
}

// EOF
