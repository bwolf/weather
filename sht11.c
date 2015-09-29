#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

#include "sht11.h"

// Configuration required in config.h:
// #define SHT11_DDR     DDRD
// #define SHT11_DDR_SCL DDD6
// #define SHT11_DDR_SDA DDD7
// #define SHT11_PORT    PORTD
// #define SHT11_SCL     PD6
// #define SHT11_SDA     PD7
// #define SHT11_PIN     PIND
// #define SHT11_PIN_SDA PIND7

// SHT commands
#define SHT11_CMD_TEMP  0x03
#define SHT11_CMD_HUMID 0x05
#define SHT11_CMD_WSTAT 0x06
#define SHT11_CMD_RSTAT 0x07
#define SHT11_CMD_RESET 0x1E

#if !(SHT11_DDR_SCL == SHT11_SCL && SHT11_DDR_SDA == SHT11_SDA && SHT11_SDA == SHT11_PIN_SDA)
# error "Malformed DDR/PORT/PIN configuration"
#endif



// This version needs external pullups on SDA

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



static uint8_t send(uint16_t b)
{
    crc8(b);

    // data
    for (uint8_t i = 0; i < 8; ++i) {
        if (b & 0x80)
            sda_hi();
        else
            sda_lo();
        b <<= 1;
        delay();
        scl_pulse();
    }

    // acknowledge
    sda_hi();
    delay();
    uint8_t ack = sda_val();
    scl_pulse();
    return ack;
}

static uint8_t recv_data(void)
{
    // data
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        // data is transmitted MSB first
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
        // CRC is transmitted LSB first
        b >>= 1;
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
    // if (no == 1)
    //     sht11_sda = SHT11_SDA1;
    // else if (no == 2)
    //     sht11_sda = SHT11_SDA2;
    // else
    //     return;

    // clrBits(PORT(SHT11_PORT), SHT11_SCL | sht11_sda);   // SCK output low, SDA input/high
    // setBits(DDR(SHT11_PORT),  SHT11_SCL);
    // clrBits(DDR(SHT11_PORT),  sht11_sda);
    // delay();

    SHT11_DDR |= (1 << SHT11_DDR_SCL); // SCK output low
    SHT11_PORT &= ~(1 << SHT11_SCL);
    SHT11_DDR &= ~(1 << SHT11_DDR_SDA); // SDA input high

    // reset communication
    for (uint8_t i = 0; i < 10; ++i) {
        scl_pulse();
        delay();
    }

    // "start" sequence
    scl_hi(); delay();
    sda_lo(); delay();
    scl_lo(); delay();
    scl_hi(); delay();
    sda_hi(); delay();
    scl_lo(); delay();
}


// Measurement sequence

uint8_t sht11_start_temp(void)
{
    crc_value = 0;
    start();
    return send(SHT11_CMD_TEMP) == 0;
}

uint8_t sht11_start_humid(void)
{
    crc_value = 0;
    start();
    return send(SHT11_CMD_HUMID) == 0;
}

uint8_t sht11_ready(void)
{
    return sda_val() == 0;
}

static int16_t result(void)
{
    if (!sht11_ready())
        return SHT11_UNAVAIL;
    int16_t v = recv_data() << 8; v |= recv_data();
    uint8_t crc = recv_crc();
    if (crc != crc_value)
        return SHT11_CRC_FAIL;
    return v;
}

int16_t sht11_result_temp(void)
{
    int16_t v = result();
    if (sht11_valid(v))
        v -= 4000;
    return v;
}

int16_t sht11_result_temp_get_raw(void)
{
    int16_t v = result();
    return v;
}

int16_t sht11_result_temp_convert_sht11(int16_t raw_temp)
{
    raw_temp -= 4000;
    return raw_temp;
}

// TODO cleanup these different conversion methods and settle to one!

int16_t sht11_result_temp_convert_sht7x(uint16_t raw_temp)
{
    return (int16_t) (raw_temp + (((int16_t) SHT11_VOLTAGE_COMPENSATION_D1) * 100));
}

int16_t sht11_result_humid_get_raw(void)
{
    int16_t v = result();
    return v;
}

int16_t sht11_result_humid_convert_sht11(int16_t raw_humid)
{
    const int32_t C1 = (int32_t)(-4.0 * 100);
    const int32_t C2 = (int32_t)(0.0405 * 100 * (1L<<28));
    const int32_t C3 = (int32_t)(-2.8e-6 * 100 * (1L<<30));
    return (int16_t)((((((C3 * raw_humid) >> 2) + C2) >> 11) * raw_humid + (1L<<16)) >> 17) + C1;
}

uint16_t sht11_result_humid_convert_sht7x(uint16_t const raw_temp, uint16_t const raw_humid)
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
    humid += (int32_t) ((sht11_result_temp_convert_sht7x(raw_temp) / 100) - 25)
        * (t1 + (t2 * (int32_t) raw_humid));

    // humid >> 18 now is the real humidity in RH% * 100

    // shift it back and round to RH% * 10
    // (the sensor doesn't really provide enough accuracy to justify more resolution)
    return (uint16_t) (((humid >> 18) + 5) / 10);
}

int16_t sht11_result_humid(void)
{
    int16_t v = result();
    if (sht11_valid(v)) {
        // inspired by Werner Hoch
        const int32_t C1 = (int32_t)(-4.0 * 100);
        const int32_t C2 = (int32_t)(0.0405 * 100 * (1L<<28));
        const int32_t C3 = (int32_t)(-2.8e-6 * 100 * (1L<<30));
        v = (int16_t)((((((C3 * v) >> 2) + C2) >> 11) * v + (1L<<16)) >> 17) + C1;
    }
    return v;
}


// Initialize

void sht11_init(void)
{
    start();
    send(SHT11_CMD_RESET);
    _delay_ms(11); // After power-up the sensor needs 11ms to get out of sleep state
}

void sht11_down(void)
{
    // Both pins as output
    SHT11_DDR |= (1 << SHT11_DDR_SCL);
    SHT11_DDR |= (1 << SHT11_DDR_SDA);

    // Both pins to GND
    SHT11_PORT &= ~(1 << SHT11_SCL);
    SHT11_PORT &= ~(1 << SHT11_SDA);
}

// EOF
