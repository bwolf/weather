// ms5611.c -- Pressure sensor M5611 from meas-spec.com.

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

#include "twim.h"

#define ADDR_W  0xEE // Module address write mode
#define ADDR_R  0xEF // Module address read mode

#define CMD_RESET    0x1E // ADC reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x40 // ADC conversion command
#define CMD_ADC_D1   0x00 // ADC D1 conversion
#define CMD_ADC_D2   0x10 // ADC D2 conversion
#define CMD_ADC_256  0x00 // ADC OSR=256
#define CMD_ADC_512  0x02 // ADC OSR=512
#define CMD_ADC_1024 0x04 // ADC OSR=1024
#define CMD_ADC_2048 0x06 // ADC OSR=2048
#define CMD_ADC_4096 0x08 // ADC OSR=4096
#define CMD_PROM_RD  0xA0 // Prom read command

//********************************************************
//! @brief Send command using I2C hardware interface.
//!
//! @return none
//********************************************************
static void i2c_send(uint8_t cmd)
{
    uint8_t ret;

    ret = twi_start(ADDR_W); // Set device address and write mode
    if (ret) { // Failed to issue start condition, possibly no device found
        twi_stop();
    } else {   // Issuing start condition ok, device accessible
        (void) twi_write(cmd);
        twi_stop();
    }
}

//********************************************************
//! @brief Send reset sequence.
//!
//! @return none
//********************************************************
static void send_reset_cmd(void)
{
    i2c_send(CMD_RESET); // Send reset sequence
    _delay_ms(3);        // Wait for the reset sequence timing
}

//********************************************************
//! @brief Preform adc conversion.
//!
//! @return 24bit result
//********************************************************
static uint32_t cmd_adc(uint8_t cmd)
{
    uint16_t ret;
    uint32_t temp = 0;

    i2c_send(CMD_ADC_CONV+cmd); // Send conversion command

    // Wait necessary conversion time
    switch (cmd & 0x0f) {
    case CMD_ADC_256:  _delay_us(900); break;
    case CMD_ADC_512:  _delay_ms(3);   break;
    case CMD_ADC_1024: _delay_ms(4);   break;
    case CMD_ADC_2048: _delay_ms(6);   break;
    case CMD_ADC_4096: _delay_ms(10);  break;
    }

    i2c_send(CMD_ADC_READ);

    ret = twi_start(ADDR_R); // Set device address and read mode
    if (ret) {
        // Failed to issue start condition, possibly no device found
        twi_stop();
    } else {
        // Issuing start condition ok, device accessible
        ret = twi_read_ack();  // Read MSB and acknowledge
        temp = 65536 * ret;
        ret = twi_read_ack();  // Read byte and acknowledge
        temp = temp + 256 * ret;
        ret = twi_read_nack(); // Read LSB and not acknowledge
        temp = temp + ret;
        twi_stop(); // Send stop condition
    }
    return temp;
}

//********************************************************
//! @brief Read calibration coefficients from PROM.
//!
//! @return factory data, coefficients, CRC
//********************************************************
static uint16_t read_prom(uint8_t prom_num)
{
    uint16_t ret;
    uint16_t rC =0;

    i2c_send(CMD_PROM_RD + prom_num * 2); // Send PROM READ command

    ret = twi_start(ADDR_R); // Set device address and read mode
    if (ret) {
        // Failed to issue start condition, possibly no device found
        twi_stop();
    } else {
        // Issuing start condition ok, device accessible
        ret = twi_read_ack();  // Read MSB and acknowledge
        rC = 256 * ret;
        ret = twi_read_nack(); // Read LSB and not acknowledge
        rC = rC + ret;
        twi_stop();
    }

    return rC;
}

static void read_prom_data(uint16_t prom_data[8])
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        prom_data[i] = read_prom(i);
    }
}

//********************************************************
//! @brief Calculate the CRC code.
//!
//! @return crc code
//********************************************************
static uint8_t calc_crc4(uint16_t prom[8])
{
    int16_t cnt;       // Simple counter
    uint16_t rem;      // CRC reminder
    uint16_t crc_read; // Original value of the CRC
    uint8_t  bitn;

    rem = 0x00;
    crc_read = prom[7];             // Save read CRC, see data sheet
    prom[7] = (0xFF00 & (prom[7])); // CRC byte is replaced by 0

    for (cnt = 0; cnt < 16; cnt++) { // Operation is performed on bytes
        // Choose LSB or MSB
        if (cnt%2==1) {
            rem ^= (uint16_t) ((prom[cnt >> 1]) & 0x00FF);
        } else {
            rem ^= (uint16_t) (prom[cnt >> 1] >> 8);
        }

        for (bitn = 8; bitn > 0; bitn--) {
            if (rem & (0x8000)) {
                rem = (rem << 1) ^ 0x3000;
            } else {
                rem = (rem << 1);
            }
        }
    }

    rem =  (0x000F & (rem >> 12)); // Final 4-bit reminder is CRC code
    prom[7] = crc_read;                // Restore the crc_read to its original place

    return (rem ^ 0x0);
}


typedef struct ms5611 {
    int16_t temperature; // Temperature in decicelsius
    uint16_t pressure;   // Pressure in millis
} ms5611_t;

// 1st order conversion
void ms5611_read(ms5611_t *ms5611)
{
    uint32_t D1;   // ADC value of the pressure conversion
    uint32_t D2;   // ADC value of the temperature conversion
    uint16_t C[8]; // Calibration coefficients [1],[2]
    float P;       // Compensated pressure value
    float T;       // Compensated temperature value
    int32_t dT;    // Difference between actual and measured temperature
    float OFF;     // Offset at actual temperature
    float SENS;    // Sensitivity at actual temperature

    // [1] address 0 contains factory data and the setup.
    // [2] address 7 contains the serial code and the CRC.

    uint8_t crc;  // CRC value of the PROM

    send_reset_cmd();
    read_prom_data(C);

    // TODO CRC is not verified
    crc = calc_crc4(C); // Calculate the CRC
    D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_4096); // read D2, temperature value
    D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_4096); // read D1, pressure value

    // TODO according data sheet all calculations can be performed with integers

    // Calculate compensated temperature
    dT = D2 - C[5] * 256;                     // 2^8  = 256
    T = (2000 + (dT * C[6]) / 8388608) / 100; // 2^23 = 8388608

    // Calculate temperature compensated pressure
    OFF =  C[2] * 131072 + dT * C[4] / 64;             // 2^17 = 131072, 2^6 = 64
    SENS = C[1] * 65536  + dT * C[3] / 128;            // 2^16 = 65536,  2^7 = 128
    P = (((D1 * SENS) / 2097152 - OFF) / 32768) / 100; // 2^21 = 2097152, 2^15 = 32768

    ms5611->temperature = T;
    ms5611->pressure = P;
}

// EOF
