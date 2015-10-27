// ms5611.c -- Pressure sensor MS5611 from meas-spec.com.

#include "config.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// #include "twim.h"
#include "spi.h"

#include "uart.h"
#include "uart_addons.h"

#include "pressure.h"

#include "ms5611.h"


// Inidication for read from ADC what we read
typedef enum { ADC_READ_PRESSURE, ADC_READ_TEMPERATURE } e_adc_read_kind_t;


// Delays for pressure, temperature according oversampling value.

#if (MS5611_OVERSAMPLING_PRESSURE == MS5611_OVERSAMPLING_256)
# define OVERSAMPLING_PRESSURE_DELAY_MS 1
#elif (MS5611_OVERSAMPLING_PRESSURE == MS5611_OVERSAMPLING_512)
# define OVERSAMPLING_PRESSURE_DELAY_MS 3
#elif (MS5611_OVERSAMPLING_PRESSURE == MS5611_OVERSAMPLING_1024)
# define OVERSAMPLING_PRESSURE_DELAY_MS 4
#elif (MS5611_OVERSAMPLING_PRESSURE == MS5611_OVERSAMPLING_2048)
# define OVERSAMPLING_PRESSURE_DELAY_MS 6
#elif (MS5611_OVERSAMPLING_PRESSURE == MS5611_OVERSAMPLING_4096)
# define OVERSAMPLING_PRESSURE_DELAY_MS 10
#else
# error "Unhandled case according specification/pressure."
#endif

#if (MS5611_OVERSAMPLING_TEMPERATURE == MS5611_OVERSAMPLING_256)
# define OVERSAMPLING_TEMPERATURE_DELAY_MS 1
#elif (MS5611_OVERSAMPLING_TEMPERATURE == MS5611_OVERSAMPLING_512)
# define OVERSAMPLING_TEMPERATURE_DELAY_MS 3
#elif (MS5611_OVERSAMPLING_TEMPERATURE == MS5611_OVERSAMPLING_1024)
# define OVERSAMPLING_TEMPERATURE_DELAY_MS 4
#elif (MS5611_OVERSAMPLING_TEMPERATURE == MS5611_OVERSAMPLING_2048)
# define OVERSAMPLING_TEMPERATURE_DELAY_MS 6
#elif (MS5611_OVERSAMPLING_TEMPERATURE == MS5611_OVERSAMPLING_4096)
# define OVERSAMPLING_TEMPERATURE_DELAY_MS 10
#else
# error "Unhandled case according specification/temperature."
#endif


// SPI interface for MS5611

#define CMD_RESET    0x1E // ADC reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x40 // ADC conversion command
#define CMD_ADC_D1   0x00 // ADC D1 conversion
#define CMD_ADC_D2   0x10 // ADC D2 conversion
#define CMD_ADC_256  0x00 // ADC OSR=256
#define CMD_ADC_512  0x02 // ADC OSR=512
#define CMD_ADC_1024 0x04 // ADC OSR=1024
#define CMD_ADC_2048 0x06 // ADC OSR=2056
#define CMD_ADC_4096 0x08 // ADC OSR=4096
#define CMD_PROM_RD  0xA0 // Prom read command

#define csb_lo() MS5611_CSB_PORT &= ~(1 << MS5611_CSB_PORT_NO)
#define csb_hi() MS5611_CSB_PORT |=  (1 << MS5611_CSB_PORT_NO)

/**
 * @brief Initialize MS5611 module.
 */
void ms5611_init(void)
{
    // Chip select port/pin as output
    MS5611_CSB_DDR |= (1 << MS5611_CSB_PORT_NO);
    csb_hi(); // Initially deselect chip (high)
}

/**
 * @brief send reset sequence
 */
static void send_reset_cmd(void)
{
    csb_lo();                         // pull CSB low to start the command
    (void) spi_fast_shift(CMD_RESET); // send reset sequence
    _delay_ms(3);                     // wait for the reset sequence timing
    csb_hi();                         // pull CSB high to finish the command
}

/**
 * @brief Preform ADC conversion.
 *
 * @return 24bit result
 */
static uint32_t send_adc_cmd(uint8_t cmd, e_adc_read_kind_t read_kind)
{
    uint16_t ret;
    uint32_t temp = 0;

    csb_lo();                                // Pull CSB low
    (void) spi_fast_shift(CMD_ADC_CONV+cmd); // Send conversion command

    // Delay according spec.
    switch (read_kind) {
    case ADC_READ_PRESSURE:    _delay_ms(OVERSAMPLING_PRESSURE_DELAY_MS);
        break;
    case ADC_READ_TEMPERATURE: _delay_ms(OVERSAMPLING_TEMPERATURE_DELAY_MS);
        break;
    default:
        break;
    }

    csb_hi();                            // Pull CSB high to finish the conversion
    csb_lo();                            // Pull CSB low to start new command
    (void) spi_fast_shift(CMD_ADC_READ); // Send ADC read command
    ret = spi_fast_shift(0x00);          // Send 0 to read 1st byte (MSB)
    temp = 65536 * ret;
    ret = spi_fast_shift(0x00);          // send 0 to read 2nd byte
    temp = temp + 256 * ret;
    ret = spi_fast_shift(0x00);          // send 0 to read 3rd byte (LSB)
    temp = temp + ret;
    csb_hi();                            // pull CSB high to finish the read command

    return temp;
}

/**
 * @brief Read calibration coefficients.
 *
 * @return coefficient
 */
static uint16_t send_prom_cmd(uint8_t coeff_num)
{
    uint16_t ret;
    uint16_t rC = 0;

    csb_lo();                                           // Pull CSB low
    (void) spi_fast_shift(CMD_PROM_RD + coeff_num * 2); // Send PROM READ command
    ret = spi_fast_shift(0x00);                         // Send 0 to read the MSB
    rC = (ret << 8);
    ret = spi_fast_shift(0x00);                         // Send 0 to read the LSB
    rC |= ret;
    csb_hi();                                           // Pull CSB high

    return rC;
}

/**
 * @brief Calculate the CRC code.
 *
 * @return CRC code
 */
static uint8_t calc_crc4(uint16_t prom[])
{
    uint16_t nrem;     // CRC reminder
    uint16_t crcread;  // Original value of the crc

    nrem = 0x00;
    crcread = prom[7]; // Save read CRC
    prom[7] = (0xFF00 & (prom[7])); // CRC byte is replaced by 0

    // The following operation is performed on a sequence of bytes,
    // this is why prom[7] is saved and restored.
    for (uint8_t n = 0; n < 16; n++) {    // Operation is performed on bytes
        // Choose LSB or MSB
        if (n % 2 == 1) {
            nrem ^= (unsigned short) ((prom[n >> 1]) & 0x00FF);
        } else {
            nrem ^= (unsigned short) (prom[n >> 1] >> 8);
        }

        for (uint8_t nbit = 8; nbit > 0; nbit--) {
            if (nrem & (0x8000)) {
                nrem = (nrem << 1) ^ 0x3000;
            } else {
                nrem = (nrem << 1);
            }
        }
    }

    nrem = (0x000F & (nrem >> 12)); // Final 4-bit reminder is CRC code
    prom[7] = crcread;             // Restore the crcread to its original place

    return (nrem ^ 0x00);
}

/**
 * @brief Read calibration coefficients.
 */
void ms5611_read_calibration_coefficients(ms5611_coeff_t *coeff)
{
    uint8_t i;

    send_reset_cmd(); // Reset after power-up

    for (i = 0; i < 8; i++) {
        coeff->c[i] = send_prom_cmd(i);
    }
}

/**
 * @brief Perform 1st order conversion.
 *
 * @param coeff Calibration coefficients read from PROM.
 */
// TODO const for coeff would be nice                        )
int8_t ms5611_read_data(ms5611_t *ms5611, ms5611_coeff_t *coeff)
{
    uint32_t D1;   // ADC value of the pressure conversion
    uint32_t D2;   // ADC value of the temperature conversion
    float P;       // Compensated pressure value
    float T;       // Compensated temperature value
    float dT;      // Difference between actual and measured temperature
    float OFF;     // Offset at actual temperature
    float SENS;    // Sensitivity at actual temperature

    send_reset_cmd(); // Reset after power-up

#undef C1
#undef C2
#undef C3
#undef C4
#undef C5
#undef C6
#define C1 coeff->c[1]
#define C2 coeff->c[2]
#define C3 coeff->c[3]
#define C4 coeff->c[4]
#define C5 coeff->c[5]
#define C6 coeff->c[6]

    // Calculate the CRC of the PROM, stored with 4 bits at prom[7] in LSB.
    if (calc_crc4(coeff->c) != (coeff->c[7] & 0x000F)) {
        uart_putsln_P("Error: ms5611/crc");
        return -1;
    }

    // Read digital pressure value, D1
    D1 = send_adc_cmd(CMD_ADC_D1 + MS5611_OVERSAMPLING_PRESSURE, ADC_READ_PRESSURE);

    // Read digital temperature value, D2
    D2 = send_adc_cmd(CMD_ADC_D2 + MS5611_OVERSAMPLING_TEMPERATURE, ADC_READ_TEMPERATURE);

    if (D2 == 0 || D1 == 0) {
        uart_puts_P(" error D1 || D2");
        return -1;
    }

#ifdef MS5611_PRINT_RAW_VALUES
    uart_crlf();
    uart_puts_P("MS5611 raw (coeff/D1/D1) ");
    for (uint8_t i = 0; i < 8; i++) {
        uart_putu8(coeff->c[i]); uart_space();
    }
    uart_putu32(D1); uart_space();
    uart_putu32(D2);
    uart_crlf();
#endif

    // TODO according data sheet all calculations can be performed with integers

    // Calculate compensated temperature
    // dT = D2 - C5 * 2^8
    dT = D2 - C5 * 256.f;

    // T = (2000 + (dT * C6) / 2^23) / 10
    T = (2000.f + (dT * C6) / 8388608.f) / 10.f;

    // Calculate temperature compensated pressure
    // OFF = C2 * 2^16 + dT * C4 / 2^7
    OFF = C2 * 65536.f + dT * C4 / 128;

    // SENS = C1 * 2^15 + dT * C3 / 2^8
    SENS = C1 * 32768.f + dT * C3 / 256.f;

    // Calculate pressure; for just 1st order conversion this would be
    // enough.
    // P = (((D1 * SENS) / 2^21 - OFF) / 2^15) / 10.f;

    // Second order conversion required at T < 20 degress C
    if (T < 200.f) {
        // T2 = dT^2 / 2^31
        float T2 = dT * dT / 2147483648.f;
        // OFF2 = 5 * (TEMP – 2000)^2 / 2^1
        float OFF2 = 5 * pow(T - 2000.f, 2) / 2.f;
        // SENS2 = 5 * (TEMP – 2000)^2 / 2^2
        float SENS2 = 5 * pow(T - 2000.f, 2) / 4.f;

        // Third order conversion required at T < - 15 degress C
        if (T < -150.f) {
            // OFF2 = OFF2 + 7 * (TEMP + 1500)^2
            OFF2 = OFF2 + 7.f * pow(T + 1500.f, 2);
            // SENS2 = SENS2 + 11 * (TEMP + 1500)^2 / 2^1
            SENS2 = SENS2 + 11 * pow(T + 1500.f, 2) / 2.f;
        }

        // Adjust TEMP
        T = T - T2;

        // Adjust OFF and SENS, used for pressure calculation
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    // P = (((D1 * SENS) / 2^21 - OFF) / 2^15) / 10.f;
    P = (((D1 * SENS) / 2097152.f - OFF) / 32768.f) / 10.f;

    ms5611->temperature = T;
    ms5611->pressure = pressure_to_nn16((uint32_t) P);

    return 0;
}

// EOF
