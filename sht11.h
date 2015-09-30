// sht11.h - Sensirion SHT7x/SHT11 sensor reading.

#ifndef SHT11_H
#define SHT11_H


// High level interface

typedef struct sht11 {
    int16_t temp;
    int16_t rh_true;
} sht11_t;

#define SHT11_READ_ERROR_TEMP_START  1
#define SHT11_READ_ERROR_HUMID_START 2
#define SHT11_READ_ERROR_INVALID     3

// Reading sht11 temperature and humidity including conversion.
//
// Usage:
//   sht11_init();
//   sht11_read();
//   sht11_down();
//
// Return: 0 on success, fills given sht11 struct
//
uint8_t sht11_read(sht11_t *sht11);



// Low level interface

void sht11_init(void);

// Power down sensor to save current.
void sht11_down(void);

uint8_t sht11_temp_start(void);
int16_t sht11_temp_get_raw(void);

uint8_t sht11_humid_start(void);
int16_t sht11_humid_get_raw(void);

#if defined(SHT11_CONVERT_SENSIRION)
float sht11_temp_convert_sensirion(int16_t raw_temp);
float sht11_humid_convert_sensirion(float Tcels, int16_t SOrh);
#elif defined(SHT11_CONVERT_SIMPLE)
int16_t sht11_temp_convert_simple(int16_t raw_temp);
int16_t sht11_humid_convert_simple(int16_t raw_humid);
#elif defined(SHT11_CONVERT_ALT_COMPENSATED)
int16_t sht11_temp_convert_alt_compensated(uint16_t raw_temp);
uint16_t sht11_humid_convert_alt_compensated(uint16_t const raw_temp, uint16_t const raw_humid);
#endif

// Predicate to check if the sht11 sensor is ready
// Return 0 unless measurement completed
uint8_t sht11_ready(void);

// Error code for unavailable SHT11 sensor
#define SHT11_UNAVAIL  -32768

// Error code for CRC error when reading from the SHT11 sensor
#define SHT11_CRC_FAIL -32767

// Predicate to check if the sensor readout is valid.
// If it is not valid, the sensor value may be compared to
// SHT11_UNAVAIL or SHT11_CRC_FAIL.
#define sht11_valid(v) ((v) > -32000)

#endif // SHT11_H
