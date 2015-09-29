// sht11.h

#ifndef SHT11_H
#define SHT11_H

/**
 * @brief Initialize SHT11
 */
void sht11_init(void);

/**
 * @brief Power down sensor to save current.
 */
void sht11_down(void);

// TODO integrate these comments into doxygen comments!
// Start measurement (humidity or temperature).
// Return "device found".
// Afterwards poll sht11_ready.

/**
 * @brief Start temperature measurement
 *
 * Afterwards poll sht11_ready.
 *
 * @return Raw temperature
 */
uint8_t sht11_start_temp(void);


int16_t sht11_result_temp_get_raw(void);
int16_t sht11_result_temp_convert_sht11(int16_t raw_temp);
int16_t sht11_result_temp_convert_sht7x(uint16_t raw_temp);

/**
 * @brief Start humidity measurement
 * @return Raw humidity
 */
// TODO cleanup this mess (different conversion methods)
uint8_t sht11_start_humid(void);
int16_t sht11_result_humid_get_raw(void);
int16_t sht11_result_humid_convert_sht11(int16_t raw_humid);
uint16_t sht11_result_humid_convert_sht7x(uint16_t const raw_temp, uint16_t const raw_humid);

/**
 * Predicate to check if the sht11 sensor is ready
 * @return 0 unless measurement completed
 */
uint8_t sht11_ready(void);

// Return result of measurement.
// H: 100*%RH (0..10000)
// T: 100*T
// Return -32xxx on failure.

/**
 * @brief Return result of temperature measurement
 * @return 100 * temperature or -32xxx on failure
 */
int16_t sht11_result_temp(void);

/**
 * @brief Return result of humidity measurement
 * @return 100 * %RH (0..1000) or -32xxx on failure
 */
int16_t sht11_result_humid(void);

/**
 * @brief Error code for unavailable SHT11 sensor
 */
#define SHT11_UNAVAIL  -32768

/**
 * @brief Error code for CRC error when reading from the SHT11 sensor
 */
#define SHT11_CRC_FAIL -32767

/**
 * @brief Predicate to check if the sensor readout is valid
 *
 * If it is not valid, the sensor value may be compared to
 * SHT11_UNAVAIL or SHT11_CRC_FAIL.
 */
#define sht11_valid(v) ((v) > -32000)

#endif // SHT11_H
