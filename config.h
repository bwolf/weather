#ifndef CONFIG_H
#define CONFIG_H

// Define F_CPU if not done
#ifndef F_CPU
# define F_CPU 1000000UL
#endif

// Features
#undef WITHOUT_UART
#undef WITHOUT_DBGLED
#undef WITHOUT_POWERDOWN

// UART for debug output
#define UART_BAUD_RATE 4800

// SCL clock in Hz for TWI/I2C
#define SCL_CLOCK 10000UL

// BMP085
#undef WITHOUT_BMP085_CALC_PRESSURE_NN
#define BMP085_WITH_UART_ERROR_MSGS

// SHT11, SHT7x configuration
#include "sht11_config.h"
#define SHT11_VOLTAGE_COMPENSATION_D1 SHT11_VOLTAGE_COMPENSATION_D1_3_3V
#define SHT11_DDR     DDRC
#define SHT11_DDR_SCL DDC0
#define SHT11_DDR_SDA DDC1
#define SHT11_PORT    PORTC
#define SHT11_SCL     PC0
#define SHT11_SDA     PC1
#define SHT11_PIN     PINC
#define SHT11_PIN_SDA PINC1

#endif // CONFIG_H
