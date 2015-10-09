#ifndef CONFIG_H
#define CONFIG_H

// Define F_CPU if not done
#ifndef F_CPU
# define F_CPU 1000000UL
#endif

// Features
#undef WITHOUT_UART
#undef WITHOUT_DBGLED
#define WITHOUT_POWERDOWN

// Features inverse
#ifndef WITHOUT_UART
# define WITH_UART
#endif
#ifndef WITHOUT_DBGLED
# define WITH_DBGLED
#endif
#ifndef WITHOUT_POWERDOWN
# define WITH_POWERDOWN
#endif

// UART for debug output
#define UART_BAUD_RATE 4800

// SCL clock in Hz for TWI/I2C
#define SCL_CLOCK 10000UL

// BMP085
#include "bmp085_config.h"
#define BMP085_OSS_VALUE BMP085_OVERSAMPLING_STANDARD
#undef WITHOUT_BMP085_CALC_PRESSURE_NN
#define BMP085_WITH_UART_ERROR_MSGS

// SHT11 configuration
#include "sht11_config.h"
#define SHT11_CONVERT_SENSIRION
#undef  SHT11_CONVERT_SIMPLE
#undef  SHT11_CONVERT_ALT_COMPENSATED
#define SHT11_VOLTAGE_COMPENSATION_D1 SHT11_VOLTAGE_COMPENSATION_D1_3_3V
#define SHT11_DDR     DDRC
#define SHT11_DDR_SCL DDC0
#define SHT11_DDR_SDA DDC1
#define SHT11_PORT    PORTC
#define SHT11_SCL     PC0
#define SHT11_SDA     PC1
#define SHT11_PIN     PINC
#define SHT11_PIN_SDA PINC1

// SPI configuration
#define SPI_PORT    PORTB
#define SPI_DDR     DDRB
#define SPI_DD_MISO DDB4
#define SPI_DD_MOSI DDB3
#define SPI_DD_SS   DDB2
#define SPI_DD_SCK  DDB5

// nRF24L01+ configuration
#define wl_module_CONFIG_DDR DDRC
#define wl_module_CONFIG_CE  PC2
#define wl_module_CONFIG_CSN PC3
// Set external interrupt on falling edge for INT0
#define WIRELESS_INTERRUPT_FALLING_EDGE() (MCUCR = (1 << ISC01) | (0 << ISC00))
// Activate external interrupt INT0
#define WIRELESS_INTERRUPT_ENABLE() (GICR = (1 << INT0))
// Name of the interrupt vector
#define WIRELESS_INTERRUPT_VECT INT0_vect

#endif // CONFIG_H
