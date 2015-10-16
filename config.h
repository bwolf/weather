#ifndef CONFIG_H
#define CONFIG_H

// Define F_CPU if not done
#ifndef F_CPU
# define F_CPU 1000000UL
#endif

// Features
#undef  WITHOUT_UART
#undef  WITHOUT_DBGLED_RED
#define WITHOUT_DBGLED_RED_TOGGLE
#undef  WITHOUT_DBGLED_GREEN
#undef  WITHOUT_DBGLED_GREEN_TOGGLE
#undef  WITHOUT_POWERDOWN

// Provide inverse features macros (for some features)
#ifndef WITHOUT_UART
# define WITH_UART
#endif

#ifndef WITHOUT_POWERDOWN
# define WITH_POWERDOWN
#endif

// UART for debug output
#define UART_BAUD_RATE 4800

// Red debug LED
#define DBGLED_RED_DDR      DDRB
#define DBGLED_RED_DDR_PIN  DDB0
#define DBGLED_RED_PORT     PORTB
#define DBGLED_RED_PORT_PIN PB0

// Green debug LED
#define DBGLED_GREEN_DDR      DDRB
#define DBGLED_GREEN_DDR_PIN  DDB1
#define DBGLED_GREEN_PORT     PORTB
#define DBGLED_GREEN_PORT_PIN PB1

// SCL clock in Hz for TWI/I2C
#define SCL_CLOCK 10000UL

// Altitude at sensor location
#define ALTITUDE_MUNICH      519
#define ALTITUDE_HOLZKIRCHEN 691

#define ALTITUDE_SENSOR_LOCATION ALTITUDE_HOLZKIRCHEN

// BMP085 configuration (temperature/pressure)
#include "bmp085_config.h"
#define BMP085_OVERSAMPLING_VALUE BMP085_OVERSAMPLING_STANDARD
#define BMP085_ALTITUDE_SENSOR ALTITUDE_SENSOR_LOCATION
#undef  WITHOUT_BMP085_CALC_PRESSURE_NN
#undef  BMP085_WITH_UART_ERROR_MSGS

// SHT11 configuration (temperature/humidity)
#include "sht11_config.h"
#undef  SHT11_CONVERT_SENSIRION
#undef  SHT11_CONVERT_SIMPLE
#undef  SHT11_CONVERT_ALT_COMPENSATED
#define SHT11_CONVERT_SENSIRION_ALL_IN_ONE_FLOATS
#define SHT11_VOLTAGE_COMPENSATION_D1 SHT11_VOLTAGE_COMPENSATION_D1_3_3V
#undef  SHT11_WITH_RAW_SENSOR_VALUES
#define SHT11_DDR     DDRC
#define SHT11_DDR_SCL DDC0
#define SHT11_DDR_SDA DDC1
#define SHT11_PORT    PORTC
#define SHT11_SCL     PC0
#define SHT11_SDA     PC1
#define SHT11_PIN     PINC
#define SHT11_PIN_SDA PINC1

// Rain sensor configuration
#define RAIN_DDR     DDRD
#define RAIN_DDR_NO  DDD3
#define RAIN_PORT    PORTD
#define RAIN_PORT_NO PORTD3
// Set external interrupt on falling edge for INT1
#define RAIN_INTERRUPT_FALLING_EDGE() (MCUCR = (1 << ISC11))
// Activate external interrupt INT1
#define RAIN_INTERRUPT_ENABLE() (GICR = (1 << INT1))
// Name of the interrupt vector
#define RAIN_INTERRUPT_VECT INT1_vect

// SPI configuration
#define SPI_PORT    PORTB
#define SPI_DDR     DDRB
#define SPI_DD_MISO DDB4
#define SPI_DD_MOSI DDB3
#define SPI_DD_SS   DDB2
#define SPI_DD_SCK  DDB5

// nRF24L01+ configuration
#define WL_MODULE_DDR  DDRC
#define WL_MODULE_PORT PORTC
#define WL_MODULE_CE   PC2
#define WL_MODULE_CSN  PC3
// Set external interrupt on falling edge for INT0
#define WIRELESS_INTERRUPT_FALLING_EDGE() (MCUCR = (1 << ISC01) | (0 << ISC00))
// Activate external interrupt INT0
#define WIRELESS_INTERRUPT_ENABLE() (GICR = (1 << INT0))
// Name of the interrupt vector
#define WIRELESS_INTERRUPT_VECT INT0_vect

#endif // CONFIG_H
