#ifndef CONFIG_H
#define CONFIG_H


// --- Define F_CPU anyways
#ifndef F_CPU
# define F_CPU 1000000UL
#endif


// --- Features
#undef  WITHOUT_UART
#define WITHOUT_DBGLED_RED
#define WITHOUT_DBGLED_RED_TOGGLE
#undef  WITHOUT_DBGLED_GREEN
#undef  WITHOUT_DBGLED_GREEN_TOGGLE
#undef  WITHOUT_POWERDOWN


// --- Provide inverse features macros (for some features)
#ifndef WITHOUT_UART
# define WITH_UART
#endif

#ifndef WITHOUT_POWERDOWN
# define WITH_POWERDOWN
#endif


// --- UART for debug output
#define UART_BAUD_RATE 4800


// --- Red debug LED
#define DBGLED_RED_DDR      DDRB
#define DBGLED_RED_DDR_PIN  DDB0
#define DBGLED_RED_PORT     PORTB
#define DBGLED_RED_PORT_PIN PB0


// --- Green debug LED
#define DBGLED_GREEN_DDR      DDRB
#define DBGLED_GREEN_DDR_PIN  DDB1
#define DBGLED_GREEN_PORT     PORTB
#define DBGLED_GREEN_PORT_PIN PB1


// --- SPI/SCL clock in Hz for TWI/I2C
#define SCL_CLOCK 10000UL


// --- Altitude at sensor location
#define ALTITUDE_MUNICH      519
#define ALTITUDE_HOLZKIRCHEN 691

#define ALTITUDE_SENSOR_LOCATION ALTITUDE_HOLZKIRCHEN


// --- MS5611 configuration (temperature/pressure)
#include "ms5611_confdefs.h"
#define MS5611_CSB_DDR     DDRD
#define MS5611_CSB_PORT    PORTD
#define MS5611_CSB_PORT_NO PD7
#define MS5611_OVERSAMPLING_PRESSURE    MS5611_OVERSAMPLING_1024
#define MS5611_OVERSAMPLING_TEMPERATURE MS5611_OVERSAMPLING_256


// --- BMP085 configuration (temperature/pressure)
#include "bmp085_confdefs.h"
#define BMP085_OVERSAMPLING_VALUE BMP085_OVERSAMPLING_ULTRA_HIGH_RESOLUTION


// --- SHT11 configuration (temperature/humidity)
#include "sht11_confdefs.h"
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


// --- Rain sensor configuration
#define RAIN_DDR     DDRD
#define RAIN_DDR_NO  DDD3
#define RAIN_PORT    PORTD
#define RAIN_PORT_NO PORTD3

#if defined(__AVR_ATmega88__)
# define RAIN_DEBOUNCE_TIMER_OFF_REG            TCCR0
# define RAIN_DEBOUNCE_TIMER_PRESCALER_REG      TCCR0
# define RAIN_DEBOUNCE_TIMER_INTERRUPT_MASK_REG TIMSK
# define RAIN_INTERRUPT_CONTROL_REG             MCUCR
# define RAIN_INTERRUPT_MASK_REG                GICR
#elif defined(__AVR_ATmega88PA__)
# define RAIN_DEBOUNCE_TIMER_OFF_REG            TCCR0B
# define RAIN_DEBOUNCE_TIMER_PRESCALER_REG      TCCR0B
# define RAIN_DEBOUNCE_TIMER_INTERRUPT_MASK_REG TIMSK0
# define RAIN_INTERRUPT_CONTROL_REG             EICRA
# define RAIN_INTERRUPT_MASK_REG                EIMSK
#else
# error "Unsupported MCU."
#endif

// Disable timer (no clock source)
#define RAIN_DEBOUNCE_TIMER_OFF() (RAIN_DEBOUNCE_TIMER_OFF_REG &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)))

// Enable timer (reset counter value/set prescaler)
#define RAIN_DEBOUNCE_TIMER_START()                                       \
    do {                                                                  \
        /* Reset counter value */                                         \
        TCNT0 = 0;                                                        \
        /* Prescaler clk/1024 */                                          \
        RAIN_DEBOUNCE_TIMER_PRESCALER_REG |= ((1 << CS02) | (1 << CS00)); \
    } while (0)

// Timer overflow interrupt enable
#define RAIN_DEBOUNCE_TIMER_OVERFLOW_INTERRUPT_ENABLE() (RAIN_DEBOUNCE_TIMER_INTERRUPT_MASK_REG |= (1 << TOIE0))
// Name of the timer overflow interrupt vector
#define RAIN_DEBOUNCE_TIMER_OVERFLOW_INTERRUPT_VECT TIMER0_OVF_vect
// Set external interrupt on falling edge for INT1
#define RAIN_INTERRUPT_FALLING_EDGE() (RAIN_INTERRUPT_CONTROL_REG = (1 << ISC11))
// Activate external interrupt INT1
#define RAIN_INTERRUPT_ENABLE() (RAIN_INTERRUPT_MASK_REG = (1 << INT1))
// Name of the interrupt vector
#define RAIN_INTERRUPT_VECT INT1_vect


// --- SPI configuration
#define SPI_PORT    PORTB
#define SPI_DDR     DDRB
#define SPI_DD_MISO DDB4
#define SPI_DD_MOSI DDB3
#define SPI_DD_SS   DDB2
#define SPI_DD_SCK  DDB5


// --- nRF24L01+ configuration
#define WL_MODULE_DDR  DDRC
#define WL_MODULE_PORT PORTC
#define WL_MODULE_CE   PC2
#define WL_MODULE_CSN  PC3

#if defined(__AVR_ATmega88__)
# define WIRELESS_INTERRUPT_CONTROL_REG MCUCR
# define WIRELESS_INTERRUPT_MASK_REG    GICR
#elif defined(__AVR_ATmega88PA__)
# define WIRELESS_INTERRUPT_CONTROL_REG EICRA
# define WIRELESS_INTERRUPT_MASK_REG    EIMSK
#else
# error "Unsupported MCU."
#endif

// Set external interrupt on falling edge for INT0
#define WIRELESS_INTERRUPT_FALLING_EDGE() (WIRELESS_INTERRUPT_CONTROL_REG = (1 << ISC01) | (0 << ISC00))
// Activate external interrupt INT0
#define WIRELESS_INTERRUPT_ENABLE() (WIRELESS_INTERRUPT_MASK_REG = (1 << INT0))
// Name of the interrupt vector
#define WIRELESS_INTERRUPT_VECT INT0_vect

// Wireless highlevel abstraction
#define WL_HIGHLEVEL_MODE_TX

#endif // CONFIG_H
