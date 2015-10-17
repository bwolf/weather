#ifndef CONFIG_H
#define CONFIG_H

// Define F_CPU if not done
#ifndef F_CPU
# define F_CPU 1000000UL
#endif

// Features
#undef  WITHOUT_UART
#undef  WITHOUT_DBGLED_RED
#undef  WITHOUT_DBGLED_RED_TOGGLE
#define WITHOUT_DBGLED_GREEN
#define WITHOUT_DBGLED_GREEN_TOGGLE
#define WITHOUT_POWERDOWN

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
