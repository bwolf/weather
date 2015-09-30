#include "config.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <util/delay.h>

#include "uart.h"
#include "uart_addons.h"

#include "twim.h"
#include "bmp085.h"
#include "sht11.h"


#ifdef WITHOUT_POWERDOWN
# warning "Compiling without power saving (powerdown)!"
#endif

#ifdef WITHOUT_UART
# warning "Building without UART, you won't see any output!"
#endif


static inline void disable_ad_converter(void)
{
#ifdef __AVR_ATmega8__
    ADCSRA &= ~(1 << ADEN);
#else
# error "Unsupported MCU"
#endif
}

static inline void disable_analog_comparator(void)
{
#ifdef __AVR_ATmega8__
    SFIOR |= (1 << ACD);
#else
# error "Unsupported MCU"
#endif
}


#ifndef WITHOUT_DBGLED
static uint8_t led = 0;

#define dbgled_init()   DDRB |= (1 << DDB0)
#define dbgled_on() led = 1; PORTB |= (1 << PB0)
#define dbgled_off() led = 0; PORTB &= ~(1 << PB0)

// static void dbgled_toggle(void)
// {
//     if (led) {
//         dbgled_off();
//     } else {
//         dbgled_on();
//     }
//     led = !led;
// }

// static void dbgled_pulse(uint8_t p)
// {
//     uint8_t n;

//     for (n = 0; n < p; n++) {
//         dbgled_on();
//         _delay_ms(50);
//         dbgled_off();
//         _delay_ms(50);
//     }
// }

#else
# warning "Building without debug LED, you won't see any blinking!"
# define dbgled_init()
# define dbgled_on()
# define dbgled_off()
# define dbgled_pulse(p) (void) p
#endif


// Predicate to check if power down is possible by checking subsystems.
static uint8_t power_down_p(void)
{
    // TODO fill predicate
    return 1;
}

// Power down all subsystems.
static void subsystems_power_down(void)
{
    // TODO power down subsystems

    // SHT11
    //sht11_down(); Is already put down after sensor reading

    // BMP085
    // bmp085_down(); // yet missing
    // TODO check BMP085 datasheet if sensor can be powered down

    // Disable TWI via TWEN bit. If this bit is written to zero, the
    // TWI is switched off and all TWI transmissions are terminated,
    // regardless of any ongoing operation.
    TWCR &= ~(1 << TWEN);
}

// Power up all subsystems
static void subsystems_power_up(void)
{
    // TODO power up subsystems

    // TWI/I2C
    //
    // Does not work after power save mode. Reset TWI register to
    // correct values.
    TWCR &= ~((1 << TWSTO) | (1 << TWEN));
    TWCR |= (1 << TWEN);
}


static bmp085_t bmp085; // TODO rename to _coefficients or include results into struct
static bmp085_results_t bmp085_results;

static sht11_t sht11;

#define ALTITUDE_MUNICH 519

static void dowork(void)
{
    dbgled_on(); // LED enable

    bmp085_read(&bmp085_results, &bmp085);

    uint32_t pNN = bmp085_calculate_pressure_nn(bmp085_results.pressure, ALTITUDE_MUNICH);
    // Debug output
    uart_puti16(bmp085_results.decicelsius); uart_space();
    uart_puti32(bmp085_results.pressure); uart_space();
    uart_putu32(pNN); uart_space();

    sht11_init();
    if (sht11_read(&sht11)) {
        uart_puts_P("sht11 error"); // Debug output
    } else {
        // Debug output
        uart_puti16(sht11.temp); uart_space();
        uart_puti16(sht11.rh_true);
    }
    sht11_down();
    uart_crlf(); // Debug output

    dbgled_off();
}


#define TIMER2_TICKS_UNTIL_WORK 10

static volatile uint8_t dowork_flag;

int __attribute__((OS_main))
main(void)
{
    disable_ad_converter();
    disable_analog_comparator();

    _delay_ms(100);
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    dbgled_init();

    twi_init();
    bmp085_init(&bmp085);
    bmp085_read_calibration_coefficients(&bmp085);

    // Use timer/counter2 in async mode to wakeup from power save.
    // An external resonator with 32.768 kHz is required on TOSC1/TOSC2.
    ASSR  = (1<< AS2); // Drive timer2 asynchronously
    _delay_ms(1000);   // Wait until external resonator is stable
    TCCR2 |= (1 << CS22) | (1 << CS20); // Prescaler 128
    while ((ASSR & (1<< TCR2UB)))       // Wait until register access has completed
        ;

    TIFR   = (1<<TOV2);  // Clear timer interrupts (*)
    TIMSK |= (1<<TOIE2); // Enable timer overflow interrupt
    // (*) Alternatively, TOV2 is cleared by writing logic one to the flag.

    sei(); // With interrupts...

#ifndef WITHOUT_POWERDOWN
    uint8_t power_down; // Remember if powered down
#endif

    uart_ff();
    while (1) {
        // Give outstanding operations time to complete (e.g. UART)
        _delay_ms(100);

        // IMPORTANT: If Timer2 is used asynchronously to do periodic
        // wake ups from sleep mode, a sleep of at least one resonator
        // cycle of Timer2 needs to be performed (~30us), to
        // reactivate the interrupt logic. Otherwise the MCU will
        // never be waked. These could be omitted if it has been
        // ensured that the main loop plus the interrupt logic take
        // longer than 30us. The following two lines ensure the
        // minimum time.
        OCR2 = 0;                    // Dummy access
        while((ASSR & (1<< OCR2UB))) // Wait until register access has completed
            ;

        if (power_down_p()) {
#ifndef WITHOUT_POWERDOWN
            power_down = 1;
            subsystems_power_down();
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            sleep_mode(); // This power downs the MCU
#endif
        }

        // Here we wakeup from sleep mode, after execution of the
        // timer2 interrupt
        _delay_ms(1000); // Wait until external resonator is stable
#ifndef WITHOUT_POWERDOWN
        if (power_down) {
            power_down = 0;
            subsystems_power_up();
        }
#endif
        if (dowork_flag) { // Flag is signaled in timer2 interrupt
            dowork_flag = 0;
            dowork();
        }
    }
}

ISR(TIMER2_OVF_vect)
{
    static uint8_t ticks;

    ++ticks;
    if (ticks == TIMER2_TICKS_UNTIL_WORK) { // New measurement cycle?
        ticks = 0;
        dowork_flag = 1; // Flag main loop to do measurement
    }
    dbgled_off(); // TODO remove
}

// EOF
