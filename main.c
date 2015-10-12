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

#include "wl_highlevel.h"


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
static volatile uint8_t dbgled_red = 0;
static volatile uint8_t dbgled_green = 0;

#define dbgled_red_init() DDRB |= (1 << DDB0)
#define dbgled_red_on()   dbgled_red = 1; PORTB |= (1 << PB0)
#define dbgled_red_off()  dbgled_red = 0; PORTB &= ~(1 << PB0)

#define dbgled_green_init() DDRB |= (1 << DDB1)
#define dbgled_green_on()   dbgled_green = 1; PORTB |= (1 << PB1)
#define dbgled_green_off()  dbgled_green = 0; PORTB &= ~(1 << PB1)

// static void dbgled_red_toggle(void)
// {
//     if (dbgled_red) {
//         dbgled_red_off();
//         dbgled_red = 0;
//     } else {
//         dbgled_red_on();
//         dbgled_red = 1;
//     }
// }

void dbgled_green_toggle(void)
{
    if (dbgled_green) {
        dbgled_green_off();
        dbgled_green = 0;
    } else {
        dbgled_green_on();
        dbgled_green = 1;
    }
}

// static void dbgled_red_pulse(uint8_t p)
// {
//     uint8_t n;

//     for (n = 0; n < p; n++) {
//         dbgled_red_on();
//         _delay_ms(50);
//         dbgled_red_off();
//         _delay_ms(50);
//     }
// }

// static void dbgled_green_pulse(uint8_t p)
// {
//     uint8_t n;

//     for (n = 0; n < p; n++) {
//         dbgled_green_on();
//         _delay_ms(50);
//         dbgled_green_off();
//         _delay_ms(50);
//     }
// }

#else
# warning "Building without red debug LED, you won't see any blinking!"
# define dbgled_red_init()
# define dbgled_red_on()
# define dbgled_red_off()
# define dbgled_red_pulse(p) (void) p
#endif


// Predicate to check if power down is possible by checking subsystems.
static uint8_t power_down_p(void)
{
    if (wireless_is_busy()) {
        return 0;
    }

    return 1;
}

// Power down all subsystems.
static void subsystems_power_down(void)
{
    // SHT11 Is always powered-down after sensor reading.
    //
    // BMP085 There is no explicit way to power-down the BMP085.
    //
    // Disable TWI via TWEN bit. If this bit is written to zero, the
    // TWI is switched off and all TWI transmissions are terminated,
    // regardless of any ongoing operation.
    TWCR &= ~(1 << TWEN);

    // Power-down nRF24L01+
    wireless_power_down();
}

// Power up all subsystems
static void subsystems_power_up(void)
{
    // See info in power_down regarding subsystems not powered-up here.
    //
    // TWI/I2C Does not work after power save mode. Reset TWI register
    // to correct values.
    TWCR &= ~((1 << TWSTO) | (1 << TWEN));
    TWCR |= (1 << TWEN);

    // Power-down nRF24L01+
    wireless_power_up();
}


static bmp085_t bmp085;
static bmp085_results_t bmp085_results;

static sht11_t sht11;

#define ALTITUDE_MUNICH      519
#define ALTITUDE_HOLZKIRCHEN 691
#define ALTITUDE_SENSOR_LOCATION ALTITUDE_HOLZKIRCHEN // TODO maybe better in EEPROM

static void dowork(void)
{
    static uint8_t once = 1;

    // Debug output
    if (once) {
        once = !once;
        uart_putsln_P("BMP085     SHT11");
        uart_putsln_P("dC  P (NN) hC   hH%");
        //             231 102464 2333 5036 -- For alignment of the header
    }

    dbgled_red_on(); // LED enable

    // TODO rename all wireless_functions to e.g. wl_hl_
    // wl_hlbusy_p()
    // wl_hlpowerup()
    // wl_hlpowerdown()
    // wl_hltransmit()
    // wl_hlinit()
    if (wireless_is_busy()) {
        // Do nothing, give wireless module time to finish
        uart_putsln_P("wireless_is_busy");
        wireless_debug_print_status();
    } else {
        bmp085_read(&bmp085_results, &bmp085);

        uint32_t pNN = bmp085_calculate_pressure_nn(bmp085_results.pressure, ALTITUDE_SENSOR_LOCATION);
        // Debug output
        uart_puti16(bmp085_results.decicelsius); uart_space();
        uart_putu32(pNN);                        uart_space();

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

        // Transmit measurements
        struct {
            int16_t bmp085_decicelsius;
            uint32_t bmp085_pNN;
            int16_t sht11_milicelsius;
            int16_t sht11_rh_true;
        } payload;

        payload.bmp085_decicelsius = bmp085_results.decicelsius;
        payload.bmp085_pNN = pNN;
        payload.sht11_milicelsius = sht11.temp;
        payload.sht11_rh_true = sht11.rh_true;

        uart_puts_P("Transsmit ");
        uart_putu8(sizeof(payload));
        uart_crlf();
        wireless_send_payload((uint8_t *) &payload, sizeof(payload));
    }

    dbgled_red_off();
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
    dbgled_red_init();
    dbgled_green_init();

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

    uart_ff();

    // Wireless setup requires interrupts
    _delay_ms(50); // TODO delay required?
    wireless_init();

#ifdef WITH_POWERDOWN
    uint8_t power_down; // Remember if powered down
#endif

    // Debug only
    _delay_ms(500);
    uart_putc(wireless_get_channel() + '0');
    uart_putsln_P(" ch");

    // static uint8_t count = 0;

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
#ifdef WITH_POWERDOWN
            power_down = 1;
            subsystems_power_down();
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            sleep_mode(); // This power downs the MCU
#endif
        }

        // Here we wakeup from sleep mode, after execution of the
        // timer2 interrupt
        _delay_ms(1000); // Wait until external resonator is stable

#ifdef WITH_POWERDOWN
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
    dbgled_red_off(); // TODO remove
}

// EOF
