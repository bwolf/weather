#include "config.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <util/delay.h>

#include <stdlib.h>

#include "uart.h"
#include "uart_addons.h"

#include "dbgled.h"

#include "adc.h"

#include "twim.h"
#include "bmp085.h"

#include "sht11.h"

#include "wl_highlevel.h"

#include "payload.h"


#ifdef WITHOUT_POWERDOWN
# warning "Compiling without power saving (powerdown)!"
#endif

#ifdef WITHOUT_UART
# warning "Building without UART, you won't see any output!"
#endif


// Predicate to check if power down is possible by checking subsystems.
static uint8_t power_down_p(void)
{
    // nRF24L01+ may be busy during transmission, so don't power down
    // unconditionally.
    return !wlhl_busy_p();
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

    // nRF24L01+
    wlhl_power_down();
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

    // nRF24L01+ powering up takes some ms!
    wlhl_power_up();
}


static bmp085_coeff_t bmp085_coeff; // BMP085 EEPROM coefficients for calculation

// Global variable containing sensor read out to be transmitted wireless
static payload_t payload;

static void dowork(void)
{
#ifdef WITH_UART
    static uint8_t once = 1;

    // Debug output
    if (once) {
        once = !once;
        uart_putsln_P("BMP085    SHT11");
        uart_putsln_P("dC  P/NN  hC   hH%");
        //             231 10246 2333 5036 -- For alignment of the header
    }
#endif

    dbgled_red_on(); // LED enable

    if (wlhl_busy_p()) {
        // Do nothing, give wireless module time to finish
        uart_putsln_P("wireless_is_busy");
    } else {
        // -- BMP086 / Pressure
        bmp085_read(&payload.bmp085, &bmp085_coeff);
        // Debug output
        uart_puti16(payload.bmp085.decicelsius); uart_space();
        uart_putu16(payload.bmp085.pressure_nn); uart_space();

        // -- SHT11 / Humidity
        sht11_init();
        if (sht11_read(&payload.sht11)) {
            uart_puts_P("sht11 error"); // Debug output
        } else {
            // Debug output
            uart_puti16(payload.sht11.temp); uart_space();
            uart_puti16(payload.sht11.rh_true);
        }
        sht11_down();

        uart_crlf(); // Debug output

        // Transmit measurements
        dbgled_green_toggle();
        wlhl_send_payload((uint8_t *) &payload, sizeof(payload));
    }

    dbgled_red_off();
}


// Do work after 10 seconds
#define TIMER2_TICKS_UNTIL_WORK 10 // TODO adjust prescaler and TICKS to sleep longer

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
    bmp085_init(&bmp085_coeff);
    bmp085_read_calibration_coefficients(&bmp085_coeff);

    // Use timer/counter2 in async mode to wakeup from power save.
    // An external resonator with 32.768 kHz is required on TOSC1/TOSC2.
    ASSR  = (1<< AS2);                   // Drive timer2 asynchronously
    _delay_ms(1000);                     // Wait until external resonator is stable

#if defined(__AVR_ATmega88__)
    TCCR2 |= (1 << CS22) | (1 << CS20);  // Prescaler 128
    while ((ASSR & (1 << TCR2UB)))       // Wait until register access has completed
        ;
    TIFR = (1 << TOV2);                  // Clear timer interrupts
    TIMSK |= (1 << TOIE2);               // Enable timer overflow interrupt
#elif defined(__AVR_ATmega88PA__)
    TCCR2B |= (1 << CS22) | (1 << CS20); // Prescaler 128
    while ((ASSR & (1 << TCR2BUB)))      // Wait until register access has completed
        ;
    TIFR2 = (1 << TOV2);                 // Clear timer interrupts
    TIMSK2 |= (1 << TOIE2);              // Enable timer overflow interrupt
#else
# error "Unsupported MCU."
#endif

    sei(); // With interrupts...

    uart_ff();

    // Wireless setup requires interrupts
    _delay_ms(50); // TODO delay required?
    wlhl_init_tx();

#ifdef WITH_POWERDOWN
    uint8_t power_down; // Remember if powered down
#endif

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
#if defined(__AVR_ATmega88__)
        OCR2 = 0;                       // Dummy access
        while ((ASSR & (1<< OCR2UB)))   // Wait until register access has completed
            ;
#elif defined(__AVR_ATmega88PA__)
        OCR2A = 0;                      // Dummy access
        while ((ASSR & (1 << OCR2BUB))) // Wait until register access has completed
            ;
#else
# error "Unsupported MCU."
#endif

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
    dbgled_red_off();
}

// EOF
