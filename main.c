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
    sht11_down();

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



// TODO this belongs to the SHT11 module
static float convert_humi(int16_t raw_temp, int16_t raw_humi)
{
    const float c1 = -2.0468f;
    const float c2 = 0.0367f;
    const float c3 = -1.5955E-6;
    const float RHlinear = c1 + c2 * raw_humi + c3 * (raw_humi * raw_humi);
    return RHlinear;
}

// TODO this belongs to the SHT11 module
static float convert_temp(int16_t raw_temperature)
{
    const float d1 = -39.6f;
    const float d2 = 0.01f;
    return (d1 + d2 * (float) raw_temperature);
}

static void sht11_read(void)
{
    uint8_t n, v;
    int16_t temp = 0;

    sht11_init();

    if ((n = sht11_start_temp())) {
        for (n = 0; n < 30; n++) {
            _delay_ms(30);
            if (sht11_ready()) {
                temp = sht11_result_temp_get_raw();
                uart_puti16(temp);
                uart_space();
                uart_puti16((int16_t) convert_temp(temp) * 10.f);
                uart_space();
                int16_t tempC = sht11_result_temp_convert_sht11(temp);
                uart_puti16(tempC);
                uart_space();
                tempC = sht11_result_temp_convert_sht7x(temp);
                uart_puti16(tempC);
            }
        }
        // FIXME what the hell is that?
        v = PINC & (1 << PINC1);
        if (!v) {
            uart_putsln_P("ERROR timeout waiting for temperature result");
            return;
        }
    } else {
        uart_puts_P("ERROR start reading temperature");
        uart_putu8(n);
        uart_crlf();
        return;
    }

    uart_space();
    _delay_ms(50);

    if ((n = sht11_start_humid())) {
        for (n = 0; n < 30; n++) {
            _delay_ms(30);
            if (sht11_ready()) {
                uint16_t humid = sht11_result_humid_get_raw();
                uart_puti16(humid);
                uart_space();
                uart_puti16((int16_t) convert_humi(temp, humid) * 10.f);
                uart_space();
                int16_t humidC = sht11_result_humid_convert_sht11(humid);
                uart_puti16(humidC);
                uart_space();
                humidC = sht11_result_humid_convert_sht7x(temp, humid);
                uart_puti16(humidC);
            }
        }
        // FIXME what the hell is that?
        v = PINC & (1 << PINC1);
        if (!v) {
            uart_putsln_P("ERROR timeout waiting for humidity result");
        }
    } else {
        uart_puts_P("ERROR start reading humidity");
        uart_putu8(n);
        uart_crlf();
    }

    uart_crlf();
}



// TODO move to config.h !!!!! or not ???????
#define BMP085_OSS_VALUE BMP085_OVERSAMPLING_STANDARD

static bmp085_t bmp085;

// TODO move this read function into the bmp085 module without uart_*, leave the rest here
static void bmp085_read(void)
{
    // uart_putsln_P("dC  P     PNN");

    // Read uncompensated temperature value
    uint16_t ut = bmp085_read_ut();

    // Read uncompensated pressure value
    uint32_t up = bmp085_read_up(BMP085_OSS_VALUE);

    // Calculate temperature in deci degress C
    int16_t decicelsius = bmp085_calculate_temperature(ut, &bmp085);
    uart_puti16(decicelsius);
    uart_putc(' ');

    // Calculate true pressure
    int32_t p = bmp085_calculate_true_pressure(up, &bmp085, BMP085_OSS_VALUE);
    uart_puti32(p);
    uart_putc(' ');

    // Calculate pressure NN
#ifndef WITHOUT_BMP085_CALC_PRESSURE_NN
    const uint16_t altitude = 691;
    uint32_t pNN = (uint32_t) (((float) p) / pow((float)1 - ((float)altitude / (float) 44330), (float)5.255));
    uart_putu32(pNN);
#endif // WITHOUT_BMP085_CALC_PRESSURE_NN

    uart_crlf();
}



static void dowork(void)
{
    dbgled_on(); // LED enable for 2s
    bmp085_read();
    sht11_read();
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