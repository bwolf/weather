// base.c -- indoor base station aka receiver

#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "uart.h"
#include "uart_addons.h"

#include "dbgled.h"

#include "nRF24l01.h"
#include "wl_module.h"
#include "wl_util.h"
#include "spi.h"

#define BMP085_DATA_TYPE_ONLY
#include "bmp085.h"
#define SHT11_DATA_TYPE_ONLY
#include "sht11.h"
#include "payload.h"

static payload_t payload;

static void get_payload_and_do_work(void)
{
    static uint8_t once = 1;
    uint8_t raw_payload[wl_module_PAYLOAD];   // holds the payload

    // Debug output
    if (once) {
        once = !once;
        uart_putsln_P("BMP085     SHT11");
        uart_putsln_P("dC  P (NN) hC  hH%");
        //             231 10165 2333 5036    -- For alignment of the header
    }

    (void) wl_module_get_data(raw_payload);   // reads the incomming Data to Array payload

    uint8_t n;
    uint8_t *p = (uint8_t *) &payload;
    for (n = 0; n < sizeof(payload); n++) {
        p[n] = raw_payload[n];
    }

    uart_puti16(payload.bmp085.decicelsius); uart_space();
    uart_putu16(payload.bmp085.pressure_nn); uart_space();
    uart_puti16(payload.sht11.temp);         uart_space();
    uart_puti16(payload.sht11.rh_true);
    uart_crlf();
}

static volatile uint8_t data_ready = 0;

int __attribute__((OS_main))
main(void)
{
    _delay_ms(100);
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    dbgled_red_init();

    uart_putsln_P("wl_module_init");
    wl_module_init();       // Init nRF Module
    _delay_ms(50);          // wait for Module
    // Configure interrupt
    WIRELESS_INTERRUPT_FALLING_EDGE();
    WIRELESS_INTERRUPT_ENABLE();
    spi_init();

    sei();                  // activate Interrupts

    wl_module_config();     // config nRF as RX Module, simple Version
    _delay_ms(10);

    uart_putc(wl_module_get_rf_ch() + '0');
    uart_putsln_P(" chan");

    {
        uint8_t k;

        wl_module_CSN_lo;        //  Pull down chip select
        k = spi_fast_shift(NOP); // Read status register
        wl_module_CSN_hi;        // Pull up chip select
        if (k & STATUS_MAX_RT) {
            // Clearing STATUS_MAX_RT
            uart_putsln_P("clearing MAX_RT");
            wl_module_config_register(STATUS, (1 << MAX_RT));
        }


        uint8_t r = 0;
        wl_module_read_register(CONFIG, &r, 1);
        uart_puts_P("CONFIG");
        uart_space();
        uart_putu8_b(r);

        wl_module_CSN_lo;        //  Pull down chip select
        r = spi_fast_shift(NOP); // Read status register
        wl_module_CSN_hi;        // Pull up chip select
        uart_puts_P("STATUS ");
        uart_space();
        uart_putu8_b(r);
    }

    uart_putsln_P("Entering receive loop");

    while (1) {
        // Poll for RX_DR Flag in STATUS register
        // while (!wl_module_data_ready()) {
        //     _delay_ms(1);
        // }

        while (!data_ready) {
            _delay_ms(10);
        }

        get_payload_and_do_work();
        dbgled_red_off();
        data_ready = 0;
    }
}

ISR(WIRELESS_INTERRUPT_VECT)
{
    uint8_t status;

    wl_module_CSN_lo;             //  Pull down chip select
    status = spi_fast_shift(NOP); //  Read status register
    wl_module_CSN_hi;             //  Pull up chip select

    // IRQ: Package has been received
    if (status & (1 << RX_DR)) {
        dbgled_red_on();
        wl_module_config_register(STATUS, (1 << RX_DR)); // Clear interrupt bit
        // Set flag to indicate main loop that data is ready
        data_ready = 1;
    }
}

// EOF
