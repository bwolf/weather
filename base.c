// base.c -- indoor base station aka receiver

#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "uart.h"
#include "uart_addons.h"

#include "dbgled.h"

#include "adc.h"

#include "wl_highlevel.h"

#define BMP085_DATA_TYPE_ONLY
#include "bmp085.h"
#define SHT11_DATA_TYPE_ONLY
#include "sht11.h"
#include "payload.h"


// UART helpers

static uint8_t comment_count = 0;

#define uart_comment(text)                              \
    uart_puts_P("# ");                                  \
    uart_putu8(comment_count++);                        \
    uart_space();                                       \
    uart_putsln_P(text)

#define uart_colon() uart_puts_P(", ")


// Business logic

static payload_t payload;

static void get_payload_and_do_work(void)
{
    static uint8_t once = 1;

    // Debug output
    if (once) {
        once = !once;
        uart_comment("BMP085    SHT11");
        uart_comment(  "d/C P/NN  h/C  h/H%");
    }

    wlhl_get_data((uint8_t *) &payload, sizeof(payload));

    // JSON output to UART: { "weather": { ... } }
    uart_puts_P("{ \"weather\": { ");
    uart_puts_P("\"station-id\": ");
    uart_putu8(payload.station_id);
    uart_colon();
    uart_puts_P("\"temp_m\": ");
    uart_puti16(payload.sht11.temp);
    uart_colon();
    uart_puts_P("\"rh-true_m\": ");
    uart_puti16(payload.sht11.rh_true);
    uart_colon();
    uart_puts_P("\"pressure-nn_c\": ");
    uart_putu16(payload.bmp085.pressure_nn);
    uart_puts_P(" } }");
    uart_crlf();
}


// Main entry point

int __attribute__((OS_main))
main(void)
{
    disable_ad_converter();
    disable_analog_comparator();

    _delay_ms(100);
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    dbgled_red_init();

    _delay_ms(2000); // Delay startup to give host time to init uart

    sei(); // activate Interrupts

    wlhl_init_rx(sizeof(payload));
    uart_comment("Receive loop");
    while (1) {
        while (!wlhl_data_ready_p()) {
            _delay_ms(500);
        }

        uart_comment("Got one");
        dbgled_red_toggle();
        get_payload_and_do_work();
    }
}

// EOF
