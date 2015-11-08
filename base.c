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


static payload_t payload;

static void get_payload_and_do_work(void)
{
    static uint8_t once = 1;

    // Debug output
    if (once) {
        once = !once;
        uart_putsln_P("BMP085    SHT11");
        uart_puts_P(  "d/C P/NN  h/C  h/H%");
#ifdef SHT11_WITH_RAW_SENSOR_VALUES
        uart_puts_P(" rawT rawH");
#endif
        uart_crlf();
    }

    wlhl_get_data((uint8_t *) &payload, sizeof(payload));

    uart_puti16(payload.bmp085.decicelsius); uart_space();
    uart_putu16(payload.bmp085.pressure_nn); uart_space();
    uart_puti16(payload.sht11.temp);         uart_space();
    uart_puti16(payload.sht11.rh_true);
#ifdef SHT11_WITH_RAW_SENSOR_VALUES
    uart_space();
    uart_puti16(payload.sht11.raw_temp); uart_space();
    uart_puti16(payload.sht11.raw_humi);
#endif
    uart_crlf();
}


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

    uart_putsln_P("Receive loop");
    while (1) {
        while (!wlhl_data_ready_p()) {
            _delay_ms(10);
        }

        get_payload_and_do_work();
        dbgled_red_off();
    }
}

// EOF
