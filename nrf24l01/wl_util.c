#include "config.h"

#include <avr/io.h>

#include <uart/uart.h>
#include <uart/uart_addons.h>

#include <spi/spi.h>

#include "wl_module.h"
#include "nRF24L01.h"

#include "wl_util.h"

void wl_util_print_config_register(void)
{
    int8_t i;
    uint8_t r;

    wl_module_read_register(CONFIG, &r, 1);

    uart_puts_P("wl:config ");
    for (i = 7; i >= 0; i--) {
        uart_putc((r >> i) & 0x01 ? '1' : '0');
    }
    uart_crlf();
}

uint8_t wl_util_print_status_register(void)
{
    int8_t i;
    uint8_t r;

    /*  Read wl_module status */
    wl_module_CSN_lo;             /*  Pull down chip select */
    r = spi_fast_shift(NOP); /*  Read status register */
    wl_module_CSN_hi;             /*  Pull up chip select */

    uart_puts_P("wl:status ");
    for (i = 7; i >= 0; i--) {
        uart_putc((r >> i) & 0x01 ? '1' : '0');
    }
    uart_crlf();

    return r;
}

/* EOF */
