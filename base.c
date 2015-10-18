// base.c -- indoor base station aka receiver

#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "uart.h"
#include "uart_addons.h"

#include "dbgled.h"

// TODO Use wl_highlevel such that includes of nRF24l01 are not needed
#include "nRF24l01.h"
#include "wl_module.h"
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

    // Debug output
    if (once) {
        once = !once;
        uart_putsln_P("BMP085     SHT11");
        uart_putsln_P("dC  P (NN) hC  hH%");
        //             231 10165 2333 5036    -- For alignment of the header
    }

    (void) wl_module_get_data_n((uint8_t *) &payload, sizeof(payload));

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

    _delay_ms(2000); // Delay startup to give host time to init uart

    uart_putsln_P("wl_module_init");
    wl_module_init();       // Init nRF Module
    _delay_ms(50);          // wait for Module
    // Configure interrupt
    WIRELESS_INTERRUPT_FALLING_EDGE();
    WIRELESS_INTERRUPT_ENABLE();
    spi_init();

    sei(); // activate Interrupts

    wl_module_config_n(sizeof(payload)); // config nRF as RX Module, with payload length
    _delay_ms(50);


    // TODO clean up this mess, integrate into wl_highlevel?
    // ----------------------------------------------------------------------
    // Debugging of restart problems after ISP
    {
        uint8_t r = 0, c = 0;

        wl_module_read_register(CONFIG, &c, 1);
        uart_puts_P("CONFIG      ");
        uart_putu8_b(c);
        uart_crlf();

        wl_module_CSN_lo;        // Pull down chip select
        r = spi_fast_shift(NOP); // Read status register
        wl_module_CSN_hi;        // Pull up chip select
        uart_puts_P("STATUS      ");
        uart_putu8_b(r);
        uart_crlf();

        // Power down chip
        c &= ~(1 << PWR_UP);
        wl_module_write_register(CONFIG, &c, 1);
        _delay_ms(5);

        // Clear MAX_RT if asserted
        if (r & (1 << MAX_RT)) {
            uart_putsln_P("Clearing MAX_RT");
            wl_module_config_register(STATUS, (1 << MAX_RT));
        }

        // Flush RX
        if (r & (1 << RX_DR)) {
            uart_putsln_P("Flushing RX");
            wl_module_CSN_lo;         // Pull down chip select
            spi_fast_shift(FLUSH_RX); // Write cmd to flush tx fifo
            wl_module_CSN_hi;         // Pull up chip select
            wl_module_config_register(STATUS, (1 << RX_DR));
        }

        // Flush TX
        if (r & (1 << TX_DS) || r & (1 << TX_FULL)) {
            uart_putsln_P("Flushing TX");
            wl_module_CSN_lo;
            spi_fast_shift(FLUSH_TX);
            wl_module_CSN_hi;
            wl_module_config_register(STATUS, (1 << TX_DS));
        }

        // Power up chip
        c |= (1 << PWR_UP);
        wl_module_write_register(CONFIG, &c, 1);
        _delay_ms(5);

        wl_module_read_register(CONFIG, &c, 1);
        uart_puts_P("CONFIG      ");
        uart_putu8_b(c);
        uart_crlf();

        wl_module_CSN_lo;        // Pull down chip select
        r = spi_fast_shift(NOP); // Read status register
        wl_module_CSN_hi;        // Pull up chip select
        uart_puts_P("STATUS      ");
        uart_putu8_b(r);
        uart_crlf();

        wl_module_read_register(EN_RXADDR, &r, 1);
        uart_puts_P("EN_RXADDR   ");
        uart_putu8_b(r);
        uart_crlf();

        // Overwrite EN_RXADDR with default values
        if (r != ((1 << ERX_P1) | (1 << ERX_P0))) {
            uart_putsln_P("Resetting EN_RXADDR");
            r = (1 << ERX_P1) | (1 << ERX_P0);
            wl_module_write_register(EN_RXADDR, &r, 1);
        }

        wl_module_read_register(FIFO_STATUS, &r, 1);
        uart_puts_P("FIFO_STATUS ");
        uart_putu8_b(r);
        uart_crlf();
    }

    uart_putsln_P("Receive loop");
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
