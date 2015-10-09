#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <spi.h>

#include "wl_module.h"
#include "wl_util.h"
#include "nRF24L01.h"

#if WIRELESS_MAX_PAYLOAD > wl_module_PAYLOAD
# error "Inconsistent wireless vs. wl_module payload size!"
#endif

// Initialize wireless module.
// Requires interrupts to be enabled.
void wireless_init(void)
{
    uint8_t k;

    // Configurate interrupt before initializing wl_module, because it
    // immediately enables SPI
    WIRELESS_INTERRUPT_FALLING_EDGE();
    WIRELESS_INTERRUPT_ENABLE();

    // Requires configured interrupt, initializes SPI
    wl_module_init();
    _delay_ms(50);

    wl_module_tx_config(wl_module_TX_NR_0); // Config Module

    _delay_ms(10);

    // Check MAX_RT
    wl_util_print_config_register();
    k = wl_util_print_status_register();
    if (k & STATUS_MAX_RT) {
        // Clearing STATUS_MAX_RT
        wl_module_config_register(STATUS, (1 << MAX_RT));
    }

    _delay_ms(10);

    // Flush data out of TX FIFO (there may be data after reset)
    wl_module_CSN_lo;           // Pull down chip select
    spi_fast_shift(FLUSH_TX);   // Write cmd to flush tx fifo
    wl_module_CSN_hi;           // Pull up chip select
}

uint8_t wireless_is_busy(void)
{
    return PTX;
}

void wireless_debug_print_status(void)
{
    wl_util_print_config_register();
    wl_util_print_status_register();
}

// Power up 24L01+ and delay according to spec 6.1.7ff
void wireless_power_up(void)
{
    wl_module_power_up();
    _delay_ms(5); // Sane delay
}

void wireless_power_down(void)
{
    wl_module_power_down();
}

void wireless_send_payload(uint8_t *data, uint8_t len)
{
    uint8_t payload[wl_module_PAYLOAD];
    uint8_t k;

    for (k = 0; k < len && k < wl_module_PAYLOAD; k++) {
        payload[k] = data[k];
    }

    // Be sure to send the correct
    // amount of bytes as configured as payload on the receiver.
    wl_module_send(payload, wl_module_PAYLOAD);
}

uint8_t wireless_get_channel(void)
{
    return wl_module_get_rf_ch();
}

void dbgled_green_toggle(void);

ISR(WIRELESS_INTERRUPT_VECT)
{
    uint8_t status;

    // TODO remove
    dbgled_green_toggle();

    //  Read wl_module status
    wl_module_CSN_lo;             //  Pull down chip select
    status = spi_fast_shift(NOP); //  Read status register
    wl_module_CSN_hi;             //  Pull up chip select

    //  IRQ: Package has been sent
    if (status & (1 << TX_DS)) {
        wl_module_config_register(STATUS, (1 << TX_DS));  //  Clear Interrupt Bit
        PTX = 0; // Unflag transmission mode
    }

    //  IRQ: Package has not been sent, send again
    if (status & (1 << MAX_RT)) {
        wl_module_config_register(STATUS, (1 << MAX_RT)); //  Clear MAX_RT Bit
        // wl_module_CE_hi;                               // Start transmission
        // _delay_us(10);

        wl_module_CE_lo;
        PTX = 0; // Abort transmission
        // dbgled_pulse(3);
    }

    //  TX_FIFO FULL <-- this is not an IRQ
    if (status & (1 << TX_FULL)) {
        wl_module_CSN_lo;         //  Pull down chip select
        spi_fast_shift(FLUSH_TX); //  Flush TX-FIFO
        wl_module_CSN_hi;         //  Pull up chip select
    }
}

// EOF
