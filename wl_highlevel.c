#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "dbgled.h"

#include "spi.h"

#include "wl_module.h"
#include "nRF24L01.h"

#include "wl_highlevel.h"

// TODO cleanup
// #if WIRELESS_MAX_PAYLOAD > wl_module_PAYLOAD
// # error "Inconsistent wireless vs. wl_module payload size!"
// #endif

// Initialize wireless module.
// Requires interrupts to be enabled.
void wlhl_init(void)
{
    uint8_t k;

    // -- Basic config
    wl_module_init();
    _delay_ms(50);

    // Configurate the interrupt
    WIRELESS_INTERRUPT_FALLING_EDGE();
    WIRELESS_INTERRUPT_ENABLE();

    // Configure SPI
    spi_init();

    // -- Config Module
    wl_module_tx_config(wl_module_TX_NR_0);

    // Wait for configuration to complete
    _delay_ms(10);

    // -- Check MAX_RT and clear it if set
    // Read wl_module status
    wl_module_CSN_lo;        //  Pull down chip select
    k = spi_fast_shift(NOP); // Read status register
    wl_module_CSN_hi;        // Pull up chip select

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

uint8_t wlhl_busy_p(void)
{
    return PTX;
}

// Power up 24L01+ and delay according to spec 6.1.7ff
void wlhl_power_up(void)
{
    wl_module_power_up();
    _delay_ms(5); // Sane delay
}

void wlhl_power_down(void)
{
    wl_module_power_down();
}

// Send given payload of length.
//
// Commentary
//
// From datasheet p. 29: With static payload length all packets
// between a transmitter and a receiver have the same length. Static
// payload length is set by the RX_PW_Px registers on the receiver
// side. The payload length on the transmitter side is set by the
// number of bytes clocked into the TX_FIFO and must equal the value
// in the RX_PW_Px register on the receiver side.
void wlhl_send_payload(uint8_t *payload, uint8_t len)
{
    // Be sure to send the correct amount of bytes as configured as
    // payload on the receiver.
    wl_module_send(payload, len);
}

ISR(WIRELESS_INTERRUPT_VECT)
{
    uint8_t status;

    dbgled_green_toggle(); // TODO remove

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
        // To manually start retransmission (auto retransmission timed out)
        // wl_module_CE_hi;                               // Start transmission
        // _delay_us(10);

        wl_module_CE_lo;
        PTX = 0; // Abort transmission
    }

    //  TX_FIFO FULL <-- this is not an IRQ
    if (status & (1 << TX_FULL)) {
        wl_module_CSN_lo;         //  Pull down chip select
        spi_fast_shift(FLUSH_TX); //  Flush TX-FIFO
        wl_module_CSN_hi;         //  Pull up chip select
    }
}

// EOF
