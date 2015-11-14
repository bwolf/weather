#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "spi.h"

#include "wl_module.h"
#include "nRF24L01.h"

#include "wl_highlevel.h"


#ifdef WL_HIGHLEVEL_MODE_TX

// SPI needs to be initialized before
void wlhl_init_tx(void)
{
    uint8_t k;

    // -- Basic config
    wl_module_init();
    _delay_ms(50);

    // Configurate the interrupt
    WIRELESS_INTERRUPT_FALLING_EDGE();
    WIRELESS_INTERRUPT_ENABLE();

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

#endif // WL_HIGHLEVEL_MODE_TX



#ifdef WL_HIGHLEVEL_MODE_RX

// Data ready flag, set by ISR, polled by wlhl_data_ready_p().
static volatile uint8_t rx_data_ready = 0;

// Requires interrupts enabled
void wlhl_init_rx(uint8_t payload_len)
{
    // -- Basic config
    wl_module_init();       // Init nRF Module
    _delay_ms(50);

    // Configure the interrupt
    WIRELESS_INTERRUPT_FALLING_EDGE();
    WIRELESS_INTERRUPT_ENABLE();

    // Configure SPI
    spi_init();

    // -- Config Module
    wl_module_config_n(payload_len); // config nRF as RX Module, with payload length
    _delay_ms(50);

    // -- Ensure the nRF24L01+ is usable.
    //
    // In particular when the MCU is reprogrammed, it may happen that
    // the nRF24L01+ may be stale in a state where it is unusable.
    //
    // Uncomment the uart_ calls to debug issues with the chip.
    uint8_t r = 0, c = 0;

    wl_module_read_register(CONFIG, &c, 1);
    // uart_puts_P("CONFIG      ");
    // uart_putu8_b(c);
    // uart_crlf();

    wl_module_CSN_lo;        // Pull down chip select
    r = spi_fast_shift(NOP); // Read status register
    wl_module_CSN_hi;        // Pull up chip select
    // uart_puts_P("STATUS      ");
    // uart_putu8_b(r);
    // uart_crlf();

    // Power down chip to prevent receiving packets while
    // configuring the cicruit.
    c &= ~(1 << PWR_UP);
    wl_module_write_register(CONFIG, &c, 1);
    _delay_ms(5);

    // Clear MAX_RT if asserted
    if (r & (1 << MAX_RT)) {
        // uart_putsln_P("Clearing MAX_RT");
        wl_module_config_register(STATUS, (1 << MAX_RT));
    }

    // Flush RX
    if (r & (1 << RX_DR)) {
        // uart_putsln_P("Flushing RX");
        wl_module_CSN_lo;         // Pull down chip select
        (void) spi_fast_shift(FLUSH_RX); // Write cmd to flush tx fifo
        wl_module_CSN_hi;         // Pull up chip select
        wl_module_config_register(STATUS, (1 << RX_DR));
    }

    // Flush TX
    if (r & (1 << TX_DS) || r & (1 << TX_FULL)) {
        // uart_putsln_P("Flushing TX");
        wl_module_CSN_lo;
        (void) spi_fast_shift(FLUSH_TX);
        wl_module_CSN_hi;
        wl_module_config_register(STATUS, (1 << TX_DS));
    }

    // Power up chip
    c |= (1 << PWR_UP);
    wl_module_write_register(CONFIG, &c, 1);
    _delay_ms(5);

    wl_module_read_register(CONFIG, &c, 1);
    // uart_puts_P("CONFIG      ");
    // uart_putu8_b(c);
    // uart_crlf();

    // wl_module_CSN_lo;        // Pull down chip select
    // r = spi_fast_shift(NOP); // Read status register
    // wl_module_CSN_hi;        // Pull up chip select
    // uart_puts_P("STATUS      ");
    // uart_putu8_b(r);
    // uart_crlf();

    wl_module_read_register(EN_RXADDR, &r, 1);
    // uart_puts_P("EN_RXADDR   ");
    // uart_putu8_b(r);
    // uart_crlf();

    // Overwrite EN_RXADDR with default values
    if (r != ((1 << ERX_P1) | (1 << ERX_P0))) {
        // uart_putsln_P("Resetting EN_RXADDR");
        r = (1 << ERX_P1) | (1 << ERX_P0);
        wl_module_write_register(EN_RXADDR, &r, 1);
    }

    // wl_module_read_register(FIFO_STATUS, &r, 1);
    // uart_puts_P("FIFO_STATUS ");
    // uart_putu8_b(r);
    // uart_crlf();
}

uint8_t wlhl_data_ready_p(void)
{
    uint8_t tmp = rx_data_ready;
    rx_data_ready = 0;
    return tmp;
}

void wlhl_get_data(uint8_t *data, uint8_t len)
{
    (void) wl_module_get_data_n(data, len);
}

// Commentary
//
// The alternative to use the interrupt is to poll for RX_DR Flag in
// the STATUS register like this:
//
// while (!wl_module_data_ready()) {
//     _delay_ms(1);
// }
//
ISR(WIRELESS_INTERRUPT_VECT)
{
    uint8_t status;

    wl_module_CSN_lo;             //  Pull down chip select
    status = spi_fast_shift(NOP); //  Read status register
    wl_module_CSN_hi;             //  Pull up chip select

    // IRQ: Package has been received
    if (status & (1 << RX_DR)) {
        wl_module_config_register(STATUS, (1 << RX_DR)); // Clear interrupt bit
        // Set flag to indicate main loop that data is ready
        rx_data_ready = 1;
    }
}

#endif // WL_HIGHLEVEL_MODE_RX

// EOF
