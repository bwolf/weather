// twim.c - TWI master according Atmel datasheet

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

#include "twim.h"


// SCL_CLOCK required for the clock calculation
#ifndef SCL_CLOCK
# error "Missing definition of SCL_CLOCK"
#endif

// F_CPU is required for the clock calculation
#ifndef F_CPU
# error "Missing definition of F_CPU"
#endif


// Init two wire interface
void twi_init(void)
{
    // TWI clock setting
    TWSR = 0; // No prescaler
    TWBR = ((F_CPU/SCL_CLOCK)-16UL)/2UL; // Must be > 10 for stable operation
}

/* Note regarding START, REPEATED START and STOP (according datasheet):
 *
 * The Master initiates and terminates a data transmission. The
 * transmission is initiated when the Master issues a START condition
 * on the bus, and it is terminated when the Master issues a STOP
 * condition. Between a START and a STOP condition, the bus is
 * considered busy, and no other master should try to seize control of
 * the bus. A special case occurs when a new START condition is issued
 * between a START and STOP condition. This is referred to as a
 * REPEATED START condition, and is used when the Master wishes to
 * initiate a new transfer without relinquishing control of the bus.
 * After a REPEATED START, the bus is considered busy until the next
 * STOP. This is identical to the START behavior, and therefore START
 * is used to describe both START and REPEATED START for the remainder
 * of this datasheet, unless otherwise noted. As depicted below, START
 * and STOP conditions are signalled by changing the level of the SDA
 * line when the SCL line is high.
 */

// Issues start condition, sends address and transfer direction.
//
// Param: address - the address and transfer direction of the TWI device.
//
// Return 0 if ok, > 0 otherwise
uint8_t twi_start(uint8_t address)
{
    uint8_t status;

    // Send START condition
    //
    // NOTE: a REPEATED START condition is sent analogus to a START
    // condition but the hardware responds with a different status
    // code.
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);

    // Wait for TWINT Flag set. This indicates that the START
    // condition has been transmitted
    while  (!(TWCR & (1<<TWINT)))
        ;

    // Check value of TWI status register. Mask prescaler bits. If
    // status different from START go to ERROR state.
    status = TWSR & 0xF8;
    if (status != TW_START && status != TW_REP_START)
        return TWI_ERROR_START_COND;

    // Send device address:
    // Load SLA_W into TWDR register. Clear TWINT bit in TWCR to start
    // transmission of address.
    TWDR = address;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // Wait for TWINT flag set. This indicates that the SLA+W has been
    // transmitted, and ACK/NACK has been received.
    while (!(TWCR & (1<<TWINT)))
        ;

    // Check value of TWI status register. Mask prescaler bits. If
    // status is different from MT_SLA_ACK go to ERROR state.
    status = TWSR & 0xF8;
    if (status != TW_MT_SLA_ACK && status != TW_MR_SLA_ACK)
        return TWI_ERROR_START_ADDR;

    return 0;
}

// Issues a repeated START condition, sends address and transfer direction.
//
// Param: address - the address and transfer direction of the TWI device.
// Return 0 if ok, > 0 otherwise
uint8_t twi_repeated_start(uint8_t address)
{
    return twi_start(address);
}

// Issues START condition, sends address and transfer direction.
// Polls for ACK to wait for device ready (blocks).
//
// Param: address - the address and transfer direction of the TWI device.
void twi_start_wait(uint8_t address)
{
    while (1) {
        // Send START condition
        TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);

        // Wait for TWINT Flag set. This indicates that the START
        // condition has been transmitted
        while  (!(TWCR & (1<<TWINT)))
            ;

        // Check value of TWI status register. Mask prescaler bits. If
        // status different from START go to ERROR state.
        uint8_t status = TWSR & 0xF8;
        if (status != TW_START && status != TW_REP_START)
            continue;

        // Send device address:
        // Load SLA_W into TWDR register. Clear TWINT bit in TWCR to start
        // transmission of address.
        TWDR = address;
        TWCR = (1<<TWINT) | (1<<TWEN);

        // Wait for TWINT flag set. This indicates that the SLA+W has been
        // transmitted, and ACK/NACK has been received.
        while (!(TWCR & (1<<TWINT)))
            ;

        // Check value of TWI status register. Mask prescaler bits. If
        // status is different from MT_SLA_ACK go to ERROR state.
        status = TWSR & 0xF8;
        if (status == TW_MT_SLA_NACK || status == TW_MR_DATA_NACK) {
            // Device is busy, send STOP condition to terminate the
            // write operation.
            TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

            // Wait until the STOP condition has been executed and the
            // bus has been released.
            while (TWCR & (1<<TWSTO))
                ;

            // Start over
            continue;
        }
        break;
    }
}

// Terminate data transfer and release TWI
void twi_stop(void)
{
    // Transmit STOP condition
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

    // Wait until the STOP condition has been executed and the
    // bus has been released.
    while (TWCR & (1<<TWSTO))
        ;
}

// Write a byte to the TWI device.
//
// Return 0 if ok, > 0 otherwise
uint8_t twi_write(uint8_t data)
{
    // Load DATA into TWDR register. Clear TWINT bit in TWCR to start
    // transmission of data.
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // Wait for TWINT flag set. This indicates that the DATA has been
    // transmitted, and ACK/NACK has been received.
    while (!(TWCR & (1<<TWINT)))
        ;

    // Check value of TWI status register. Mask prescaler bits. If
    // status different from MT_DATA_ACK go to ERROR state.
    if ((TWSR & 0xF8) != TW_MT_DATA_ACK)
        return 1;

    return 0;
}

// Read a byte from the TWI device, request more data.
uint8_t twi_read_ack(void)
{
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    while (!(TWCR & (1<<TWINT)))
        ;

    return TWDR;
}

// Read a byte from the TWI device, followed by a STOP condition.
uint8_t twi_read_nack(void)
{
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)))
        ;

    return TWDR;
}

// EOF
