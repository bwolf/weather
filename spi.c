/*
  Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without
  restriction, including without limitation the rights to use, copy,
  modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  DEALINGS IN THE SOFTWARE.

  $Id$
*/

#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include "spi.h"

// A definition like the following is required from config.h
// #define SPI_PORT    PORTB
// #define SPI_DDR     DDRB
// #define SPI_DD_MISO DDB4
// #define SPI_DD_MOSI DDB3
// #define SPI_DD_SS   DDB2
// #define SPI_DD_SCK  DDB5

#ifndef SPI_PORT
# error "Missing definition for SPI port."
#endif

#ifndef SPI_DDR
# error "Missing definition for SPI DDR."
#endif

#ifndef SPI_DD_MISO
# error "Missing definition for SPI DDR MISO."
#endif

#ifndef SPI_DD_MOSI
# error "Missing definition for SPI DDR MOSI."
#endif

#ifndef SPI_DD_SS
# error "Missing definition for SPI DDR SS."
#endif

#ifndef SPI_DD_SCK
# error "Missing definition for SPI DDR SCK."
#endif

// Initialize pins for SPI communication
void spi_init()
{
    SPI_DDR &= ~((1 << SPI_DD_MOSI) | (1 << SPI_DD_MISO) | (1 << SPI_DD_SS) | (1 << SPI_DD_SCK));

    // Define the following pins as output
    SPI_DDR |= ((1 << SPI_DD_MOSI) | (1 << SPI_DD_SS) | (1 << SPI_DD_SCK));

    SPCR = ((1 << SPE)  |               // SPI Enable
            (0 << SPIE) |               // SPI Interupt Enable
            (0 << DORD) |               // Data Order (0:MSB first / 1:LSB first)
            (1 << MSTR) |               // Master/Slave select
            (0 << SPR1) | (1 << SPR0) | // SPI Clock Rate
            (0 << CPOL) |               // Clock Polarity (0:SCK low / 1:SCK hi when idle)
            (0 << CPHA));               // Clock Phase (0:leading / 1:trailing edge sampling)

    SPSR = (1 << SPI2X);                // Double Clock Rate
}

// Shift full array through target device
void spi_transfer_sync (uint8_t *dataout, uint8_t *datain, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        SPDR = dataout[i];
        while ((SPSR & (1 << SPIF)) == 0)
            ;
        datain[i] = SPDR;
    }
}

// Shift full array to target device without receiving any byte
void spi_transmit_sync (uint8_t *dataout, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        SPDR = dataout[i];
        while ((SPSR & (1 << SPIF)) == 0)
            ;
    }
}

// Clocks only one byte to target device and returns the received one
uint8_t spi_fast_shift (uint8_t data)
{
    SPDR = data;
    while ((SPSR & (1 << SPIF)) == 0)
        ;
    return SPDR;
}

/* EOF */
