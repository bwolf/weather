/* Copyright 2016 Marcus Geiger. All rights reserved. Use of this
 * source code is governed by a Apache 2.0 license that can be found
 * in the LICENSE file.
 */

#ifndef TWIM_H
#define TWIM_H

void twi_init(void);

// Data direction to read from TWI device as used by twi_start().
#define TWI_READ 1

// Data direction to write to TWI device as used by twi_start().
#define TWI_WRITE 0

// Error codes for twi_start
#define TWI_ERROR_START_COND 1
#define TWI_ERROR_START_ADDR 2

uint8_t twi_start(uint8_t address);
uint8_t twi_repeated_start(uint8_t address);
void twi_start_wait(uint8_t address);

void twi_stop(void);

uint8_t twi_write(uint8_t data);
uint8_t twi_read_ack(void);
uint8_t twi_read_nack(void);

#endif // TWIM_H
