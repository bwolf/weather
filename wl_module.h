/*
 * Copyright (c) 2011 by Ernst Buchmann
 * Copyright (c) 2015 by Marcus Geiger
 *
 * Code based on the work of Stefan Engelke and Brennan Ball
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef WL_MODULE_H
#define WL_MODULE_H

//#include <avr/io.h>

// External variables of WL-Module
extern volatile uint8_t PTX;

// WL-Module settings
#define wl_module_CH         2

// Note: The payload constant has been disabled, since we changed the
// function interface to use the given payload directly without
// copying the data to a fixed buffer first and then transfer it to
// nRF24L01+.
//
// #define wl_module_PAYLOAD    16

#define wl_module_RF_DR_HIGH 0 // 0 = 1Mbps, 1 = 2Mpbs
#define wl_module_RF_SETUP   (RF_SETUP_RF_PWR_0 | RF_SETUP_RF_DR_250)
#define wl_module_CONFIG     ((0 << MASK_RX_DR) | (1 << EN_CRC) | (1 << CRCO))
#define wl_module_TX_NR_0    0
#define wl_module_TX_NR_1    1
#define wl_module_TX_NR_2    2
#define wl_module_TX_NR_3    3
#define wl_module_TX_NR_4    4
#define wl_module_TX_NR_5    5

// Pin definitions for chip select and chip enabled of the wl-module
// are defined in config.h and referenced in wl_module.c.

// Definitions for selecting and enabling wl_module module
#define wl_module_CSN_hi WL_MODULE_PORT |=  (1 << WL_MODULE_CSN);
#define wl_module_CSN_lo WL_MODULE_PORT &= ~(1 << WL_MODULE_CSN);
#define wl_module_CE_hi  WL_MODULE_PORT |=  (1 << WL_MODULE_CE);
#define wl_module_CE_lo  WL_MODULE_PORT &= ~(1 << WL_MODULE_CE);

// Defines for setting the wl_module registers for transmitting or receiving mode
#define TX_POWERUP wl_module_config_register(CONFIG, wl_module_CONFIG | ((1 << PWR_UP) | (0 << PRIM_RX)))
#define RX_POWERUP wl_module_config_register(CONFIG, wl_module_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX)))

// Public standard functions
void wl_module_init(void);
void wl_module_config(void);
void wl_module_config_n(uint8_t payload_len);
void wl_module_send(uint8_t *value, uint8_t len);
void wl_module_set_RADDR(uint8_t *adr);
void wl_module_set_TADDR(uint8_t *adr);
uint8_t wl_module_data_ready(void);
// uint8_t wl_module_get_data(uint8_t *data);
uint8_t wl_module_get_data_n(uint8_t *data, uint8_t len);

// Public functions
uint8_t wl_module_get_status(void);
uint8_t wl_module_get_rx_pipe_reading_status(void);
uint8_t wl_module_get_one_byte(uint8_t command);
void wl_module_set_rx_pw(unsigned char payloadwidth, unsigned char rxpipenum);
uint8_t wl_module_get_rx_pw(uint8_t rxpipenum);
void wl_module_set_tx_addr(uint8_t *address, uint8_t len);
void wl_module_set_rx_addr(uint8_t *address, uint8_t len, uint8_t rxpipenum);
void wl_module_tx_config(uint8_t tx_nr);
// void wl_module_rx_config(void);
void wl_module_get_rx_addr(uint8_t *data, uint8_t rxpipenum, uint8_t len);
uint8_t wl_module_get_rx_pipe(void);
uint8_t wl_module_get_rx_pipe_from_status(uint8_t status);
uint8_t wl_module_fifo_tx_empty(void); // returns true if TX_EMPTY bit in FIFO_STATUS register is set, false otherwise
uint8_t wl_module_fifo_rx_empty(void);
uint8_t wl_module_get_rf_ch(void);
uint8_t wl_module_get_rf_setup(void);
uint8_t wl_module_get_plos_cnt(void);
uint8_t wl_module_get_arc_cnt(void);
void wl_module_set_as_tx(void); // activate module with existing config
void wl_module_power_down(void); // powers down the module with existing config
void wl_module_power_up(void); // power up the module with existing config

// Public extended functions
void wl_module_config_register(uint8_t reg, uint8_t value);
void wl_module_read_register(uint8_t reg, uint8_t *value, uint8_t len);
void wl_module_write_register(uint8_t reg, uint8_t *value, uint8_t len);

#endif /* WL_MODULE_H */
