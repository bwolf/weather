// uart_addons.c

#include "config.h"

#include <avr/pgmspace.h>
#include <stdlib.h>

#include "uart.h"

#ifndef WITHOUT_UART

void uart_putu8_kernel(uint8_t n, uint8_t base)
{
    char buf[5];

    itoa(n, buf, base);
    uart_puts(buf);
}

void uart_putu8_b(uint8_t r)
{
    int8_t n;

    for (n = 7; n >= 0; n--) {
        uart_putc(((r >> n) & 0x01) ? '1' : '0');
    }
}

void uart_putu16_kernel(uint16_t n, uint8_t base)
{
    char buf[10];

    utoa(n, buf, base);
    uart_puts(buf);
}

void uart_puti16(int16_t n)
{
    char buf[10];

    itoa(n, buf, 10);
    uart_puts(buf);
}

void uart_putu32_kernel(uint32_t n, uint8_t base)
{
    char buf[20];

    ultoa(n, buf, base);
    uart_puts(buf);
}

void uart_puti32(int32_t n)
{
    char buf[20];

    ltoa(n, buf, 10);
    uart_puts(buf);
}

void uart_crlf(void)
{
    uart_puts_P("\r\n");
}

#endif // WITHOUT_UART
