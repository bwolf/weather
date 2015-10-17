// uart_addons.h

#ifndef UART_ADDONS_H
#define UART_ADDONS_H

#ifndef WITHOUT_UART

void uart_putu8_kernel(uint8_t n, uint8_t base);
#define uart_putu8(n) uart_putu8_kernel(n, 10)
#define uart_putu8_x(n) uart_putu8_kernel(n, 16)

void uart_putu8_b(uint8_t r);

void uart_putu16_kernel(uint16_t n, uint8_t base);
#define uart_putu16(n) uart_putu16_kernel(n, 10)
#define uart_putu16_x(n) uart_putu16_kernel(n, 16)

void uart_puti16(int16_t n);

void uart_putu32_kernel(uint32_t n, uint8_t base);
#define uart_putu32(n) uart_putu32_kernel(n, 10)
#define uart_putu32_x(n) uart_putu32_kernel(n, 16)

void uart_puti32(int32_t n);

void uart_crlf(void);

#define uart_putsln(s) uart_puts(s); uart_crlf()
#define uart_putsln_P(s) uart_puts_P(s); uart_crlf()
#define uart_ff() uart_putc(0x0C)
#define uart_space() uart_putc(0x20)

#else // WITHOUT_UART
# define uart_putu8(u) (void) u
# define uart_putu8_x(u) (void) u
# define uart_putu8_b(u) (void) u
# define uart_putu16(u) (void) u
# define uart_putu16_x(u) (void) u
# define uart_puti16(i) (void) i
# define uart_putu32(u) (void) u
# define uart_putu32_x(u) (void) u
# define uart_puti32(u) (void) u
# define uart_crlf()
# define uart_putsln(s) (void) s
# define uart_putsln_P(s) (void) s
# define uart_ff()
# define uart_space()
#endif // WITHOUT_UART

#endif
