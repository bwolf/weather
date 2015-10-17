// uart_addons.h

#ifndef UART_ADDONS_H
#define UART_ADDONS_H

#ifndef WITHOUT_UART

void uart_putu8_kernel(uint8_t n, uint8_t base);
static inline void uart_putu8(uint8_t n) { uart_putu8_kernel(n, 10); }
static inline void uart_putu8_x(uint8_t n) { uart_putu8_kernel(n, 16); }

void uart_putu8_b(uint8_t r);

void uart_putu16_kernel(uint16_t n, uint8_t base);
static inline void uart_putu16(uint16_t n) { uart_putu16_kernel(n, 10); }
static inline void uart_putu16_x(uint16_t n) { uart_putu16_kernel(n, 16); }

void uart_puti16(int16_t n);

void uart_putu32_kernel(uint32_t n, uint8_t base);
static inline void uart_putu32(uint32_t n) { uart_putu32_kernel(n, 10); }
static inline void uart_putu32_x(uint32_t n) { uart_putu32_kernel(n, 16); }
void uart_puti32(int32_t n);

static void uart_crlf(void) { uart_puts_P("\r\n"); }
static inline void uart_ff(void) { uart_putc(0x0C); }
static inline void uart_space(void) { uart_putc(0x20); }

static inline void uart_putsln(const char *s) { uart_puts(s); uart_crlf(); }
#define uart_putsln_P(s) uart_puts_P(s); uart_crlf()

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
