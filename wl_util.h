#ifndef WL_UTIL_H
#define WL_UTIL_H


#ifndef WITHOUT_UART
void wl_util_print_config_register(void);
uint8_t wl_util_print_status_register(void);
#else
# define wl_util_print_config_register()
# define wl_util_print_status_register()
#endif

#endif /* WL_UTIL_H */
