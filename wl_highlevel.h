// wireles.h

#ifndef WIRELESS_H
#define WL_HIGHLEVEL_H

#define WIRELESS_MAX_PAYLOAD 16

void    wlhl_init(void);
uint8_t wlhl_is_busy(void);
void    wlhl_debug_print_status(void);
void    wlhl_power_up(void);
void    wlhl_power_down(void);
void    wlhl_send_payload(uint8_t* data, uint8_t len);
uint8_t wlhl_get_channel(void);

#endif // WL_HIGHLEVEL_H
