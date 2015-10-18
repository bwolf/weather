// wireles.h

#ifndef WL_HIGHLEVEL_H
#define WL_HIGHLEVEL_H

// TODO cleanup
// #define WIRELESS_MAX_PAYLOAD 16

void    wlhl_init(void);
uint8_t wlhl_busy_p(void);
void    wlhl_power_up(void);
void    wlhl_power_down(void);
void    wlhl_send_payload(uint8_t* data, uint8_t len);

#endif // WL_HIGHLEVEL_H
