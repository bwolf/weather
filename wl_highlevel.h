// wireles.h

#ifndef WIRELESS_H
#define WL_HIGHLEVEL_H

#define WIRELESS_MAX_PAYLOAD 16

void wireless_init(void);
uint8_t wireless_is_busy(void);
void wireless_debug_print_status(void);
void wireless_power_up(void);
void wireless_power_down(void);
void wireless_send_payload(uint8_t* data, uint8_t len);
uint8_t wireless_get_channel(void);

#endif // WL_HIGHLEVEL_H
