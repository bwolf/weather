// wireles.h

#ifndef WL_HIGHLEVEL_H
#define WL_HIGHLEVEL_H

#if defined(WL_HIGHLEVEL_MODE_TX)
void    wlhl_init_tx(void);
uint8_t wlhl_busy_p(void);
void    wlhl_power_up(void);
void    wlhl_power_down(void);
void    wlhl_send_payload(uint8_t* data, uint8_t len);
#elif defined(WL_HIGHLEVEL_MODE_RX)
void    wlhl_init_rx(uint8_t payload_len);
uint8_t wlhl_data_ready_p(void);
void    wlhl_get_data(uint8_t *payload, uint8_t len);
#else // WL_HIGHLEVEL_xyz
# error "Either WL_HIGHLEVEL_MODE_TX or WL_HIGHLEVEL_MODE_RX must be defined."
#endif // WL_HIGHLEVEL_xyz

#endif // WL_HIGHLEVEL_H
