#pragma once

#include <host/ble_hs.h>
#include <stdint.h>

extern uint16_t conn_handle;
extern uint16_t encoder_handle;
extern bool notify_state;


/// @brief initializes the buttons task
/// @return 0 if ok, >0 if not ok
uint8_t ble_init(void);
