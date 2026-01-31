#ifndef _BTCLIENT_H
#define _BTCLIENT_H

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/sync.h"

typedef struct hid_state {
    uint32_t buttons;
    uint32_t buttons_toggled;  // indicates the button changed value
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint8_t l2;
    uint8_t r2;
    uint8_t hat;
    uint8_t pad;
} hid_state_t;

typedef enum {
    BLINK_OFF = 0,
    BLINK_SLOW = 1,
    BLINK_MED = 2,
    BLINK_FAST = 3
} blink_state_t;

extern blink_state_t blink_state;
extern bool hid_host_descriptor_available;

int btclient_setup();
void btclient_get_hid_state(hid_state_t* dest);

#endif // _BTCLIENT_H
