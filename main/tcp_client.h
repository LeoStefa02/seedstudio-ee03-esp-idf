#pragma once
#include "esp_err.h"

#define CMD_INIT      0x01
#define CMD_BTN_DOWN  0x02
#define CMD_BTN_UP    0x03
#define CMD_BTN_EXIT  0x04

esp_err_t tcp_client_start(uint8_t *fb, size_t fb_size);