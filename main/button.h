#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

// Commands matching Python server
#define CMD_INIT      0x01
#define CMD_BTN_DOWN  0x02
#define CMD_BTN_UP    0x03
#define CMD_BTN_EXIT  0x04

// Initialise GPIO and start button task
esp_err_t button_init(void);

// Block until a button command is available
// Returns CMD_BTN_UP / CMD_BTN_DOWN / CMD_BTN_EXIT
bool button_wait(uint8_t *cmd, TickType_t timeout);