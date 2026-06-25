#pragma once

#include "esp_err.h"

// Connect to WiFi in STA mode.
// Blocks until connected or max retries exceeded.
// Returns ESP_OK on success, ESP_FAIL if connection failed.
esp_err_t wifi_connect(const char *ssid, const char *password);

// Returns the current IP address as a string.
// Valid only after wifi_connect() returns ESP_OK.
const char *wifi_get_ip(void);