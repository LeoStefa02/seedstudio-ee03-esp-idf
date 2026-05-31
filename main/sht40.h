#pragma once

#include "esp_err.h"


esp_err_t sht40_init(void);

int8_t sht40_read_temperature(void);