#pragma once
#include "esp_err.h"
#include <stdint.h>

// Discover EPaperViewer on the local network via mDNS.
// Blocks until found or timeout expires.
// ip_out must be at least 16 bytes.
esp_err_t mdns_discover_server(char    *ip_out,
                                size_t   ip_len,
                                uint16_t *port_out);