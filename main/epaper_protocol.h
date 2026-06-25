#ifndef EPAPER_PROTOCOL_H
#define EPAPER_PROTOCOL_H
#include <stdint.h>

// ── Grid ──────────────────────────────────────────────────────
#define SCREEN_W       1872
#define SCREEN_H       1404

// ── Display modes ─────────────────────────────────────────────
#define MODE_INIT        0
#define MODE_DU          1
#define MODE_GC16        2

// ── Buttons ───────────────────────────────────────────────────
#define BTN_UP           0    // GPIO2
#define BTN_DOWN         1    // GPIO3
#define BTN_EXIT         2    // GPIO5

#endif // EPAPER_PROTOCOL_H