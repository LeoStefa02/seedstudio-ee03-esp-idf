#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"

// =============================================================
// EE03 Board — Authoritative Pin Definitions
// Source: Seeed Seeed_GFX library hardware extraction
// =============================================================

// --- Buttons (for reference, unused in this project) ---
#define PIN_BUTTON1         2    // Active LOW, internal pull-up
#define PIN_BUTTON2         3    // Active LOW, internal pull-up
#define PIN_BUTTON3         5    // Active LOW, internal pull-up

// --- Battery (for reference, unused in this project) ---
#define PIN_ADC_EN          6
#define PIN_BAT_ADC         1

// --- IT8951 SPI Preambles ---
#define IT8951_PREAMBLE_CMD     0x6000
#define IT8951_PREAMBLE_DATA    0x0000
#define IT8951_PREAMBLE_READ    0x1000

// --- IT8951 Commands ---
#define IT8951_CMD_SYS_RUN      0x0001
#define IT8951_CMD_GET_DEV_INFO 0x0302
#define IT8951_CMD_REG_WR       0x0011
#define IT8951_CMD_LD_IMG_AREA  0x0021
#define IT8951_CMD_LD_IMG_END   0x0022
#define IT8951_CMD_LD_IMG       0x0020
#define IT8951_CMD_VCOM         0x0039
#define IT8951_CMD_REG_RD       0x0010
#define IT8951_CMD_DPY_AREA     0x0034

// --- IT8951 Registers ---
#define IT8951_REG_I80CPCR  0x0004   // I80 Control Protocol Config Register
#define IT8951_REG_LISAR_H  0x0008
#define IT8951_REG_LISAR_L  0x000A

// =============================================================
// EE03 Board
// =============================================================
#define PIN_SPI_SCK         7
#define PIN_SPI_MOSI        9
#define PIN_SPI_MISO        8
#define PIN_EPD_CS          44
#define PIN_EPD_BUSY        4    // LOW = busy, HIGH = ready
#define PIN_EPD_RST         38   // Active LOW
#define PIN_PWR_EN          43

// =============================================================
// Display Geometry
// =============================================================
#define EPD_WIDTH           1404
#define EPD_HEIGHT          1872
#define EPD_FB_SIZE         (EPD_WIDTH * EPD_HEIGHT / 2)  // 1,314,144 bytes

// Bounce buffer size for SPI DMA transfers — must be <= internal SRAM size
#define SPI_BOUNCE_BUF_SIZE 4096

static const char *TAG = "EPAPER";

// Internal SRAM bounce buffer for SPI DMA transfers
// PSRAM cannot be read directly by the SPI DMA engine
static uint8_t *s_bounce_buf = NULL;

static spi_device_handle_t s_epd_spi = NULL;
static uint32_t s_img_buf_addr = 0;

// Global pointer — will hold the address of our PSRAM framebuffer
static uint8_t *s_framebuffer = NULL;

// Wait for IT8951 BUSY/HRDY pin to go HIGH (ready)
// BUSY LOW = chip is busy, BUSY HIGH = chip is ready
static void it8951_wait_busy(void)
{
    while (gpio_get_level(PIN_EPD_BUSY) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ----------------------------------------------------------
// Primitive 1: Transfer a single 16-bit word over SPI
// Handles the raw byte-swap (IT8951 is big-endian)
// ----------------------------------------------------------
static void spi_write_word(uint16_t word)
{
    spi_transaction_t t = {
        .length    = 16,               // bits
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    // Big-endian: send high byte first
    t.tx_data[0] = (word >> 8) & 0xFF;
    t.tx_data[1] = (word >> 0) & 0xFF;
    t.flags = SPI_TRANS_USE_TXDATA;
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_epd_spi, &t));
}

// ----------------------------------------------------------
// Primitive 2: Read a single 16-bit word over SPI
// ----------------------------------------------------------
static uint16_t spi_read_word(void)
{
    spi_transaction_t t = {
        .length    = 16,
        .rxlength  = 16,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
        .flags     = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    t.tx_data[0] = 0x00;
    t.tx_data[1] = 0x00;
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_epd_spi, &t));
    return ((uint16_t)t.rx_data[0] << 8) | t.rx_data[1];
}

// ----------------------------------------------------------
// Primitive 3: Send a command word to the IT8951
// Full sequence: CS_LOW → preamble → BUSY → cmd → CS_HIGH
// ----------------------------------------------------------
static void it8951_write_cmd(uint16_t cmd)
{
    it8951_wait_busy();
    gpio_set_level(PIN_EPD_CS, 0);
    spi_write_word(IT8951_PREAMBLE_CMD);    // 0x6000
    it8951_wait_busy();
    spi_write_word(cmd);
    gpio_set_level(PIN_EPD_CS, 1);
}

// ----------------------------------------------------------
// Primitive 5: Read a data word from the IT8951
// Full sequence: CS_LOW → preamble → BUSY → dummy → BUSY → read → CS_HIGH
// ----------------------------------------------------------
static uint16_t it8951_read_data(void)
{
    gpio_set_level(PIN_EPD_CS, 0);
    spi_write_word(IT8951_PREAMBLE_READ);   // 0x1000
    it8951_wait_busy();
    spi_read_word();                        // dummy read — required by protocol
    it8951_wait_busy();
    uint16_t val = spi_read_word();         // actual data
    gpio_set_level(PIN_EPD_CS, 1);
    return val;
}

// ----------------------------------------------------------
// Primitive 4: Send a data word to the IT8951
// Full sequence: CS_LOW → preamble → BUSY → data → CS_HIGH
// ----------------------------------------------------------
static void it8951_write_data(uint16_t data)
{
    it8951_wait_busy();
    gpio_set_level(PIN_EPD_CS, 0);
    spi_write_word(IT8951_PREAMBLE_DATA);   // 0x0000
    it8951_wait_busy();
    spi_write_word(data);
    gpio_set_level(PIN_EPD_CS, 1);
}


// IT8951 device info — returned by GET_DEV_INFO command
typedef struct {
    uint16_t width;          // Panel width in pixels
    uint16_t height;         // Panel height in pixels
    uint16_t img_buf_addr_l; // Image buffer address low word
    uint16_t img_buf_addr_h; // Image buffer address high word
    uint16_t fw_version[8];  // Firmware version string (16 bytes)
    uint16_t lut_version[8]; // LUT version string (16 bytes)
} it8951_dev_info_t;

static it8951_dev_info_t s_dev_info = {0};

static void it8951_get_dev_info(void)
{
    // Send GET_DEV_INFO command
    it8951_write_cmd(IT8951_CMD_GET_DEV_INFO);

    // Read back 40 bytes = 20 x 16-bit words
    uint16_t *p = (uint16_t *)&s_dev_info;
    gpio_set_level(PIN_EPD_CS, 0);
    spi_write_word(IT8951_PREAMBLE_READ);   // 0x1000
    it8951_wait_busy();
    spi_read_word();                        // dummy read
    it8951_wait_busy();
    for (int i = 0; i < sizeof(it8951_dev_info_t) / 2; i++) {
        p[i] = spi_read_word();
    }
    gpio_set_level(PIN_EPD_CS, 1);
}

// ----------------------------------------------------------
// Write a value to an IT8951 internal register
// Sequence: CMD(REG_WR) → DATA(reg_addr) → DATA(value)
// ----------------------------------------------------------
static void it8951_write_reg(uint16_t reg_addr, uint16_t value)
{
    it8951_write_cmd(IT8951_CMD_REG_WR);
    it8951_write_data(reg_addr);
    it8951_write_data(value);
}

static uint16_t it8951_read_reg(uint16_t reg_addr)
{
    it8951_write_cmd(IT8951_CMD_REG_RD);   // 0x0010
    it8951_write_data(reg_addr);
    return it8951_read_data();
}

// ----------------------------------------------------------
// Wake the IT8951 and set the image buffer base address
// Must be called once after every hardware reset
// ----------------------------------------------------------

static void it8951_init(void)
{
    // Wake the controller
    it8951_write_cmd(IT8951_CMD_SYS_RUN);
    vTaskDelay(pdMS_TO_TICKS(10));

    it8951_write_cmd(IT8951_CMD_VCOM);
    it8951_write_data(0x0001); // -2.0V, default is usually around -1.7V
    it8951_write_data(2150);

    it8951_write_reg(IT8951_REG_I80CPCR, 0x0001);

    // Set LISAR — try original addresses 0x0008/0x000A
    it8951_write_reg(IT8951_REG_LISAR_H,
                     (uint16_t)(s_img_buf_addr >> 16));
    it8951_write_reg(IT8951_REG_LISAR_L,
                     (uint16_t)(s_img_buf_addr & 0xFFFF));

    // Read back LISAR with BOTH known address conventions
    // Convention A: 0x0008/0x000A
    uint16_t a_h = it8951_read_reg(0x0008);
    uint16_t a_l = it8951_read_reg(0x000A);

    // Convention B: 0x0208/0x020A (MCSR base)
    uint16_t b_h = it8951_read_reg(0x0208);
    uint16_t b_l = it8951_read_reg(0x020A);

    ESP_LOGI(TAG, "LISAR target  : H=0x%04X L=0x%04X (addr=0x%08X)",
             (uint16_t)(s_img_buf_addr >> 16),
             (uint16_t)(s_img_buf_addr & 0xFFFF),
             (unsigned)s_img_buf_addr);
    ESP_LOGI(TAG, "LISAR @ 0x0008/0x000A : H=0x%04X L=0x%04X", a_h, a_l);
    ESP_LOGI(TAG, "LISAR @ 0x0208/0x020A : H=0x%04X L=0x%04X", b_h, b_l);
}

static void generate_test_pattern(void)
{
    // Simple solid mid-gray fill for first confirmation
    // Gray level 8 of 15 = 0x88 packed (two pixels per byte)
    // If this shows as a uniform gray, the pipeline is confirmed
    memset(s_framebuffer, 0xFF, EPD_FB_SIZE);
}

static void it8951_display_image(void)
{
    ESP_LOGI(TAG, "--- Display sequence start ---");

    // Step 1: INIT baseline
    ESP_LOGI(TAG, "Sending INIT refresh...");
    it8951_write_cmd(IT8951_CMD_DPY_AREA);
    it8951_write_data(0);
    it8951_write_data(0);
    it8951_write_data(s_dev_info.width);
    it8951_write_data(s_dev_info.height);
    it8951_write_data(0);                   // mode 0 = INIT
    it8951_wait_busy();
    ESP_LOGI(TAG, "INIT complete");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Step 2: Reload LISAR
    it8951_write_reg(IT8951_REG_LISAR_H,
                     (uint16_t)(s_img_buf_addr >> 16));
    it8951_write_reg(IT8951_REG_LISAR_L,
                     (uint16_t)(s_img_buf_addr & 0xFFFF));
    ESP_LOGI(TAG, "LISAR reloaded");

    // Step 3: LD_IMG (0x0020) — simpler than LD_IMG_AREA, no coords needed
    ESP_LOGI(TAG, "Sending LD_IMG command...");
    it8951_write_cmd(IT8951_CMD_LD_IMG);
    // ARG: endian=0, bpp=2 (4bpp), rotate=0
    it8951_write_data((0 << 8) | (0x2 << 4) | 0x0);
    ESP_LOGI(TAG, "LD_IMG ARG sent, streaming pixels...");

    // Step 4: Stream pixel data — all black, no PSRAM involved
    gpio_set_level(PIN_EPD_CS, 0);
    spi_write_word(IT8951_PREAMBLE_DATA);
    it8951_wait_busy();

    uint32_t remaining = EPD_FB_SIZE;
    uint32_t offset    = 0;
    uint32_t chunks    = 0;

    while (remaining > 0) {
        size_t chunk = (remaining > SPI_BOUNCE_BUF_SIZE)
                     ? SPI_BOUNCE_BUF_SIZE : remaining;

        // Direct fill — bypass PSRAM entirely for this test
        memset(s_bounce_buf, 0x00, chunk);  // 0x00 = black

        spi_transaction_t t = {
            .length    = chunk * 8,
            .tx_buffer = s_bounce_buf,
            .rx_buffer = NULL,
        };
        ESP_ERROR_CHECK(spi_device_polling_transmit(s_epd_spi, &t));

        offset    += chunk;
        remaining -= chunk;
        chunks++;

        // Log progress every 256KB
        if ((offset % (256 * 1024)) == 0) {
            ESP_LOGI(TAG, "  Streamed %lu / %d bytes (%lu chunks so far)",
                     (unsigned long)offset, EPD_FB_SIZE, (unsigned long)chunks);
        }
    }

    gpio_set_level(PIN_EPD_CS, 1);
    ESP_LOGI(TAG, "Pixel stream complete: %lu chunks, %d bytes total",
             (unsigned long)chunks, EPD_FB_SIZE);

    // Step 5: LD_IMG_END
    ESP_LOGI(TAG, "Sending LD_IMG_END...");
    it8951_write_cmd(IT8951_CMD_LD_IMG_END);
    it8951_wait_busy();
    ESP_LOGI(TAG, "LD_IMG_END complete");

    // Step 6: DU refresh — simplest waveform, just B&W threshold
    // More forgiving than GC16 for first confirmation
    ESP_LOGI(TAG, "Sending DPY_AREA DU (mode 1)...");
    it8951_write_cmd(IT8951_CMD_DPY_AREA);
    it8951_write_data(0);
    it8951_write_data(0);
    it8951_write_data(s_dev_info.width);
    it8951_write_data(s_dev_info.height);
    it8951_write_data(1);                   // mode 1 = DU (direct, B&W only)
    it8951_wait_busy();
    ESP_LOGI(TAG, "DU refresh complete — screen should be BLACK");
}


void app_main(void)
{
    // Allocate SPI bounce buffer in INTERNAL SRAM
    s_bounce_buf = (uint8_t *)heap_caps_malloc(
        SPI_BOUNCE_BUF_SIZE,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA
    );
    if (s_bounce_buf == NULL) {
        ESP_LOGE(TAG, "FATAL: Failed to allocate SPI bounce buffer!");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    ESP_LOGI(TAG, "SPI bounce buffer allocated at: %p (internal SRAM)",
             (void *)s_bounce_buf);

    ESP_LOGI(TAG, "=== E-Paper Driver Initializing ===");
    ESP_LOGI(TAG, "Target framebuffer size: %d bytes (%.2f MB)",
             EPD_FB_SIZE,
             (float)EPD_FB_SIZE / (1024.0f * 1024.0f));

    // ----------------------------------------------------------
    // Step 1: Report free memory BEFORE allocation
    // ----------------------------------------------------------
    size_t free_spiram_before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t free_internal_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    ESP_LOGI(TAG, "Free PSRAM before alloc   : %u bytes", (unsigned)free_spiram_before);
    ESP_LOGI(TAG, "Free internal RAM before  : %u bytes", (unsigned)free_internal_before);

    // ----------------------------------------------------------
    // Step 2: Allocate framebuffer in PSRAM
    // ----------------------------------------------------------
    s_framebuffer = (uint8_t *)heap_caps_malloc(EPD_FB_SIZE, MALLOC_CAP_SPIRAM);

    if (s_framebuffer == NULL) {
        ESP_LOGE(TAG, "FATAL: Failed to allocate %d bytes in PSRAM!", EPD_FB_SIZE);
        ESP_LOGE(TAG, "Check menuconfig: Component config > ESP PSRAM > Support for external RAM");
        // Halt — nothing else can proceed without the framebuffer
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Framebuffer allocated at : %p", (void *)s_framebuffer);

    // ----------------------------------------------------------
    // Step 3: Prove the memory is writable (write + verify)
    // ----------------------------------------------------------
    ESP_LOGI(TAG, "Testing PSRAM write/read...");

    memset(s_framebuffer, 0xAB, EPD_FB_SIZE);

    // Spot-check first, middle, and last byte
    bool ok = (s_framebuffer[0]              == 0xAB) &&
              (s_framebuffer[EPD_FB_SIZE / 2] == 0xAB) &&
              (s_framebuffer[EPD_FB_SIZE - 1] == 0xAB);

    if (!ok) {
        ESP_LOGE(TAG, "FATAL: PSRAM read-back verification failed!");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "PSRAM read-back: OK");

    // ----------------------------------------------------------
    // Step 4: Report free memory AFTER allocation
    // ----------------------------------------------------------
    size_t free_spiram_after = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Free PSRAM after alloc    : %u bytes", (unsigned)free_spiram_after);
    ESP_LOGI(TAG, "PSRAM consumed            : %u bytes",
             (unsigned)(free_spiram_before - free_spiram_after));

    // ----------------------------------------------------------
    // Clear the framebuffer to 0x00 (black) ready for later use
    // ----------------------------------------------------------
    memset(s_framebuffer, 0x00, EPD_FB_SIZE);
    ESP_LOGI(TAG, "Framebuffer cleared to 0x00");

    ESP_LOGI(TAG, "=== Phase 2 Complete — PSRAM OK ===");

    // ----------------------------------------------------------
    // Phase 4a: Configure all EPD GPIO pins
    // Do this BEFORE PWR_EN so UART logging still works
    // ----------------------------------------------------------

    // BUSY pin — input
    gpio_config_t busy_cfg = {
        .pin_bit_mask = (1ULL << PIN_EPD_BUSY),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&busy_cfg));

    // RST pin — output, idle HIGH
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << PIN_EPD_RST),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_RST, 1));

    // CS pin — output, idle HIGH
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << PIN_EPD_CS),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cs_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));

    ESP_LOGI(TAG, "EPD GPIO pins configured");
    ESP_LOGI(TAG, "BUSY pin state before RST: %d (expect: don't care)",
             gpio_get_level(PIN_EPD_BUSY));

    // ----------------------------------------------------------
    // Phase 3: PWR_EN — Enable PMIC rails
    // ----------------------------------------------------------
    ESP_LOGI(TAG, "Enabling PWR_EN...");

    gpio_config_t pwr_en_cfg = {
        .pin_bit_mask         = (1ULL << PIN_PWR_EN),
        .mode                 = GPIO_MODE_OUTPUT,
        .pull_up_en           = GPIO_PULLUP_DISABLE,
        .pull_down_en         = GPIO_PULLDOWN_DISABLE,
        .intr_type            = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&pwr_en_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_PWR_EN, 1));

    // Wait for rails to stabilize
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "PWR_EN HIGH — rails should be stable");
    ESP_LOGI(TAG, "BUSY pin state after PWR_EN: %d", gpio_get_level(PIN_EPD_BUSY));

    // ----------------------------------------------------------
    // Phase 4b: Hardware reset the IT8951
    // ----------------------------------------------------------
    ESP_LOGI(TAG, "Pulsing IT8951 RST LOW...");
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_RST, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_RST, 1));
    ESP_LOGI(TAG, "RST pulse complete — waiting for BUSY HIGH...");

    // ----------------------------------------------------------
    // Phase 4c: Wait for BUSY HIGH with full logging
    // ----------------------------------------------------------
    int elapsed = 0;
    while (gpio_get_level(PIN_EPD_BUSY) == 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
        ESP_LOGI(TAG, "  BUSY still LOW... (%d ms elapsed)", elapsed);
        if (elapsed >= 5000) {
            ESP_LOGE(TAG, "TIMEOUT: IT8951 BUSY never went HIGH after %d ms", elapsed);
            ESP_LOGE(TAG, "Possible causes:");
            ESP_LOGE(TAG, "  1. BUSY pin mapping wrong (currently GPIO%d)", PIN_EPD_BUSY);
            ESP_LOGE(TAG, "  2. IT8951 not getting power");
            ESP_LOGE(TAG, "  3. RST pin mapping wrong (currently GPIO%d)", PIN_EPD_RST);
            // Halt — do not restart, we want to read the log
            while (1) { vTaskDelay(pdMS_TO_TICKS(5000)); }
        }
    }

    ESP_LOGI(TAG, "SUCCESS: BUSY HIGH after %d ms — IT8951 is alive!", elapsed);

    // ----------------------------------------------------------
    // Now take GPIO43 — UART goes silent from this point
    // Phase 4d (SPI init) will be added next
    // ----------------------------------------------------------
    ESP_LOGI(TAG, "=== Phase 4 GPIO complete. Proceeding to SPI init. ===");
    vTaskDelay(pdMS_TO_TICKS(100));

    // ----------------------------------------------------------
    // Phase 4d: Initialize SPI bus
    // ----------------------------------------------------------
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = PIN_SPI_MOSI,   // GPIO9
        .miso_io_num     = PIN_SPI_MISO,   // GPIO8
        .sclk_io_num     = PIN_SPI_SCK,    // GPIO7
        .quadwp_io_num   = -1,             // Not used
        .quadhd_io_num   = -1,             // Not used
        .max_transfer_sz = 4096,           // Bytes per transaction
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI bus initialized");

    // ----------------------------------------------------------
    // Phase 4e: Add IT8951 as SPI device
    // Mode 0 (CPOL=0, CPHA=0), CS managed manually via GPIO44
    // ----------------------------------------------------------
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz  = 2 * 1000 * 1000,  // 10 MHz — conservative start
        .mode            = 0,                  // SPI Mode 0
        .spics_io_num    = -1,                 // CS managed manually
        .queue_size      = 1,
        .flags           = 0,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_epd_spi));
    ESP_LOGI(TAG, "IT8951 SPI device added at 10MHz");
    ESP_LOGI(TAG, "=== Phase 4 Complete — SPI ready ===");
    vTaskDelay(pdMS_TO_TICKS(100));

    // ----------------------------------------------------------
    // Phase 5: First SPI transaction — GET_DEV_INFO
    // This reads back screen dimensions and buffer address
    // UART is still alive here — we log everything before silence
    // ----------------------------------------------------------
    ESP_LOGI(TAG, "Sending GET_DEV_INFO to IT8951...");

    it8951_get_dev_info();
    s_img_buf_addr = ((uint32_t)s_dev_info.img_buf_addr_h << 16)
               |  (uint32_t)s_dev_info.img_buf_addr_l;
    ESP_LOGI(TAG, "  ImgBuf addr stored: 0x%08X", (unsigned)s_img_buf_addr);

    ESP_LOGI(TAG, "IT8951 Device Info:");
    ESP_LOGI(TAG, "  Panel width  : %d px", s_dev_info.width);
    ESP_LOGI(TAG, "  Panel height : %d px", s_dev_info.height);
    ESP_LOGI(TAG, "  ImgBuf addr  : 0x%04X%04X",
             s_dev_info.img_buf_addr_h,
             s_dev_info.img_buf_addr_l);

    if ((s_dev_info.width  == EPD_WIDTH  && s_dev_info.height == EPD_HEIGHT) ||
      (s_dev_info.width  == EPD_HEIGHT && s_dev_info.height == EPD_WIDTH)) {
        ESP_LOGI(TAG, "  Geometry OK: %dx%d (portrait/landscape match)",
                s_dev_info.width, s_dev_info.height);
    } else {
        ESP_LOGW(TAG, "  WARNING: Unexpected geometry %dx%d",
                 s_dev_info.width, s_dev_info.height);
    }

    ESP_LOGI(TAG, "=== Phase 5a Complete ===");
    vTaskDelay(pdMS_TO_TICKS(100));

    // ----------------------------------------------------------
    // Phase 5b: IT8951 software init
    // ----------------------------------------------------------
    ESP_LOGI(TAG, "Running IT8951 init sequence...");
    it8951_init();
    ESP_LOGI(TAG, "SYS_RUN sent");
    ESP_LOGI(TAG, "VCOM set to -2.0V");
    ESP_LOGI(TAG, "LISAR set to 0x%08X", (unsigned)s_img_buf_addr);
    ESP_LOGI(TAG, "=== Phase 5b Complete — IT8951 ready for image load ===");
    vTaskDelay(pdMS_TO_TICKS(100));

    // ----------------------------------------------------------
    // Phase 6: Generate test pattern and push to display
    // ----------------------------------------------------------
    // Step 1: INIT baseline
    ESP_LOGI(TAG, "Sending INIT refresh...");
    it8951_write_cmd(IT8951_CMD_DPY_AREA);
    it8951_write_data(0);
    it8951_write_data(0);
    it8951_write_data(s_dev_info.width);
    it8951_write_data(s_dev_info.height);
    it8951_write_data(0);                   // mode 0 = INIT
    it8951_wait_busy();
    ESP_LOGI(TAG, "INIT complete");
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "=== Running 256x256 Injection Test ===");

    // 1. Target LISAR back to base memory
    it8951_write_reg(IT8951_REG_LISAR_H, (uint16_t)(s_img_buf_addr >> 16));
    it8951_write_reg(IT8951_REG_LISAR_L, (uint16_t)(s_img_buf_addr & 0xFFFF));

    // 2. Define a small, perfectly aligned 256x256 window
    it8951_write_cmd(IT8951_CMD_LD_IMG_AREA);
    it8951_write_data((0 << 8) | (0x2 << 4) | 0x0); // Endian=0, 4bpp, Rotate=0
    it8951_write_data(0);   // X start
    it8951_write_data(0);   // Y start
    it8951_write_data(256); // Width
    it8951_write_data(256); // Height

    // 3. Start Data Phase
    it8951_wait_busy(); // CRITICAL: Wait before touching CS!
    gpio_set_level(PIN_EPD_CS, 0);
    spi_write_word(IT8951_PREAMBLE_DATA);
    it8951_wait_busy();

    // 4. Stream Solid Black (0x00) using safe DMA bounce buffer
    // 256x256 @ 4bpp = 32,768 bytes.
    // SPI_BOUNCE_BUF_SIZE is 4096. We send 8 chunks.
    memset(s_bounce_buf, 0x00, SPI_BOUNCE_BUF_SIZE);

    for (int i = 0; i < 8; i++) {
        spi_transaction_t t = {
            .length    = SPI_BOUNCE_BUF_SIZE * 8, 
            .tx_buffer = s_bounce_buf,
            .rx_buffer = NULL,
        };
        ESP_ERROR_CHECK(spi_device_transmit(s_epd_spi, &t));
    }

    gpio_set_level(PIN_EPD_CS, 1);
    
    // 5. End Image Load
    it8951_write_cmd(IT8951_CMD_LD_IMG_END);
    
    // 6. Reset LISAR again before drawing
    it8951_write_reg(IT8951_REG_LISAR_H, (uint16_t)(s_img_buf_addr >> 16));
    it8951_write_reg(IT8951_REG_LISAR_L, (uint16_t)(s_img_buf_addr & 0xFFFF));

    // 7. Trigger Display (Mode 1 - A2 Black/White)
    it8951_write_cmd(IT8951_CMD_DPY_AREA);
    it8951_write_data(0);
    it8951_write_data(0);
    it8951_write_data(s_dev_info.width);
    it8951_write_data(s_dev_info.height);
    it8951_write_data(1);

    // Refresh complete — idle forever
    while (1) { vTaskDelay(pdMS_TO_TICKS(5000)); }
}
