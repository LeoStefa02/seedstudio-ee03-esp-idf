#include "sht40.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define PIN_I2C_SDA  42
#define PIN_I2C_SCL  41
#define SHT40_I2C_ADDR   0x44
#define SHT40_CMD_MEAS   0xFD

static const char *TAG = "SHT40";

static i2c_master_bus_handle_t  s_i2c_bus  = NULL;
static i2c_master_dev_handle_t  s_sht40    = NULL;

esp_err_t sht40_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port             = I2C_NUM_0,
        .sda_io_num           = PIN_I2C_SDA,   // GPIO42
        .scl_io_num           = PIN_I2C_SCL,   // GPIO41
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,  // board has 4.7K but no harm
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SHT40_I2C_ADDR,
        .scl_speed_hz    = 400000,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_sht40);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SHT40 device add failed: %s", esp_err_to_name(err));
        return err;
    }

    // Probe to confirm it is alive
    err = i2c_master_probe(s_i2c_bus, SHT40_I2C_ADDR, 50);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SHT40 not found at 0x44");
        return err;
    }

    ESP_LOGI(TAG, "SHT40 found at 0x44");
    return ESP_OK;
}

int8_t sht40_read_temperature(void)
{
    uint8_t   data[6] = {0};

    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t cmd = SHT40_CMD_MEAS;   // 0xFD
    ESP_ERROR_CHECK(i2c_master_transmit(s_sht40, &cmd, 1, 200));

    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_ERROR_CHECK(i2c_master_receive(s_sht40, data, sizeof(data), 200));

    // Convert raw bytes → Celsius (integer only, no float)
    uint16_t raw_t  = ((uint16_t)data[0] << 8) | data[1];
    int32_t  t_x100 = -4500 + ((int32_t)raw_t * 17500) / 65535;

    uint16_t raw_h   = ((uint16_t)data[3] << 8) | data[4];
    int32_t  rh_x100 = ((int32_t)raw_h * 12500 / 65535) - 600;
    if (rh_x100 < 0)     rh_x100 = 0;
    if (rh_x100 > 10000) rh_x100 = 10000;

    return (int8_t)(t_x100 / 100);
}