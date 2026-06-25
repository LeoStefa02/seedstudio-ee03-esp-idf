#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wifi.h"
#include "tcp_client.h"
#include "button.h"
#include "it8951.h"

#define WIFI_SSID "Dalmazio Birazio"
#define WIFI_PASSWORD "123pollo"

static const char *TAG = "MAIN";

static uint8_t *s_framebuffer = NULL;

void app_main(void)
{
    ESP_ERROR_CHECK(wifi_connect(WIFI_SSID, WIFI_PASSWORD));
    ESP_ERROR_CHECK(IT8951_Init());


    const IT8951DevInfo dev_info = *IT8951_GetDevInfo();

    // Check if malloc size is correct. On the library they do not divide
    s_framebuffer = (uint8_t *)heap_caps_malloc(dev_info.usPanelW * dev_info.usPanelH / 2, MALLOC_CAP_SPIRAM);
    if (s_framebuffer == NULL)
    {
        ESP_LOGE(TAG, "FATAL: Failed to allocate %d bytes in PSRAM!", dev_info.usPanelW * dev_info.usPanelH / 2);
        ESP_LOGE(TAG, "Check menuconfig: Component config > ESP PSRAM > Support for external RAM");
        // Halt — nothing else can proceed without the framebuffer
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Framebuffer allocated at : %p", (void *)s_framebuffer);

    ESP_ERROR_CHECK(button_init());
    ESP_ERROR_CHECK(tcp_client_start(s_framebuffer, dev_info.usPanelW * dev_info.usPanelH / 2));
}
