#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sht40.h"
#include "it8951.h"

static const char *TAG = "MAIN";

static uint8_t *s_framebuffer = NULL;

void app_main(void)
{
    sht40_init();

    if (IT8951_Init())
    {
        ESP_LOGE(TAG, "IT8951 initialization failed!");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

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

    IT8951LdImgInfo ld_img_info = {
        .usEndianType = 0,  // Big Endian
        .usPixelFormat = 2, // 4bpp (16 gray levels)
        .usRotate = 0,      // No rotation
        .ulStartFBAddr = (uint32_t)s_framebuffer,
        .ulImgBufBaseAddr = IT8951_GetImgBufAddr(),
    };

    IT8951AreaImgInfo area_img_info = {
        .usX = 0,
        .usY = 0,
        .usWidth = dev_info.usPanelW,
        .usHeight = dev_info.usPanelH,
    };


    memset(s_framebuffer, 0x00, dev_info.usPanelW * dev_info.usPanelH / 2); // Black
    IT8951_WaitForDisplayReady();

    IT8951_HostAreaPackedPixelWrite(&ld_img_info, &area_img_info);
    IT8951_DisplayArea(0, 0, dev_info.usPanelW, dev_info.usPanelH, 1, 25);

    vTaskDelay(pdMS_TO_TICKS(2000));

    memset(s_framebuffer, 0xFF, dev_info.usPanelW * dev_info.usPanelH / 2); // Black
    IT8951_WaitForDisplayReady();

    IT8951_HostAreaPackedPixelWrite(&ld_img_info, &area_img_info);
    IT8951_DisplayArea(0, 0, dev_info.usPanelW, dev_info.usPanelH, 1, 25);

    vTaskDelay(pdMS_TO_TICKS(2000));

    IT8951AreaImgInfo partial_area = {
        .usX      = 0,
        .usY      = 0,
        .usWidth  = 800, // 800 % 8 == 0 (Perfect 32-bit alignment!)
        .usHeight = 600,
    };

    size_t partial_size = (800 * 600) / 2;
    memset(s_framebuffer, 0x88, partial_size);
    IT8951_WaitForDisplayReady();

    IT8951_HostAreaPackedPixelWrite(&ld_img_info, &partial_area);
    IT8951_DisplayArea(0, 0, 800, 600, 2, sht40_read_temperature());

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
