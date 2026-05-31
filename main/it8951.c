#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "it8951.h"

#define VCOM 1300 // mV
#define SPI_BOUNCE_BUF_SIZE 4096

static const char *TAG = "IT8951";

static spi_device_handle_t s_epd_spi = NULL;
static uint32_t s_img_buf_addr = 0;
static uint8_t *s_bounce_buf = NULL;
static IT8951DevInfo dev_info;

void IT8951_WaitBusy(void)
{
    esp_rom_delay_us(100); // Short delay to avoid busy-waiting too tightly
    while (gpio_get_level(PIN_EPD_BUSY) == 0)
    {
        vTaskDelay(1);
    }
}

void IT8951_WriteWord(uint16_t word)
{
    spi_transaction_t t = {
        .length = 16, // bits
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    // Big-endian: send high byte first
    t.tx_data[0] = (word >> 8) & 0xFF;
    t.tx_data[1] = (word >> 0) & 0xFF;
    t.flags = SPI_TRANS_USE_TXDATA;
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_epd_spi, &t));
}

uint16_t IT8951_ReadWord(void)
{
    spi_transaction_t t = {
        .length = 16,
        .rxlength = 16,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    t.tx_data[0] = 0x00;
    t.tx_data[1] = 0x00;
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_epd_spi, &t));
    return ((uint16_t)t.rx_data[0] << 8) | t.rx_data[1];
}

void IT8951_WriteCmd(uint16_t cmd)
{
    IT8951_WaitBusy();
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 0));
    IT8951_WriteWord(IT8951_PREAMBLE_CMD); // 0x6000
    IT8951_WaitBusy();
    IT8951_WriteWord(cmd);
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));
}

void IT8951_WriteData(uint16_t data)
{
    IT8951_WaitBusy();
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 0));
    IT8951_WriteWord(IT8951_PREAMBLE_DATA); // 0x0000
    IT8951_WaitBusy();
    IT8951_WriteWord(data);
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));
}

void IT8951_WriteDataBuf(const uint16_t *data, size_t len)
{
    IT8951_WaitBusy();
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 0));
    IT8951_WriteWord(IT8951_PREAMBLE_DATA); // 0x0000
    IT8951_WaitBusy();

    for (size_t i = 0; i < len; i++)
    {
        IT8951_WriteWord(data[i]);
    }

    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));
}

uint16_t IT8951_ReadData(void)
{
    uint16_t data;

    IT8951_WaitBusy();
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 0));
    IT8951_WriteWord(IT8951_PREAMBLE_READ); // 0x1000
    IT8951_WaitBusy();
    IT8951_ReadWord();
    IT8951_WaitBusy();
    data = IT8951_ReadWord();
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));
    return data;
}

void IT8951_ReadDataBuf(uint16_t *data, size_t len)
{
    IT8951_WaitBusy();
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 0));
    IT8951_WriteWord(IT8951_PREAMBLE_READ); // 0x1000
    IT8951_WaitBusy();
    IT8951_ReadWord();
    IT8951_WaitBusy();

    for (size_t i = 0; i < len; i++)
    {
        data[i] = IT8951_ReadWord();
    }

    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));
}

void IT8951_SendCmdArg(uint16_t cmd, const uint16_t *args, size_t arg_len)
{
    IT8951_WriteCmd(cmd);
    for (size_t i = 0; i < arg_len; i++)
    {
        IT8951_WriteData(args[i]);
    }
}

void IT8951_SystemRun(void)
{
    IT8951_WriteCmd(IT8951_TCON_SYS_RUN);
}

void IT8951_Standby(void)
{
    IT8951_WriteCmd(IT8951_TCON_STANDBY);
}

void IT8951_Sleep(void)
{
    IT8951_WriteCmd(IT8951_TCON_SLEEP);
}

uint16_t IT8951_ReadRegister(uint16_t reg_addr)
{
    IT8951_WriteCmd(IT8951_TCON_REG_RD);
    IT8951_WriteData(reg_addr);
    return IT8951_ReadData();
}

void IT8951_WriteRegister(uint16_t reg_addr, uint16_t value)
{
    IT8951_WriteCmd(IT8951_TCON_REG_WR);
    IT8951_WriteData(reg_addr);
    IT8951_WriteData(value);
}

uint16_t IT8951_GetVCOM(void)
{
    IT8951_WriteCmd(USDEF_I80_CMD_VCOM);
    IT8951_WriteData(0x0000); // Dummy data to trigger read
    return IT8951_ReadData();
}

void IT8951_SetVCOM(uint16_t vcom_mv)
{
    IT8951_WriteCmd(USDEF_I80_CMD_VCOM);
    IT8951_WriteData(0x0001); // Subcommand for setting VCOM
    IT8951_WriteData(vcom_mv);
}

void IT8951_LoadImgStart(IT8951LdImgInfo *pLdImgInfo)
{
    uint16_t arg = (pLdImgInfo->usEndianType << 8) | (pLdImgInfo->usPixelFormat << 4) | (pLdImgInfo->usRotate);
    IT8951_WriteCmd(IT8951_TCON_LD_IMG_AREA);
    IT8951_WriteData(arg);
}

void IT8951_LoadImgAreaStart(IT8951LdImgInfo *pLdImgInfo, IT8951AreaImgInfo *pAreaImgInfo)
{
    uint16_t arg[5] = {
        (pLdImgInfo->usEndianType << 8) | (pLdImgInfo->usPixelFormat << 4) | (pLdImgInfo->usRotate),
        (pAreaImgInfo->usX),
        (pAreaImgInfo->usY),
        (pAreaImgInfo->usWidth),
        (pAreaImgInfo->usHeight),
    };

    IT8951_SendCmdArg(IT8951_TCON_LD_IMG_AREA, arg, sizeof(arg) / sizeof(arg[0]));
}

void IT8951_LoadImgEnd(void)
{
    IT8951_WriteCmd(IT8951_TCON_LD_IMG_END);
}

void IT8951_GetDeviceInfo(IT8951DevInfo *dev_info)
{
    uint16_t *dev_info_words = (uint16_t *)dev_info;

    IT8951_WriteCmd(USDEF_I80_CMD_GET_DEV_INFO);
    IT8951_ReadDataBuf(dev_info_words, sizeof(IT8951DevInfo) / 2);

    dev_info = (IT8951DevInfo *)dev_info_words;
    ESP_LOGI(TAG, "Panel: %d x %d", dev_info->usPanelW, dev_info->usPanelH);
    ESP_LOGI(TAG, "Image Buffer Address: %X", dev_info->usImgBufAddrL | (dev_info->usImgBufAddrH << 16));
    ESP_LOGI(TAG, "FW Version: %s", (char *)dev_info->usFWVersion);
    ESP_LOGI(TAG, "LUT Version: %s", (char *)dev_info->usLUTVersion);
}

void IT8951_SetImgBufBaseAddr(uint32_t addr)
{
    uint16_t wordH = (uint16_t)((addr >> 16) & 0x0000FFFF);
    uint16_t wordL = (uint16_t)(addr & 0x0000FFFF);

    IT8951_WriteRegister(LISAR + 2, wordH);
    IT8951_WriteRegister(LISAR, wordL);
}

void IT8951_WaitForDisplayReady(void)
{
    while (IT8951_ReadRegister(LUTAFSR))
        ;
}

void IT8951_SetTemperature(int8_t temp_celsius)
{
    IT8951_WriteCmd(IT8951_TCON_SET_TEMP);
    IT8951_WriteData(0x01);
    IT8951_WriteData((uint16_t)temp_celsius);
}

void IT8951_SPIInit(void)
{
    // ----------------------------------------------------------
    // Initialize SPI bus
    // ----------------------------------------------------------
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_SPI_MOSI, // GPIO9
        .miso_io_num = PIN_SPI_MISO, // GPIO8
        .sclk_io_num = PIN_SPI_SCK,  // GPIO7
        .quadwp_io_num = -1,         // Not used
        .quadhd_io_num = -1,         // Not used
        .max_transfer_sz = 4096,     // Bytes per transaction
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI bus initialized");

    // ----------------------------------------------------------
    // Add IT8951 as SPI device
    // Mode 0 (CPOL=0, CPHA=0), CS managed manually via GPIO44
    // ----------------------------------------------------------
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz — conservative start
        .mode = 0,                          // SPI Mode 0
        .spics_io_num = -1,                 // CS managed manually
        .queue_size = 1,
        .flags = 0,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_epd_spi));
    ESP_LOGI(TAG, "IT8951 SPI device added at 10MHz");
    vTaskDelay(pdMS_TO_TICKS(100));


}

void IT8951_GPIOInit(void)
{
    // ----------------------------------------------------------
    // Configure all EPD GPIO pins
    // ----------------------------------------------------------

    // BUSY pin — input
    gpio_config_t busy_cfg = {
        .pin_bit_mask = (1ULL << PIN_EPD_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&busy_cfg));

    // RST pin — output, idle HIGH
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << PIN_EPD_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_RST, 1));

    // CS pin — output, idle HIGH
    gpio_config_t cs_cfg = {
        .pin_bit_mask = (1ULL << PIN_EPD_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cs_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));

    ESP_LOGI(TAG, "EPD GPIO pins configured");
    ESP_LOGI(TAG, "BUSY pin state before RST: %d (expect: don't care)",
             gpio_get_level(PIN_EPD_BUSY));

    gpio_config_t pwr_en_cfg = {
        .pin_bit_mask = (1ULL << PIN_PWR_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&pwr_en_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_PWR_EN, 1));

    // Wait for rails to stabilize
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "PWR_EN HIGH — rails should be stable");
    ESP_LOGI(TAG, "BUSY pin state after PWR_EN: %d", gpio_get_level(PIN_EPD_BUSY));
}

uint8_t IT8951_Init(void)
{
    // Allocate SPI bounce buffer in INTERNAL SRAM
    s_bounce_buf = (uint8_t *)heap_caps_malloc(
        SPI_BOUNCE_BUF_SIZE,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (s_bounce_buf == NULL)
    {
        ESP_LOGE(TAG, "FATAL: Failed to allocate SPI bounce buffer!");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "SPI bounce buffer allocated at: %p (internal SRAM)",
             (void *)s_bounce_buf);

    IT8951_SPIInit();
    IT8951_GPIOInit();

    ESP_LOGI(TAG, "E-Paper Driver Initializing");
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_RST, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_RST, 1));
    IT8951_GetDeviceInfo(&dev_info);


    s_img_buf_addr = dev_info.usImgBufAddrL | (dev_info.usImgBufAddrH << 16);
    IT8951_WriteRegister(I80CPCR, 0x0001);

    if (VCOM != IT8951_GetVCOM())
    {
        ESP_LOGI(TAG, "Setting VCOM to %d mV", VCOM);
        IT8951_SetVCOM(VCOM);
    }

    return 0;
}


void IT8951_HostAreaPackedPixelWrite(IT8951LdImgInfo *pLdImgInfo, IT8951AreaImgInfo *pAreaImgInfo)
{

    uint8_t *pFrameBuf = (uint8_t *)pLdImgInfo->ulStartFBAddr;
    size_t total_bytes = (pAreaImgInfo->usWidth * pAreaImgInfo->usHeight) / 2; // 4bpp = 2 pixels per byte

    // Set image buffer base address
    IT8951_SetImgBufBaseAddr(pLdImgInfo->ulImgBufBaseAddr);
    IT8951_LoadImgAreaStart(pLdImgInfo, pAreaImgInfo);

    IT8951_WaitBusy();
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 0));
    IT8951_WriteWord(IT8951_PREAMBLE_DATA);
    IT8951_WaitBusy();

    // ==========================================================
    // Phase 2: Stream Data via DMA Chunks
    // ==========================================================
    size_t sent_bytes = 0;
    while (sent_bytes < total_bytes)
    {
        size_t chunk = (total_bytes - sent_bytes > SPI_BOUNCE_BUF_SIZE)
                           ? SPI_BOUNCE_BUF_SIZE
                           : (total_bytes - sent_bytes);

        // Copy from PSRAM (pFrameBuf) to Internal SRAM (s_bounce_buf)
        memcpy(s_bounce_buf, pFrameBuf + sent_bytes, chunk);

        spi_transaction_t t = {
            .length = chunk * 8, // Length in bits
            .tx_buffer = s_bounce_buf,
            .rx_buffer = NULL,
        };
        // MUST use interrupt-driven transmit here so FreeRTOS Watchdog doesn't bite!
        ESP_ERROR_CHECK(spi_device_transmit(s_epd_spi, &t));

        sent_bytes += chunk;
    }

    // ==========================================================
    // Phase 3: Close the SPI Gate ONCE
    // ==========================================================
    gpio_set_level(PIN_EPD_CS, 1);

    IT8951_LoadImgEnd();
}

void IT8951_DisplayArea(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t dpyMode, uint8_t temp)
{
    IT8951_SetTemperature(temp);

    IT8951_WriteCmd(USDEF_I80_CMD_DPY_AREA);
    IT8951_WriteData(x);
    IT8951_WriteData(y);
    IT8951_WriteData(width);
    IT8951_WriteData(height);
    IT8951_WriteData(dpyMode);
}

const IT8951DevInfo *IT8951_GetDevInfo(void)
{
    return &dev_info;
}

uint32_t IT8951_GetImgBufAddr(void)
{
    return s_img_buf_addr;
}