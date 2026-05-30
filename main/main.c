#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Built in I80 Command Code
#define IT8951_TCON_SYS_RUN 0x0001
#define IT8951_TCON_STANDBY 0x0002
#define IT8951_TCON_SLEEP 0x0003
#define IT8951_TCON_REG_RD 0x0010
#define IT8951_TCON_REG_WR 0x0011
#define IT8951_TCON_MEM_BST_RD_T 0x0012
#define IT8951_TCON_MEM_BST_RD_S 0x0013
#define IT8951_TCON_MEM_BST_WR 0x0014
#define IT8951_TCON_MEM_BST_END 0x0015
#define IT8951_TCON_LD_IMG 0x0020
#define IT8951_TCON_LD_IMG_AREA 0x0021
#define IT8951_TCON_LD_IMG_END 0x0022
#define IT8951_TCON_SET_TEMP 0x0040

// --- IT8951 SPI Preambles ---
#define IT8951_PREAMBLE_CMD 0x6000
#define IT8951_PREAMBLE_DATA 0x0000
#define IT8951_PREAMBLE_READ 0x1000

// I80 User defined command code
#define USDEF_I80_CMD_DPY_AREA 0x0034
#define USDEF_I80_CMD_GET_DEV_INFO 0x0302
#define USDEF_I80_CMD_DPY_BUF_AREA 0x0037
#define USDEF_I80_CMD_VCOM 0x0039

//-----------------------------------------------------------------------
// IT8951 TCon Registers defines
//-----------------------------------------------------------------------
// Register Base Address
#define DISPLAY_REG_BASE 0x1000 // Register RW access for I80 only
// Base Address of Basic LUT Registers
#define LUT0EWHR (DISPLAY_REG_BASE + 0x00)  // LUT0 Engine Width Height Reg
#define LUT0XYR (DISPLAY_REG_BASE + 0x40)   // LUT0 XY Reg
#define LUT0BADDR (DISPLAY_REG_BASE + 0x80) // LUT0 Base Address Reg
#define LUT0MFN (DISPLAY_REG_BASE + 0xC0)   // LUT0 Mode and Frame number Reg
#define LUT01AF (DISPLAY_REG_BASE + 0x114)  // LUT0 and LUT1 Active Flag Reg
// Update Parameter Setting Register
#define UP0SR (DISPLAY_REG_BASE + 0x134) // Update Parameter0 Setting Reg

#define UP1SR (DISPLAY_REG_BASE + 0x138)     // Update Parameter1 Setting Reg
#define LUT0ABFRV (DISPLAY_REG_BASE + 0x13C) // LUT0 Alpha blend and Fill rectangle Value
#define UPBBADDR (DISPLAY_REG_BASE + 0x17C)  // Update Buffer Base Address
#define LUT0IMXY (DISPLAY_REG_BASE + 0x180)  // LUT0 Image buffer X/Y offset Reg
#define LUTAFSR (DISPLAY_REG_BASE + 0x224)   // LUT Status Reg (status of All LUT Engines)

#define BGVR (DISPLAY_REG_BASE + 0x250) // Bitmap (1bpp) image color table

//-------System Registers----------------
#define SYS_REG_BASE 0x0000

// Address of System Registers
#define I80CPCR (SYS_REG_BASE + 0x04)

//-------Memory Converter Registers----------------
#define MCSR_BASE_ADDR 0x0200
#define MCSR (MCSR_BASE_ADDR + 0x0000)
#define LISAR (MCSR_BASE_ADDR + 0x0008)

// EE03 Board
#define PIN_SPI_SCK 7
#define PIN_SPI_MOSI 9
#define PIN_SPI_MISO 8
#define PIN_EPD_CS 44
#define PIN_EPD_BUSY 4 // LOW = busy, HIGH = ready
#define PIN_EPD_RST 38 // Active LOW
#define PIN_PWR_EN 43

#define I2C_MASTER_SCL_IO           6
#define I2C_MASTER_SDA_IO           5
#define I2C_MASTER_NUM              0 
#define I2C_MASTER_FREQ_HZ          100000
#define SHT31_SENSOR_ADDR           0x44

#define VCOM 1300
#define SPI_BOUNCE_BUF_SIZE 4096

static const char *TAG = "EPAPER";

typedef struct
{
    uint16_t usPanelW;
    uint16_t usPanelH;
    uint16_t usImgBufAddrL;
    uint16_t usImgBufAddrH;
    uint16_t usFWVersion[8];  // 16 Bytes String
    uint16_t usLUTVersion[8]; // 16 Bytes String
} IT8951DevInfo;

typedef struct IT8951LdImgInfo
{
    uint16_t usEndianType;     // little or Big Endian
    uint16_t usPixelFormat;    // bpp
    uint16_t usRotate;         // Rotate mode
    uint32_t ulStartFBAddr;    // Start address of source Frame buffer
    uint32_t ulImgBufBaseAddr; // Base address of target image buffer

} IT8951LdImgInfo;

// structure prototype 2
typedef struct IT8951AreaImgInfo
{
    uint16_t usX;
    uint16_t usY;
    uint16_t usWidth;
    uint16_t usHeight;
} IT8951AreaImgInfo;

static spi_device_handle_t s_epd_spi = NULL;
static uint8_t *s_framebuffer = NULL;
static uint32_t s_img_buf_addr = 0;
static uint8_t *s_bounce_buf = NULL;
static IT8951DevInfo dev_info;

// Global handle used to communicate specifically with the SHT31
static i2c_master_dev_handle_t s_sht31_handle;

static esp_err_t i2c_master_init(void)
{
    // 1. Configure the I2C Master Bus
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    // 2. Configure the specific SHT31 Device on that bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT31_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Add the device and save the handle to our global variable
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_sht31_handle);
}

static int8_t read_sht31_temperature(void)
{
    uint8_t cmd[2] = {0x24, 0x00}; // Command: Measure, High Repeatability
    uint8_t data[6];

    // 1. Transmit the measurement command to the sensor
    esp_err_t err = i2c_master_transmit(s_sht31_handle, cmd, sizeof(cmd), -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command to SHT31");
        return 25; // Safe fallback
    }

    // 2. Wait for the sensor to physically measure the air (takes ~15ms)
    vTaskDelay(pdMS_TO_TICKS(20));

    // 3. Receive the 6 bytes of measurement data
    err = i2c_master_receive(s_sht31_handle, data, sizeof(data), -1);
    if (err == ESP_OK) {
        // Convert the raw 16-bit data to Celsius
        uint16_t raw_temp = (data[0] << 8) | data[1];
        float celsius = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
        
        ESP_LOGI(TAG, "SHT31 Real Temperature: %.1f C", celsius);
        return (int8_t)celsius;
    } else {
        ESP_LOGE(TAG, "Failed to read data from SHT31");
        return 25; // Safe fallback
    }
}

static void IT8951_WaitBusy(void)
{
    esp_rom_delay_us(100); // Short delay to avoid busy-waiting too tightly
    while (gpio_get_level(PIN_EPD_BUSY) == 0)
    {
        vTaskDelay(1);
    }
}

static void IT8951_WriteWord(uint16_t word)
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

static uint16_t IT8951_ReadWord(void)
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

static void IT8951_WriteCmd(uint16_t cmd)
{
    IT8951_WaitBusy();
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 0));
    IT8951_WriteWord(IT8951_PREAMBLE_CMD); // 0x6000
    IT8951_WaitBusy();
    IT8951_WriteWord(cmd);
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));
}

static void IT8951_WriteData(uint16_t data)
{
    IT8951_WaitBusy();
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 0));
    IT8951_WriteWord(IT8951_PREAMBLE_DATA); // 0x0000
    IT8951_WaitBusy();
    IT8951_WriteWord(data);
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));
}

static void IT8951_WriteDataBuf(const uint16_t *data, size_t len)
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

static uint16_t IT8951_ReadData(void)
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

static void IT8951_ReadDataBuf(uint16_t *data, size_t len)
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

static void IT8951_SendCmdArg(uint16_t cmd, const uint16_t *args, size_t arg_len)
{
    IT8951_WriteCmd(cmd);
    for (size_t i = 0; i < arg_len; i++)
    {
        IT8951_WriteData(args[i]);
    }
}

static void IT8951_SystemRun(void)
{
    IT8951_WriteCmd(IT8951_TCON_SYS_RUN);
}

static void IT8951_Standby(void)
{
    IT8951_WriteCmd(IT8951_TCON_STANDBY);
}

static void IT8951_Sleep(void)
{
    IT8951_WriteCmd(IT8951_TCON_SLEEP);
}

static uint16_t IT8951_ReadRegister(uint16_t reg_addr)
{
    IT8951_WriteCmd(IT8951_TCON_REG_RD);
    IT8951_WriteData(reg_addr);
    return IT8951_ReadData();
}

static void IT8951_WriteRegister(uint16_t reg_addr, uint16_t value)
{
    IT8951_WriteCmd(IT8951_TCON_REG_WR);
    IT8951_WriteData(reg_addr);
    IT8951_WriteData(value);
}

static uint16_t IT8951_GetVCOM(void)
{
    IT8951_WriteCmd(USDEF_I80_CMD_VCOM);
    IT8951_WriteData(0x0000); // Dummy data to trigger read
    return IT8951_ReadData();
}

static void IT8951_SetVCOM(uint16_t vcom_mv)
{
    IT8951_WriteCmd(USDEF_I80_CMD_VCOM);
    IT8951_WriteData(0x0001); // Subcommand for setting VCOM
    IT8951_WriteData(vcom_mv);
}

static void IT8951_LoadImgStart(IT8951LdImgInfo *pLdImgInfo)
{
    uint16_t arg = (pLdImgInfo->usEndianType << 8) | (pLdImgInfo->usPixelFormat << 4) | (pLdImgInfo->usRotate);
    IT8951_WriteCmd(IT8951_TCON_LD_IMG_AREA);
    IT8951_WriteData(arg);
}

static void IT8951_LoadImgAreaStart(IT8951LdImgInfo *pLdImgInfo, IT8951AreaImgInfo *pAreaImgInfo)
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

static void IT8951_LoadImgEnd(void)
{
    IT8951_WriteCmd(IT8951_TCON_LD_IMG_END);
}

static void IT8951_GetDeviceInfo(IT8951DevInfo *dev_info)
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

static void IT8951_SetImgBufBaseAddr(uint32_t addr)
{
    uint16_t wordH = (uint16_t)((addr >> 16) & 0x0000FFFF);
    uint16_t wordL = (uint16_t)(addr & 0x0000FFFF);

    IT8951_WriteRegister(LISAR + 2, wordH);
    IT8951_WriteRegister(LISAR, wordL);
}

static void IT8951_WaitForDisplayReady(void)
{
    while (IT8951_ReadRegister(LUTAFSR))
        ;
}

static void IT8951_SetTemperature(int8_t temp_celsius)
{
    IT8951_WriteCmd(IT8951_TCON_SET_TEMP);
    IT8951_WriteData(0x01);
    IT8951_WriteData((uint16_t)temp_celsius);
}

static uint8_t IT8951_Init(void)
{
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_CS, 1));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_RST, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(PIN_EPD_RST, 1));
    IT8951_GetDeviceInfo(&dev_info);

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

    s_img_buf_addr = dev_info.usImgBufAddrL | (dev_info.usImgBufAddrH << 16);
    IT8951_WriteRegister(I80CPCR, 0x0001);

    if (VCOM != IT8951_GetVCOM())
    {
        ESP_LOGI(TAG, "Setting VCOM to %d mV", VCOM);
        IT8951_SetVCOM(VCOM);
    }

    IT8951_SetTemperature(25);

    return 0;
}

static void IT8951_HostAreaPackedPixelWrite(IT8951LdImgInfo *pLdImgInfo, IT8951AreaImgInfo *pAreaImgInfo)
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

static void IT8951_DisplayArea(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t dpyMode)
{
    IT8951_WriteCmd(USDEF_I80_CMD_DPY_AREA);
    IT8951_WriteData(x);
    IT8951_WriteData(y);
    IT8951_WriteData(width);
    IT8951_WriteData(height);
    IT8951_WriteData(dpyMode);
}

void app_main(void)
{

    i2c_master_init();
    int8_t real_temp = read_sht31_temperature();

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

    ESP_LOGI(TAG, "E-Paper Driver Initializing");

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

    if (IT8951_Init())
    {
        ESP_LOGE(TAG, "IT8951 initialization failed!");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    IT8951LdImgInfo ld_img_info = {
        .usEndianType = 0,  // Big Endian
        .usPixelFormat = 2, // 4bpp (16 gray levels)
        .usRotate = 0,      // No rotation
        .ulStartFBAddr = (uint32_t)s_framebuffer,
        .ulImgBufBaseAddr = s_img_buf_addr,
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
    IT8951_DisplayArea(0, 0, dev_info.usPanelW, dev_info.usPanelH, 1);

    vTaskDelay(pdMS_TO_TICKS(2000));

    memset(s_framebuffer, 0xFF, dev_info.usPanelW * dev_info.usPanelH / 2); // Black
    IT8951_WaitForDisplayReady();

    IT8951_HostAreaPackedPixelWrite(&ld_img_info, &area_img_info);
    IT8951_DisplayArea(0, 0, dev_info.usPanelW, dev_info.usPanelH, 1);

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
    IT8951_DisplayArea(0, 0, 800, 600, 2);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
