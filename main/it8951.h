#pragma once

#include <stdint.h>
#include <stddef.h>
#include "driver/spi_master.h"

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


#define VCOM 1300
#define SPI_BOUNCE_BUF_SIZE 4096


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


// =============================================================
// Public API
// =============================================================

// Initialize SPI bus, GPIO pins, and IT8951
uint8_t IT8951_Init(void);

// Load pixels from framebuffer into IT8951 memory
void IT8951_HostAreaPackedPixelWrite(
    IT8951LdImgInfo     *pLdImgInfo,
    IT8951AreaImgInfo   *pAreaImgInfo);

// Trigger display refresh
void IT8951_DisplayArea(
    uint16_t x, uint16_t y,
    uint16_t width, uint16_t height,
    uint16_t dpyMode, uint8_t temp);

// Block until display engine is idle
void IT8951_WaitForDisplayReady(void);

// Block until BUSY pin goes HIGH
void IT8951_WaitBusy(void);

// Access to device info populated during Init
const IT8951DevInfo *IT8951_GetDevInfo(void);

// Access to image buffer address populated during Init
uint32_t IT8951_GetImgBufAddr(void);