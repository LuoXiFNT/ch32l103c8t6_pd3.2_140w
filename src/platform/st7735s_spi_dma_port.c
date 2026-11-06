/**
 * @file st7735s_spi_dma_port.c
 * @brief SPI1, DMA and GPIO implementation of the ST7735S host interface.
 *
 * Everything the panel driver needs from the MCU lives here: the four control pins
 * (RES, CS, DC, BLK), the SPI1 byte stream with its TX DMA channel, and the blocking
 * delay used by the power-on sequence. Chip select is framed by this port, so the
 * driver never touches a pin.
 *
 * @author LuoXiFNT
 * @date 2026-11-05
 * @lastModified 2026-11-06
 */

#include "st7735s_spi_dma_port.h"

#include "ch32l103_init.h"
#include "ch32_spi_dma.h"

/**
 * @brief Send one command byte with D/CX low, framed by a chip-select pulse.
 * @param user Unused bus context.
 * @param cmd Command code.
 */
static void ST7735S_PortWriteCmd(void *user, uint8_t cmd)
{
    (void)user;

    ST7735S_CS_LOW();
    ST7735S_DC_LOW();
    CH32_SPI_DMA_WriteByte(cmd);
    ST7735S_DC_HIGH();
    ST7735S_CS_HIGH();
}

/**
 * @brief Send data bytes with D/CX high, blocking; used by the init sequence.
 * @param user Unused bus context.
 * @param data Bytes to send.
 * @param len Number of bytes.
 */
static void ST7735S_PortWriteData(void *user, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    (void)user;

    ST7735S_CS_LOW();
    ST7735S_DC_HIGH();
    for (i = 0u; i < len; i++) {
        CH32_SPI_DMA_WriteByte(data[i]);
    }
    ST7735S_CS_HIGH();
}

/**
 * @brief Start a DMA transfer with D/CX high; used for address and pixel data.
 * @param user Unused bus context.
 * @param data Bytes to send.
 * @param len Number of bytes.
 * @return true when the transfer started.
 */
static bool ST7735S_PortStartPixels(void *user, const uint8_t *data, uint16_t len)
{
    (void)user;

    if (!CH32_SPI_DMA_IsIdle()) {
        return false;
    }

    ST7735S_CS_LOW();
    ST7735S_DC_HIGH();
    if (!CH32_SPI_DMA_Write(data, len)) {
        ST7735S_CS_HIGH();
        return false;
    }

    return true;
}

/**
 * @brief Report whether the SPI bus has finished the current transfer.
 * @param user Unused bus context.
 * @return true when the bus has drained.
 */
static bool ST7735S_PortIsIdle(void *user)
{
    (void)user;
    return CH32_SPI_DMA_IsIdle();
}

/**
 * @brief Advance the SPI transfer and release chip select once it has drained.
 * @param user Unused bus context.
 */
static void ST7735S_PortTask(void *user)
{
    (void)user;

    CH32_SPI_DMA_Task();
    if (CH32_SPI_DMA_IsIdle()) {
        /* The shift register is empty, so the frame is really over. */
        ST7735S_CS_HIGH();
        ST7735S_DC_HIGH();
    }
}

/**
 * @brief Drive the panel reset line.
 * @param user Unused bus context.
 * @param level 1 releases the panel, 0 keeps it in reset.
 */
static void ST7735S_PortSetReset(void *user, uint8_t level)
{
    (void)user;

    if (level != 0u) {
        ST7735S_RES_HIGH();
    } else {
        ST7735S_RES_LOW();
    }
}

/**
 * @brief Drive the backlight switch.
 * @param user Unused bus context.
 * @param level 1 switches the LEDs on, 0 switches them off.
 */
static void ST7735S_PortSetBacklight(void *user, uint8_t level)
{
    (void)user;

    if (level != 0u) {
        ST7735S_BLK_HIGH();
    } else {
        ST7735S_BLK_LOW();
    }
}

/**
 * @brief Blocking millisecond delay for the panel power-on timing.
 * @param user Unused bus context.
 * @param ms Milliseconds to wait.
 */
static void ST7735S_PortDelayMs(void *user, uint16_t ms)
{
    (void)user;
    CH32L103_DelayMs(ms);
}

/** @brief Configure the panel control pins and release the panel from reset. */
static void ST7735S_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = ST7735S_RES_GPIO_PIN;
    GPIO_Init(ST7735S_RES_GPIO_PORT, &gpio);
    ST7735S_RES_HIGH();

    gpio.GPIO_Pin = ST7735S_CS_GPIO_PIN;
    GPIO_Init(ST7735S_CS_GPIO_PORT, &gpio);
    ST7735S_CS_HIGH();

    gpio.GPIO_Pin = ST7735S_BLK_GPIO_PIN;
    GPIO_Init(ST7735S_BLK_GPIO_PORT, &gpio);
    /* Keep the backlight off during initialization; the display service turns it
     * on once the first frame is on the panel. */
    ST7735S_BLK_LOW();

    gpio.GPIO_Pin = ST7735S_DC_GPIO_PIN;
    GPIO_Init(ST7735S_DC_GPIO_PORT, &gpio);
    ST7735S_DC_HIGH();
}

/** @brief Configure the SPI1 peripheral and its transmit DMA channel. */
static void ST7735S_SPI1_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    SPI_InitTypeDef spi = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_SPI1, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    gpio.GPIO_Pin = SPI1_SCK_GPIO_PIN | SPI1_MOSI_GPIO_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(SPI1_GPIO_PORT, &gpio);
    GPIO_SetBits(SPI1_GPIO_PORT, SPI1_SCK_GPIO_PIN | SPI1_MOSI_GPIO_PIN);

    gpio.GPIO_Pin = SPI1_MISO_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(SPI1_GPIO_PORT, &gpio);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7u;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);

    DMA_Cmd(SPI1_DMA_TX_CH, DISABLE);
    DMA_ClearITPendingBit(SPI1_DMA_TX_FLAG);

    /* One line per interrupt (320 bytes at 12 MHz 闂?210 us), so the LCD DMA sits
     * below the tick and the USART link, which both have tighter deadlines. */
    CH32L103_NvicConfig(SPI1_DMA_TX_IRQn,
                        CH32L103_IRQ_PREEMPT_SERVICES,
                        CH32L103_IRQ_SUB_THIRD);
}

/** @brief Host interface handed to the panel driver. */
static const st7735s_bus_t s_bus = {
    ST7735S_PortWriteCmd,
    ST7735S_PortWriteData,
    ST7735S_PortStartPixels,
    ST7735S_PortIsIdle,
    ST7735S_PortTask,
    ST7735S_PortSetReset,
    ST7735S_PortSetBacklight,
    ST7735S_PortDelayMs,
    0
};

const st7735s_bus_t *ST7735S_SPI_DMA_InitDefault(void)
{
    ST7735S_GPIO_Init();
    ST7735S_SPI1_Init();
    return &s_bus;
}
