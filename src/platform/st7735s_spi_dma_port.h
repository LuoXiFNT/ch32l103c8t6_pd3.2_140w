/**
 * @file st7735s_spi_dma_port.h
 * @brief SPI host interface for the ST7735S panel on the CH32L103 board.
 *
 * Platform layer: this file and its .c implement st7735s_bus_t with SPI1, DMA and
 * four GPIO pins, so they are what gets rewritten when the platform changes. The
 * driver in src/drivers/lcd_st7735s stays platform independent.
 *
 * @author LuoXiFNT
 * @date 2026-11-05
 * @lastModified 2026-11-05
 */

#ifndef ST7735S_SPI_DMA_PORT_H
#define ST7735S_SPI_DMA_PORT_H

#include <stdint.h>

#include "st7735s.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure the panel pins and SPI1, and return the host interface.
 *
 * Drives the backlight low first, then sets up RES/CS/DC as outputs plus the SPI1
 * pins and its TX DMA channel. Call it as early as possible in the board init so the
 * panel stays dark, then pass the returned bus to st7735s_init().
 *
 * @return const st7735s_bus_t* Static host interface, never NULL.
 */
const st7735s_bus_t *ST7735S_SPI_DMA_InitDefault(void);


#ifdef __cplusplus
}
#endif
#endif /* ST7735S_SPI_DMA_PORT_H */
