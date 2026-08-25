/**
 * @file ch32l103_spi_dma.h
 * @brief SPI1 DMA transfer state and public driver interface.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef CH32L103_SPI_DMA_H
#define CH32L103_SPI_DMA_H

#include "debug.h"

/** @brief SPI DMA transfer phases. */
typedef enum {
    CH32_SPI_DMA_STATE_IDLE = 0U, /**< No transfer is active. */
    CH32_SPI_DMA_STATE_TRANSFER,  /**< DMA is feeding SPI. */
    CH32_SPI_DMA_STATE_DRAIN      /**< DMA ended; SPI shift register is draining. */
} ch32_spi_dma_state_t;

/** @brief Send one byte synchronously over SPI1. */
void CH32_SPI_DMA_WriteByte(uint8_t data);
/** @brief Start a non-blocking SPI1 DMA write. */
uint8_t CH32_SPI_DMA_Write(const uint8_t *data, uint16_t len);
/** @brief Return non-zero when SPI1 is ready for a new transfer. */
uint8_t CH32_SPI_DMA_IsIdle(void);
/** @brief Finish the post-DMA SPI drain phase when BSY clears. */
void CH32_SPI_DMA_Task(void);
/** @brief Handle SPI1 TX DMA completion. */
void CH32_SPI_DMA_TxHandler(void);

#endif
