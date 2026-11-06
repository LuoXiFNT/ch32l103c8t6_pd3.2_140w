/**
 * @file ch32_spi_dma.h
 * @brief SPI1 DMA transfer state and public driver interface.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef CH32_SPI_DMA_H
#define CH32_SPI_DMA_H

#include <stdbool.h>
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief SPI DMA transfer phases.
 *
 * A transfer walks IDLE -> TRANSFER -> DRAIN -> IDLE. DRAIN exists because the DMA
 * completion interrupt fires as soon as the last byte has been handed to the shift
 * register, while the bus is still clocking it out: the caller may only release CS
 * and start the next command once BSY has cleared, which CH32_SPI_DMA_Task() waits
 * for.
 */
typedef enum {
    CH32_SPI_DMA_STATE_IDLE = 0u, /**< No transfer is active. */
    CH32_SPI_DMA_STATE_TRANSFER,  /**< DMA is feeding SPI. */
    CH32_SPI_DMA_STATE_DRAIN      /**< DMA ended; SPI shift register is draining. */
} ch32_spi_dma_state_t;

/** @brief Send one byte synchronously over SPI1; used during panel init only. */
void CH32_SPI_DMA_WriteByte(uint8_t data);
/** @brief Start a non-blocking SPI1 DMA write; returns 0 when the bus is busy. */
bool CH32_SPI_DMA_Write(const uint8_t *data, uint16_t len);
/** @brief Return non-zero when SPI1 is ready for a new transfer. */
bool CH32_SPI_DMA_IsIdle(void);
/** @brief Finish the post-DMA SPI drain phase when BSY clears. */
void CH32_SPI_DMA_Task(void);
/** @brief Handle SPI1 TX DMA completion. */
void CH32_SPI_DMA_TxHandler(void);


#ifdef __cplusplus
}
#endif
#endif /* CH32_SPI_DMA_H */
