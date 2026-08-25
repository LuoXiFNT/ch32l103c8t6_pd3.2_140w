/**
 * @file ch32l103_i2c_dma.h
 * @brief CH32L103 I2C DMA queue types, requests, and interrupt interfaces.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef __CH32L103_I2C_DMA_H
#define __CH32L103_I2C_DMA_H

#include "debug.h"

/** Maximum number of queued I2C transactions. */
#define CH32_I2C_DMA_QUEUE_LEN       16U
/** Maximum payload size of one queued transaction. */
#define CH32_I2C_DMA_MAX_FRAME_SIZE  9U
/** Transaction timeout in milliseconds. */
#define CH32_I2C_DMA_TIMEOUT_MS      100U

typedef enum {
    CH32_I2C_OP_WRITE = 0U,
    CH32_I2C_OP_READ_REG
} ch32_i2c_op_t;

typedef enum {
    CH32_I2C_STATE_IDLE = 0U,
    CH32_I2C_STATE_START,
    CH32_I2C_STATE_ADDR,
    CH32_I2C_STATE_TX_DATA,
    CH32_I2C_STATE_RX_DATA,
    CH32_I2C_STATE_WAIT_BTF,
    CH32_I2C_STATE_STOP,
    CH32_I2C_STATE_ERROR
} ch32_i2c_state_t;

/** @brief One queued I2C transaction. */
typedef struct {
    uint8_t *tx_data; /**< Transmit buffer, normally frame. */
    uint8_t *rx_data; /**< Receive destination for read operations. */
    uint16_t dev_addr; /**< 8-bit I2C address (7-bit address shifted left). */
    uint16_t size; /**< Number of bytes to transfer. */
    uint8_t op; /**< ch32_i2c_op_t operation code. */
    uint8_t reg; /**< Register address for a read operation. */
    uint8_t frame[CH32_I2C_DMA_MAX_FRAME_SIZE]; /**< Inline transmit storage. */
} ch32_i2c_node_t;

/** @brief I2C peripheral, DMA channels, queue, and transaction state. */
typedef struct {
    I2C_TypeDef *i2c; /**< I2C peripheral instance. */
    DMA_Channel_TypeDef *dma_tx; /**< TX DMA channel. */
    DMA_Channel_TypeDef *dma_rx; /**< RX DMA channel. */
    uint32_t dma_tx_flags; /**< TX DMA interrupt flag. */
    uint32_t dma_rx_flags; /**< RX DMA interrupt flag. */

    ch32_i2c_node_t queue[CH32_I2C_DMA_QUEUE_LEN];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t state;
    volatile uint8_t busy;
    volatile uint8_t error;
    volatile uint8_t recover_pending;
    volatile uint16_t timeout_ms;
    volatile uint16_t completed_count;
    volatile uint16_t recover_count;
    volatile uint16_t queue_full_count;
} ch32_i2c_dma_t;

/** @brief Initialize an I2C DMA queue and bind its peripheral resources. */
void CH32_I2C_DMA_Init(ch32_i2c_dma_t *dma, I2C_TypeDef *i2c,
                        DMA_Channel_TypeDef *dma_tx, uint32_t dma_tx_flags,
                        DMA_Channel_TypeDef *dma_rx, uint32_t dma_rx_flags);

/** @brief Queue a write transaction. @return 1 when queued, 0 on failure. */
uint8_t CH32_I2C_DMA_Write(ch32_i2c_dma_t *dma, uint16_t dev_addr,
                           const uint8_t *data, uint16_t size);
/** @brief Queue a register-read transaction. @return 1 when queued, 0 on failure. */
uint8_t CH32_I2C_DMA_Read(ch32_i2c_dma_t *dma, uint16_t dev_addr,
                          uint8_t reg, uint8_t *rx_buf, uint16_t size);
/** @brief Return non-zero when the queue and peripheral are idle. */
uint8_t CH32_I2C_DMA_IsIdle(ch32_i2c_dma_t *dma);
/** @brief Return non-zero when a bus or DMA error is latched. */
uint8_t CH32_I2C_DMA_HadError(ch32_i2c_dma_t *dma);
/** @brief Clear the latched error flag. */
void    CH32_I2C_DMA_ClearError(ch32_i2c_dma_t *dma);
/** @brief Advance the transaction state machine. */
void    CH32_I2C_DMA_Task(ch32_i2c_dma_t *dma);
/** @brief Advance transaction timeout accounting. */
void    CH32_I2C_DMA_TickMs(ch32_i2c_dma_t *dma, uint8_t elapsed_ms);

/** @brief Handle I2C event interrupts. */
void CH32_I2C_DMA_EvtHandler(ch32_i2c_dma_t *dma);
/** @brief Handle I2C error interrupts. */
void CH32_I2C_DMA_ErrHandler(ch32_i2c_dma_t *dma);
/** @brief Handle TX DMA completion interrupts. */
void CH32_I2C_DMA_TxHandler(ch32_i2c_dma_t *dma);
/** @brief Handle RX DMA completion interrupts. */
void CH32_I2C_DMA_RxHandler(ch32_i2c_dma_t *dma);

#endif
