/**
 * @file ch32_i2c_dma.h
 * @brief CH32L103 I2C DMA queue types, requests, and interrupt interfaces.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef CH32_I2C_DMA_H
#define CH32_I2C_DMA_H

#include <stdbool.h>
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of queued I2C transactions. */
#define CH32_I2C_DMA_QUEUE_LEN       16u
/** Maximum payload size of one queued transaction. */
#define CH32_I2C_DMA_MAX_FRAME_SIZE  9u
/** Transaction timeout in milliseconds. */
#define CH32_I2C_DMA_TIMEOUT_MS      100u

typedef enum {
    CH32_I2C_OP_WRITE = 0u,
    CH32_I2C_OP_READ_REG
} ch32_i2c_op_t;

typedef enum {
    CH32_I2C_STATE_IDLE = 0u, /**< No transaction in flight. */
    CH32_I2C_STATE_START,     /**< START generated, waiting for the start bit. */
    CH32_I2C_STATE_ADDR,      /**< Address byte sent, waiting for acknowledge. */
    CH32_I2C_STATE_TX_DATA,   /**< Sending the register address of a read. */
    CH32_I2C_STATE_RX_DATA,   /**< Repeated START and RX DMA setup. */
    CH32_I2C_STATE_WAIT_BTF,  /**< Waiting for the last byte to shift out. */
    CH32_I2C_STATE_STOP       /**< STOP generated. */
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
    volatile uint8_t head;           /**< Queue write index; advanced by clients. */
    volatile uint8_t tail;           /**< Queue read index; advanced on completion. */
    volatile uint8_t state;          /**< ch32_i2c_state_t of the active transaction. */
    volatile uint8_t busy;           /**< 1 while a transaction owns the peripheral. */
    volatile uint8_t error;          /**< Latched bus error; cleared by the client. */
    volatile uint8_t recover_pending;/**< Set on error, consumed by the recovery. */
    volatile uint16_t timeout_ms;    /**< Owned by the tick; reset when a transfer starts. */
    volatile uint16_t completed_count; /**< Successful transactions since boot. */
    volatile uint16_t recover_count;   /**< Peripheral resets since boot. */
    volatile uint16_t queue_full_count;/**< Requests rejected because the queue was full. */
} ch32_i2c_dma_t;

/** @brief Initialize an I2C DMA queue and bind its peripheral resources. */
void CH32_I2C_DMA_Init(ch32_i2c_dma_t *dma, I2C_TypeDef *i2c,
                       DMA_Channel_TypeDef *dma_tx, uint32_t dma_tx_flags,
                       DMA_Channel_TypeDef *dma_rx, uint32_t dma_rx_flags);

/** @brief Queue a write transaction; returns true when queued. */
bool CH32_I2C_DMA_Write(ch32_i2c_dma_t *dma, uint16_t dev_addr,
                        const uint8_t *data, uint16_t size);
/** @brief Queue a register-read transaction; returns true when queued. */
bool CH32_I2C_DMA_Read(ch32_i2c_dma_t *dma, uint16_t dev_addr,
                       uint8_t reg, uint8_t *rx_buf, uint16_t size);
/** @brief Return non-zero when the queue and peripheral are idle. */
bool CH32_I2C_DMA_IsIdle(ch32_i2c_dma_t *dma);
/** @brief Return non-zero when a bus or DMA error is latched. */
bool CH32_I2C_DMA_HadError(ch32_i2c_dma_t *dma);
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


#ifdef __cplusplus
}
#endif
#endif /* CH32_I2C_DMA_H */
