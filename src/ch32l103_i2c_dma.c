/**
 * @file ch32l103_i2c_dma.c
 * @brief Non-blocking I2C2 transaction queue and DMA event handling.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "ch32l103_i2c_dma.h"

/**
 * @brief Start the next queued transaction when the peripheral is idle.
 * @param dma Pointer to the I2C DMA context.
 */
static void i2c_dma_start_next(ch32_i2c_dma_t *dma);
/**
 * @brief Recover the I2C peripheral after a bus or timeout error.
 * @param dma Pointer to the I2C DMA context.
 */
static void i2c_dma_recover(ch32_i2c_dma_t *dma);

void CH32_I2C_DMA_Init(ch32_i2c_dma_t *dma, I2C_TypeDef *i2c,
                       DMA_Channel_TypeDef *dma_tx, uint32_t dma_tx_flags,
                       DMA_Channel_TypeDef *dma_rx, uint32_t dma_rx_flags)
{
    uint8_t i;

    if (dma == 0) {
        return;
    }

    dma->i2c = i2c;
    dma->dma_tx = dma_tx;
    dma->dma_rx = dma_rx;
    dma->dma_tx_flags = dma_tx_flags;
    dma->dma_rx_flags = dma_rx_flags;
    dma->head = 0;
    dma->tail = 0;
    dma->state = CH32_I2C_STATE_IDLE;
    dma->busy = 0;
    dma->error = 0;
    dma->recover_pending = 0;
    dma->completed_count = 0U;
    dma->recover_count = 0U;
    dma->queue_full_count = 0U;
    dma->timeout_ms = 0;

    for (i = 0U; i < CH32_I2C_DMA_QUEUE_LEN; i++) {
        dma->queue[i].tx_data = dma->queue[i].frame;
        dma->queue[i].rx_data = 0;
    }

}

/**
 * @brief Copy a transaction into the circular queue.
 * @param dma Pointer to the I2C DMA context.
 * @param node Transaction descriptor to enqueue.
 * @return uint8_t 1 if the transaction was queued, otherwise 0.
 */
static uint8_t i2c_dma_push(ch32_i2c_dma_t *dma, const ch32_i2c_node_t *node)
{
    uint8_t next;
    ch32_i2c_node_t *dst;

    if (dma == 0 || node == 0) {
        return 0U;
    }

    next = (uint8_t)((dma->head + 1U) % CH32_I2C_DMA_QUEUE_LEN);
    if (next == dma->tail) {
        dma->queue_full_count++;
        return 0U;
    }

    dst = &dma->queue[dma->head];
    *dst = *node;
    dst->tx_data = dst->frame;
    dma->head = next;
    return 1U;
}

uint8_t CH32_I2C_DMA_Write(ch32_i2c_dma_t *dma, uint16_t dev_addr,
                           const uint8_t *data, uint16_t size)
{
    ch32_i2c_node_t node;
    uint16_t i;

    if (dma == 0 || data == 0 || size == 0U || size > CH32_I2C_DMA_MAX_FRAME_SIZE) {
        return 0U;
    }

    node.op = CH32_I2C_OP_WRITE;
    node.dev_addr = dev_addr;
    node.size = size;
    node.reg = 0U;
    node.rx_data = 0;
    for (i = 0U; i < size; i++) {
        node.frame[i] = data[i];
    }
    node.tx_data = node.frame;

    __disable_irq();
    if (i2c_dma_push(dma, &node) == 0U) {
        __enable_irq();
        return 0U;
    }

    i2c_dma_start_next(dma);
    __enable_irq();
    return 1U;
}

uint8_t CH32_I2C_DMA_Read(ch32_i2c_dma_t *dma, uint16_t dev_addr,
                          uint8_t reg, uint8_t *rx_buf, uint16_t size)
{
    ch32_i2c_node_t node;

    if (dma == 0 || rx_buf == 0 || size == 0U || size > CH32_I2C_DMA_MAX_FRAME_SIZE) {
        return 0U;
    }

    node.op = CH32_I2C_OP_READ_REG;
    node.dev_addr = dev_addr;
    node.size = size;
    node.reg = reg;
    node.rx_data = rx_buf;
    node.tx_data = node.frame;

    __disable_irq();
    if (i2c_dma_push(dma, &node) == 0U) {
        __enable_irq();
        return 0U;
    }

    i2c_dma_start_next(dma);
    __enable_irq();
    return 1U;
}

uint8_t CH32_I2C_DMA_IsIdle(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return 1U;
    }

    return (dma->busy == 0U && dma->head == dma->tail) ? 1U : 0U;
}

uint8_t CH32_I2C_DMA_HadError(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return 0U;
    }

    return dma->error;
}

void CH32_I2C_DMA_ClearError(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return;
    }

    __disable_irq();
    dma->error = 0U;
    __enable_irq();
}

/**
 * @brief Clear the I2C ADDR flag by reading STAR1 and STAR2.
 * @param i2c Pointer to the I2C peripheral.
 */
static void i2c_dma_clear_addr(I2C_TypeDef *i2c)
{
    (void)i2c->STAR1;
    (void)i2c->STAR2;
}

/**
 * @brief Configure the TX DMA channel for the active transaction.
 * @param dma Pointer to the I2C DMA context.
 * @param mem_addr Source buffer address in memory.
 * @param size Number of bytes to transfer.
 */
static void i2c_dma_configure_dma_tx(ch32_i2c_dma_t *dma, uint32_t mem_addr, uint16_t size)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    DMA_Cmd(dma->dma_tx, DISABLE);
    DMA_DeInit(dma->dma_tx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dma->i2c->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = mem_addr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(dma->dma_tx, &DMA_InitStructure);
    DMA_ClearITPendingBit(dma->dma_tx_flags);
    DMA_ITConfig(dma->dma_tx, DMA_IT_TC, ENABLE);
}

/**
 * @brief Configure the RX DMA channel for the active transaction.
 * @param dma Pointer to the I2C DMA context.
 * @param mem_addr Destination buffer address in memory.
 * @param size Number of bytes to transfer.
 */
static void i2c_dma_configure_dma_rx(ch32_i2c_dma_t *dma, uint32_t mem_addr, uint16_t size)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    DMA_Cmd(dma->dma_rx, DISABLE);
    DMA_DeInit(dma->dma_rx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dma->i2c->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = mem_addr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(dma->dma_rx, &DMA_InitStructure);
    DMA_ClearITPendingBit(dma->dma_rx_flags);
    DMA_ITConfig(dma->dma_rx, DMA_IT_TC, ENABLE);
}

/**
 * @brief Arm I2C interrupts and generate START for the queue head.
 * @param dma Pointer to the I2C DMA context.
 */
static void i2c_dma_start_next(ch32_i2c_dma_t *dma)
{
    if (dma == 0 || dma->busy != 0U || dma->tail == dma->head) {
        return;
    }

    dma->busy = 1U;
    dma->state = CH32_I2C_STATE_START;
    dma->timeout_ms = 0U;

    I2C_AcknowledgeConfig(dma->i2c, ENABLE);
    I2C_NACKPositionConfig(dma->i2c, I2C_NACKPosition_Current);
    I2C_DMALastTransferCmd(dma->i2c, DISABLE);
    I2C_DMACmd(dma->i2c, DISABLE);
    I2C_ITConfig(dma->i2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
    I2C_GenerateSTART(dma->i2c, ENABLE);
}

/**
 * @brief Stop the active transfer, advance the queue, and start the next one.
 * @param dma Pointer to the I2C DMA context.
 */
static void i2c_dma_finish_current(ch32_i2c_dma_t *dma)
{
    I2C_ITConfig(dma->i2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    I2C_DMACmd(dma->i2c, DISABLE);
    I2C_DMALastTransferCmd(dma->i2c, DISABLE);
    I2C_AcknowledgeConfig(dma->i2c, ENABLE);
    I2C_NACKPositionConfig(dma->i2c, I2C_NACKPosition_Current);
    dma->busy = 0U;
    dma->state = CH32_I2C_STATE_IDLE;
    dma->timeout_ms = 0U;
    dma->recover_pending = 0U;
    dma->tail = (uint8_t)((dma->tail + 1U) % CH32_I2C_DMA_QUEUE_LEN);
    dma->completed_count++;
    i2c_dma_start_next(dma);
}

/**
 * @brief Reset I2C and DMA state after a failed transaction.
 * @param dma Pointer to the I2C DMA context.
 */
static void i2c_dma_recover(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return;
    }

    I2C_ITConfig(dma->i2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    DMA_Cmd(dma->dma_tx, DISABLE);
    DMA_Cmd(dma->dma_rx, DISABLE);
    DMA_ITConfig(dma->dma_tx, DMA_IT_TC, DISABLE);
    DMA_ITConfig(dma->dma_rx, DMA_IT_TC, DISABLE);
    I2C_DMACmd(dma->i2c, DISABLE);
    I2C_DMALastTransferCmd(dma->i2c, DISABLE);
    I2C_GenerateSTOP(dma->i2c, ENABLE);
    I2C_SoftwareResetCmd(dma->i2c, ENABLE);
    I2C_SoftwareResetCmd(dma->i2c, DISABLE);
    I2C_Cmd(dma->i2c, ENABLE);
    I2C_AcknowledgeConfig(dma->i2c, ENABLE);
    I2C_NACKPositionConfig(dma->i2c, I2C_NACKPosition_Current);

    dma->head = 0U;
    dma->tail = 0U;
    dma->busy = 0U;
    dma->state = CH32_I2C_STATE_IDLE;
    dma->timeout_ms = 0U;
    dma->recover_pending = 0U;
    dma->error = 1U;
    dma->recover_count++;
}

void CH32_I2C_DMA_EvtHandler(ch32_i2c_dma_t *dma)
{
    ch32_i2c_node_t *node;
    uint16_t star1;

    if (dma == 0 || dma->busy == 0U) {
        return;
    }

    node = &dma->queue[dma->tail];
    star1 = I2C_ReadRegister(dma->i2c, I2C_Register_STAR1);

    switch (dma->state) {
    case CH32_I2C_STATE_START:
        if ((star1 & I2C_STAR1_SB) != 0U) {
            I2C_Send7bitAddress(dma->i2c, (uint8_t)node->dev_addr, I2C_Direction_Transmitter);
            dma->state = CH32_I2C_STATE_ADDR;
        }
        break;

    case CH32_I2C_STATE_ADDR:
        if ((star1 & I2C_STAR1_ADDR) != 0U) {
            i2c_dma_clear_addr(dma->i2c);

            if (node->op == CH32_I2C_OP_WRITE) {
                i2c_dma_configure_dma_tx(dma, (uint32_t)node->tx_data, node->size);
                I2C_DMACmd(dma->i2c, ENABLE);
                DMA_Cmd(dma->dma_tx, ENABLE);
                I2C_ITConfig(dma->i2c, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
                dma->state = CH32_I2C_STATE_WAIT_BTF;
            } else {
                I2C_SendData(dma->i2c, node->reg);
                dma->state = CH32_I2C_STATE_TX_DATA;
            }
        }
        break;

    case CH32_I2C_STATE_TX_DATA:
        if ((star1 & I2C_STAR1_BTF) != 0U) {
            I2C_GenerateSTART(dma->i2c, ENABLE);
            dma->state = CH32_I2C_STATE_RX_DATA;
        }
        break;

    case CH32_I2C_STATE_RX_DATA:
        if ((star1 & I2C_STAR1_SB) != 0U) {
            I2C_Send7bitAddress(dma->i2c, (uint8_t)node->dev_addr, I2C_Direction_Receiver);
        } else if ((star1 & I2C_STAR1_ADDR) != 0U) {
            i2c_dma_configure_dma_rx(dma, (uint32_t)node->rx_data, node->size);
            I2C_DMALastTransferCmd(dma->i2c, ENABLE);
            I2C_DMACmd(dma->i2c, ENABLE);
            DMA_Cmd(dma->dma_rx, ENABLE);
            i2c_dma_clear_addr(dma->i2c);
            I2C_ITConfig(dma->i2c, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
            dma->state = CH32_I2C_STATE_STOP;
        }
        break;

    case CH32_I2C_STATE_WAIT_BTF:
        if ((star1 & I2C_STAR1_BTF) != 0U) {
            I2C_GenerateSTOP(dma->i2c, ENABLE);
            i2c_dma_finish_current(dma);
        }
        break;

    default:
        break;
    }
}

void CH32_I2C_DMA_ErrHandler(ch32_i2c_dma_t *dma)
{
    uint16_t star1;

    if (dma == 0) {
        return;
    }

    star1 = I2C_ReadRegister(dma->i2c, I2C_Register_STAR1);
    I2C_ClearITPendingBit(dma->i2c,
                          I2C_IT_BERR | I2C_IT_ARLO | I2C_IT_AF |
                          I2C_IT_OVR | I2C_IT_TIMEOUT | I2C_IT_PECERR);
    if ((star1 & (I2C_STAR1_BERR | I2C_STAR1_ARLO | I2C_STAR1_AF |
                  I2C_STAR1_OVR | I2C_STAR1_TIMEOUT | I2C_STAR1_PECERR)) != 0U) {
        dma->error = 1U;
    }
    dma->recover_pending = 1U;
    I2C_ITConfig(dma->i2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
}

void CH32_I2C_DMA_TxHandler(ch32_i2c_dma_t *dma)
{
    if (dma == 0 || DMA_GetITStatus(dma->dma_tx_flags) == RESET) {
        return;
    }

    DMA_ClearITPendingBit(dma->dma_tx_flags);
    DMA_ITConfig(dma->dma_tx, DMA_IT_TC, DISABLE);
    DMA_Cmd(dma->dma_tx, DISABLE);
    I2C_DMACmd(dma->i2c, DISABLE);

    if (dma->busy == 0U) {
        return;
    }

    dma->state = CH32_I2C_STATE_WAIT_BTF;
    I2C_ITConfig(dma->i2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
}

void CH32_I2C_DMA_RxHandler(ch32_i2c_dma_t *dma)
{
    if (dma == 0 || DMA_GetITStatus(dma->dma_rx_flags) == RESET) {
        return;
    }

    DMA_ClearITPendingBit(dma->dma_rx_flags);
    DMA_ITConfig(dma->dma_rx, DMA_IT_TC, DISABLE);
    DMA_Cmd(dma->dma_rx, DISABLE);
    I2C_DMACmd(dma->i2c, DISABLE);

    if (dma->busy == 0U) {
        return;
    }

    I2C_GenerateSTOP(dma->i2c, ENABLE);
    i2c_dma_finish_current(dma);
}

void CH32_I2C_DMA_Task(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return;
    }

    if (dma->recover_pending != 0U) {
        i2c_dma_recover(dma);
        return;
    }

    if (dma->state == CH32_I2C_STATE_WAIT_BTF) {
        CH32_I2C_DMA_EvtHandler(dma);
    }
}

void CH32_I2C_DMA_TickMs(ch32_i2c_dma_t *dma, uint8_t elapsed_ms)
{
    uint16_t timeout_ms;

    if ((dma == 0) || (dma->busy == 0U)) {
        return;
    }

    timeout_ms = (uint16_t)(dma->timeout_ms + elapsed_ms);
    dma->timeout_ms = timeout_ms;
    if (timeout_ms >= CH32_I2C_DMA_TIMEOUT_MS) {
        dma->error = 1U;
        dma->recover_pending = 1U;
    }
}
