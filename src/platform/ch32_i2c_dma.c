/**
 * @file ch32_i2c_dma.c
 * @brief Non-blocking I2C2 transaction queue and DMA event handling.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#include "ch32_i2c_dma.h"

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
    dma->completed_count = 0u;
    dma->recover_count = 0u;
    dma->queue_full_count = 0u;
    dma->timeout_ms = 0;

    for (i = 0u; i < CH32_I2C_DMA_QUEUE_LEN; i++) {
        dma->queue[i].tx_data = dma->queue[i].frame;
        dma->queue[i].rx_data = 0;
    }

}

/**
 * @brief Copy a transaction into the circular queue.
 * @param dma Pointer to the I2C DMA context.
 * @param node Transaction descriptor to enqueue.
 * @return true when the transaction was queued.
 */
static uint8_t i2c_dma_push(ch32_i2c_dma_t *dma, const ch32_i2c_node_t *node)
{
    uint8_t next;
    ch32_i2c_node_t *dst;

    if (dma == 0 || node == 0) {
        return 0u;
    }

    next = (uint8_t)((dma->head + 1u) % CH32_I2C_DMA_QUEUE_LEN);
    if (next == dma->tail) {
        dma->queue_full_count++;
        return 0u;
    }

    dst = &dma->queue[dma->head];
    *dst = *node;
    dst->tx_data = dst->frame;
    dma->head = next;
    return 1u;
}

bool CH32_I2C_DMA_Write(ch32_i2c_dma_t *dma, uint16_t dev_addr,
                        const uint8_t *data, uint16_t size)
{
    ch32_i2c_node_t node;
    uint16_t i;

    if (dma == 0 || data == 0 || size == 0u || size > CH32_I2C_DMA_MAX_FRAME_SIZE) {
        return false;
    }

    node.op = CH32_I2C_OP_WRITE;
    node.dev_addr = dev_addr;
    node.size = size;
    node.reg = 0u;
    node.rx_data = 0;
    for (i = 0u; i < size; i++) {
        node.frame[i] = data[i];
    }
    node.tx_data = node.frame;

    __disable_irq();
    if (i2c_dma_push(dma, &node) == 0u) {
        __enable_irq();
        return false;
    }

    i2c_dma_start_next(dma);
    __enable_irq();
    return true;
}

bool CH32_I2C_DMA_Read(ch32_i2c_dma_t *dma, uint16_t dev_addr,
                       uint8_t reg, uint8_t *rx_buf, uint16_t size)
{
    ch32_i2c_node_t node;

    if (dma == 0 || rx_buf == 0 || size == 0u || size > CH32_I2C_DMA_MAX_FRAME_SIZE) {
        return false;
    }

    node.op = CH32_I2C_OP_READ_REG;
    node.dev_addr = dev_addr;
    node.size = size;
    node.reg = reg;
    node.rx_data = rx_buf;
    node.tx_data = node.frame;

    __disable_irq();
    if (i2c_dma_push(dma, &node) == 0u) {
        __enable_irq();
        return false;
    }

    i2c_dma_start_next(dma);
    __enable_irq();
    return true;
}

bool CH32_I2C_DMA_IsIdle(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return true;
    }

    return dma->busy == 0u && dma->head == dma->tail;
}

bool CH32_I2C_DMA_HadError(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return false;
    }

    return dma->error;
}

void CH32_I2C_DMA_ClearError(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return;
    }

    __disable_irq();
    dma->error = 0u;
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
 * @brief Configure the DMA channel of the active transaction.
 *
 * The two channels differ only in direction, so one helper programs both.
 *
 * @param dma Pointer to the I2C DMA context.
 * @param mem_addr Buffer address in memory.
 * @param size Number of bytes to transfer.
 * @param tx true for the transmit channel, false for the receive channel.
 */
static void i2c_dma_configure_channel(ch32_i2c_dma_t *dma, uint32_t mem_addr, uint16_t size, bool tx)
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    DMA_Channel_TypeDef *channel = tx ? dma->dma_tx : dma->dma_rx;

    DMA_Cmd(channel, DISABLE);
    DMA_DeInit(channel);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dma->i2c->DATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = mem_addr;
    DMA_InitStructure.DMA_DIR = tx ? DMA_DIR_PeripheralDST : DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = size;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(channel, &DMA_InitStructure);
    DMA_ClearITPendingBit(tx ? dma->dma_tx_flags : dma->dma_rx_flags);
    DMA_ITConfig(channel, DMA_IT_TC, ENABLE);
}

/**
 * @brief Arm I2C interrupts and generate START for the queue head.
 * @param dma Pointer to the I2C DMA context.
 */
static void i2c_dma_start_next(ch32_i2c_dma_t *dma)
{
    if (dma == 0 || dma->busy != 0u || dma->tail == dma->head) {
        return;
    }

    dma->busy = 1u;
    dma->state = CH32_I2C_STATE_START;
    dma->timeout_ms = 0u;

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
    dma->busy = 0u;
    dma->state = CH32_I2C_STATE_IDLE;
    dma->timeout_ms = 0u;
    dma->recover_pending = 0u;
    dma->tail = (uint8_t)((dma->tail + 1u) % CH32_I2C_DMA_QUEUE_LEN);
    dma->completed_count++;
    i2c_dma_start_next(dma);
}

/**
 * @brief Reset I2C and DMA state after a failed transaction.
 *
 * Consumes recover_pending first: an error that arrives while the peripheral is
 * being reset re-arms the flag and is handled on the next pass, so no recovery
 * request can be lost.
 *
 * @param dma Pointer to the I2C DMA context.
 */
static void i2c_dma_recover(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return;
    }

    dma->recover_pending = 0u;

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

    dma->head = 0u;
    dma->tail = 0u;
    dma->busy = 0u;
    dma->state = CH32_I2C_STATE_IDLE;
    dma->timeout_ms = 0u;
    dma->error = 1u;
    dma->recover_count++;
}

void CH32_I2C_DMA_EvtHandler(ch32_i2c_dma_t *dma)
{
    ch32_i2c_node_t *node;
    uint16_t star1;

    if (dma == 0 || dma->busy == 0u) {
        return;
    }

    node = &dma->queue[dma->tail];
    star1 = I2C_ReadRegister(dma->i2c, I2C_Register_STAR1);

    switch (dma->state) {
    case CH32_I2C_STATE_START:
        if ((star1 & I2C_STAR1_SB) != 0u) {
            I2C_Send7bitAddress(dma->i2c, (uint8_t)node->dev_addr, I2C_Direction_Transmitter);
            dma->state = CH32_I2C_STATE_ADDR;
        }
        break;

    case CH32_I2C_STATE_ADDR:
        if ((star1 & I2C_STAR1_ADDR) != 0u) {
            i2c_dma_clear_addr(dma->i2c);

            if (node->op == CH32_I2C_OP_WRITE) {
                i2c_dma_configure_channel(dma, (uint32_t)node->tx_data, node->size, true);
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
        if ((star1 & I2C_STAR1_BTF) != 0u) {
            I2C_GenerateSTART(dma->i2c, ENABLE);
            dma->state = CH32_I2C_STATE_RX_DATA;
        }
        break;

    case CH32_I2C_STATE_RX_DATA:
        if ((star1 & I2C_STAR1_SB) != 0u) {
            I2C_Send7bitAddress(dma->i2c, (uint8_t)node->dev_addr, I2C_Direction_Receiver);
        } else if ((star1 & I2C_STAR1_ADDR) != 0u) {
            i2c_dma_configure_channel(dma, (uint32_t)node->rx_data, node->size, false);
            I2C_DMALastTransferCmd(dma->i2c, ENABLE);
            I2C_DMACmd(dma->i2c, ENABLE);
            DMA_Cmd(dma->dma_rx, ENABLE);
            i2c_dma_clear_addr(dma->i2c);
            I2C_ITConfig(dma->i2c, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
            dma->state = CH32_I2C_STATE_STOP;
        }
        break;

    case CH32_I2C_STATE_WAIT_BTF:
        if ((star1 & I2C_STAR1_BTF) != 0u) {
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
                  I2C_STAR1_OVR | I2C_STAR1_TIMEOUT | I2C_STAR1_PECERR)) != 0u) {
        dma->error = 1u;
    }
    dma->recover_pending = 1u;
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

    if (dma->busy == 0u) {
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

    if (dma->busy == 0u) {
        return;
    }

    I2C_GenerateSTOP(dma->i2c, ENABLE);
    i2c_dma_finish_current(dma);
}

/**
 * @brief Advance the queue: recover after an error, time a stalled transfer, or
 *        finish a write that nobody reported.
 *
 * The WAIT_BTF poll is not redundant. Once the TX DMA has pushed the last byte the
 * event interrupt is re-enabled, but the BTF flag for a single-byte transfer can be
 * raised before that interrupt is taken; when it is taken later no further event
 * arrives, and the transfer would sit in WAIT_BTF forever. Re-checking the state
 * here means a BTF that nobody reported is still noticed on the next pass, so a
 * write can never stall the shared bus.
 *
 * The timeout decision lives here, in the main-loop context, so the tick handler
 * only counts and never touches the recovery bookkeeping.
 *
 * @param dma Pointer to the I2C DMA context.
 */
void CH32_I2C_DMA_Task(ch32_i2c_dma_t *dma)
{
    if (dma == 0) {
        return;
    }

    if (dma->recover_pending != 0u) {
        i2c_dma_recover(dma);
        return;
    }

    /* A transaction that stops making progress is treated as a bus error: the
     * peripheral is reset and the clients restart their own sequences. Without
     * this the queue would stay busy forever and every client would block. */
    if ((dma->busy != 0u) && (dma->timeout_ms >= CH32_I2C_DMA_TIMEOUT_MS)) {
        dma->error = 1u;
        dma->recover_pending = 1u;
        return;
    }

    if (dma->state == CH32_I2C_STATE_WAIT_BTF) {
        CH32_I2C_DMA_EvtHandler(dma);
    }
}

/**
 * @brief Count one tick of transfer time.
 *
 * Runs in the system tick and only saturates `timeout_ms`; the main loop reads it
 * in CH32_I2C_DMA_Task() and owns the error/recovery bookkeeping. The value is also
 * reset from the main loop when a transfer starts or completes, so the worst case of
 * that overlap is a timeout that fires one tick late, never a lost transaction.
 *
 * @param dma Pointer to the I2C DMA context.
 * @param elapsed_ms Milliseconds elapsed since the previous call.
 */
void CH32_I2C_DMA_TickMs(ch32_i2c_dma_t *dma, uint8_t elapsed_ms)
{
    if ((dma == 0) || (dma->busy == 0u)) {
        return;
    }

    if (dma->timeout_ms < CH32_I2C_DMA_TIMEOUT_MS) {
        dma->timeout_ms = (uint16_t)(dma->timeout_ms + elapsed_ms);
    }
}
