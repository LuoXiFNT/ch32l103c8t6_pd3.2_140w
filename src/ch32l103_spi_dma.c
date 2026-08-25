/**
 * @file ch32l103_spi_dma.c
 * @brief SPI1 byte and DMA transfer implementation for the LCD bus.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "ch32l103_spi_dma.h"
#include "ch32l103_init.h"

/** Current SPI DMA transfer phase. */
static volatile uint8_t s_spi_dma_state = CH32_SPI_DMA_STATE_IDLE;

void CH32_SPI_DMA_WriteByte(uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
    }
    SPI_I2S_SendData(SPI1, data);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {
    }
}

uint8_t CH32_SPI_DMA_Write(const uint8_t *data, uint16_t len)
{
    DMA_InitTypeDef dma = {0};

    if ((data == 0) || (len == 0U) ||
        (s_spi_dma_state != CH32_SPI_DMA_STATE_IDLE)) {
        return 0U;
    }

    DMA_Cmd(SPI1_DMA_TX_CH, DISABLE);
    DMA_DeInit(SPI1_DMA_TX_CH);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)data;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = len;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(SPI1_DMA_TX_CH, &dma);
    DMA_ClearITPendingBit(SPI1_DMA_TX_FLAG);
    DMA_ITConfig(SPI1_DMA_TX_CH, DMA_IT_TC, ENABLE);

    s_spi_dma_state = CH32_SPI_DMA_STATE_TRANSFER;
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
    DMA_Cmd(SPI1_DMA_TX_CH, ENABLE);
    return 1U;
}

uint8_t CH32_SPI_DMA_IsIdle(void)
{
    return (s_spi_dma_state == CH32_SPI_DMA_STATE_IDLE) ? 1U : 0U;
}

void CH32_SPI_DMA_Task(void)
{
    if ((s_spi_dma_state == CH32_SPI_DMA_STATE_DRAIN) &&
        (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == RESET)) {
        s_spi_dma_state = CH32_SPI_DMA_STATE_IDLE;
    }
}

void CH32_SPI_DMA_TxHandler(void)
{
    if (DMA_GetITStatus(SPI1_DMA_TX_FLAG) == RESET) {
        return;
    }

    DMA_ClearITPendingBit(SPI1_DMA_TX_FLAG);
    DMA_ITConfig(SPI1_DMA_TX_CH, DMA_IT_TC, DISABLE);
    DMA_Cmd(SPI1_DMA_TX_CH, DISABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

    if (s_spi_dma_state == CH32_SPI_DMA_STATE_TRANSFER) {
        s_spi_dma_state = CH32_SPI_DMA_STATE_DRAIN;
    }
}
