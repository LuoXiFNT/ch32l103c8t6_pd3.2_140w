/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32l103_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/10/30
 * Description        : Main Interrupt Service Routines.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "ch32l103_it.h"
#include "board_config.h"
#include "board_link.h"
#include "ch32l103_i2c_dma.h"
#include "ch32l103_init.h"
#if BOARD_IS_MAIN
#include "ch32l103_spi_dma.h"
#endif

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#if BOARD_IS_MAIN
void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#endif
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C2_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C2_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
    while (1)
    {
    }
}


/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while (1)
    {
    }
}

#if BOARD_IS_MAIN
void DMA1_Channel3_IRQHandler(void)
{
    CH32_SPI_DMA_TxHandler();
}
#endif

void DMA1_Channel4_IRQHandler(void)
{
    CH32_I2C_DMA_TxHandler(&g_i2c2_dma);
}

void DMA1_Channel5_IRQHandler(void)
{
    CH32_I2C_DMA_RxHandler(&g_i2c2_dma);
}

void I2C2_EV_IRQHandler(void)
{
    CH32_I2C_DMA_EvtHandler(&g_i2c2_dma);
}

void I2C2_ER_IRQHandler(void)
{
    CH32_I2C_DMA_ErrHandler(&g_i2c2_dma);
}

void USART1_IRQHandler(void)
{
    BoardLink_USART1_IRQHandler();
}

