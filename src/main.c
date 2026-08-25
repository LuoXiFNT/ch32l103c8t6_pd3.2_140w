/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/26
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "ch32l103_init.h"
#include "board_config.h"
#include "board_link.h"
#include "ch211_i2c_dma_port.h"
#include "display_service.h"
#include "fault_service.h"
#include "fan_service.h"
#include "ina226.h"
#include "ina226_service.h"
#if BOARD_IS_MAIN
#include "lcd.h"
#endif
#include "mp2980_i2c_dma_port.h"
#include "pd.h"

/** @brief Provide PD with the measured local C-channel VBUS voltage. */
static uint16_t PD_VbusSenseMv(void)
{
    ina226_service_sample_t channel_c;

    INA226_Service_GetSamples(&channel_c, 0);
    return ina226_convert_bus_voltage_mv(channel_c.vbus_raw);
}

int main(void)
{
    /* Configure board hardware and the shared I2C/SPI DMA transports. */
    CH32L103_BoardInit();

    /* Register each I2C client before its service begins queuing transfers. */
    INA226_Service_Init(&g_i2c2_dma);
    (void)CH211_I2C_DMA_InitDefault(&g_i2c2_dma);
    (void)MP2980_I2C_DMA_InitDefault(&g_i2c2_dma);

    /* PD receives VBUS feedback through the board-level measurement callback. */
    PD_Init(PD_VbusSenseMv);

    while (1) {

#if BOARD_IS_MAIN
        /* Advance the LCD SPI DMA transfer state machine. */
        LCD_Task();
#endif
        /* Advance the PD protocol state machine from the system tick. */
        PD_TaskFromTick();

        /* Complete queued I2C transfers and recover all clients after bus errors. */
        CH32_I2C_DMA_Task(&g_i2c2_dma);
        if (CH32_I2C_DMA_HadError(&g_i2c2_dma) != 0U) {
            INA226_Service_Abort();
            MP2980_I2C_DMA_Abort();
            CH211_I2C_DMA_Abort();
            CH32_I2C_DMA_ClearError(&g_i2c2_dma);
        }
        MP2980_I2C_DMA_Task();
        CH211_I2C_DMA_Task();
        INA226_Service_I2C_DMA_Task();

        /* Update measurements first, then run board protection and reporting services. */
        INA226_Service_UpdateSamples();
        Fault_Task();
        BoardLink_Task();
#if BOARD_IS_MAIN
        /* Main board only: update the fan control and the rendered LCD content. */
        Fan_Task();
        Display_Task();
#endif
    }
}
