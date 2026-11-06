/**
 * @file main.c
 * @brief Firmware entry point: initialize the board, then run the cooperative loop.
 *
 * There is no RTOS. The TIM1 tick (see ch32l103_it.c) only advances the millisecond
 * counters of the services; every state machine is advanced here, one bounded step
 * per pass, in an order that keeps measurements one pass ahead of the logic that
 * consumes them.
 *
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 *
 * Derived from the WCH CH32L103 demo project; the original copyright notice is kept
 * in the sdk/ tree.
 */

#include <stdbool.h>

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
    bool bus_error;

    /* Board hardware, transports, and every driver/display initialization. */
    CH32L103_BoardInit();

    /* PD receives VBUS feedback through the board-level measurement callback. */
    PD_Init(PD_VbusSenseMv);

    while (1) {
        /* 1. Time-driven state machines: PD protocol and the LCD transfer engine.
         *    Both are tick-driven and only advance when their timer says so. */
#if BOARD_IS_MAIN
        lcd_task();
#endif
        PD_TaskFromTick();

        /* 2. Shared I2C bus: finish the transfer in flight, recover the queue after a
         *    bus error, then let each driver and the INA226 sampler queue at most one
         *    transfer.
         *
         *    Recovery is per client and deliberately conservative. A bus error is not
         *    a protection event: the PD contract stays in force, so the MP2980 port
         *    keeps the current limit it has already programmed (USB PD 3.2 section
         *    4.1.5.1 requires OCP not to act while the drawn current is valid for the
         *    contract, so that limit must not be forgotten or rewritten needlessly) and
         *    only drops the voltage cache, while CH211 restarts its command chain.
         *    Real protection events go through Fault_Task() ->
         *    PD_UpdateFaults(), which sends the Alert, removes VBUS and renegotiates
         *    after the fault clears (sections 4.1.5.1 and 4.1.5.3).
         *
         *    The error flag is cleared only after every client has seen it. */
        CH32_I2C_DMA_Task(&g_i2c2_dma);
        bus_error = CH32_I2C_DMA_HadError(&g_i2c2_dma);
        if (bus_error) {
            INA226_Service_Abort();   /* the sampler has no recovery of its own */
        }
        MP2980_I2C_DMA_Task();
        CH211_I2C_DMA_Task();
        INA226_Service_I2C_DMA_Task();
        if (bus_error) {
            CH32_I2C_DMA_ClearError(&g_i2c2_dma);
        }

        /* 3. Measurements first, so protection and reporting see fresh data. */
        INA226_Service_UpdateSamples();

        /* 4. Board protection, then the link that publishes it to the other board. */
        Fault_Task();
        BoardLink_Task();

        /* 5. Main board only: fan duty follows the measured power, then the LCD
         *    content is rebuilt when the refresh slot is due. */
#if BOARD_IS_MAIN
        Fan_Task();
        Display_Task();
#endif
    }
}
