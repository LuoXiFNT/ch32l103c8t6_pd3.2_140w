/**
 * @file ch32l103_it.c
 * @brief Interrupt vector implementations for the CH32L103 platform.
 *
 * Every handler only forwards to the module that owns the peripheral state; the
 * work itself (bus transfers, rendering, protocol handling) runs in the main loop.
 * The TIM1 handler is the system tick: it advances the millisecond counters of all
 * services and must never do more than that.
 *
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-05
 */

#include "ch32l103_it.h"

#include "board_config.h"
#include "board_link.h"
#include "ch32_i2c_dma.h"
#include "ch32l103_init.h"
#include "display_service.h"
#include "fault_service.h"
#include "led_service.h"
#include "pd.h"
#if BOARD_IS_MAIN
#include "ch32_spi_dma.h"
#endif

/* The WCH-Interrupt-fast attribute selects the hardware prologue of the QingKe
 * core; nested interrupts only work when every vector uses it. */
void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#if BOARD_IS_MAIN
void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#endif
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C2_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C2_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/** @brief Status LED toggle period in milliseconds. */
#define LED_TOGGLE_PERIOD_MS    500u

/**
 * @brief Advance every service timebase by one system tick.
 *
 * All services accumulate the elapsed time themselves, so the tick period stays a
 * single constant: change BOARD_TICK_MS and every timer follows, with the same
 * wall-clock behaviour.
 */
static void CH32L103_Tick(void)
{
    CH32L103_TickUs();
    LED_RunTickMs(BOARD_TICK_MS, LED_TOGGLE_PERIOD_MS);
    PD_TickMs(BOARD_TICK_MS);
    CH32_I2C_DMA_TickMs(&g_i2c2_dma, BOARD_TICK_MS);
    Display_TickMs(BOARD_TICK_MS);
    Fault_TickMs(BOARD_TICK_MS);
    BoardLink_TickMs(BOARD_TICK_MS);
}

/** @brief Non-maskable interrupt: nothing can be reported, so stop here. */
void NMI_Handler(void)
{
    while (1) {
    }
}

/** @brief Hard fault: reset the MCU instead of spinning on a dead bus. */
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while (1) {
    }
}

/** @brief System tick: clear the update flag first, then advance all timebases. */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        CH32L103_Tick();
    }
}

#if BOARD_IS_MAIN
/** @brief SPI1 TX DMA completion: move the LCD engine to its drain phase. */
void DMA1_Channel3_IRQHandler(void)
{
    CH32_SPI_DMA_TxHandler();
}
#endif

/** @brief I2C2 TX DMA completion. */
void DMA1_Channel4_IRQHandler(void)
{
    CH32_I2C_DMA_TxHandler(&g_i2c2_dma);
}

/** @brief I2C2 RX DMA completion. */
void DMA1_Channel5_IRQHandler(void)
{
    CH32_I2C_DMA_RxHandler(&g_i2c2_dma);
}

/** @brief I2C2 event interrupt. */
void I2C2_EV_IRQHandler(void)
{
    CH32_I2C_DMA_EvtHandler(&g_i2c2_dma);
}

/** @brief I2C2 error interrupt; the main loop performs the actual recovery. */
void I2C2_ER_IRQHandler(void)
{
    CH32_I2C_DMA_ErrHandler(&g_i2c2_dma);
}

/** @brief USART1 board-link interrupt. */
void USART1_IRQHandler(void)
{
    BoardLink_USART1_IRQHandler();
}
