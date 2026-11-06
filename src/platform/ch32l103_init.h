/**
 * @file ch32l103_init.h
 * @brief Board pin assignments and CH32L103 initialization interface.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef CH32L103_INIT_H
#define CH32L103_INIT_H

#include <stdint.h>

#include "ch32_i2c_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CH32L103_LINK_USART_BAUD 115200u

/* System tick period in milliseconds. TIM1 fires this often and every service
 * advances by exactly this much, so the whole firmware timebase follows this one
 * value: change it here and all timers keep their wall-clock behaviour. */
#define BOARD_TICK_MS            1u

/* Board GPIO assignment.
 *
 * Inputs are pulled up because every fault and interrupt line on this board is
 * active low and driven by an open-drain or push-pull output of the monitoring IC:
 * a floating pin would read as a fault. The LED, the LCD control lines and the fan
 * PWM are push-pull outputs. */
#define LED_GPIO_PORT                 GPIOB
#define LED_GPIO_PIN                  GPIO_Pin_5

#define MP2980_INT_GPIO_PORT          GPIOA
#define MP2980_INT_GPIO_PIN           GPIO_Pin_0
#define CH217K_FLAG_GPIO_PORT         GPIOA
#define CH217K_FLAG_GPIO_PIN          GPIO_Pin_8
#define LMR33630_PG_GPIO_PORT         GPIOB
#define LMR33630_PG_GPIO_PIN          GPIO_Pin_8
#define LM5069_PG_GPIO_PORT           GPIOB
#define LM5069_PG_GPIO_PIN            GPIO_Pin_9
#define INA226_ALERT_A_GPIO_PORT      GPIOB
#define INA226_ALERT_A_GPIO_PIN       GPIO_Pin_12
#define INA226_ALERT_C_GPIO_PORT      GPIOB
#define INA226_ALERT_C_GPIO_PIN       GPIO_Pin_13
#define CH211_INT_GPIO_PORT           GPIOB
#define CH211_INT_GPIO_PIN            GPIO_Pin_14

#define LINK_USART_GPIO_PORT          GPIOA
#define LINK_USART_TX_GPIO_PIN        GPIO_Pin_9
#define LINK_USART_RX_GPIO_PIN        GPIO_Pin_10

#define FAN_GPIO_PORT                 GPIOA
#define FAN_PWM_GPIO_PIN              GPIO_Pin_1
#define FAN_TACH_GPIO_PIN             GPIO_Pin_2

#define ST7735S_RES_GPIO_PORT             GPIOA
#define ST7735S_RES_GPIO_PIN              GPIO_Pin_3
#define ST7735S_CS_GPIO_PORT              GPIOA
#define ST7735S_CS_GPIO_PIN               GPIO_Pin_4
#define ST7735S_BLK_GPIO_PORT             GPIOB
#define ST7735S_BLK_GPIO_PIN              GPIO_Pin_0
#define ST7735S_DC_GPIO_PORT              GPIOB
#define ST7735S_DC_GPIO_PIN               GPIO_Pin_1

/* Display module control lines, shared by the platform init and the driver.
 * BLK high lights the LEDs (low-side NPN with a 10 kOhm base pull-down to GND),
 * so the backlight is dark whenever the pin is low or undriven. */
#define ST7735S_RES_LOW()                 GPIO_ResetBits(ST7735S_RES_GPIO_PORT, ST7735S_RES_GPIO_PIN)
#define ST7735S_RES_HIGH()                GPIO_SetBits(ST7735S_RES_GPIO_PORT, ST7735S_RES_GPIO_PIN)
#define ST7735S_CS_LOW()                  GPIO_ResetBits(ST7735S_CS_GPIO_PORT, ST7735S_CS_GPIO_PIN)
#define ST7735S_CS_HIGH()                 GPIO_SetBits(ST7735S_CS_GPIO_PORT, ST7735S_CS_GPIO_PIN)
#define ST7735S_DC_LOW()                  GPIO_ResetBits(ST7735S_DC_GPIO_PORT, ST7735S_DC_GPIO_PIN)
#define ST7735S_DC_HIGH()                 GPIO_SetBits(ST7735S_DC_GPIO_PORT, ST7735S_DC_GPIO_PIN)
#define ST7735S_BLK_LOW()                 GPIO_ResetBits(ST7735S_BLK_GPIO_PORT, ST7735S_BLK_GPIO_PIN)
#define ST7735S_BLK_HIGH()                GPIO_SetBits(ST7735S_BLK_GPIO_PORT, ST7735S_BLK_GPIO_PIN)

/* Board DMA assignment. */
#define SPI1_GPIO_PORT                 GPIOA
#define SPI1_SCK_GPIO_PIN              GPIO_Pin_5
#define SPI1_MISO_GPIO_PIN             GPIO_Pin_6
#define SPI1_MOSI_GPIO_PIN             GPIO_Pin_7
#define SPI1_DMA_TX_CH                 DMA1_Channel3
#define SPI1_DMA_TX_FLAG               DMA1_FLAG_TC3
#define SPI1_DMA_TX_IRQn               DMA1_Channel3_IRQn

#define I2C2_GPIO_PORT                GPIOB
#define I2C2_SCL_GPIO_PIN             GPIO_Pin_10
#define I2C2_SDA_GPIO_PIN             GPIO_Pin_11
#define I2C2_DMA_TX_CH                DMA1_Channel4
#define I2C2_DMA_TX_FLAG              DMA1_FLAG_TC4
#define I2C2_DMA_TX_IRQn              DMA1_Channel4_IRQn
#define I2C2_DMA_RX_CH                DMA1_Channel5
#define I2C2_DMA_RX_FLAG              DMA1_FLAG_TC5
#define I2C2_DMA_RX_IRQn              DMA1_Channel5_IRQn

#define PD_CC_GPIO_PORT               GPIOB
#define PD_CC1_GPIO_PIN               GPIO_Pin_6
#define PD_CC2_GPIO_PIN               GPIO_Pin_7

/**
 * @name Interrupt priority levels
 *
 * Interrupt nesting is enabled (INTSYSCR_INEST in ch32l103_misc.h), so a PFIC
 * priority byte is "preemption priority << 7 | sub priority << 5": two preemption
 * levels, four sub levels each. Only interrupts with a different preemption priority
 * preempt each other; within one preemption level the sub priority only decides who
 * runs first when both are pending.
 *
 * The bus and the PD timing sit in the upper level because they cannot tolerate
 * delay; the system tick and the slow services sit below them and accept being
 * deferred by a few microseconds.
 * @{
 */
#define CH32L103_IRQ_PREEMPT_CRITICAL   0u /**< I2C bus, GoodCRC timer. */
#define CH32L103_IRQ_PREEMPT_SERVICES   1u /**< System tick, USART link, LCD DMA. */

#define CH32L103_IRQ_SUB_FIRST          0u
#define CH32L103_IRQ_SUB_SECOND         1u
#define CH32L103_IRQ_SUB_THIRD          2u
/** @} */

extern ch32_i2c_dma_t g_i2c2_dma;

void CH32L103_BoardInit(void);
void CH32L103_FanSetDutyPermille(uint16_t duty_permille);

/**
 * @brief Enable one interrupt with an explicit priority.
 *
 * NVIC_SetPriority() writes the priority byte verbatim, so passing it a small level
 * number silently maps every interrupt to the highest preemption level. This helper
 * applies the documented encoding instead, so the named levels above mean what they
 * say; use it for every vector.
 *
 * @param irq Interrupt to enable.
 * @param preempt_priority Preemption level, 0 is the most urgent.
 * @param sub_priority Sub priority inside that level, 0 is the most urgent.
 */
void CH32L103_NvicConfig(IRQn_Type irq, uint8_t preempt_priority, uint8_t sub_priority);

/**
 * @brief Monotonic microsecond counter, 1 us resolution.
 *
 * Backed by the 1 MHz system tick timer plus the tick counter, so it can time
 * intervals shorter than one tick (for example the 1100 us CRCReceiveTimer of the PD
 * protocol layer). The value wraps after about 71 minutes; compare two samples with an
 * unsigned difference.
 *
 * @return uint32_t Microseconds since start-up.
 */
uint32_t CH32L103_GetUs(void);

/** @brief Advance the microsecond base by one system tick. Called from the tick ISR. */
void CH32L103_TickUs(void);

/**
 * @brief Blocking microsecond delay.
 *
 * Busy-waits on the same 1 MHz counter that CH32L103_GetUs() reads, so the firmware has
 * one time base instead of a separate millisecond timer for delays. Only the hardware
 * counter is read, so the wait works whether or not interrupts are enabled; TIM1 has to
 * be running, which BoardInit guarantees before the first delay is used.
 *
 * @param us Microseconds to wait.
 */
void CH32L103_DelayUs(uint32_t us);

/**
 * @brief Blocking millisecond delay built from CH32L103_DelayUs().
 * @param ms Milliseconds to wait.
 */
void CH32L103_DelayMs(uint32_t ms);


#ifdef __cplusplus
}
#endif
#endif /* CH32L103_INIT_H */
