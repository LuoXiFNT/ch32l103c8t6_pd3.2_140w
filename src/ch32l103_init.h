/**
 * @file ch32l103_init.h
 * @brief Board pin assignments and CH32L103 initialization interface.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef CH32L103_INIT_H
#define CH32L103_INIT_H

#include "ch32l103_i2c_dma.h"

#define CH32L103_LINK_USART_BAUD 115200U

/* Board GPIO assignment. */
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

#define LCD_RES_GPIO_PORT             GPIOA
#define LCD_RES_GPIO_PIN              GPIO_Pin_3
#define LCD_CS_GPIO_PORT              GPIOA
#define LCD_CS_GPIO_PIN               GPIO_Pin_4
#define LCD_BLK_GPIO_PORT             GPIOB
#define LCD_BLK_GPIO_PIN              GPIO_Pin_0
#define LCD_DC_GPIO_PORT              GPIOB
#define LCD_DC_GPIO_PIN               GPIO_Pin_1

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

extern ch32_i2c_dma_t g_i2c2_dma;

void CH32L103_BoardInit(void);
void CH32L103_FanSetDutyPermille(u16 duty_permille);

#endif
