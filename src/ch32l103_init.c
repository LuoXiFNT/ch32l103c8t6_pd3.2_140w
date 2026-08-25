/**
 * @file ch32l103_init.c
 * @brief Board GPIO, peripheral, timer, SPI, and I2C initialization.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "ch32l103_init.h"

#include "board_config.h"
#include "board_link.h"
#include "ch32l103_i2c_dma.h"
#include "ch32l103_spi_dma.h"
#include "ch32l103_usbpd.h"
#include "display_service.h"
#include "fault_service.h"
#include "led_service.h"
#if BOARD_IS_MAIN
#include "lcd.h"
#include "lcd_init.h"
#endif
#include "pd.h"

#define TIM1_COUNTER_HZ        1000000UL
#define TIM1_TASK_PERIOD_MS    2U
#define FAN_PWM_ARR            5999U
#define FAN_PWM_COMPARE_50PCT  3000U
#define PD_GOODCRC_REPLY_DELAY_US 30U

ch32_i2c_dma_t g_i2c2_dma;

/** @brief Configure board-level power-good, alert, and interrupt inputs. */
static void Board_SignalGPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;

    gpio.GPIO_Pin = MP2980_INT_GPIO_PIN;
    GPIO_Init(MP2980_INT_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = CH217K_FLAG_GPIO_PIN;
    GPIO_Init(CH217K_FLAG_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = LMR33630_PG_GPIO_PIN;
    GPIO_Init(LMR33630_PG_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = LM5069_PG_GPIO_PIN;
    GPIO_Init(LM5069_PG_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = INA226_ALERT_A_GPIO_PIN;
    GPIO_Init(INA226_ALERT_A_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = INA226_ALERT_C_GPIO_PIN;
    GPIO_Init(INA226_ALERT_C_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = CH211_INT_GPIO_PIN;
    GPIO_Init(CH211_INT_GPIO_PORT, &gpio);
}

/** @brief Configure the status LED output and switch it off initially. */
static void LED_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = LED_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_GPIO_PORT, &gpio);

    LED_OFF();
}

#if BOARD_IS_MAIN
/** @brief Configure the LCD SPI1 peripheral and its transmit DMA channel. */
static void SPI1_DMA_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    SPI_InitTypeDef spi = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_SPI1, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    gpio.GPIO_Pin = SPI1_SCK_GPIO_PIN | SPI1_MOSI_GPIO_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(SPI1_GPIO_PORT, &gpio);
    GPIO_SetBits(SPI1_GPIO_PORT, SPI1_SCK_GPIO_PIN | SPI1_MOSI_GPIO_PIN);

    gpio.GPIO_Pin = SPI1_MISO_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(SPI1_GPIO_PORT, &gpio);

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7U;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);

    DMA_Cmd(SPI1_DMA_TX_CH, DISABLE);
    DMA_ClearITPendingBit(SPI1_DMA_TX_FLAG);

    nvic.NVIC_IRQChannel = SPI1_DMA_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}
#endif

/** @brief Configure the board-link USART1 pins, baud rate, and interrupts. */
static void USART1_Link_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef usart = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_USART1, ENABLE);

    gpio.GPIO_Pin = LINK_USART_TX_GPIO_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
#if BOARD_IS_SUB
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(LINK_USART_GPIO_PORT, &gpio);
#endif

#if BOARD_IS_MAIN
    gpio.GPIO_Pin = LINK_USART_RX_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(LINK_USART_GPIO_PORT, &gpio);
#endif

    USART_DeInit(USART1);
    usart.USART_BaudRate = CH32L103_LINK_USART_BAUD;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
#if BOARD_IS_SUB
    usart.USART_Mode = USART_Mode_Tx;
#else
    usart.USART_Mode = USART_Mode_Rx;
#endif
    USART_Init(USART1, &usart);

#if BOARD_IS_MAIN
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#else
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
#endif
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
}

/** @brief Configure USB-PD CC inputs and the GoodCRC timing timer. */
static void USBPD_Hardware_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_TimeBaseInitTypeDef timer = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM4, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBPD, ENABLE);

    gpio.GPIO_Pin = PD_CC1_GPIO_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    if (((CHIPID & 0xF0U) == 0x10U) ||
        (((CHIPID & 0xF0U) == 0x20U) &&
         (((USBPD_CFG & 0x0FU) == ((~(USBPD_CFG >> 4U)) & 0x0FU)) &&
          ((USBPD_CFG & 0x0FU) == 0x01U)))) {
        gpio.GPIO_Mode = GPIO_Mode_IPU;
    } else {
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    GPIO_Init(PD_CC_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = PD_CC2_GPIO_PIN;
    if (((CHIPID & 0xF0U) == 0x10U) ||
        (((CHIPID & 0xF0U) == 0x20U) &&
         (((USBPD_CFG & 0x0F00U) == ((~(USBPD_CFG >> 4U)) & 0x0F00U)) &&
          ((USBPD_CFG & 0x0F00U) == 0x0100U)))) {
        gpio.GPIO_Mode = GPIO_Mode_IPU;
    } else {
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    GPIO_Init(PD_CC_GPIO_PORT, &gpio);

    AFIO->CR |= USBPD_IN_HVT;

    TIM_DeInit(TIM4);
    timer.TIM_Period = PD_GOODCRC_REPLY_DELAY_US - 1U;
    timer.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000UL) - 1UL);
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM4, &timer);
    TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Single);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_SetPriority(TIM4_IRQn, 1);
}

#if BOARD_IS_MAIN
/** @brief Configure LCD reset, chip-select, backlight, and data/command pins. */
static void LCD_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = LCD_RES_GPIO_PIN;
    GPIO_Init(LCD_RES_GPIO_PORT, &gpio);
    GPIO_SetBits(LCD_RES_GPIO_PORT, LCD_RES_GPIO_PIN);

    gpio.GPIO_Pin = LCD_CS_GPIO_PIN;
    GPIO_Init(LCD_CS_GPIO_PORT, &gpio);
    GPIO_SetBits(LCD_CS_GPIO_PORT, LCD_CS_GPIO_PIN);

    gpio.GPIO_Pin = LCD_BLK_GPIO_PIN;
    GPIO_Init(LCD_BLK_GPIO_PORT, &gpio);
    GPIO_SetBits(LCD_BLK_GPIO_PORT, LCD_BLK_GPIO_PIN);

    gpio.GPIO_Pin = LCD_DC_GPIO_PIN;
    GPIO_Init(LCD_DC_GPIO_PORT, &gpio);
    GPIO_SetBits(LCD_DC_GPIO_PORT, LCD_DC_GPIO_PIN);

}

/** @brief Configure the fan PWM output and tachometer input. */
static void FAN_TIM2_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_TimeBaseInitTypeDef tim = {0};
    TIM_OCInitTypeDef oc = {0};
    TIM_ICInitTypeDef ic = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, DISABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, DISABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, DISABLE);

    gpio.GPIO_Pin = FAN_PWM_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(FAN_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = FAN_TACH_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(FAN_GPIO_PORT, &gpio);

    TIM_DeInit(TIM2);
    tim.TIM_Period = FAN_PWM_ARR;
    tim.TIM_Prescaler = 0U;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM2, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = FAN_PWM_COMPARE_50PCT;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC2Init(TIM2, &oc);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    ic.TIM_Channel = TIM_Channel_3;
    ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter = 0U;
    TIM_ICInit(TIM2, &ic);

    TIM_ARRPreloadConfig(TIM2, DISABLE);
    TIM_SetCompare2(TIM2, FAN_PWM_COMPARE_50PCT);
    TIM_Cmd(TIM2, ENABLE);
}
#endif

void CH32L103_FanSetDutyPermille(u16 duty_permille)
{
#if BOARD_IS_MAIN
    u32 compare;

    if (duty_permille > 1000U) {
        duty_permille = 1000U;
    }

    compare = ((u32)(FAN_PWM_ARR + 1U) * duty_permille + 500U) / 1000U;
    if (compare > (u32)(FAN_PWM_ARR + 1U)) {
        compare = FAN_PWM_ARR + 1U;
    }

    TIM_SetCompare2(TIM2, (u16)compare);
#else
    (void)duty_permille;
#endif
}

/** @brief Configure the 2 ms board service timer interrupt. */
static void TIM1_INT_Init(void)
{
    NVIC_InitTypeDef nvic = {0};
    TIM_TimeBaseInitTypeDef tim = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_TIM1, ENABLE);

    tim.TIM_Period = (uint16_t)(((TIM1_COUNTER_HZ / 1000UL) *
                                 TIM1_TASK_PERIOD_MS) - 1UL);
    tim.TIM_Prescaler = (uint16_t)((SystemCoreClock / TIM1_COUNTER_HZ) - 1UL);
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

    nvic.NVIC_IRQChannel = TIM1_UP_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/** @brief Configure I2C2, DMA channels, and their interrupt handlers. */
static void I2C2_DMA_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    I2C_InitTypeDef i2c = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C2, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    gpio.GPIO_Pin = I2C2_SCL_GPIO_PIN | I2C2_SDA_GPIO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C2_GPIO_PORT, &gpio);

    I2C_DeInit(I2C2);
    i2c.I2C_ClockSpeed = 400000U;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_16_9;
    i2c.I2C_OwnAddress1 = 0x00U;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &i2c);

    I2C_DMACmd(I2C2, DISABLE);
    I2C_DMALastTransferCmd(I2C2, DISABLE);
    I2C_Cmd(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    NVIC_EnableIRQ(I2C2_EV_IRQn);
    NVIC_SetPriority(I2C2_EV_IRQn, 2);
    NVIC_EnableIRQ(I2C2_ER_IRQn);
    NVIC_SetPriority(I2C2_ER_IRQn, 1);
    NVIC_EnableIRQ(I2C2_DMA_TX_IRQn);
    NVIC_SetPriority(I2C2_DMA_TX_IRQn, 2);
    NVIC_EnableIRQ(I2C2_DMA_RX_IRQn);
    NVIC_SetPriority(I2C2_DMA_RX_IRQn, 2);

    CH32_I2C_DMA_Init(&g_i2c2_dma,
                      I2C2,
                      I2C2_DMA_TX_CH,
                      I2C2_DMA_TX_FLAG,
                      I2C2_DMA_RX_CH,
                      I2C2_DMA_RX_FLAG);
}

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        LED_RunTickMs(TIM1_TASK_PERIOD_MS, 500U);
        PD_TickMs(TIM1_TASK_PERIOD_MS);
        CH32_I2C_DMA_TickMs(&g_i2c2_dma, TIM1_TASK_PERIOD_MS);
        Display_TickMs(TIM1_TASK_PERIOD_MS);
        Fault_TickMs(TIM1_TASK_PERIOD_MS);
        BoardLink_TickMs(TIM1_TASK_PERIOD_MS);
    }
}

void CH32L103_BoardInit(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    Board_SignalGPIO_Init();
    LED_GPIO_Init();
    USART1_Link_Init();
    USBPD_Hardware_Init();
    TIM1_INT_Init();
#if BOARD_IS_MAIN
    FAN_TIM2_Init();
    LCD_GPIO_Init();
    SPI1_DMA_Init();
    LCD_Init();
    LCD_AsyncInit();
#else
    /* Match the LCD reset/sleep-out delay before enabling I2C2. */
    Delay_Ms(320U);
#endif

    I2C2_DMA_Init();
}
