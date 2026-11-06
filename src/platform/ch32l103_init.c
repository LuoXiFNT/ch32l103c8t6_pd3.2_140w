/**
 * @file ch32l103_init.c
 * @brief Board GPIO, peripheral, timer, and I2C initialization.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#include "ch32l103_init.h"

#include "board_config.h"
#include "ch211_i2c_dma_port.h"
#include "ch32_i2c_dma.h"
#include "ch32l103_usbpd.h"
#include "ina226_service.h"
#include "led_service.h"
#include "mp2980_i2c_dma_port.h"
#if BOARD_IS_MAIN
#include "lcd.h"
#include "st7735s_spi_dma_port.h"
#endif
/* Needed by both roles: the sub board borrows the panel's blocking boot time so it
 * reaches the main loop at the same instant as the main board. */
#include "st7735s.h"

#define TIM1_COUNTER_HZ        1000000ul
/** Microseconds between two TIM1 reload events. */
#define TIM1_RELOAD_US         ((TIM1_COUNTER_HZ / 1000ul) * BOARD_TICK_MS)
#define FAN_PWM_ARR            5999u
#define FAN_PWM_COMPARE_50PCT  3000u
#define PD_GOODCRC_REPLY_DELAY_US 30u

ch32_i2c_dma_t g_i2c2_dma;

/** Microseconds accumulated by the tick; TIM1 supplies the sub-millisecond part. */
static volatile uint32_t s_tick_us;

void CH32L103_NvicConfig(IRQn_Type irq, uint8_t preempt_priority, uint8_t sub_priority)
{
    /* PFIC byte with nesting enabled: bit7 = preemption, bits6:5 = sub priority. */
    NVIC_SetPriority(irq, (uint8_t)((preempt_priority << 7u) | (sub_priority << 5u)));
    NVIC_EnableIRQ(irq);
}

void CH32L103_TickUs(void)
{
    s_tick_us += (uint32_t)BOARD_TICK_MS * 1000ul;
}

uint32_t CH32L103_GetUs(void)
{
    /* TIM1 runs at 1 MHz and reloads once per BOARD_TICK_MS, so its counter is the
     * microsecond offset inside the current tick. Reading it together with the
     * accumulated base gives a monotonic counter; a tick landing between the two reads
     * can only make the result up to one tick old, which no caller depends on. */
    return s_tick_us + (uint32_t)TIM1->CNT;
}

void CH32L103_DelayUs(uint32_t us)
{
    uint16_t last = (uint16_t)TIM1->CNT;
    uint32_t remaining = us;

    /* The counter reloads every TIM1_RELOAD_US, so each step counts the distance to the
     * previous sample and adds the wrap when the counter rolled over. Only the hardware
     * counter is read, which keeps the wait independent of the tick interrupt. */
    while (remaining != 0u) {
        uint16_t now = (uint16_t)TIM1->CNT;
        uint16_t delta = (now >= last) ? (uint16_t)(now - last) :
                                        (uint16_t)(TIM1_RELOAD_US - last + now);

        last = now;
        remaining = (remaining > (uint32_t)delta) ? (remaining - (uint32_t)delta) : 0u;
    }
}

void CH32L103_DelayMs(uint32_t ms)
{
    while (ms != 0ul) {
        CH32L103_DelayUs(1000ul);
        ms--;
    }
}

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

/** @brief Configure the board-link USART1 pins, baud rate, and interrupts. */
static void USART1_Link_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef usart = {0};

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

    CH32L103_NvicConfig(USART1_IRQn,
                        CH32L103_IRQ_PREEMPT_SERVICES,
                        CH32L103_IRQ_SUB_SECOND);

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
    if (((CHIPID & 0xF0u) == 0x10u) ||
        (((CHIPID & 0xF0u) == 0x20u) &&
         (((USBPD_CFG & 0x0Fu) == ((~(USBPD_CFG >> 4u)) & 0x0Fu)) &&
          ((USBPD_CFG & 0x0Fu) == 0x01u)))) {
        gpio.GPIO_Mode = GPIO_Mode_IPU;
    } else {
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    GPIO_Init(PD_CC_GPIO_PORT, &gpio);

    gpio.GPIO_Pin = PD_CC2_GPIO_PIN;
    if (((CHIPID & 0xF0u) == 0x10u) ||
        (((CHIPID & 0xF0u) == 0x20u) &&
         (((USBPD_CFG & 0x0F00u) == ((~(USBPD_CFG >> 4u)) & 0x0F00u)) &&
          ((USBPD_CFG & 0x0F00u) == 0x0100u)))) {
        gpio.GPIO_Mode = GPIO_Mode_IPU;
    } else {
        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    }
    GPIO_Init(PD_CC_GPIO_PORT, &gpio);

    AFIO->CR |= USBPD_IN_HVT;

    TIM_DeInit(TIM4);
    timer.TIM_Period = PD_GOODCRC_REPLY_DELAY_US - 1u;
    timer.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000ul) - 1ul);
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_RepetitionCounter = 0u;
    TIM_TimeBaseInit(TIM4, &timer);
    TIM_SelectOnePulseMode(TIM4, TIM_OPMode_Single);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    /* GoodCRC is due 30 us after the message; it must not queue behind the tick. */
    CH32L103_NvicConfig(TIM4_IRQn,
                        CH32L103_IRQ_PREEMPT_CRITICAL,
                        CH32L103_IRQ_SUB_FIRST);
}

#if BOARD_IS_MAIN
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
    tim.TIM_Prescaler = 0u;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_RepetitionCounter = 0u;
    TIM_TimeBaseInit(TIM2, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = FAN_PWM_COMPARE_50PCT;
    /* The fan connector is driven through an external NMOS inverter, so the timer
     * runs active-low: the pin is low for CCR ticks, the inverter turns that into a
     * high level on the fan PWM input, and a larger compare value therefore means a
     * longer fan-on time. Do not "fix" this to TIM_OCPolarity_High without removing
     * the inverter. */
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
    ic.TIM_ICFilter = 0u;
    TIM_ICInit(TIM2, &ic);

    TIM_ARRPreloadConfig(TIM2, DISABLE);
    TIM_SetCompare2(TIM2, FAN_PWM_COMPARE_50PCT);
    TIM_Cmd(TIM2, ENABLE);
}
#endif

/**
 * @brief Set the fan PWM duty cycle.
 *
 * The pin drives the fan connector through an external NMOS inverter and the timer
 * is configured active-low, so the compare value maps straight to the duty seen by
 * the fan: 0 is off and 1000 is full speed.
 *
 * @param duty_permille Requested duty in thousandths; values above 1000 are clamped.
 */
void CH32L103_FanSetDutyPermille(uint16_t duty_permille)
{
#if BOARD_IS_MAIN
    uint32_t compare;

    if (duty_permille > 1000u) {
        duty_permille = 1000u;
    }

    compare = ((uint32_t)(FAN_PWM_ARR + 1u) * duty_permille + 500u) / 1000u;
    TIM_SetCompare2(TIM2, (uint16_t)compare);
#else
    (void)duty_permille;
#endif
}

/** @brief Configure the BOARD_TICK_MS board service timer interrupt. */
static void TIM1_INT_Init(void)
{
    TIM_TimeBaseInitTypeDef tim = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_TIM1, ENABLE);

    tim.TIM_Period = (uint16_t)(((TIM1_COUNTER_HZ / 1000ul) * BOARD_TICK_MS) - 1ul);
    tim.TIM_Prescaler = (uint16_t)((SystemCoreClock / TIM1_COUNTER_HZ) - 1ul);
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tim);

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

    /* The tick only advances counters, so it yields to the bus and to the PD timer. */
    CH32L103_NvicConfig(TIM1_UP_IRQn,
                        CH32L103_IRQ_PREEMPT_SERVICES,
                        CH32L103_IRQ_SUB_FIRST);

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
    i2c.I2C_ClockSpeed = 400000u;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_16_9;
    i2c.I2C_OwnAddress1 = 0x00u;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &i2c);

    I2C_DMACmd(I2C2, DISABLE);
    I2C_DMALastTransferCmd(I2C2, DISABLE);
    I2C_Cmd(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    /* The bus handlers must not be delayed: an I2C byte time at 400 kHz is 22.5 us and
     * a late event interrupt breaks the transaction. The error handler goes first so a
     * bus problem is cleared before the next event is processed. */
    CH32L103_NvicConfig(I2C2_ER_IRQn,
                        CH32L103_IRQ_PREEMPT_CRITICAL,
                        CH32L103_IRQ_SUB_FIRST);
    CH32L103_NvicConfig(I2C2_EV_IRQn,
                        CH32L103_IRQ_PREEMPT_CRITICAL,
                        CH32L103_IRQ_SUB_SECOND);
    CH32L103_NvicConfig(I2C2_DMA_TX_IRQn,
                        CH32L103_IRQ_PREEMPT_CRITICAL,
                        CH32L103_IRQ_SUB_THIRD);
    CH32L103_NvicConfig(I2C2_DMA_RX_IRQn,
                        CH32L103_IRQ_PREEMPT_CRITICAL,
                        CH32L103_IRQ_SUB_THIRD);

    CH32_I2C_DMA_Init(&g_i2c2_dma,
                      I2C2,
                      I2C2_DMA_TX_CH,
                      I2C2_DMA_TX_FLAG,
                      I2C2_DMA_RX_CH,
                      I2C2_DMA_RX_FLAG);
}

void CH32L103_BoardInit(void)
{
#if BOARD_IS_MAIN
    const st7735s_bus_t *lcd_bus;
#endif

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();

#if BOARD_IS_MAIN
    /* Configure the panel pins, SPI1 and its DMA first, so the backlight is driven
     * low at the earliest possible moment after reset. */
    lcd_bus = ST7735S_SPI_DMA_InitDefault();
#endif

    Board_SignalGPIO_Init();
    LED_GPIO_Init();
    USART1_Link_Init();
    USBPD_Hardware_Init();
    TIM1_INT_Init();
#if BOARD_IS_MAIN
    FAN_TIM2_Init();
    st7735s_init(lcd_bus, (st7735s_rotation_t)LCD_PANEL_ROTATION);
    lcd_init();
#else
    /* The sub board has no panel, but it must not reach the main loop earlier than
     * the main board, which spends exactly this long inside st7735s_init(). The
     * value comes from st7735s.h, so both boards stay in sync automatically. */
    CH32L103_DelayMs(ST7735S_INIT_BLOCKING_MS);
#endif

    I2C2_DMA_Init();

    /* Register every I2C client on the shared bus here, so main() only schedules
     * tasks and never runs driver initialization itself. */
    INA226_Service_Init(&g_i2c2_dma);
    (void)CH211_I2C_DMA_InitDefault(&g_i2c2_dma);
    (void)MP2980_I2C_DMA_InitDefault(&g_i2c2_dma);
}
