/**
 * @file fault_service.c
 * @brief Fault sampling, debounce, reporting, and recovery coordination.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "fault_service.h"

#include "board_config.h"
#include "board_link.h"
#include "ch211_i2c_dma_port.h"
#include "ch32l103_init.h"
#include "ina226.h"
#include "ina226_service.h"
#include "mp2980_i2c_dma_port.h"
#include "pd.h"

#include <stdbool.h>

/* Fault sources, debounce state, I2C diagnostics, and public task flow. */

#define FAULT_POLL_MS           100U
#define FAULT_GPIO_POLL_MS      20U
#define FAULT_CONFIRM_COUNT     2U
#define FAULT_GPIO_CONFIRM_COUNT 5U
#define FAULT_C_BLANK_MS        400U
#define FAULT_C_APDO_BLANK_MS   800U
#define FAULT_C_EPR_BLANK_MS    1200U
#define FAULT_BITS_MAX          16U

#define FAULT_A_GPIO_BITS       (PD_FAULT_LM5069_PG | \
                                 PD_FAULT_LMR33630_PG | \
                                 PD_FAULT_CH217K_FLAG | \
                                 PD_FAULT_INA226_A_ALERT)
#define FAULT_C_GPIO_BITS       (PD_FAULT_MP2980_INT | \
                                 PD_FAULT_CH211_INT | \
                                 PD_FAULT_INA226_C_ALERT)
#define FAULT_GPIO_BITS         (FAULT_A_GPIO_BITS | FAULT_C_GPIO_BITS)
#define FAULT_C_DISPLAY_BITS    (PD_FAULT_LM5069_PG | \
                                 PD_FAULT_MP2980_INT | \
                                 PD_FAULT_CH211_INT | \
                                 PD_FAULT_INA226_C_ALERT | \
                                 PD_FAULT_MP2980_OCP | \
                                 PD_FAULT_MP2980_OTP | \
                                 PD_FAULT_MP2980_OVP | \
                                 PD_FAULT_MP2980_PG | \
                                 PD_FAULT_CH211_OTP | \
                                 PD_FAULT_CH211_OVP | \
                                 PD_FAULT_CH211_PG)
#define FAULT_PD_BLOCKING_BITS  (FAULT_A_GPIO_BITS | \
                                 PD_FAULT_INA226_C_ALERT | \
                                 PD_FAULT_MP2980_OCP | \
                                 PD_FAULT_MP2980_OTP | \
                                 PD_FAULT_MP2980_OVP | \
                                 PD_FAULT_CH211_OTP | \
                                 PD_FAULT_CH211_OVP)

typedef enum {
    C_FAULT_I2C_IDLE = 0U,
    C_FAULT_I2C_READ_MP2980,
    C_FAULT_I2C_CLEAR_MP2980,
    C_FAULT_I2C_WAIT_MP2980_CLEAR,
    C_FAULT_I2C_READ_CH211
} c_fault_i2c_state_t;

static volatile u16 s_fault_poll_counter_ms = 0U;
static volatile u16 s_fault_gpio_poll_counter_ms = 0U;
static volatile u8 s_fault_poll_due = 1U;
static volatile u8 s_fault_gpio_poll_due = 1U;
static volatile u8 s_fault_a = 0U;
static volatile u8 s_fault_c = 0U;
static volatile u16 s_c_fault_blank_ms = 0U;

static c_fault_i2c_state_t s_c_fault_i2c_state = C_FAULT_I2C_IDLE;
static u8 s_c_fault_i2c_active = 0U;
static u8 s_c_fault_set_count = 0U;
static u8 s_c_fault_clear_count = 0U;
static u8 s_c_fault_ready_prev = 0U;
static u8 s_c_fault_discard_count = 0U;
static u16 s_c_fault_i2c_sample_bits = 0U;
static u16 s_c_fault_i2c_active_bits = 0U;
static u16 s_fault_gpio_active_bits = 0U;
static u8 s_fault_gpio_set_count[FAULT_BITS_MAX];
static u8 s_fault_gpio_clear_count[FAULT_BITS_MAX];
static u8 s_c_monitor_ready_prev = 0U;
static u16 s_c_fault_last_mv = 0U;
static u16 s_c_fault_last_ma = 0U;


void Fault_TickMs(u16 elapsed_ms)
{
    s_fault_poll_counter_ms = (u16)(s_fault_poll_counter_ms + elapsed_ms);
    if (s_fault_poll_counter_ms >= FAULT_POLL_MS) {
        s_fault_poll_counter_ms = 0U;
        s_fault_poll_due = 1U;
    }

    s_fault_gpio_poll_counter_ms = (u16)(s_fault_gpio_poll_counter_ms + elapsed_ms);
    if (s_fault_gpio_poll_counter_ms >= FAULT_GPIO_POLL_MS) {
        s_fault_gpio_poll_counter_ms = 0U;
        s_fault_gpio_poll_due = 1U;
    }

    if (s_c_fault_blank_ms > elapsed_ms) {
        s_c_fault_blank_ms = (u16)(s_c_fault_blank_ms - elapsed_ms);
    } else {
        s_c_fault_blank_ms = 0U;
    }
}

/** @brief Read one active-low fault input. */
static u8 Fault_PinIsAsserted(GPIO_TypeDef *port, u16 pin)
{
    return (GPIO_ReadInputDataBit(port, pin) == Bit_RESET) ? 1U : 0U;
}

/** @brief Debounce GPIO fault bits and update their latched state. */
static void Fault_DebounceGpioBits(u16 sample_bits)
{
    u8 i;

    for (i = 0U; i < FAULT_BITS_MAX; i++) {
        u16 bit = (u16)(1U << i);

        if ((FAULT_GPIO_BITS & bit) == 0U) {
            continue;
        }

        if ((sample_bits & bit) != 0U) {
            s_fault_gpio_clear_count[i] = 0U;
            if (s_fault_gpio_set_count[i] < FAULT_GPIO_CONFIRM_COUNT) {
                s_fault_gpio_set_count[i]++;
            }
            if (s_fault_gpio_set_count[i] >= FAULT_GPIO_CONFIRM_COUNT) {
                s_fault_gpio_active_bits = (u16)(s_fault_gpio_active_bits | bit);
            }
        } else {
            s_fault_gpio_set_count[i] = 0U;
            if (s_fault_gpio_clear_count[i] < FAULT_GPIO_CONFIRM_COUNT) {
                s_fault_gpio_clear_count[i]++;
            }
            if (s_fault_gpio_clear_count[i] >= FAULT_GPIO_CONFIRM_COUNT) {
                s_fault_gpio_active_bits = (u16)(s_fault_gpio_active_bits & (u16)(~bit));
            }
        }
    }
}

/** @brief Sample all enabled active-low GPIO fault inputs. */
static u16 Fault_ReadGpioBits(u8 c_monitor_enable)
{
    u16 bits = 0U;

    if (Fault_PinIsAsserted(LM5069_PG_GPIO_PORT, LM5069_PG_GPIO_PIN) != 0U) {
        bits |= PD_FAULT_LM5069_PG;
    }
    if (Fault_PinIsAsserted(LMR33630_PG_GPIO_PORT, LMR33630_PG_GPIO_PIN) != 0U) {
        bits |= PD_FAULT_LMR33630_PG;
    }
    if (Fault_PinIsAsserted(CH217K_FLAG_GPIO_PORT, CH217K_FLAG_GPIO_PIN) != 0U) {
        bits |= PD_FAULT_CH217K_FLAG;
    }
    if (Fault_PinIsAsserted(INA226_ALERT_A_GPIO_PORT, INA226_ALERT_A_GPIO_PIN) != 0U) {
        bits |= PD_FAULT_INA226_A_ALERT;
    }

    if (c_monitor_enable != 0U) {
        if (Fault_PinIsAsserted(MP2980_INT_GPIO_PORT, MP2980_INT_GPIO_PIN) != 0U) {
            bits |= PD_FAULT_MP2980_INT;
        }
        if (Fault_PinIsAsserted(CH211_INT_GPIO_PORT, CH211_INT_GPIO_PIN) != 0U) {
            bits |= PD_FAULT_CH211_INT;
        }
        if (Fault_PinIsAsserted(INA226_ALERT_C_GPIO_PORT, INA226_ALERT_C_GPIO_PIN) != 0U) {
            bits |= PD_FAULT_INA226_C_ALERT;
        }
    }

    return bits;
}

/** @brief Publish the current A-channel fault indicator. */
static void Fault_A(u8 active)
{
    s_fault_a = active;
}

/** @brief Publish the current C-channel fault indicator. */
static void Fault_C(u8 active)
{
    s_fault_c = active;
}

/** @brief Reset the C-channel I2C fault polling state machine. */
static void Fault_CI2CReset(void);

/** @brief Select the post-contract fault-monitoring blanking interval. */
static u16 Fault_CBlankMsForStatus(const pd_status_t *status)
{
    if (status == 0) {
        return FAULT_C_BLANK_MS;
    }

    switch (status->active_supply_type) {
    case PD_SUPPLY_TYPE_EPR_FIXED:
    case PD_SUPPLY_TYPE_EPR_AVS:
        return FAULT_C_EPR_BLANK_MS;

    case PD_SUPPLY_TYPE_SPR_PPS:
    case PD_SUPPLY_TYPE_SPR_AVS:
        return FAULT_C_APDO_BLANK_MS;

    default:
        return FAULT_C_BLANK_MS;
    }
}

/** @brief Check whether the C output is ready for fault sampling. */
static u8 Fault_COutputReadyForCheck(const pd_status_t *status)
{
    if ((status == 0) || (status->connected == 0U)) {
        return 0U;
    }

    if ((status->contract_mv != 0U) && (status->state == PD_STATE_READY)) {
        return 1U;
    }

    return 0U;
}

/** @brief Restart C-channel blanking after a contract or readiness change. */
static void Fault_UpdateCBlanking(const pd_status_t *status, u8 c_ready)
{
    if ((status == 0) || (c_ready == 0U)) {
        s_c_monitor_ready_prev = 0U;
        s_c_fault_last_mv = 0U;
        s_c_fault_last_ma = 0U;
        s_c_fault_blank_ms = 0U;
        return;
    }

    if ((s_c_monitor_ready_prev == 0U) ||
        (s_c_fault_last_mv != status->contract_mv) ||
        (s_c_fault_last_ma != status->contract_ma)) {
        s_c_fault_blank_ms = Fault_CBlankMsForStatus(status);
        Fault_CI2CReset();
    }

    s_c_monitor_ready_prev = 1U;
    s_c_fault_last_mv = status->contract_mv;
    s_c_fault_last_ma = status->contract_ma;
}

/** @brief Translate MP2980 interrupt status into PD fault bits. */
static u16 Fault_MP2980StatusBits(const mp2980_interrupt_status_t *status)
{
    u16 bits = 0U;

    if (status == 0) {
        return 0U;
    }

    if (status->ocp != false) {
        bits |= PD_FAULT_MP2980_OCP;
    }
    if (status->otp != false) {
        bits |= PD_FAULT_MP2980_OTP;
    }
    if (status->ovp != false) {
        bits |= PD_FAULT_MP2980_OVP;
    }
    if (status->png != false) {
        bits |= PD_FAULT_MP2980_PG;
    }

    return bits;
}

/** @brief Translate CH211 system status into PD fault bits. */
static u16 Fault_CH211StatusBits(const ch211_sys_status_t *status)
{
    u16 bits = 0U;

    if (status == 0) {
        return 0U;
    }

    if (status->over_temperature_reset != false) {
        bits |= PD_FAULT_CH211_OTP;
    }
    if (status->vbus_over_voltage != false) {
        bits |= PD_FAULT_CH211_OVP;
    }
    if ((status->vbus_ready == false) || (status->vbus_exist == false)) {
        bits |= PD_FAULT_CH211_PG;
    }

    return bits;
}

/** @brief Reset the C-channel I2C fault sampling state machine. */
static void Fault_CI2CReset(void)
{
    s_c_fault_i2c_state = C_FAULT_I2C_IDLE;
    s_c_fault_i2c_active = 0U;
    s_c_fault_set_count = 0U;
    s_c_fault_clear_count = 0U;
    s_c_fault_ready_prev = 0U;
    s_c_fault_discard_count = 0U;
    s_c_fault_i2c_sample_bits = 0U;
    s_c_fault_i2c_active_bits = 0U;
}

/** @brief Debounce one combined C-channel I2C fault sample. */
static void Fault_CI2CApplySample(u16 fault_bits)
{
    if (fault_bits != 0U) {
        s_c_fault_clear_count = 0U;
        if (s_c_fault_set_count < FAULT_CONFIRM_COUNT) {
            s_c_fault_set_count++;
        }
        if (s_c_fault_set_count >= FAULT_CONFIRM_COUNT) {
            s_c_fault_i2c_active = 1U;
            s_c_fault_i2c_active_bits = (u16)(s_c_fault_i2c_active_bits | fault_bits);
        }
    } else {
        s_c_fault_set_count = 0U;
        if (s_c_fault_clear_count < FAULT_CONFIRM_COUNT) {
            s_c_fault_clear_count++;
        }
        if (s_c_fault_clear_count >= FAULT_CONFIRM_COUNT) {
            s_c_fault_i2c_active = 0U;
            s_c_fault_i2c_active_bits = 0U;
        }
    }
}

/** @brief Run the non-blocking MP2980/CH211 fault read sequence. */
static void Fault_CI2CTask(u8 enable)
{
    mp2980_interrupt_status_t mp_status;
    ch211_sys_status_t ch_status;
    mp2980_status_t mp_ret;
    ch211_status_t ch_ret;

    if (enable == 0U) {
        Fault_CI2CReset();
        return;
    }

    if (s_c_fault_ready_prev == 0U) {
        s_c_fault_ready_prev = 1U;
        s_c_fault_discard_count = 1U;
        s_fault_poll_due = 1U;
    }

    if (s_c_fault_i2c_state == C_FAULT_I2C_IDLE) {
        if (s_fault_poll_due == 0U) {
            return;
        }
        if ((CH32_I2C_DMA_IsIdle(&g_i2c2_dma) == 0U) ||
            (MP2980_I2C_DMA_IsBusy() != 0U) ||
            (CH211_I2C_DMA_IsBusy() != 0U)) {
            return;
        }
        s_fault_poll_due = 0U;
        s_c_fault_i2c_sample_bits = 0U;
        s_c_fault_i2c_state = C_FAULT_I2C_READ_MP2980;
    }

    if (s_c_fault_i2c_state == C_FAULT_I2C_READ_MP2980) {
        u16 mp_bits;

        mp_ret = MP2980_I2C_DMA_ReadInterruptStatus(&mp_status);
        if (mp_ret == MP2980_BUSY) {
            return;
        }
        if (mp_ret == MP2980_OK) {
            mp_bits = Fault_MP2980StatusBits(&mp_status);
            if (mp_bits != 0U) {
                s_c_fault_i2c_sample_bits = (u16)(s_c_fault_i2c_sample_bits | mp_bits);
            }
            if (mp_status.raw != 0U) {
                s_c_fault_i2c_state = C_FAULT_I2C_CLEAR_MP2980;
                return;
            }
        }
        s_c_fault_i2c_state = C_FAULT_I2C_READ_CH211;
    }

    if (s_c_fault_i2c_state == C_FAULT_I2C_CLEAR_MP2980) {
        mp_ret = MP2980_I2C_DMA_ClearInterruptStatus();
        if (mp_ret == MP2980_BUSY) {
            return;
        }
        if (mp_ret == MP2980_OK) {
            s_c_fault_i2c_state = C_FAULT_I2C_WAIT_MP2980_CLEAR;
        } else {
            s_c_fault_i2c_state = C_FAULT_I2C_READ_CH211;
        }
    }

    if (s_c_fault_i2c_state == C_FAULT_I2C_WAIT_MP2980_CLEAR) {
        if (CH32_I2C_DMA_IsIdle(&g_i2c2_dma) == 0U) {
            return;
        }
        s_c_fault_i2c_state = C_FAULT_I2C_READ_CH211;
    }

    if (s_c_fault_i2c_state == C_FAULT_I2C_READ_CH211) {
        u16 ch_bits;

        ch_ret = CH211_I2C_DMA_ReadSysStatus(&ch_status);
        if (ch_ret == CH211_BUSY) {
            return;
        }
        if (ch_ret == CH211_OK) {
            ch_bits = Fault_CH211StatusBits(&ch_status);
            if (ch_bits != 0U) {
                s_c_fault_i2c_sample_bits = (u16)(s_c_fault_i2c_sample_bits | ch_bits);
            }
        }
        if (s_c_fault_discard_count != 0U) {
            s_c_fault_discard_count--;
        } else {
            Fault_CI2CApplySample(s_c_fault_i2c_sample_bits);
        }
        s_c_fault_i2c_state = C_FAULT_I2C_IDLE;
    }
}

void Fault_Task(void)
{
    pd_status_t pd_status;
    u8 c_ready;
    u8 c_monitor;
    u8 c_fault = 0U;
    u8 a_fault = 0U;
    u16 gpio_bits;
    u16 pd_fault_bits = 0U;
    u16 pd_blocking_bits;

    PD_GetStatus(&pd_status);
    c_ready = Fault_COutputReadyForCheck(&pd_status);
    Fault_UpdateCBlanking(&pd_status, c_ready);
    c_monitor = ((c_ready != 0U) && (s_c_fault_blank_ms == 0U)) ? 1U : 0U;

    if (s_fault_gpio_poll_due != 0U) {
        s_fault_gpio_poll_due = 0U;
        Fault_DebounceGpioBits(Fault_ReadGpioBits(c_monitor));
    }

    gpio_bits = s_fault_gpio_active_bits;
    pd_fault_bits = (u16)(gpio_bits & FAULT_A_GPIO_BITS);
    a_fault = ((gpio_bits & FAULT_A_GPIO_BITS) != 0U) ? 1U : 0U;

    if (c_monitor != 0U) {
        Fault_CI2CTask(1U);
        pd_fault_bits = (u16)(pd_fault_bits | (gpio_bits & FAULT_C_GPIO_BITS));
        if (s_c_fault_i2c_active != 0U) {
            pd_fault_bits |= s_c_fault_i2c_active_bits;
        }
        c_fault = ((pd_fault_bits & FAULT_C_DISPLAY_BITS) != 0U) ? 1U : 0U;
    } else {
        Fault_CI2CTask(0U);
    }

    pd_blocking_bits = (u16)(pd_fault_bits & FAULT_PD_BLOCKING_BITS);
    PD_UpdateFaults(pd_blocking_bits);

    Fault_A(a_fault);
    Fault_C(c_fault);
    BoardLink_SetFaults(s_fault_c, s_fault_a);

    {
        board_link_channel_t c_ch;
        board_link_channel_t a_ch;

        BoardLink_BuildChannels(&c_ch, &a_ch);
        BoardLink_SetLocalData(&c_ch, &a_ch, pd_fault_bits);
    }
}
