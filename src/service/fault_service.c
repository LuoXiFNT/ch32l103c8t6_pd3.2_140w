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

/* Fault sources, debounce state, I2C diagnostics, and public task flow.
 *
 * INA226 ALERT: both channels are wired to the MCU and sampled below, but the
 * chip-side alert is deliberately left unarmed (MASK_ENABLE and ALERT_LIMIT keep
 * their power-on values; see the note in ina226_service.c). Its sources duplicate
 * the MP2980 and CH211 protection, and clearing a latched alert needs an extra
 * MASK_ENABLE read that would land inside the PD timing-critical sequence, so
 * PD_FAULT_INA226_A_ALERT and PD_FAULT_INA226_C_ALERT stay 0 by design. */

#define FAULT_POLL_MS           100u
#define FAULT_GPIO_POLL_MS      20u
#define FAULT_CONFIRM_COUNT     2u
#define FAULT_GPIO_CONFIRM_COUNT 5u
#define FAULT_C_BLANK_MS        400u
#define FAULT_C_APDO_BLANK_MS   800u
#define FAULT_C_EPR_BLANK_MS    1200u
#define FAULT_BITS_MAX          16u

/* Board topology, as reflected by the three groups below:
 *
 *   LM5069 hot-swap  - shared by the A and C output paths, not by the MCU supply
 *   A path           - LMR33630 buck + CH217K limiter, monitored by INA226 channel A
 *   C path           - MP2980 + CH211, monitored by INA226 channel C
 *
 * Both output paths can run at full load at the same time, so there is no power
 * sharing to arbitrate: a fault on one path must leave the other untouched. The
 * masks are therefore built from three disjoint groups instead of one flat list. */
#define FAULT_SHARED_BITS       (PD_FAULT_LM5069_PG)
#define FAULT_A_ONLY_BITS       (PD_FAULT_LMR33630_PG | \
                                 PD_FAULT_CH217K_FLAG | \
                                 PD_FAULT_INA226_A_ALERT)
#define FAULT_C_ONLY_BITS       (PD_FAULT_MP2980_INT | \
                                 PD_FAULT_CH211_INT | \
                                 PD_FAULT_INA226_C_ALERT)

/* Bits sampled from the GPIO pins, per path; the shared input appears in both. */
#define FAULT_A_GPIO_BITS       (FAULT_SHARED_BITS | FAULT_A_ONLY_BITS)
#define FAULT_C_GPIO_BITS       (FAULT_SHARED_BITS | FAULT_C_ONLY_BITS)
#define FAULT_GPIO_BITS         (FAULT_SHARED_BITS | FAULT_A_ONLY_BITS | FAULT_C_ONLY_BITS)

/* Conditions that make the C panel show FAULT: its own GPIO sources plus the
 * decoded MP2980/CH211 status. The shared input counts because it feeds this path. */
#define FAULT_C_DISPLAY_BITS    (FAULT_C_GPIO_BITS | \
                                 PD_FAULT_MP2980_OCP | \
                                 PD_FAULT_MP2980_OTP | \
                                 PD_FAULT_MP2980_OVP | \
                                 PD_FAULT_MP2980_PG | \
                                 PD_FAULT_CH211_OTP | \
                                 PD_FAULT_CH211_OVP | \
                                 PD_FAULT_CH211_PG)

/* Conditions that must stop the C output and be reported to the sink. A-path-only
 * faults are deliberately absent: they must never disturb the C contract. */
#define FAULT_PD_BLOCKING_BITS  (PD_FAULT_LM5069_PG | \
                                 PD_FAULT_INA226_C_ALERT | \
                                 PD_FAULT_MP2980_OCP | \
                                 PD_FAULT_MP2980_OTP | \
                                 PD_FAULT_MP2980_OVP | \
                                 PD_FAULT_CH211_OTP | \
                                 PD_FAULT_CH211_OVP)

typedef enum {
    C_FAULT_I2C_IDLE = 0u,
    C_FAULT_I2C_READ_MP2980,
    C_FAULT_I2C_CLEAR_MP2980,
    C_FAULT_I2C_WAIT_MP2980_CLEAR,
    C_FAULT_I2C_READ_CH211
} c_fault_i2c_state_t;

/* Timing state. The tick handler (TIM1 interrupt) advances the counters and the
 * blanking timer; everything else below is owned by Fault_Task() in the main loop. */
static volatile uint16_t s_fault_poll_counter_ms = 0u;      /**< Time since the I2C poll. */
static volatile uint16_t s_fault_gpio_poll_counter_ms = 0u; /**< Time since the GPIO poll. */
static volatile bool s_fault_poll_due = true;               /**< I2C poll slot reached. */
static volatile bool s_fault_gpio_poll_due = true;          /**< GPIO poll slot reached. */
static volatile uint8_t s_fault_a = 0u;                     /**< Published A-channel fault. */
static volatile uint8_t s_fault_c = 0u;                     /**< Published C-channel fault. */
static volatile uint16_t s_c_fault_blank_ms = 0u;           /**< Monitoring blanking left. */

/* C-channel I2C fault monitor: reads the MP2980 and CH211 status registers, clears
 * the latched MP2980 interrupts, and debounces the combined result. */
static c_fault_i2c_state_t s_c_fault_i2c_state = C_FAULT_I2C_IDLE; /**< Chain position. */
static bool s_c_fault_i2c_active = false;                          /**< Debounced C fault from the I2C chain. */
static uint8_t s_c_fault_set_count = 0u;                           /**< Consecutive samples with a fault. */
static uint8_t s_c_fault_clear_count = 0u;                         /**< Consecutive samples without one. */
static bool s_c_fault_ready_prev = false;                          /**< Monitoring was active last pass. */
static uint8_t s_c_fault_discard_count = 0u;                       /**< Samples to drop after (re)start. */
static uint16_t s_c_fault_i2c_sample_bits = 0u;                    /**< Bits collected in this chain pass. */
static uint16_t s_c_fault_i2c_active_bits = 0u;                    /**< Debounced bits from the I2C chain. */

/* GPIO fault monitor: one debounce counter pair per fault bit. */
static uint16_t s_fault_gpio_active_bits = 0u;           /**< Debounced GPIO fault bits. */
static uint8_t s_fault_gpio_set_count[FAULT_BITS_MAX];   /**< Samples asserting each bit. */
static uint8_t s_fault_gpio_clear_count[FAULT_BITS_MAX]; /**< Samples releasing each bit. */

/* Contract tracking: a new contract restarts blanking and drops stale samples, so a
 * supply transition is never reported as a fault. */
static bool s_c_monitor_ready_prev = false; /**< C output was ready last pass. */
static uint16_t s_c_fault_last_mv = 0u;     /**< Contract voltage seen last pass. */
static uint16_t s_c_fault_last_ma = 0u;     /**< Contract current seen last pass. */


/**
 * @brief Advance the fault-monitor timers; called from the system tick.
 *
 * Only the counters and the blanking timer live here, so the tick handler stays
 * short. The polling flags are consumed by Fault_Task().
 *
 * @param elapsed_ms Milliseconds since the previous call.
 */
void Fault_TickMs(uint16_t elapsed_ms)
{
    s_fault_poll_counter_ms = (uint16_t)(s_fault_poll_counter_ms + elapsed_ms);
    if (s_fault_poll_counter_ms >= FAULT_POLL_MS) {
        s_fault_poll_counter_ms = 0u;
        s_fault_poll_due = true;
    }

    s_fault_gpio_poll_counter_ms = (uint16_t)(s_fault_gpio_poll_counter_ms + elapsed_ms);
    if (s_fault_gpio_poll_counter_ms >= FAULT_GPIO_POLL_MS) {
        s_fault_gpio_poll_counter_ms = 0u;
        s_fault_gpio_poll_due = true;
    }

    if (s_c_fault_blank_ms > elapsed_ms) {
        s_c_fault_blank_ms = (uint16_t)(s_c_fault_blank_ms - elapsed_ms);
    } else {
        s_c_fault_blank_ms = 0u;
    }
}

/** @brief Read one active-low fault input. */
static bool Fault_PinIsAsserted(GPIO_TypeDef *port, uint16_t pin)
{
    return (GPIO_ReadInputDataBit(port, pin) == Bit_RESET) ? 1u : 0u;
}

/** @brief Debounce GPIO fault bits and update their latched state. */
static void Fault_DebounceGpioBits(uint16_t sample_bits)
{
    uint8_t i;

    for (i = 0u; i < FAULT_BITS_MAX; i++) {
        uint16_t bit = (uint16_t)(1u << i);

        if ((FAULT_GPIO_BITS & bit) == 0u) {
            continue;
        }

        if ((sample_bits & bit) != 0u) {
            s_fault_gpio_clear_count[i] = 0u;
            if (s_fault_gpio_set_count[i] < FAULT_GPIO_CONFIRM_COUNT) {
                s_fault_gpio_set_count[i]++;
            }
            if (s_fault_gpio_set_count[i] >= FAULT_GPIO_CONFIRM_COUNT) {
                s_fault_gpio_active_bits = (uint16_t)(s_fault_gpio_active_bits | bit);
            }
        } else {
            s_fault_gpio_set_count[i] = 0u;
            if (s_fault_gpio_clear_count[i] < FAULT_GPIO_CONFIRM_COUNT) {
                s_fault_gpio_clear_count[i]++;
            }
            if (s_fault_gpio_clear_count[i] >= FAULT_GPIO_CONFIRM_COUNT) {
                s_fault_gpio_active_bits = (uint16_t)(s_fault_gpio_active_bits & (uint16_t)(~bit));
            }
        }
    }
}

/** @brief Sample all enabled active-low GPIO fault inputs. */
static uint16_t Fault_ReadGpioBits(bool c_monitor_enable)
{
    uint16_t bits = 0u;

    if (Fault_PinIsAsserted(LM5069_PG_GPIO_PORT, LM5069_PG_GPIO_PIN)) {
        bits |= PD_FAULT_LM5069_PG;
    }
    if (Fault_PinIsAsserted(LMR33630_PG_GPIO_PORT, LMR33630_PG_GPIO_PIN)) {
        bits |= PD_FAULT_LMR33630_PG;
    }
    if (Fault_PinIsAsserted(CH217K_FLAG_GPIO_PORT, CH217K_FLAG_GPIO_PIN)) {
        bits |= PD_FAULT_CH217K_FLAG;
    }
    if (Fault_PinIsAsserted(INA226_ALERT_A_GPIO_PORT, INA226_ALERT_A_GPIO_PIN)) {
        bits |= PD_FAULT_INA226_A_ALERT;
    }

    if (c_monitor_enable) {
        if (Fault_PinIsAsserted(MP2980_INT_GPIO_PORT, MP2980_INT_GPIO_PIN)) {
            bits |= PD_FAULT_MP2980_INT;
        }
        if (Fault_PinIsAsserted(CH211_INT_GPIO_PORT, CH211_INT_GPIO_PIN)) {
            bits |= PD_FAULT_CH211_INT;
        }
        if (Fault_PinIsAsserted(INA226_ALERT_C_GPIO_PORT, INA226_ALERT_C_GPIO_PIN)) {
            bits |= PD_FAULT_INA226_C_ALERT;
        }
    }

    return bits;
}

static void Fault_CI2CReset(void);

/** @brief Select the post-contract fault-monitoring blanking interval. */
static uint16_t Fault_CBlankMsForStatus(const pd_status_t *status)
{
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
static bool Fault_COutputReadyForCheck(const pd_status_t *status)
{
    if (status->connected == 0u) {
        return 0u;
    }

    if ((status->contract_mv != 0u) && (status->state == PD_STATE_READY)) {
        return 1u;
    }

    return 0u;
}

/** @brief Restart C-channel blanking after a contract or readiness change. */
static void Fault_UpdateCBlanking(const pd_status_t *status, uint8_t c_ready)
{
    if (c_ready == 0u) {
        s_c_monitor_ready_prev = false;
        s_c_fault_last_mv = 0u;
        s_c_fault_last_ma = 0u;
        s_c_fault_blank_ms = 0u;
        return;
    }

    if ((!s_c_monitor_ready_prev) ||
        (s_c_fault_last_mv != status->contract_mv) ||
        (s_c_fault_last_ma != status->contract_ma)) {
        s_c_fault_blank_ms = Fault_CBlankMsForStatus(status);
        Fault_CI2CReset();
    }

    s_c_monitor_ready_prev = true;
    s_c_fault_last_mv = status->contract_mv;
    s_c_fault_last_ma = status->contract_ma;
}

/** @brief Translate MP2980 interrupt status into PD fault bits. */
static uint16_t Fault_MP2980StatusBits(const mp2980_interrupt_status_t *status)
{
    uint16_t bits = 0u;

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
static uint16_t Fault_CH211StatusBits(const ch211_sys_status_t *status)
{
    uint16_t bits = 0u;

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
    s_c_fault_i2c_active = false;
    s_c_fault_set_count = 0u;
    s_c_fault_clear_count = 0u;
    s_c_fault_ready_prev = false;
    s_c_fault_discard_count = 0u;
    s_c_fault_i2c_sample_bits = 0u;
    s_c_fault_i2c_active_bits = 0u;
}

/** @brief Debounce one combined C-channel I2C fault sample. */
static void Fault_CI2CApplySample(uint16_t fault_bits)
{
    if (fault_bits != 0u) {
        s_c_fault_clear_count = 0u;
        if (s_c_fault_set_count < FAULT_CONFIRM_COUNT) {
            s_c_fault_set_count++;
        }
        if (s_c_fault_set_count >= FAULT_CONFIRM_COUNT) {
            s_c_fault_i2c_active = true;
            s_c_fault_i2c_active_bits = (uint16_t)(s_c_fault_i2c_active_bits | fault_bits);
        }
    } else {
        s_c_fault_set_count = 0u;
        if (s_c_fault_clear_count < FAULT_CONFIRM_COUNT) {
            s_c_fault_clear_count++;
        }
        if (s_c_fault_clear_count >= FAULT_CONFIRM_COUNT) {
            s_c_fault_i2c_active = false;
            s_c_fault_i2c_active_bits = 0u;
        }
    }
}

/** @brief Run the non-blocking MP2980/CH211 fault read sequence. */
static void Fault_CI2CTask(uint8_t enable)
{
    mp2980_interrupt_status_t mp_status;
    ch211_sys_status_t ch_status;
    mp2980_status_t mp_ret;
    ch211_status_t ch_ret;

    if (enable == 0u) {
        Fault_CI2CReset();
        return;
    }

    if (!s_c_fault_ready_prev) {
        s_c_fault_ready_prev = true;
        s_c_fault_discard_count = 1u;
        s_fault_poll_due = true;
    }

    if (s_c_fault_i2c_state == C_FAULT_I2C_IDLE) {
        if (!s_fault_poll_due) {
            return;
        }
        if ((!CH32_I2C_DMA_IsIdle(&g_i2c2_dma)) ||
            (MP2980_I2C_DMA_IsBusy()) ||
            (CH211_I2C_DMA_IsBusy())) {
            return;
        }
        s_fault_poll_due = false;
        s_c_fault_i2c_sample_bits = 0u;
        s_c_fault_i2c_state = C_FAULT_I2C_READ_MP2980;
    }

    if (s_c_fault_i2c_state == C_FAULT_I2C_READ_MP2980) {
        uint16_t mp_bits;

        mp_ret = MP2980_I2C_DMA_ReadInterruptStatus(&mp_status);
        if (mp_ret == MP2980_BUSY) {
            return;
        }
        if (mp_ret == MP2980_OK) {
            mp_bits = Fault_MP2980StatusBits(&mp_status);
            if (mp_bits != 0u) {
                s_c_fault_i2c_sample_bits = (uint16_t)(s_c_fault_i2c_sample_bits | mp_bits);
            }
            if (mp_status.raw != 0u) {
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
        if (!CH32_I2C_DMA_IsIdle(&g_i2c2_dma)) {
            return;
        }
        s_c_fault_i2c_state = C_FAULT_I2C_READ_CH211;
    }

    if (s_c_fault_i2c_state == C_FAULT_I2C_READ_CH211) {
        uint16_t ch_bits;

        ch_ret = CH211_I2C_DMA_GetSysStatus(&ch_status);
        if (ch_ret == CH211_BUSY) {
            return;
        }
        if (ch_ret == CH211_OK) {
            ch_bits = Fault_CH211StatusBits(&ch_status);
            if (ch_bits != 0u) {
                s_c_fault_i2c_sample_bits = (uint16_t)(s_c_fault_i2c_sample_bits | ch_bits);
            }
        }
        if (s_c_fault_discard_count != 0u) {
            s_c_fault_discard_count--;
        } else {
            Fault_CI2CApplySample(s_c_fault_i2c_sample_bits);
        }
        s_c_fault_i2c_state = C_FAULT_I2C_IDLE;
    }
}

/**
 * @brief Sample, debounce, aggregate, and publish the board faults.
 *
 * Runs one bounded step per call: the GPIO inputs are polled on their own slower
 * schedule, the C-channel I2C chain advances one transfer per pass, and the result is
 * published to PD, to the display flags and to the board link. Nothing here blocks.
 */
void Fault_Task(void)
{
    pd_status_t pd_status;
    uint8_t c_ready;
    bool c_monitor;
    uint8_t c_fault = 0u;
    uint8_t a_fault = 0u;
    uint16_t gpio_bits;
    uint16_t pd_fault_bits = 0u;
    uint16_t pd_blocking_bits;

    PD_GetStatus(&pd_status);
    c_ready = Fault_COutputReadyForCheck(&pd_status);
    Fault_UpdateCBlanking(&pd_status, c_ready);
    c_monitor = c_ready && (s_c_fault_blank_ms == 0u);

    if (s_fault_gpio_poll_due) {
        s_fault_gpio_poll_due = false;
        Fault_DebounceGpioBits(Fault_ReadGpioBits(c_monitor));
    }

    gpio_bits = s_fault_gpio_active_bits;
    pd_fault_bits = (uint16_t)(gpio_bits & FAULT_A_GPIO_BITS);
    a_fault = ((gpio_bits & FAULT_A_GPIO_BITS) != 0u) ? 1u : 0u;

    if (c_monitor) {
        Fault_CI2CTask(1u);
        pd_fault_bits = (uint16_t)(pd_fault_bits | (gpio_bits & FAULT_C_GPIO_BITS));
        if (s_c_fault_i2c_active) {
            pd_fault_bits |= s_c_fault_i2c_active_bits;
        }
        c_fault = ((pd_fault_bits & FAULT_C_DISPLAY_BITS) != 0u) ? 1u : 0u;
    } else {
        Fault_CI2CTask(0u);
        /* Without a C contract there is nothing to protect, but a shared input fault
         * is still worth reporting on this panel. */
        c_fault = ((gpio_bits & FAULT_SHARED_BITS) != 0u) ? 1u : 0u;
    }

    pd_blocking_bits = (uint16_t)(pd_fault_bits & FAULT_PD_BLOCKING_BITS);
    PD_UpdateFaults(pd_blocking_bits);

    s_fault_a = a_fault;
    s_fault_c = c_fault;
    BoardLink_SetFaults(s_fault_c, s_fault_a);

    {
        board_link_channel_t c_ch;
        board_link_channel_t a_ch;

        BoardLink_BuildChannels(&c_ch, &a_ch);
        BoardLink_SetLocalData(&c_ch, &a_ch, pd_fault_bits);
    }
}
