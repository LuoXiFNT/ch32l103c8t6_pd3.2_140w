/**
 * @file pd_power.c
 * @brief Non-blocking MP2980/CH211 power transition state machine.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#include "pd_internal.h"

/* Voltage request sequencing, settle timing, readiness, and fault recovery. */

void pd_power_abort(void)
{
    MP2980_I2C_DMA_Abort();
}

void pd_power_request(bool enable, uint16_t mv, uint16_t ma)
{
    uint16_t base_mv;
    uint16_t delta_mv;
    uint16_t margin_ms = PD_POWER_SETTLE_MARGIN_MS;
    uint16_t min_ms = PD_POWER_SETTLE_MIN_MS;
    bool downshift_discharge = false;

    if (s_debug_power_request_count != 0xFFu) {
        s_debug_power_request_count++;
    }

    s_power.target_enable = enable;
    s_power.target_mv = mv;
    s_power.target_ma = ma;
    s_power.wait_ms = 0u;
    s_power.downshift_failed = false;

    if (enable) {
        base_mv = (s_power.output_mv != 0u) ? s_power.output_mv : PD_VSAFE5V_MV;
        delta_mv = (mv > base_mv) ? (uint16_t)(mv - base_mv) : (uint16_t)(base_mv - mv);
        if ((s_power.output_mv != 0u) &&
            (s_power.output_mv > mv) &&
            ((uint16_t)(s_power.output_mv - mv) >= PD_POWER_DOWNSHIFT_MIN_MV)) {
            downshift_discharge = true;
        }
        if ((s_power.output_mv != 0u) &&
            (s_active_supply_type == PD_SUPPLY_TYPE_SPR_PPS)) {
            if (delta_mv <= PD_PPS_SMALL_STEP_MAX_MV) {
                margin_ms = PD_APDO_STEP_SETTLE_MARGIN_MS;
                min_ms = PD_APDO_STEP_SETTLE_MIN_MS;
            } else {
                margin_ms = PD_PPS_LARGE_SETTLE_MARGIN_MS;
            }
        } else if ((s_power.output_mv != 0u) &&
                   (delta_mv <= PD_AVS_SMALL_STEP_MAX_MV) &&
                   ((s_active_supply_type == PD_SUPPLY_TYPE_SPR_AVS) ||
                    (s_active_supply_type == PD_SUPPLY_TYPE_EPR_AVS))) {
            margin_ms = PD_APDO_STEP_SETTLE_MARGIN_MS;
            min_ms = PD_APDO_STEP_SETTLE_MIN_MS;
        }
        s_power.settle_ms = (uint16_t)((delta_mv + PD_POWER_SLEW_MV_PER_MS - 1u) /
                                       PD_POWER_SLEW_MV_PER_MS);
        s_power.settle_ms = (uint16_t)(s_power.settle_ms + margin_ms);
        if (s_power.settle_ms < min_ms) {
            s_power.settle_ms = min_ms;
        }
    } else {
        s_power.settle_ms = 0u;
        s_power.output_mv = 0u;
    }
    s_power.downshift_discharge = downshift_discharge;

    if (enable) {
        s_power.state = (s_power.output_mv == 0u) ?
                        PD_POWER_DISABLE_VBUS_DISCHARGE :
                        PD_POWER_SET_MP2980;
    } else {
        s_power.state = PD_POWER_CLOSE_HVCP;
    }
}

void pd_power_recover_to_vsafe5v(void)
{
    MP2980_I2C_DMA_Abort();
    CH211_I2C_DMA_Abort();
    if (s_power.target_enable) {
        pd_power_request(true, PD_VSAFE5V_MV, PD_VSAFE5V_MA);
    }
}

bool pd_power_is_ready_on(void)
{
    return (s_power.state == PD_POWER_READY_ON);
}


bool pd_power_request_is(bool enable, uint16_t mv)
{
    if (s_power.target_enable != enable) {
        return false;
    }
    if ((enable != 0u) && (s_power.target_mv != mv)) {
        return false;
    }
    return true;
}


bool pd_power_has_error(void)
{
    return (s_power.state == PD_POWER_ERROR);
}

uint16_t pd_power_vbus_min_mv(void)
{
    return s_power.vbus_min_report_mv;
}

/** @brief Calculate the timeout for the active voltage transition. */
static uint16_t pd_power_wait_timeout_ms(void)
{
    uint16_t timeout_ms = (uint16_t)(s_power.settle_ms + PD_POWER_EXTRA_TIMEOUT_MS);

    return (timeout_ms < PD_POWER_TIMEOUT_MS) ? PD_POWER_TIMEOUT_MS : timeout_ms;
}

/** @brief Check whether VBUS has fallen far enough for a downshift. */
static bool pd_power_vbus_ready_for_downshift(void)
{
    uint16_t vbus_mv = pd_sense_vbus_mv();
    uint16_t ready_mv;

    if (vbus_mv == 0u) {
        return true;
    }

    ready_mv = (uint16_t)(s_power.target_mv + PD_POWER_VBUS_READY_MARGIN_MV);
    return (vbus_mv <= ready_mv);
}

/** @brief Check whether measured VBUS is within the target tolerance. */
static bool pd_power_vbus_at_target(void)
{
    uint16_t vbus_mv = pd_sense_vbus_mv();
    uint16_t low_mv;
    uint16_t high_mv;

    if (vbus_mv == 0u) {
        return false;
    }

    low_mv = (s_power.target_mv > PD_POWER_VBUS_TARGET_TOLERANCE_MV) ?
             (uint16_t)(s_power.target_mv - PD_POWER_VBUS_TARGET_TOLERANCE_MV) :
             0u;
    high_mv = (s_power.target_mv <
               (uint16_t)(0xFFFFu - PD_POWER_VBUS_TARGET_TOLERANCE_MV)) ?
              (uint16_t)(s_power.target_mv + PD_POWER_VBUS_TARGET_TOLERANCE_MV) :
              0xFFFFu;
    return ((vbus_mv >= low_mv) && (vbus_mv <= high_mv));
}

/** @brief Publish the completed power request and enter the ready state. */
static void pd_power_finish_ready_on(void)
{
    s_power.output_mv = s_power.target_mv;
    s_power.downshift_discharge = false;
    s_power.downshift_failed = false;
    s_power.state = PD_POWER_READY_ON;
}

/** @brief Latch a power-state error and reset state timing. */
static void pd_power_enter_error(void)
{
    if (s_power.state != PD_POWER_ERROR) {
        pd_debug_note_failure(PD_DEBUG_EVENT_POWER_ERROR);
    }
    s_power.state = PD_POWER_ERROR;
    s_power.last_state = PD_POWER_ERROR;
    s_power.state_elapsed_ms = 0u;
}


void pd_power_task(uint8_t elapsed_ms)
{
    uint16_t vbus_mv;

    s_power.wait_ms = (uint16_t)(s_power.wait_ms + elapsed_ms);
    MP2980_I2C_DMA_Task();
    CH211_I2C_DMA_Task();

    /* Track the deepest output sag while the port is powering a load: a heavy load can
     * latch itself off after a short dip that the instantaneous reading no longer shows. */
    if (s_power.target_enable) {
        vbus_mv = pd_sense_vbus_mv();
        if ((vbus_mv != 0u) &&
            ((s_power.vbus_min_mv == 0u) || (vbus_mv < s_power.vbus_min_mv))) {
            s_power.vbus_min_mv = vbus_mv;
        }
        s_power.vbus_min_window_ms =
            (uint16_t)(s_power.vbus_min_window_ms + elapsed_ms);
        if (s_power.vbus_min_window_ms >= PD_VBUS_MIN_WINDOW_MS) {
            if (s_power.vbus_min_mv != 0u) {
                s_power.vbus_min_report_mv = s_power.vbus_min_mv;
            }
            s_power.vbus_min_mv = 0u;
            s_power.vbus_min_window_ms = 0u;
        }
    }

    if (s_power.state != s_power.last_state) {
        s_power.last_state = s_power.state;
        s_power.state_enter_ms = s_power.wait_ms;
        s_power.state_elapsed_ms = 0u;
        if (s_power.state == PD_POWER_READY_ON) {
            pd_debug_note_event(PD_DEBUG_EVENT_POWER_READY);
        } else if (s_power.state == PD_POWER_ERROR) {
            pd_debug_note_failure(PD_DEBUG_EVENT_POWER_ERROR);
        }
    } else if (s_power.state_elapsed_ms < (uint16_t)(0xFFFFu - elapsed_ms)) {
        s_power.state_elapsed_ms = (uint16_t)(s_power.state_elapsed_ms + elapsed_ms);
    }
    if ((s_power.state != PD_POWER_OFF) &&
        (s_power.state != PD_POWER_READY_OFF) &&
        (s_power.state != PD_POWER_READY_ON) &&
        (s_power.state != PD_POWER_ERROR)) {
        if (s_power.state_elapsed_ms >= PD_POWER_STATE_TIMEOUT_MS) {
            pd_power_enter_error();
            /* Drop driver-level transactions as well as the PD state.  A
             * lost I2C completion must not keep the next contract or the
             * INA226 monitor waiting forever. */
            MP2980_I2C_DMA_Abort();
            CH211_I2C_DMA_Abort();
        }
    }

    switch (s_power.state) {
    case PD_POWER_OFF:
        pd_power_request(false, 0u, 0u);
        break;

    case PD_POWER_CLOSE_HVCP:
        if (CH211_I2C_DMA_SetHvcpLow(true) == CH211_OK) {
            s_power.state = PD_POWER_WAIT_HVCP_CLOSED;
        }
        break;

    case PD_POWER_WAIT_HVCP_CLOSED:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_power.state = PD_POWER_DISABLE_MP2980;
        }
        break;

    case PD_POWER_DISABLE_MP2980:
        if (MP2980_I2C_DMA_DisablePower() == MP2980_OK) {
            s_power.state = PD_POWER_WAIT_MP2980_DISABLED;
        }
        break;

    case PD_POWER_WAIT_MP2980_DISABLED:
        if (MP2980_I2C_DMA_IsBusy() == 0u) {
            s_power.state = PD_POWER_ENABLE_MP2980_DISCHARGE;
        }
        break;

    case PD_POWER_ENABLE_MP2980_DISCHARGE:
        if (MP2980_I2C_DMA_SetDischarge(true) == MP2980_OK) {
            s_power.state = PD_POWER_WAIT_MP2980_DISCHARGE_ON;
        }
        break;

    case PD_POWER_WAIT_MP2980_DISCHARGE_ON:
        if (MP2980_I2C_DMA_IsBusy() == 0u) {
            s_power.state = PD_POWER_ENABLE_VBUS_DISCHARGE;
        }
        break;

    case PD_POWER_ENABLE_VBUS_DISCHARGE:
        if (CH211_I2C_DMA_SetVbusDischarge(true) == CH211_OK) {
            s_power.state = PD_POWER_WAIT_VBUS_DISCHARGE_ON;
        }
        break;

    case PD_POWER_WAIT_VBUS_DISCHARGE_ON:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_power.state = PD_POWER_VBUS_DISCHARGE;
        }
        break;

    case PD_POWER_VBUS_DISCHARGE:
        if ((s_power.target_enable) ||
            ((uint16_t)(s_power.wait_ms - s_power.state_enter_ms) >= PD_POWER_VBUS_DISCHARGE_MS)) {
            s_power.state = PD_POWER_DISABLE_VBUS_DISCHARGE;
        }
        break;

    case PD_POWER_DISABLE_VBUS_DISCHARGE:
        if (CH211_I2C_DMA_SetVbusDischarge(false) == CH211_OK) {
            s_power.state = PD_POWER_WAIT_VBUS_DISCHARGE_OFF;
        }
        break;

    case PD_POWER_WAIT_VBUS_DISCHARGE_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_power.state = PD_POWER_DISABLE_MP2980_DISCHARGE;
        }
        break;

    case PD_POWER_DISABLE_MP2980_DISCHARGE:
        if (MP2980_I2C_DMA_SetDischarge(false) == MP2980_OK) {
            s_power.state = PD_POWER_WAIT_MP2980_DISCHARGE_OFF;
        }
        break;

    case PD_POWER_WAIT_MP2980_DISCHARGE_OFF:
        if (MP2980_I2C_DMA_IsBusy() == 0u) {
            s_power.state = (s_power.target_enable) ?
                            PD_POWER_SET_MP2980 :
                            PD_POWER_READY_OFF;
        }
        break;

    case PD_POWER_SET_MP2980:
        if (MP2980_I2C_DMA_SetOutputVoltageCurrentMv(s_power.target_mv,
                                                     s_power.target_ma) == MP2980_OK) {
            s_power.wait_ms = 0u;
            s_power.state = (s_power.downshift_discharge) ?
                            PD_POWER_ENABLE_DOWNSHIFT_DISCHARGE :
                            PD_POWER_WAIT_MP2980;
        }
        break;

    case PD_POWER_WAIT_MP2980:
        if (s_power.wait_ms >= pd_power_wait_timeout_ms()) {
            pd_power_enter_error();
            MP2980_I2C_DMA_Abort();
            CH211_I2C_DMA_Abort();
            break;
        }
        if (s_power.wait_ms < s_power.settle_ms) {
            break;
        }
        if ((s_power.output_mv != 0u) &&
            (pd_power_vbus_at_target())) {
            pd_power_finish_ready_on();
        } else if (s_power.output_mv == 0u) {
            s_power.state = PD_POWER_OPEN_HVCP;
        }
        break;

    case PD_POWER_OPEN_HVCP:
        if (CH211_I2C_DMA_EnableHvcpAuto(true) == CH211_OK) {
            s_power.state = PD_POWER_WAIT_HVCP_OPEN;
        }
        break;

    case PD_POWER_WAIT_HVCP_OPEN:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            pd_power_finish_ready_on();
        }
        break;

    case PD_POWER_ENABLE_DOWNSHIFT_DISCHARGE:
        if (CH211_I2C_DMA_SetVbusDischarge(true) == CH211_OK) {
            s_power.state = PD_POWER_WAIT_DOWNSHIFT_DISCHARGE_ON;
        }
        break;

    case PD_POWER_WAIT_DOWNSHIFT_DISCHARGE_ON:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_power.wait_ms = 0u;
            s_power.state = PD_POWER_DOWNSHIFT_DISCHARGE;
        }
        break;

    case PD_POWER_DOWNSHIFT_DISCHARGE:
        if (s_power.wait_ms >= pd_power_wait_timeout_ms()) {
            s_power.downshift_failed = true;
            s_power.state = PD_POWER_DISABLE_DOWNSHIFT_DISCHARGE;
            CH211_I2C_DMA_Abort();
            break;
        }
        if (s_power.wait_ms < s_power.settle_ms) {
            break;
        }
        if (pd_power_vbus_ready_for_downshift()) {
            s_power.state = PD_POWER_DISABLE_DOWNSHIFT_DISCHARGE;
        }
        break;

    case PD_POWER_DISABLE_DOWNSHIFT_DISCHARGE:
        if (CH211_I2C_DMA_SetVbusDischarge(false) == CH211_OK) {
            s_power.state = PD_POWER_WAIT_DOWNSHIFT_DISCHARGE_OFF;
        }
        break;

    case PD_POWER_WAIT_DOWNSHIFT_DISCHARGE_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            if (s_power.downshift_failed) {
                pd_power_enter_error();
            } else if (pd_power_vbus_at_target()) {
                pd_power_finish_ready_on();
            }
        }
        break;

    case PD_POWER_ERROR:
        break;

    default:
        break;
    }
}
