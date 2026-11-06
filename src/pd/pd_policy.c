/**
 * @file pd_policy.c
 * @brief PD policy coordination, alert/status responses, and EPR scheduling.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#include "pd_internal.h"

/* Message encoding helpers and policy response scheduling. */

uint32_t pd_get_u32_le(const uint8_t *buf)
{
    return ((uint32_t)buf[0]) |
           ((uint32_t)buf[1] << 8u) |
           ((uint32_t)buf[2] << 16u) |
           ((uint32_t)buf[3] << 24u);
}

void pd_put_u32_le(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)(value & 0xFFu);
    buf[1] = (uint8_t)((value >> 8u) & 0xFFu);
    buf[2] = (uint8_t)((value >> 16u) & 0xFFu);
    buf[3] = (uint8_t)((value >> 24u) & 0xFFu);
}

void pd_put_u16_le(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFFu);
    buf[1] = (uint8_t)((value >> 8u) & 0xFFu);
}

uint32_t pd_vdm_version_bits(uint8_t sop)
{
    uint8_t revision = (sop == UPD_SOP0) ? s_pd.pd_revision :
                                          s_cable_pd_revision;

    return revision ? PD_VDM_VERSION_20 : 0ul;
}

uint32_t pd_make_structured_vdm(uint16_t svid,
                                uint8_t cmd,
                                uint32_t cmdt,
                                uint8_t sop)
{
    return ((uint32_t)svid << 16u) |
           PD_VDM_STRUCTURED |
           pd_vdm_version_bits(sop) |
           (cmdt & PD_VDM_CMDT_MASK) |
           (uint32_t)(cmd & PD_VDM_CMD_MASK);
}

uint16_t pd_current_limit_ma(void)
{
    return s_cable_current_ma;
}

bool pd_tx_start_revision(void)
{
    uint8_t payload[4];

    pd_put_u32_le(payload, PD_REVISION_DATA_OBJECT);
    return pd_tx_start(DEF_TYPE_REVISION, payload, sizeof(payload));
}

bool pd_tx_start_epr_mode(uint8_t action, uint8_t data)
{
    uint8_t payload[4];
    uint32_t eprmdo;

    eprmdo = (((uint32_t)action & PD_EPR_MODE_FIELD_MASK) << PD_EPR_MODE_ACTION_SHIFT) |
             (((uint32_t)data & PD_EPR_MODE_FIELD_MASK) << PD_EPR_MODE_DATA_SHIFT);
    pd_put_u32_le(payload, eprmdo);
    return pd_tx_start(DEF_TYPE_EPR_MODE, payload, sizeof(payload));
}

bool pd_tx_start_extended_control(uint8_t type, uint8_t data)
{
    uint8_t payload[PD_EXT_CONTROL_DATA_LEN];

    payload[0] = type;
    payload[1] = data;
    return pd_tx_start_extended(PD_EXT_EXTENDED_CONTROL,
                                payload,
                                PD_EXT_CONTROL_DATA_LEN);
}

bool pd_fault_active(void)
{
    return (s_fault_bits != 0u);
}

/** @brief Translate internal fault bits into a USB-PD Alert Data Object. */
static uint32_t pd_fault_alert_do(uint16_t fault_bits)
{
    uint32_t ado = 0ul;

    if ((fault_bits & (PD_FAULT_MP2980_OCP |
                       PD_FAULT_INA226_C_ALERT |
                       PD_FAULT_INA226_A_ALERT)) != 0u) {
        ado |= PD_ALERT_ADO_OCP;
    }
    if ((fault_bits & (PD_FAULT_MP2980_OTP |
                       PD_FAULT_CH211_OTP)) != 0u) {
        ado |= PD_ALERT_ADO_OTP;
    }
    if ((fault_bits & (PD_FAULT_MP2980_OVP |
                       PD_FAULT_CH211_OVP)) != 0u) {
        ado |= PD_ALERT_ADO_OVP;
    }
    if ((fault_bits & (PD_FAULT_MP2980_PG |
                       PD_FAULT_CH211_PG |
                       PD_FAULT_LM5069_PG |
                       PD_FAULT_LMR33630_PG)) != 0u) {
        ado |= PD_ALERT_ADO_SOURCE_INPUT;
    }
    if ((fault_bits & (PD_FAULT_MP2980_INT |
                       PD_FAULT_CH211_INT |
                       PD_FAULT_CH217K_FLAG)) != 0u) {
        ado |= PD_ALERT_ADO_OPERATING_COND;
    }
    if ((fault_bits & (PD_FAULT_MP2980_PG |
                       PD_FAULT_CH211_PG |
                       PD_FAULT_LM5069_PG |
                       PD_FAULT_LMR33630_PG |
                       PD_FAULT_CH217K_FLAG)) != 0u) {
        ado |= PD_ALERT_ADO_POWER_STATE;
    }
    if ((ado == 0ul) && (fault_bits != 0u)) {
        ado = PD_ALERT_ADO_OPERATING_COND;
    }

    return ado;
}

/** @brief Translate internal fault bits into USB-PD status event bits. */
static uint8_t pd_fault_status_events(uint16_t fault_bits)
{
    uint8_t events = 0u;

    if ((fault_bits & (PD_FAULT_MP2980_OCP |
                       PD_FAULT_INA226_C_ALERT |
                       PD_FAULT_INA226_A_ALERT)) != 0u) {
        events |= PD_STATUS_EVENT_OCP;
    }
    if ((fault_bits & (PD_FAULT_MP2980_OTP |
                       PD_FAULT_CH211_OTP)) != 0u) {
        events |= PD_STATUS_EVENT_OTP;
    }
    if ((fault_bits & (PD_FAULT_MP2980_OVP |
                       PD_FAULT_CH211_OVP)) != 0u) {
        events |= PD_STATUS_EVENT_OVP;
    }
    if ((fault_bits & (PD_FAULT_MP2980_PG |
                       PD_FAULT_CH211_PG |
                       PD_FAULT_LM5069_PG |
                       PD_FAULT_LMR33630_PG)) != 0u) {
        events |= PD_STATUS_EVENT_SOURCE_INPUT;
    }
    if ((fault_bits & (PD_FAULT_MP2980_INT |
                       PD_FAULT_CH211_INT |
                       PD_FAULT_CH217K_FLAG)) != 0u) {
        events |= PD_STATUS_EVENT_OPERATING;
    }
    if ((fault_bits & (PD_FAULT_MP2980_PG |
                       PD_FAULT_MP2980_INT |
                       PD_FAULT_CH211_PG |
                       PD_FAULT_CH211_INT |
                       PD_FAULT_LM5069_PG |
                       PD_FAULT_LMR33630_PG |
                       PD_FAULT_CH217K_FLAG)) != 0u) {
        events |= PD_STATUS_EVENT_POWER;
    }

    return events;
}

bool pd_tx_start_alert(void)
{
    uint8_t payload[4];
    uint32_t ado;

    if (!pd_fault_active()) {
        return 0u;
    }

    ado = pd_fault_alert_do(s_fault_bits);
    pd_put_u32_le(payload, ado);
    return pd_tx_start(DEF_TYPE_ALERT, payload, sizeof(payload));
}

bool pd_tx_start_status(void)
{
    uint8_t payload[PD_EXT_STATUS_DATA_LEN];
    uint8_t events = pd_fault_status_events(s_fault_bits);

    memset(payload, 0, sizeof(payload));
    payload[1] = (uint8_t)(PD_STATUS_INPUT_EXTERNAL | PD_STATUS_INPUT_DC);
    payload[3] = events;
    payload[4] = ((events & PD_STATUS_EVENT_OTP) != 0u) ?
                 PD_STATUS_TEMP_OVER :
                 PD_STATUS_TEMP_NORMAL;

    return pd_tx_start_extended(DEF_TYPE_GET_STATUS_R, payload, sizeof(payload));
}

bool pd_tx_start_status_chunk(uint8_t chunk_number)
{
    uint8_t payload[PD_EXT_STATUS_DATA_LEN];
    uint8_t events = pd_fault_status_events(s_fault_bits);

    memset(payload, 0, sizeof(payload));
    payload[1] = (uint8_t)(PD_STATUS_INPUT_EXTERNAL | PD_STATUS_INPUT_DC);
    payload[3] = events;
    payload[4] = ((events & PD_STATUS_EVENT_OTP) != 0u) ?
                 PD_STATUS_TEMP_OVER :
                 PD_STATUS_TEMP_NORMAL;

    return pd_tx_start_extended_chunk(DEF_TYPE_GET_STATUS_R,
                                      payload,
                                      PD_EXT_STATUS_DATA_LEN,
                                      chunk_number);
}

bool pd_tx_start_pps_status(void)
{
    uint8_t payload[PD_PPS_STATUS_DATA_LEN];
    uint16_t mv = (s_power.output_mv != 0u) ? s_power.output_mv : s_power.target_mv;
    uint16_t ma = s_pd.contract_ma;
    uint16_t voltage_20mv;
    uint16_t current_50ma;

    if (mv == 0u) {
        mv = PD_VSAFE5V_MV;
    }
    if (ma == 0u) {
        ma = pd_effective_pps_ma();
    }

    voltage_20mv = (uint16_t)((mv + 10u) / 20u);
    current_50ma = (uint16_t)((ma + 25u) / 50u);
    if (current_50ma > 0xFFu) {
        current_50ma = 0xFFu;
    }

    pd_put_u16_le(&payload[0], voltage_20mv);
    payload[2] = (uint8_t)current_50ma;
    payload[3] = 0u;
    return pd_tx_start_extended(PD_EXT_PPS_STATUS, payload, PD_PPS_STATUS_DATA_LEN);
}

void pd_alert_task(void)
{
    pd_tx_result_t result;

    if (s_fault_alert_tx_active) {
        if (pd_tx_take_result(&result) != 0u) {
            s_fault_alert_tx_active = false;
            if (result == PD_TX_RESULT_OK) {
                if (s_fault_bits == 0u) {
                    s_fault_alert_pending = false;
                    s_fault_alert_sent = false;
                } else if (s_fault_alert_tx_bits == s_fault_bits) {
                    s_fault_alert_pending = false;
                    s_fault_alert_sent = true;
                } else {
                    s_fault_alert_pending = true;
                    s_fault_alert_sent = true;
                }
            } else {
                s_fault_alert_pending = (s_fault_bits != 0u);
            }
            s_fault_alert_tx_bits = 0u;
        }
        return;
    }

    if ((!s_fault_alert_pending) ||
        (!pd_fault_active()) ||
        (!s_pd.connected) ||
        (s_pd.state != PD_STATE_READY) ||
        (s_rx_msg_pending) ||
        (pd_is_power_transition_state()) ||
        (!pd_tx_is_idle()) ||
        (s_tx.result != PD_TX_RESULT_NONE)) {
        return;
    }

    if (pd_tx_start_alert()) {
        s_fault_alert_tx_active = true;
        s_fault_alert_tx_bits = s_fault_bits;
    }
}

/** @brief Check whether the discovered cable permits EPR operation. */
static bool pd_epr_cable_ready(void)
{
    return ((s_cable_epr_capable) && (s_cable_current_ma >= PD_5A_CURRENT_MA) &&
            (s_cable_max_vbus_mv >= PD_EPR_MIN_CABLE_VBUS_MV));
}

bool pd_epr_start_mode_response(uint8_t action, uint8_t data);

/** @brief Progress the cable check required before acknowledging EPR entry. */
static uint8_t pd_epr_enter_wait_done(void)
{
    if (pd_fault_active()) {
        s_epr_fail_reason = PD_EPR_FAIL_FAULT;
        return pd_epr_start_mode_response(PD_EPR_MODE_ACTION_FAILED,
                                          PD_EPR_MODE_FAIL_UNABLE);
    }
    if (pd_epr_cable_ready()) {
        return pd_epr_start_mode_response(PD_EPR_MODE_ACTION_ENTER_OK, 0u);
    }
    if ((s_cable_state == PD_CABLE_DONE) ||
        (s_epr_enter_wait_ms >= PD_EPR_ENTER_CABLE_WAIT_MS)) {
        s_epr_fail_reason = PD_EPR_FAIL_UNAVAILABLE;
        return pd_epr_start_mode_response(PD_EPR_MODE_ACTION_FAILED,
                                          PD_EPR_MODE_FAIL_CABLE);
    }
    if (s_cable_state == PD_CABLE_IDLE) {
        pd_cable_start_discovery();
    }
    if (s_cable_state == PD_CABLE_FAILED) {
        return 0u;
    }
    return 0u;
}


bool pd_epr_start_mode_response(uint8_t action, uint8_t data)
{
    s_epr_last_tx_action = action;
    s_epr_last_tx_data = data;
    if (action == PD_EPR_MODE_ACTION_FAILED) {
        pd_debug_note_epr_entry_failure(s_epr_fail_reason);
    }
    if (pd_tx_start_epr_mode(action, data)) {
        s_epr_mode_tx_active = true;
        s_ext_response_tx_active = true;
        return true;
    }
    s_epr_fail_reason = PD_EPR_FAIL_BUSY;
    return false;
}


void pd_ext_response_task(uint8_t elapsed_ms)
{
    pd_tx_result_t result;

    if (((s_epr_enter_wait_active) ||
         ((s_epr_mode_tx_active) &&
          (s_epr_last_tx_action == PD_EPR_MODE_ACTION_ENTER_ACK))) &&
        (s_epr_enter_wait_ms < PD_EPR_ENTER_CABLE_WAIT_MS)) {
        uint16_t enter_ms = (uint16_t)(s_epr_enter_wait_ms + elapsed_ms);
        s_epr_enter_wait_ms = (enter_ms > PD_EPR_ENTER_CABLE_WAIT_MS) ?
                              PD_EPR_ENTER_CABLE_WAIT_MS : enter_ms;
    }

    if (s_epr_enter_wait_active) {
        if ((!pd_tx_is_idle()) ||
            (s_tx.result != PD_TX_RESULT_NONE) ||
            (s_rx_msg_pending) ||
            (pd_is_power_transition_state())) {
            return;
        }
        if (pd_epr_enter_wait_done() != 0u) {
            s_epr_enter_wait_active = false;
            s_epr_enter_wait_ms = 0u;
        }
        return;
    }

    if (!s_ext_response_tx_active) {
        return;
    }
    if (pd_tx_take_result(&result) != 0u) {
        if (s_epr_mode_tx_active) {
            if (result == PD_TX_RESULT_OK) {
                s_epr_mode_tx_active = false;
                s_ext_response_tx_active = false;
                if (s_epr_last_tx_action == PD_EPR_MODE_ACTION_ENTER_ACK) {
                    if ((!pd_epr_cable_ready()) &&
                        ((s_cable_state == PD_CABLE_IDLE) ||
                         (s_cable_state == PD_CABLE_FAILED))) {
                        pd_cable_start_discovery();
                    }
                    s_epr_enter_wait_active = true;
                    return;
                }
                if (s_epr_last_tx_action == PD_EPR_MODE_ACTION_ENTER_OK) {
                    s_epr_mode_active = true;
                    s_epr_source_cap_pending = true;
                    s_epr_source_cap_retry = 0u;
                    s_epr_fail_reason = PD_EPR_FAIL_NONE;
                } else if (s_epr_last_tx_action == PD_EPR_MODE_ACTION_FAILED) {
                    pd_epr_exit_to_spr_recovery();
                } else if (s_epr_last_tx_action == PD_EPR_MODE_ACTION_EXIT) {
                    s_epr_mode_active = false;
                    s_epr_source_cap_pending = false;
                    s_epr_source_cap_tx_active = false;
                    s_epr_source_cap_retry = 0u;
                    s_source_cap_refresh_pending = true;
                    s_source_cap_refresh_retries = 1u;
                    s_source_cap_refresh_wait_ms = 0u;
                }
            } else {
                s_epr_fail_reason = PD_EPR_FAIL_TX_FAIL;
                pd_epr_exit_to_spr_recovery();
            }
        } else if (s_epr_source_cap_tx_active) {
            s_epr_source_cap_tx_active = false;
            if (result == PD_TX_RESULT_OK) {
                pd_debug_note_epr_chunk_phase(
                    (s_debug_epr_chunk_phase == 4u) ? 5u : 2u);
                s_epr_source_cap_retry = 0u;
            } else {
                pd_debug_note_epr_chunk_phase(0x0Eu);
                s_epr_fail_reason = PD_EPR_FAIL_TX_FAIL;
                s_epr_source_cap_retry = 0u;
            }
        }
        s_ext_response_tx_active = false;
    }
}


void pd_epr_source_cap_task(void)
{
    if ((!s_epr_source_cap_pending) ||
        (!s_pd.connected) ||
        (!s_epr_mode_active) ||
        (!pd_epr_is_available()) ||
        (pd_fault_active()) ||
        (s_rx_msg_pending) ||
        (pd_is_power_transition_state()) ||
        (!pd_tx_is_idle()) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        ((s_pd.state != PD_STATE_READY) &&
         (s_pd.state != PD_STATE_WAIT_REQUEST))) {
        return;
    }

    if (pd_tx_start_epr_source_cap_ext()) {
        s_epr_source_cap_pending = false;
        s_epr_source_cap_tx_active = true;
        s_ext_response_tx_active = true;
    }
}
