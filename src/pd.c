/**
 * @file pd.c
 * @brief Core USB Power Delivery context, message dispatch, and public API.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "pd_internal.h"

/* Persistent protocol context, request staging, diagnostics, and task flow. */

__attribute__((aligned(4))) uint8_t s_pd_rx_buf[PD_RX_BUF_SIZE];
__attribute__((aligned(4))) uint8_t s_pd_rx_dma_buf[PD_RX_BUF_SIZE];
__attribute__((aligned(4))) uint8_t s_pd_rx_queued_buf[PD_RX_BUF_SIZE];
__attribute__((aligned(4))) uint8_t s_pd_tx_buf[PD_TX_BUF_SIZE];
uint8_t s_pd_goodcrc_buf[PD_GOODCRC_LEN];

static const pd_fixed_pdo_t s_default_pdos[PD_SOURCE_MAX_FIXED_PDOS] = {
    { 5000U,  PD_DEFAULT_CURRENT_MA },
    { 9000U,  PD_DEFAULT_CURRENT_MA },
    { 12000U, PD_DEFAULT_CURRENT_MA },
    { 15000U, PD_DEFAULT_CURRENT_MA },
    { 20000U, PD_5A_CURRENT_MA }
};

pd_fixed_pdo_t s_pdos[PD_SOURCE_MAX_FIXED_PDOS];
uint8_t s_pdo_count;
volatile uint8_t s_rx_msg_pending;
volatile uint8_t s_rx_msg_queued;
volatile uint8_t s_rx_msg_len;
volatile uint8_t s_rx_msg_queued_len;
volatile uint8_t s_rx_sop;
volatile uint8_t s_rx_msg_queued_sop;
volatile uint16_t s_tick_ms;
volatile uint8_t s_goodcrc_tx_active;
volatile uint8_t s_goodcrc_tx_pending;
volatile uint8_t s_goodcrc_watchdog_ms;
volatile uint8_t s_goodcrc_sop;
pd_context_t s_pd;
pd_cc_detect_t s_cc;
pd_tx_context_t s_tx;
pd_power_context_t s_power;
volatile uint8_t s_debug_power_request_reason;
volatile uint8_t s_debug_power_request_enable;
volatile uint8_t s_debug_power_request_count;
volatile uint8_t s_debug_goodcrc_rx_msg_type;
volatile uint8_t s_debug_goodcrc_rx_msg_id;
volatile uint8_t s_debug_goodcrc_fail_msg_type;
volatile uint8_t s_debug_goodcrc_fail_msg_id;
volatile uint8_t s_debug_goodcrc_fail_stage;
volatile uint8_t s_debug_rx_hard_reset_count;
volatile uint8_t s_debug_rx_hard_reset_state;
volatile uint8_t s_debug_rx_hard_reset_power_state;
pd_frontend_state_t s_frontend_state;
pd_frontend_state_t s_frontend_last_state;
pd_cable_state_t s_cable_state;
pd_power_control_cb_t s_power_cb;
static pd_vbus_sense_cb_t s_vbus_sense_cb;
uint16_t s_frontend_wait_ms;
uint16_t s_cable_wait_ms;
uint16_t s_pd_tx_clk_cnt;
uint16_t s_pd_rx_clk_cnt;
uint8_t s_cable_msg_id;
uint8_t s_cable2_msg_id;
uint8_t s_cable_pd_revision;
uint8_t s_last_msg_type;
uint8_t s_last_num_do;
uint8_t s_last_extended;
uint8_t s_last_request_pdo;
uint8_t s_last_rdo_pos;
uint32_t s_last_rdo;
uint32_t s_last_spr_rdo;
uint16_t s_last_request_mv;
uint16_t s_last_request_ma;
uint8_t s_last_request_is_pps;
uint8_t s_last_request_is_epr;
uint8_t s_pps_active;
uint8_t s_epr_mode_active;
uint8_t s_active_supply_type;
uint8_t s_epr_mode_tx_active;
uint8_t s_epr_source_cap_pending;
uint8_t s_epr_source_cap_tx_active;
uint8_t s_epr_source_cap_retry;
uint8_t s_epr_advertise_ready;
uint8_t s_epr_last_rx_action;
uint8_t s_epr_last_tx_action;
uint8_t s_epr_last_tx_data;
uint8_t s_epr_fail_reason;
uint8_t s_epr_enter_wait_active;
uint16_t s_epr_enter_wait_ms;
uint8_t s_supports_5a;
uint8_t s_source_cap_refresh_pending;
uint8_t s_source_cap_refresh_retries;
uint16_t s_source_cap_refresh_wait_ms;
uint16_t s_cable_current_ma;
uint16_t s_cable_max_vbus_mv;
uint8_t s_cable_epr_capable;
uint32_t s_last_vdm_header;
uint32_t s_last_cable_vdo;
uint8_t s_last_vdm_sop;
uint8_t s_last_vdm_cmd;
uint8_t s_last_vdm_cmdt;
uint8_t s_cable_retry;
volatile uint8_t s_pd_reset_pending;
uint16_t s_fault_bits;
uint8_t s_fault_alert_pending;
uint8_t s_fault_alert_sent;
uint8_t s_fault_alert_tx_active;
uint16_t s_fault_alert_tx_bits;
uint8_t s_ext_response_tx_active;
uint8_t s_partner_revision;
static uint8_t s_state_tx_active;
static uint8_t s_fault_recovery_pending;
static uint8_t s_epr_keepalive_pending;
static uint8_t s_pd_revision_locked;
typedef enum {
    PD_REVISION_QUERY_IDLE = 0U,
    PD_REVISION_QUERY_DELAY,
    PD_REVISION_QUERY_TX,
    PD_REVISION_QUERY_WAIT,
    PD_REVISION_QUERY_DONE
} pd_revision_query_state_t;
static pd_revision_query_state_t s_revision_query_state;
static uint8_t s_revision_query_tx_active;
static uint16_t s_revision_query_wait_ms;
static uint8_t s_staged_request_valid;
static uint8_t s_staged_request_pdo;
static uint8_t s_staged_request_is_pps;
static uint8_t s_staged_request_is_epr;
static uint8_t s_staged_request_supply_type;
static uint16_t s_staged_request_mv;
static uint16_t s_staged_request_ma;
static uint32_t s_staged_request_rdo;
static uint32_t s_staged_request_spr_rdo;

static uint8_t s_deferred_request_valid;
static uint8_t s_deferred_request_len;
static uint8_t s_deferred_request_sop;
static uint8_t s_deferred_request_buf[PD_RX_BUF_SIZE];
static uint8_t s_replaying_deferred_request;
static uint8_t s_rx_last_msg_id[3];
static uint8_t s_rx_last_msg_id_valid[3];
static uint8_t s_debug_last_event;
static uint8_t s_debug_last_failure;
static uint8_t s_debug_rx_msg_id;
static uint8_t s_debug_rx_msg_id_valid;
static uint8_t s_debug_rx_sop;
static uint16_t s_debug_event_seq;
static uint16_t s_debug_request_count;
static uint16_t s_debug_accept_ok_count;
static uint16_t s_debug_accept_fail_count;
static uint16_t s_debug_power_ready_count;
static uint16_t s_debug_power_error_count;
static uint16_t s_debug_ps_rdy_ok_count;
static uint16_t s_debug_ps_rdy_fail_count;
static uint16_t s_debug_soft_reset_count;
static uint16_t s_debug_hard_reset_count;
uint8_t s_debug_epr_chunk_phase;
uint8_t s_debug_goodcrc_recover_count;
static uint8_t s_debug_epr_entry_fail_reason;
static uint8_t s_debug_epr_entry_fail_state;
static uint16_t s_source_epr_keepalive_ms;
static uint16_t s_source_pps_comm_ms;

/* Diagnostic event counters and EPR entry tracing. */
void pd_debug_note_event(uint8_t event)
{
    s_debug_last_event = event;
    s_debug_event_seq++;

    switch (event) {
    case PD_DEBUG_EVENT_RX_REQUEST:
    case PD_DEBUG_EVENT_RX_EPR_REQUEST:
        s_debug_request_count++;
        break;
    case PD_DEBUG_EVENT_ACCEPT_OK:
        s_debug_accept_ok_count++;
        break;
    case PD_DEBUG_EVENT_ACCEPT_FAIL:
        s_debug_accept_fail_count++;
        break;
    case PD_DEBUG_EVENT_POWER_READY:
        s_debug_power_ready_count++;
        break;
    case PD_DEBUG_EVENT_POWER_ERROR:
        s_debug_power_error_count++;
        break;
    case PD_DEBUG_EVENT_PS_RDY_OK:
        s_debug_ps_rdy_ok_count++;
        break;
    case PD_DEBUG_EVENT_PS_RDY_FAIL:
        s_debug_ps_rdy_fail_count++;
        break;
    case PD_DEBUG_EVENT_SOFT_RESET:
        s_debug_soft_reset_count++;
        break;
    case PD_DEBUG_EVENT_HARD_RESET:
        s_debug_hard_reset_count++;
        break;
    default:
        break;
    }
}

void pd_debug_note_failure(uint8_t event)
{
    s_debug_last_failure = event;
    pd_debug_note_event(event);
}

void pd_debug_note_epr_chunk_phase(uint8_t phase)
{
    s_debug_epr_chunk_phase = phase;
}

void pd_debug_begin_epr_entry(void)
{
    s_debug_last_failure = PD_DEBUG_EVENT_NONE;
    s_debug_epr_chunk_phase = 0U;
    s_debug_epr_entry_fail_reason = PD_EPR_FAIL_NONE;
    s_debug_epr_entry_fail_state = (uint8_t)s_pd.state;
    pd_debug_note_event(PD_DEBUG_EVENT_EPR_MODE_ENTER);
}

void pd_debug_note_epr_entry_failure(uint8_t reason)
{
    s_debug_epr_entry_fail_reason = reason;
    s_debug_epr_entry_fail_state = (uint8_t)s_pd.state;
}

/** @brief Reset internal PD diagnostic counters and trace state. */
static void pd_debug_reset(void)
{
    s_debug_last_event = PD_DEBUG_EVENT_NONE;
    s_debug_last_failure = PD_DEBUG_EVENT_NONE;
    s_debug_rx_msg_id = 0U;
    s_debug_rx_msg_id_valid = 0U;
    s_debug_rx_sop = PD_RX_SOP0;
    s_debug_event_seq = 0U;
    s_debug_request_count = 0U;
    s_debug_accept_ok_count = 0U;
    s_debug_accept_fail_count = 0U;
    s_debug_power_ready_count = 0U;
    s_debug_power_error_count = 0U;
    s_debug_ps_rdy_ok_count = 0U;
    s_debug_ps_rdy_fail_count = 0U;
    s_debug_soft_reset_count = 0U;
    s_debug_hard_reset_count = 0U;
    s_debug_epr_chunk_phase = 0U;
    s_debug_epr_entry_fail_reason = PD_EPR_FAIL_NONE;
    s_debug_epr_entry_fail_state = 0U;
}

void pd_reset_source_comm_timers(void)
{
    s_source_epr_keepalive_ms = 0U;
    s_source_pps_comm_ms = 0U;
}

/* Deferred and staged Request handling used across policy transitions. */
/** @brief Discard the deferred Request message, if any. */
static void pd_clear_deferred_request(void)
{
    s_deferred_request_valid = 0U;
    s_deferred_request_len = 0U;
    s_deferred_request_sop = PD_RX_SOP0;
}

/** @brief Save the current Request message for replay after a transition. */
static void pd_defer_current_request(void)
{
    uint8_t i;

    if ((s_rx_msg_len < 10U) || (s_rx_msg_len > PD_RX_BUF_SIZE)) {
        return;
    }
    for (i = 0U; i < s_rx_msg_len; i++) {
        s_deferred_request_buf[i] = s_pd_rx_buf[i];
    }
    s_deferred_request_len = s_rx_msg_len;
    s_deferred_request_sop = s_rx_sop;
    s_deferred_request_valid = 1U;
    pd_debug_note_event(PD_DEBUG_EVENT_REQUEST_DEFERRED);
}

/** @brief Map an RX SOP identifier to an internal message-ID slot. */
static uint8_t pd_rx_sop_index(uint8_t sop)
{
    if (sop == PD_RX_SOP1_HRST) {
        return 1U;
    }
    if (sop == PD_RX_SOP2_CRST) {
        return 2U;
    }
    return 0U;
}

/** @brief Clear duplicate-detection message IDs for all SOP types. */
static void pd_clear_rx_message_ids(void)
{
    uint8_t i;

    for (i = 0U; i < 3U; i++) {
        s_rx_last_msg_id[i] = 0U;
        s_rx_last_msg_id_valid[i] = 0U;
    }
}

/** @brief Check whether the current RX message repeats its previous ID. */
static uint8_t pd_rx_message_is_duplicate(uint8_t msg_type)
{
    uint8_t index;
    uint8_t msg_id;

    if (msg_type == DEF_TYPE_SOFT_RESET) {
        return 0U;
    }

    index = pd_rx_sop_index(s_rx_sop);
    msg_id = (uint8_t)((s_pd_rx_buf[1] >> 1U) & 0x07U);
    if ((s_rx_last_msg_id_valid[index] != 0U) &&
        (s_rx_last_msg_id[index] == msg_id)) {
        return 1U;
    }
    s_rx_last_msg_id[index] = msg_id;
    s_rx_last_msg_id_valid[index] = 1U;
    return 0U;
}

/** @brief Clear the staged Request values waiting for policy approval. */
static void pd_clear_staged_request(void)
{
    s_staged_request_valid = 0U;
    s_staged_request_pdo = 0U;
    s_staged_request_is_pps = 0U;
    s_staged_request_is_epr = 0U;
    s_staged_request_supply_type = PD_SUPPLY_TYPE_NONE;
    s_staged_request_mv = 0U;
    s_staged_request_ma = 0U;
    s_staged_request_rdo = 0UL;
    s_staged_request_spr_rdo = 0UL;
}

/** @brief Store a validated Request until its power transition completes. */
static void pd_stage_request(uint16_t mv, uint16_t ma, uint8_t pdo_index)
{
    s_staged_request_valid = 1U;
    s_staged_request_pdo = pdo_index;
    s_staged_request_is_pps = s_last_request_is_pps;
    s_staged_request_is_epr = s_last_request_is_epr;
    s_staged_request_supply_type = s_active_supply_type;
    s_staged_request_mv = mv;
    s_staged_request_ma = ma;
    s_staged_request_rdo = s_last_rdo;
    s_staged_request_spr_rdo = s_last_spr_rdo;
}

/** @brief Apply the staged Request to the active PD contract. */
static uint8_t pd_commit_staged_request(void)
{
    if (s_staged_request_valid == 0U) {
        return 0U;
    }

    s_last_request_pdo = s_staged_request_pdo;
    s_last_request_mv = s_staged_request_mv;
    s_last_request_ma = s_staged_request_ma;
    s_last_request_is_pps = s_staged_request_is_pps;
    s_last_request_is_epr = s_staged_request_is_epr;
    s_active_supply_type = s_staged_request_supply_type;
    s_last_rdo = s_staged_request_rdo;
    s_last_spr_rdo = s_staged_request_spr_rdo;
    s_pd.contract_mv = s_staged_request_mv;
    s_pd.contract_ma = s_staged_request_ma;
    s_pd.selected_pdo = s_staged_request_pdo;
    s_pps_active = s_staged_request_is_pps;
    pd_clear_staged_request();
    return 1U;
}

/** @brief Check whether the active contract uses an EPR supply type. */
static uint8_t pd_active_contract_is_epr(void)
{
    return ((s_epr_mode_active != 0U) ||
            (s_last_request_is_epr != 0U) ||
            (s_active_supply_type == PD_SUPPLY_TYPE_EPR_FIXED) ||
            (s_active_supply_type == PD_SUPPLY_TYPE_EPR_AVS)) ? 1U : 0U;
}

void pd_epr_exit_to_spr_recovery(void)
{
    if (s_epr_mode_tx_active != 0U) {
        s_ext_response_tx_active = 0U;
    }
    s_epr_keepalive_pending = 0U;
    s_epr_mode_active = 0U;
    s_epr_mode_tx_active = 0U;
    s_epr_source_cap_pending = 0U;
    s_epr_source_cap_tx_active = 0U;
    s_epr_source_cap_retry = 0U;
    s_epr_enter_wait_active = 0U;
    s_epr_enter_wait_ms = 0U;
    s_last_request_is_epr = 0U;
    if ((s_active_supply_type == PD_SUPPLY_TYPE_EPR_FIXED) ||
        (s_active_supply_type == PD_SUPPLY_TYPE_EPR_AVS)) {
        s_active_supply_type = PD_SUPPLY_TYPE_NONE;
    }
    if (s_epr_fail_reason == PD_EPR_FAIL_NONE) {
        s_epr_fail_reason = PD_EPR_FAIL_SPR_FALLBACK;
    }
}

/* Core protocol state transitions and revision negotiation. */
/** @brief Identify an EPR KeepAlive extended-control request. */
static uint8_t pd_is_epr_keepalive_request(uint8_t msg_type,
                                           uint8_t extended,
                                           uint8_t num_do)
{
    uint16_t ext_header;
    uint16_t data_len;
    uint8_t chunk_number;

    if ((msg_type != PD_EXT_EXTENDED_CONTROL) ||
        (extended == 0U) ||
        (num_do == 0U) ||
        (s_rx_msg_len < 10U)) {
        return 0U;
    }

    ext_header = (uint16_t)s_pd_rx_buf[2] |
                 ((uint16_t)s_pd_rx_buf[3] << 8U);
    data_len = (uint16_t)(ext_header & PD_EXT_HEADER_DATA_SIZE_MASK);
    chunk_number = (uint8_t)((ext_header >> PD_EXT_HEADER_CHUNK_NUM_SHIFT) &
                             PD_EXT_HEADER_CHUNK_NUM_MASK);
    if (((ext_header & PD_EXT_HEADER_REQUEST_CHUNK) != 0U) ||
        (chunk_number != 0U) ||
        (data_len == 0U) ||
        (data_len > PD_EXT_CONTROL_DATA_LEN)) {
        return 0U;
    }

    if ((s_pd_rx_buf[4] == PD_EXT_CTRL_EPR_KEEP_ALIVE) &&
        ((data_len < PD_EXT_CONTROL_DATA_LEN) || (s_pd_rx_buf[5] == 0U))) {
        return 1U;
    }
    return 0U;
}

/** @brief Check whether an auxiliary policy owner has a pending TX result. */
static uint8_t pd_tx_result_has_aux_owner(void)
{
    if ((s_ext_response_tx_active != 0U) ||
        (s_fault_alert_tx_active != 0U) ||
        (s_revision_query_tx_active != 0U)) {
        return 1U;
    }
    if ((s_cable_state == PD_CABLE_WAIT_DISCOVER_ID) &&
        (s_tx.sop == UPD_SOP1)) {
        return 1U;
    }
    return 0U;
}

/** @brief Clear state-owned transmit ownership and result context. */
static void pd_clear_state_tx_context(void)
{
    s_state_tx_active = 0U;
    if (pd_tx_result_has_aux_owner() == 0U) {
        s_tx.result = PD_TX_RESULT_NONE;
    }
}

/** @brief Consume a transmit result owned by the active PD state. */
static uint8_t pd_state_take_tx_result(pd_tx_result_t *result)
{
    pd_tx_result_t dropped;

    if (s_state_tx_active == 0U) {
        if ((s_tx.result != PD_TX_RESULT_NONE) &&
            (pd_tx_result_has_aux_owner() == 0U)) {
            (void)pd_tx_take_result(&dropped);
        }
        return 0U;
    }

    if (pd_tx_take_result(result) == 0U) {
        return 0U;
    }
    s_state_tx_active = 0U;
    return 1U;
}

/** @brief Start a control message owned by the active policy state. */
static uint8_t pd_state_start_control(uint8_t msg_type)
{
    if (pd_tx_start_control(msg_type) == 0U) {
        return 0U;
    }
    s_state_tx_active = 1U;
    return 1U;
}

/** @brief Start a hard-reset sequence owned by the active policy state. */
static void pd_state_start_hard_reset(void)
{
    if ((pd_tx_is_idle() == 0U) || (s_tx.result != PD_TX_RESULT_NONE)) {
        return;
    }
    pd_tx_start_hard_reset();
    if (pd_tx_is_idle() == 0U) {
        s_state_tx_active = 1U;
    }
}

/** @brief Reset the partner revision-query state machine. */
static void pd_revision_query_reset(void)
{
    s_partner_revision = PD_PARTNER_REVISION_UNKNOWN;
    s_revision_query_state = PD_REVISION_QUERY_IDLE;
    s_revision_query_tx_active = 0U;
    s_revision_query_wait_ms = 0U;
}

/** @brief Complete a revision query with the negotiated revision value. */
static void pd_revision_query_finish(uint8_t revision)
{
    uint8_t disable_spr_avs =
        (((revision == PD_PARTNER_REVISION_30) ||
          (revision == PD_PARTNER_REVISION_31)) &&
         ((s_partner_revision == PD_PARTNER_REVISION_UNKNOWN) ||
          (s_partner_revision == PD_PARTNER_REVISION_3X))) ? 1U : 0U;

    s_partner_revision = revision;
    s_revision_query_state = PD_REVISION_QUERY_DONE;
    s_revision_query_tx_active = 0U;
    s_revision_query_wait_ms = 0U;

    if (disable_spr_avs != 0U) {
        s_source_cap_refresh_pending = 1U;
        s_source_cap_refresh_retries = 1U;
        s_source_cap_refresh_wait_ms = 0U;
    }
}

/** @brief Complete a revision query when the partner does not respond. */
static void pd_revision_query_finish_unresolved(void)
{
    s_revision_query_state = PD_REVISION_QUERY_DONE;
    s_revision_query_tx_active = 0U;
    s_revision_query_wait_ms = 0U;
}

/** @brief Process a message received while querying partner revision support. */
static uint8_t pd_revision_query_handle_message(uint8_t msg_type,
                                                uint8_t num_do,
                                                uint8_t extended)
{
    uint32_t rmdo;
    uint8_t major;
    uint8_t minor;

    if ((s_rx_sop != PD_RX_SOP0) ||
        (s_revision_query_state != PD_REVISION_QUERY_WAIT) ||
        (extended != 0U)) {
        return 0U;
    }

    if ((msg_type == DEF_TYPE_REVISION) && (num_do == 1U)) {
        rmdo = pd_get_u32_le(&s_pd_rx_buf[2]);
        major = (uint8_t)((rmdo & PD_RMDO_REV_MAJOR_MASK) >>
                          PD_RMDO_REV_MAJOR_SHIFT);
        minor = (uint8_t)((rmdo & PD_RMDO_REV_MINOR_MASK) >>
                          PD_RMDO_REV_MINOR_SHIFT);
        if ((major > 3U) || ((major == 3U) && (minor >= 2U))) {
            pd_revision_query_finish(PD_PARTNER_REVISION_32);
        } else if ((major == 3U) && (minor == 1U)) {
            pd_revision_query_finish(PD_PARTNER_REVISION_31);
        } else {
            pd_revision_query_finish(PD_PARTNER_REVISION_30);
        }
        return 1U;
    }

    if ((msg_type == DEF_TYPE_NOT_SUPPORT) && (num_do == 0U)) {
        pd_revision_query_finish(PD_PARTNER_REVISION_30);
        return 1U;
    }
    return 0U;
}

/** @brief Advance the partner revision-query timeout and retry state. */
static void pd_revision_query_task(uint8_t elapsed_ms)
{
    pd_tx_result_t result;
    uint32_t next_ms;

    if (s_revision_query_state == PD_REVISION_QUERY_DELAY) {
        if ((s_pd.connected == 0U) ||
            (s_pd.contract_mv == 0U) ||
            (s_pd.state != PD_STATE_READY)) {
            return;
        }
        next_ms = (uint32_t)s_revision_query_wait_ms + elapsed_ms;
        s_revision_query_wait_ms = (next_ms > 0xFFFFUL) ?
                                   0xFFFFU : (uint16_t)next_ms;
        if ((s_revision_query_wait_ms < PD_REVISION_QUERY_DELAY_MS) ||
            (s_rx_msg_pending != 0U) ||
            (s_goodcrc_tx_pending != 0U) ||
            (s_goodcrc_tx_active != 0U) ||
            (s_ext_response_tx_active != 0U) ||
            ((s_epr_mode_active != 0U) &&
             (s_debug_epr_chunk_phase >= 1U) &&
             (s_debug_epr_chunk_phase < 5U)) ||
            (s_fault_alert_tx_active != 0U) ||
            (s_source_cap_refresh_pending != 0U) ||
            (pd_tx_is_idle() == 0U) ||
            (s_tx.result != PD_TX_RESULT_NONE)) {
            return;
        }
        if (pd_tx_start_control(DEF_TYPE_GET_REVISION) != 0U) {
            s_revision_query_state = PD_REVISION_QUERY_TX;
            s_revision_query_tx_active = 1U;
            s_revision_query_wait_ms = 0U;
        }
        return;
    }

    if (s_revision_query_state == PD_REVISION_QUERY_TX) {
        if (pd_tx_take_result(&result) == 0U) {
            return;
        }
        s_revision_query_tx_active = 0U;
        if (result == PD_TX_RESULT_OK) {
            s_revision_query_state = PD_REVISION_QUERY_WAIT;
            s_revision_query_wait_ms = 0U;
        } else {
            pd_revision_query_finish_unresolved();
        }
        return;
    }

    if (s_revision_query_state == PD_REVISION_QUERY_WAIT) {
        next_ms = (uint32_t)s_revision_query_wait_ms + elapsed_ms;
        s_revision_query_wait_ms = (next_ms > 0xFFFFUL) ?
                                   0xFFFFU : (uint16_t)next_ms;
        if (s_revision_query_wait_ms >= PD_SENDER_RESPONSE_MAX_MS) {
            pd_revision_query_finish_unresolved();
        }
    }
}

void pd_set_state(pd_state_t state)
{
    pd_state_t old_state = s_pd.state;

    s_pd.state = state;
    s_pd.state_timer_ms = 0U;

    if (state != old_state) {
        if (state == PD_STATE_SEND_SOFT_RESET) {
            pd_debug_note_event(PD_DEBUG_EVENT_SOFT_RESET);
        } else if (state == PD_STATE_HARD_RESET) {
            pd_debug_note_event(PD_DEBUG_EVENT_HARD_RESET);
        }
    }
    if (state == PD_STATE_READY) {
        pd_reset_source_comm_timers();
    }

    if ((state == PD_STATE_DISCONNECTED) ||
        (state == PD_STATE_ATTACHED)) {
        s_pd_revision_locked = 0U;
        pd_revision_query_reset();
        pd_clear_contract_info();
        pd_clear_deferred_request();
        s_epr_mode_active = 0U;
        s_epr_mode_tx_active = 0U;
        s_epr_source_cap_pending = 0U;
        s_epr_source_cap_tx_active = 0U;
        s_epr_source_cap_retry = 0U;
        s_epr_advertise_ready = 0U;
        s_epr_keepalive_pending = 0U;
        s_epr_last_rx_action = 0U;
        s_epr_last_tx_action = 0U;
        s_epr_last_tx_data = 0U;
        s_epr_fail_reason = PD_EPR_FAIL_NONE;
        s_epr_enter_wait_active = 0U;
        s_epr_enter_wait_ms = 0U;
        s_source_cap_refresh_pending = 0U;
        s_source_cap_refresh_retries = 0U;
        s_source_cap_refresh_wait_ms = 0U;
        s_ext_response_tx_active = 0U;
        s_fault_alert_tx_active = 0U;
        pd_clear_staged_request();
    }

    if (state == PD_STATE_HARD_RESET) {
        /* A Hard Reset resets Protocol Layer MessageID counters in both
         * directions.  Otherwise the Sink's first post-reset MessageID 0
         * frame can be discarded as a duplicate of pre-reset traffic. */
        pd_clear_rx_message_ids();
        /* SOP and SOP' revision detection both restart after Hard Reset. */
        s_pd.pd_revision = 1U;
        s_cable_pd_revision = 1U;
        s_pd_revision_locked = 0U;
        pd_revision_query_reset();
        if (pd_active_contract_is_epr() != 0U) {
            pd_epr_exit_to_spr_recovery();
        }
        s_epr_mode_active = 0U;
        s_epr_mode_tx_active = 0U;
        s_epr_source_cap_pending = 0U;
        s_epr_source_cap_tx_active = 0U;
        s_epr_source_cap_retry = 0U;
        s_epr_keepalive_pending = 0U;
        s_epr_enter_wait_active = 0U;
        s_epr_enter_wait_ms = 0U;
        s_ext_response_tx_active = 0U;
        s_fault_alert_tx_active = 0U;
        pd_clear_staged_request();
        pd_clear_deferred_request();
        pd_cable_reset();
        s_debug_power_request_reason = PD_DEBUG_POWER_REQ_HARD_RESET;
        s_debug_power_request_enable = 1U;
        pd_power_recover_to_vsafe5v();
    }

    if ((state == PD_STATE_DISCONNECTED) ||
        (state == PD_STATE_ATTACHED) ||
        (state == PD_STATE_SEND_SOURCE_CAP) ||
        (state == PD_STATE_SEND_ACCEPT) ||
        (state == PD_STATE_SEND_CONTROL) ||
        (state == PD_STATE_SEND_SOFT_RESET) ||
        (state == PD_STATE_WAIT_SOFT_RESET_ACCEPT) ||
        (state == PD_STATE_HARD_RESET) ||
        (state == PD_STATE_SEND_EPR_KEEPALIVE_ACK) ||
        (state == PD_STATE_WAIT_REQUEST) ||
        (state == PD_STATE_APPLY_POWER) ||
        (state == PD_STATE_READY)) {
        pd_clear_state_tx_context();
    }
    if (state == PD_STATE_READY) {
        pd_clear_staged_request();
    }
}

/* Contract reset, source capability refresh, and message response helpers. */
void pd_clear_contract_info(void)
{
    pd_clear_staged_request();
    s_pd.contract_mv = 0U;
    s_pd.contract_ma = 0U;
    s_pd.selected_pdo = 0U;
    s_last_request_pdo = 0U;
    s_last_request_mv = 0U;
    s_last_request_ma = 0U;
    s_last_request_is_pps = 0U;
    s_last_request_is_epr = 0U;
    s_pps_active = 0U;
    s_active_supply_type = PD_SUPPLY_TYPE_NONE;
    s_last_rdo = 0UL;
    s_last_spr_rdo = 0UL;
}

void pd_reset_context(uint8_t disable_power)
{
    memset(&s_pd, 0, sizeof(s_pd));
    memset(&s_tx, 0, sizeof(s_tx));
    memset(&s_cc, 0, sizeof(s_cc));
    memset(&s_power, 0, sizeof(s_power));
    pd_debug_reset();
    s_last_msg_type = 0U;
    s_last_num_do = 0U;
    s_last_extended = 0U;
    s_last_request_pdo = 0U;
    s_last_rdo_pos = 0U;
    s_last_rdo = 0UL;
    s_last_spr_rdo = 0UL;
    s_last_request_mv = 0U;
    s_last_request_ma = 0U;
    s_last_request_is_pps = 0U;
    s_last_request_is_epr = 0U;
    s_pps_active = 0U;
    s_epr_mode_active = 0U;
    s_active_supply_type = PD_SUPPLY_TYPE_NONE;
    s_epr_mode_tx_active = 0U;
    s_epr_source_cap_pending = 0U;
    s_epr_source_cap_tx_active = 0U;
    s_epr_source_cap_retry = 0U;
    s_epr_advertise_ready = 0U;
    s_epr_keepalive_pending = 0U;
    s_epr_last_rx_action = 0U;
    s_epr_last_tx_action = 0U;
    s_epr_last_tx_data = 0U;
    s_epr_fail_reason = PD_EPR_FAIL_NONE;
    s_epr_enter_wait_active = 0U;
    s_epr_enter_wait_ms = 0U;
    s_source_cap_refresh_pending = 0U;
    s_source_cap_refresh_retries = 0U;
    s_source_cap_refresh_wait_ms = 0U;
    s_rx_msg_pending = 0U;
    s_rx_msg_queued = 0U;
    s_rx_msg_len = 0U;
    s_rx_msg_queued_len = 0U;
    s_rx_sop = PD_RX_SOP0;
    s_rx_msg_queued_sop = PD_RX_SOP0;
    s_cable_epr_capable = 0U;
    s_goodcrc_tx_active = 0U;
    s_goodcrc_tx_pending = 0U;
    s_goodcrc_watchdog_ms = 0U;
    s_goodcrc_sop = UPD_SOP0;
    s_pd_reset_pending = 0U;
    s_ext_response_tx_active = 0U;
    s_state_tx_active = 0U;
    s_replaying_deferred_request = 0U;
    s_fault_recovery_pending = 0U;
    s_pd_revision_locked = 0U;
    pd_revision_query_reset();
    pd_reset_source_comm_timers();
    pd_clear_staged_request();
    s_fault_bits = 0U;
    s_fault_alert_pending = 0U;
    s_fault_alert_sent = 0U;
    s_fault_alert_tx_active = 0U;
    s_fault_alert_tx_bits = 0U;
    s_pd.state = PD_STATE_DISCONNECTED;
    pd_clear_deferred_request();
    pd_clear_rx_message_ids();
    s_pd.pd_revision = 1U;
    s_cable_pd_revision = 1U;
    USBPD->PORT_CC1 = CC_CMP_66 | CC_PU_330;
    USBPD->PORT_CC2 = CC_CMP_66 | CC_PU_330;
    MP2980_I2C_DMA_Abort();
    pd_frontend_restart();
    pd_cable_reset();
    pd_goodcrc_timer_stop();
    if (disable_power != 0U) {
        s_debug_power_request_reason = PD_DEBUG_POWER_REQ_FAULT_OFF;
        s_debug_power_request_enable = 0U;
        pd_power_request(0U, 0U, 0U);
    }
    pd_set_phy_rx_mode();
}

/** @brief Queue a control message and the state to enter after transmission. */
static void pd_queue_control(uint8_t msg_type, pd_state_t next_state)
{
    s_pd.control_msg = msg_type;
    s_pd.next_state = next_state;
    pd_set_state(PD_STATE_SEND_CONTROL);
}

/** @brief Start soft-reset recovery after a failed negotiation exchange. */
static void pd_start_soft_reset_recovery(void)
{
    __disable_irq();
    s_rx_msg_pending = 0U;
    s_rx_msg_queued = 0U;
    s_rx_msg_len = 0U;
    s_rx_msg_queued_len = 0U;
    __enable_irq();

    pd_clear_staged_request();
    pd_clear_deferred_request();
    pd_clear_rx_message_ids();
    s_pd.msg_id = 0U;
    pd_set_state(PD_STATE_SEND_SOFT_RESET);
}

/** @brief Check whether a message should be ignored during power transition. */
static uint8_t pd_ignore_message_during_transition(uint8_t msg_type,
                                                   uint8_t num_do)
{
    uint8_t epr_action;

    if ((msg_type == DEF_TYPE_GOODCRC) || (msg_type == DEF_TYPE_SOFT_RESET)) {
        return 0U;
    }

    /* RX policy runs before the main state consumes the preceding TX result.
     * Serialize a fast EPR Enter behind an in-flight Source AMS instead of
     * rejecting it merely because the software state has not reached READY. */
    if ((msg_type == DEF_TYPE_EPR_MODE) &&
        (num_do == 1U) &&
        (s_rx_msg_len >= 10U)) {
        epr_action = (uint8_t)((pd_get_u32_le(&s_pd_rx_buf[2]) >>
                                PD_EPR_MODE_ACTION_SHIFT) &
                               PD_EPR_MODE_FIELD_MASK);
        if ((epr_action == PD_EPR_MODE_ACTION_ENTER) &&
            (s_pd.contract_mv != 0U) &&
            (s_state_tx_active != 0U) &&
            (pd_tx_is_idle() != 0U) &&
            (s_tx.result == PD_TX_RESULT_OK) &&
            ((s_pd.state == PD_STATE_SEND_SOURCE_CAP) ||
             (s_pd.state == PD_STATE_SEND_PS_RDY) ||
             (s_pd.state == PD_STATE_SEND_CONTROL))) {
            pd_defer_current_request();
            return 1U;
        }
    }
    if (pd_is_power_transition_state() == 0U) {
        return 0U;
    }
    if (((msg_type == DEF_TYPE_REQUEST) && (num_do == 1U)) ||
        ((msg_type == DEF_TYPE_EPR_REQUEST) && (num_do == 2U))) {
        /* Preserve the newest Request without starting another AMS until the
         * current Accept -> power transition -> PS_RDY sequence completes. */
        pd_defer_current_request();
    }
    return 1U;
}

/** @brief Check whether the configured source caps contain a 5 A PDO. */
static uint8_t pd_has_5a_pdo(void)
{
    uint8_t i;

    for (i = 0U; i < s_pdo_count; i++) {
        if (s_pdos[i].ma >= PD_5A_CURRENT_MA) {
            return 1U;
        }
    }
    if (pd_current_limit_ma() >= PD_5A_CURRENT_MA) {
        return 1U;
    }
    return 0U;
}

/** @brief Return the configured fixed-PDO current limit at a voltage. */
static uint16_t pd_fixed_pdo_max_ma(uint16_t mv)
{
    return (mv >= 20000U) ? PD_5A_CURRENT_MA : PD_DEFAULT_CURRENT_MA;
}

/** @brief Check whether a newly discovered 5 A cable needs cap refresh. */
static uint8_t pd_5a_refresh_needed(void)
{
    return ((s_supports_5a != 0U) &&
            (s_cable_current_ma >= PD_5A_CURRENT_MA) &&
            (s_pd.contract_ma != 0U) &&
            (s_pd.contract_ma < PD_5A_CURRENT_MA) &&
            (s_source_cap_refresh_retries != 0U) &&
            (pd_has_5a_pdo() != 0U)) ? 1U : 0U;
}

/** @brief Record that a source-capability refresh transmission has started. */
static void pd_source_cap_refresh_started(void)
{
    if (s_source_cap_refresh_pending != 0U) {
        s_source_cap_refresh_pending = 0U;
        s_source_cap_refresh_wait_ms = 0U;
        if (s_source_cap_refresh_retries != 0U) {
            s_source_cap_refresh_retries--;
        }
    }
}

/** @brief Start the source-capability message selected by the policy state. */
static uint8_t pd_state_start_source_cap(void)
{
    uint8_t started;

    if (s_epr_mode_active != 0U) {
        started = pd_tx_start_epr_source_cap_ext();
    } else {
        started = pd_tx_start_source_cap();
    }
    if (started == 0U) {
        return 0U;
    }
    s_state_tx_active = 1U;
    if (s_epr_mode_active != 0U) {
        s_epr_source_cap_pending = 0U;
        s_epr_source_cap_retry = 0U;
    }
    pd_source_cap_refresh_started();
    return 1U;
}

uint8_t pd_is_power_transition_state(void)
{
    return ((s_pd.state == PD_STATE_SEND_ACCEPT) ||
            (s_pd.state == PD_STATE_APPLY_POWER) ||
            (s_pd.state == PD_STATE_SEND_PS_RDY)) ? 1U : 0U;
}

/** @brief Start one extended response chunk owned by the policy state. */
static uint8_t pd_tx_extended_response(uint8_t msg_type, uint8_t chunk_number)
{
    pd_tx_result_t old_result;

    if ((s_pd.pd_revision == 0U) ||
        (pd_tx_is_idle() == 0U)) {
        return 0U;
    }
    if (s_tx.result != PD_TX_RESULT_NONE) {
        (void)pd_tx_take_result(&old_result);
    }

    if (msg_type == DEF_TYPE_SRC_CAP) {
        if (pd_tx_start_source_cap_ext_chunk(chunk_number) != 0U) {
            s_ext_response_tx_active = 1U;
            return 1U;
        }
        return 0U;
    }
    if (msg_type == DEF_TYPE_GET_STATUS_R) {
        if (pd_tx_start_status_chunk(chunk_number) != 0U) {
            s_ext_response_tx_active = 1U;
            return 1U;
        }
        return 0U;
    }
    if (msg_type == PD_EXT_EPR_SOURCE_CAP) {
        if (pd_tx_start_epr_source_cap_ext_chunk(chunk_number) != 0U) {
            s_epr_source_cap_tx_active = 1U;
            s_ext_response_tx_active = 1U;
            return 1U;
        }
        return 0U;
    }
    return 0U;
}

/** @brief Start the partner revision response for the active query. */
static uint8_t pd_tx_revision_response(void)
{
    pd_tx_result_t old_result;

    if ((s_pd.pd_revision == 0U) ||
        (pd_tx_is_idle() == 0U)) {
        return 0U;
    }
    if (s_tx.result != PD_TX_RESULT_NONE) {
        (void)pd_tx_take_result(&old_result);
    }

    if (pd_tx_start_revision() != 0U) {
        s_ext_response_tx_active = 1U;
        return 1U;
    }
    return 0U;
}

/** @brief Dispatch an incoming extended message to its policy handler. */
static void pd_handle_extended_message(uint8_t msg_type, uint8_t num_do)
{
    uint16_t ext_header;
    uint16_t data_len;
    uint8_t chunk_number;
    uint8_t ext_ctrl_type;
    uint8_t ext_ctrl_data;
    pd_tx_result_t old_result;

    if (pd_is_power_transition_state() != 0U) {
        return;
    }
    if ((num_do == 0U) || (s_rx_msg_len < 6U)) {
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
        return;
    }

    ext_header = (uint16_t)s_pd_rx_buf[2] |
                 ((uint16_t)s_pd_rx_buf[3] << 8U);
    data_len = (uint16_t)(ext_header & PD_EXT_HEADER_DATA_SIZE_MASK);
    chunk_number = (uint8_t)((ext_header >> PD_EXT_HEADER_CHUNK_NUM_SHIFT) &
                             PD_EXT_HEADER_CHUNK_NUM_MASK);

    if ((ext_header & PD_EXT_HEADER_REQUEST_CHUNK) != 0U) {
        if ((msg_type == PD_EXT_EPR_SOURCE_CAP) &&
            ((ext_header & PD_EXT_HEADER_CHUNKED) != 0U) &&
            (data_len == 0U)) {
            pd_debug_note_epr_chunk_phase(3U);
        }
        if (pd_tx_extended_response(msg_type, chunk_number) != 0U) {
            return;
        }
    }

    if ((msg_type == PD_EXT_EXTENDED_CONTROL) &&
        (chunk_number == 0U) &&
        (data_len >= 1U) &&
        (data_len <= PD_EXT_CONTROL_DATA_LEN) &&
        (s_rx_msg_len >= 10U)) {
        ext_ctrl_type = s_pd_rx_buf[4];
        ext_ctrl_data = (data_len >= PD_EXT_CONTROL_DATA_LEN) ? s_pd_rx_buf[5] : 0U;
        if ((ext_ctrl_type == PD_EXT_CTRL_EPR_GET_SRC_CAP) &&
            (ext_ctrl_data == 0U)) {
            if (pd_tx_extended_response(PD_EXT_EPR_SOURCE_CAP, 0U) != 0U) {
                return;
            }
        } else if ((ext_ctrl_type == PD_EXT_CTRL_EPR_KEEP_ALIVE) &&
                   (ext_ctrl_data == 0U) &&
                   (s_epr_mode_active != 0U)) {
            if (s_tx.result != PD_TX_RESULT_NONE) {
                (void)pd_tx_take_result(&old_result);
            }
            if (pd_tx_start_extended_control(PD_EXT_CTRL_EPR_KEEP_ALIVE_A, 0U) != 0U) {
                s_ext_response_tx_active = 1U;
                return;
            }
        }
        s_epr_fail_reason = (uint8_t)(20U + ext_ctrl_type);
    }

    pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
}

/* Incoming message dispatch and main policy state machine. */
static void pd_handle_message(void)
{
    uint8_t msg_type = (uint8_t)(s_pd_rx_buf[0] & 0x1FU);
    uint8_t num_do = (uint8_t)((s_pd_rx_buf[1] >> 4U) & 0x07U);
    uint8_t extended = (uint8_t)((s_pd_rx_buf[1] & 0x80U) != 0U);
    uint16_t mv;
    uint16_t ma;
    uint8_t pdo_index;
    uint8_t old_pps_active;
    uint8_t old_active_supply_type;
    uint32_t old_last_spr_rdo;

    s_last_msg_type = msg_type;
    s_last_num_do = num_do;
    s_last_extended = extended;

    /* Stale SOP traffic must not preempt the Hard Reset/Source Startup
     * sequence or seed the post-reset duplicate detector. SOP' cable
     * discovery remains valid while the Source is in ATTACHED. */
    if ((s_rx_sop == PD_RX_SOP0) &&
        ((s_pd.state == PD_STATE_DISABLED) ||
         (s_pd.state == PD_STATE_DISCONNECTED) ||
         (s_pd.state == PD_STATE_HARD_RESET) ||
         (s_pd.state == PD_STATE_ATTACHED))) {
        return;
    }

    if ((s_rx_sop == PD_RX_SOP1_HRST) || (s_rx_sop == PD_RX_SOP2_CRST)) {
        s_cable_pd_revision = ((s_pd_rx_buf[0] & 0xC0U) == 0x80U) ? 1U : 0U;
    } else if ((s_pd_revision_locked == 0U) &&
               (extended == 0U) &&
               (msg_type == DEF_TYPE_REQUEST) &&
               (num_do == 1U)) {
        uint8_t header_revision = (uint8_t)(s_pd_rx_buf[0] & 0xC0U);

        /* USB PD 6.1.3.1: the Sink's initial Request selects PD2.0 or PD3.x. */
        if (header_revision == 0x80U) {
            s_pd.pd_revision = 1U;
            s_pd_revision_locked = 1U;
            s_partner_revision = PD_PARTNER_REVISION_3X;
            s_revision_query_state = PD_REVISION_QUERY_DELAY;
            s_revision_query_wait_ms = 0U;
        } else if ((header_revision == 0x40U) ||
                   (header_revision == 0x00U)) {
            s_pd.pd_revision = 0U;
            s_pd_revision_locked = 1U;
            s_partner_revision = PD_PARTNER_REVISION_20;
            s_revision_query_state = PD_REVISION_QUERY_DONE;
        }
    }

    if ((s_replaying_deferred_request == 0U) &&
        (pd_rx_message_is_duplicate(msg_type) != 0U)) {
        return;
    }

    if ((s_rx_sop == PD_RX_SOP1_HRST) || (s_rx_sop == PD_RX_SOP2_CRST)) {
        if ((msg_type == DEF_TYPE_VENDOR_DEFINED) && (num_do != 0U)) {
            if ((s_rx_sop == PD_RX_SOP1_HRST) &&
                (s_cable_state == PD_CABLE_WAIT_DISCOVER_ID)) {
                pd_cable_handle_message(num_do);
            } else {
                pd_vdm_handle_message(num_do);
            }
        }
        return;
    }

    /* This implementation keeps several auxiliary AMS responses inside the
     * READY state, so count any non-duplicate SOP traffic as communication. */
    pd_reset_source_comm_timers();

    s_debug_rx_msg_id = (uint8_t)((s_pd_rx_buf[1] >> 1U) & 0x07U);
    s_debug_rx_msg_id_valid = 1U;
    s_debug_rx_sop = s_rx_sop;
    if ((extended == 0U) && (msg_type == DEF_TYPE_REQUEST) && (num_do == 1U)) {
        pd_debug_note_event(PD_DEBUG_EVENT_RX_REQUEST);
    } else if ((extended == 0U) &&
               (msg_type == DEF_TYPE_EPR_REQUEST) &&
               (num_do == 2U)) {
        pd_debug_note_event(PD_DEBUG_EVENT_RX_EPR_REQUEST);
    }

    if (s_pd.state == PD_STATE_SEND_SOFT_RESET) {
        return;
    }

    if (pd_revision_query_handle_message(msg_type, num_do, extended) != 0U) {
        return;
    }

    if (pd_is_epr_keepalive_request(msg_type, extended, num_do) != 0U) {
        if (s_epr_mode_active != 0U) {
            pd_reset_source_comm_timers();
            pd_debug_note_event(PD_DEBUG_EVENT_KEEPALIVE_RX);
            s_epr_keepalive_pending = 0U;
            pd_set_state(PD_STATE_SEND_EPR_KEEPALIVE_ACK);
            return;
        }
    }

    if (pd_ignore_message_during_transition(msg_type, num_do) != 0U) {
        return;
    }

    if (s_pd.state == PD_STATE_WAIT_SOFT_RESET_ACCEPT) {
        if ((extended == 0U) &&
            (msg_type == DEF_TYPE_ACCEPT) &&
            (num_do == 0U)) {
            pd_set_state(PD_STATE_SEND_SOURCE_CAP);
        } else {
            pd_set_state(PD_STATE_HARD_RESET);
        }
        return;
    }

    if (extended != 0U) {
        if (msg_type != DEF_TYPE_GOODCRC) {
            pd_handle_extended_message(msg_type, num_do);
        }
    } else if ((msg_type == DEF_TYPE_VENDOR_DEFINED) && (num_do != 0U)) {
        pd_vdm_handle_message(num_do);
    } else if ((msg_type == DEF_TYPE_BIST) && (num_do != 0U)) {
        return;
    } else if ((msg_type == DEF_TYPE_REQUEST) && (num_do == 1U)) {
        if (s_epr_mode_active != 0U) {
            pd_set_state(PD_STATE_HARD_RESET);
            return;
        }
        old_pps_active = s_pps_active;
        old_active_supply_type = s_active_supply_type;
        old_last_spr_rdo = s_last_spr_rdo;
        if (pd_validate_request(&mv, &ma, &pdo_index) != 0U) {
            pd_stage_request(mv, ma, pdo_index);
            s_pps_active = old_pps_active;
            s_active_supply_type = old_active_supply_type;
            s_last_spr_rdo = old_last_spr_rdo;
            s_pd.control_msg = 0U;
            pd_set_state(PD_STATE_SEND_ACCEPT);
        } else {
            s_pps_active = old_pps_active;
            s_active_supply_type = old_active_supply_type;
            s_last_spr_rdo = old_last_spr_rdo;
            s_last_request_pdo = 0U;
            s_last_request_mv = 0U;
            s_last_request_ma = 0U;
            s_last_request_is_pps = 0U;
            pd_queue_control(DEF_TYPE_REJECT,
                             (pd_fault_active() != 0U) ? s_pd.state : PD_STATE_SEND_SOURCE_CAP);
        }
    } else if ((msg_type == DEF_TYPE_EPR_MODE) && (num_do == 1U)) {
        if (pd_handle_epr_mode_request() != 0U) {
            s_pd.control_msg = 0U;
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type == DEF_TYPE_EPR_REQUEST) {
        if (s_epr_mode_active == 0U) {
            pd_set_state(PD_STATE_HARD_RESET);
            return;
        }
        old_pps_active = s_pps_active;
        old_active_supply_type = s_active_supply_type;
        old_last_spr_rdo = s_last_spr_rdo;
        if ((num_do == 2U) &&
            (pd_validate_epr_request(&mv, &ma, &pdo_index) != 0U)) {
            pd_stage_request(mv, ma, pdo_index);
            s_pps_active = old_pps_active;
            s_active_supply_type = old_active_supply_type;
            s_last_spr_rdo = old_last_spr_rdo;
            s_pd.control_msg = 0U;
            pd_set_state(PD_STATE_SEND_ACCEPT);
        } else {
            s_pps_active = old_pps_active;
            s_active_supply_type = old_active_supply_type;
            s_last_spr_rdo = old_last_spr_rdo;
            s_last_request_pdo = 0U;
            s_last_request_mv = 0U;
            s_last_request_ma = 0U;
            s_last_request_is_epr = 0U;
            if (s_epr_fail_reason == PD_EPR_FAIL_NONE) {
                s_epr_fail_reason = PD_EPR_FAIL_BAD_REQUEST;
            }
            pd_queue_control(DEF_TYPE_REJECT, s_pd.state);
        }
    } else if (msg_type == DEF_TYPE_GET_SRC_CAP) {
        if (pd_is_power_transition_state() != 0U) {
            return;
        }
        if (s_epr_mode_active != 0U) {
            if (pd_tx_start_source_cap() != 0U) {
                s_ext_response_tx_active = 1U;
                return;
            }
            pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
            return;
        }
        s_pd.control_msg = 0U;
        s_pd.source_cap_retry = 0U;
        pd_set_state(PD_STATE_SEND_SOURCE_CAP);
    } else if (msg_type == DEF_TYPE_GET_SRC_CAP_EX) {
        if (pd_is_power_transition_state() != 0U) {
            return;
        }
        if (pd_tx_extended_response(DEF_TYPE_SRC_CAP, 0U) != 0U) {
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type == DEF_TYPE_SOFT_RESET) {
        pd_clear_staged_request();
        pd_clear_deferred_request();
        s_pd.msg_id = 0U;
        pd_clear_rx_message_ids();
        pd_queue_control(DEF_TYPE_ACCEPT, PD_STATE_SEND_SOURCE_CAP);
    } else if (msg_type == DEF_TYPE_GET_STATUS) {
        if (pd_is_power_transition_state() != 0U) {
            return;
        }
        if (pd_tx_extended_response(DEF_TYPE_GET_STATUS_R, 0U) != 0U) {
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type == DEF_TYPE_GET_PPS_STATUS) {
        if (pd_is_power_transition_state() != 0U) {
            return;
        }
        if (pd_tx_start_pps_status() != 0U) {
            s_ext_response_tx_active = 1U;
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type == DEF_TYPE_GET_REVISION) {
        if (pd_tx_revision_response() != 0U) {
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type == DEF_TYPE_WAIT) {
        return;
    } else if (msg_type == DEF_TYPE_PING) {
        return;
    } else if (msg_type == DEF_TYPE_GOTOMIN) {
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type != DEF_TYPE_GOODCRC) {
        if (pd_is_power_transition_state() != 0U) {
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    }
}

/** @brief Advance the main PD policy state machine for one time slice. */
static void pd_process_state(uint8_t elapsed_ms)
{
    pd_tx_result_t result;

    s_pd.state_timer_ms = (uint16_t)(s_pd.state_timer_ms + elapsed_ms);
    if ((pd_fault_active() != 0U) && (s_power.target_enable != 0U)) {
        s_debug_power_request_reason = PD_DEBUG_POWER_REQ_FAULT_OFF;
        s_debug_power_request_enable = 0U;
        pd_power_request(0U, 0U, 0U);
    }
    if ((s_fault_recovery_pending != 0U) && (pd_fault_active() == 0U)) {
        if ((s_power.target_enable != 0U) ||
            (s_power.state == PD_POWER_ERROR)) {
            s_debug_power_request_reason = PD_DEBUG_POWER_REQ_FAULT_REC;
            s_debug_power_request_enable = 0U;
            pd_power_request(0U, 0U, 0U);
            return;
        }
        if ((s_power.state != PD_POWER_READY_OFF) &&
            (s_power.state != PD_POWER_OFF)) {
            return;
        }
        s_fault_recovery_pending = 0U;
        if (s_pd.connected != 0U) {
            s_pd.source_cap_retry = 0U;
            pd_set_state(PD_STATE_ATTACHED);
        }
    }

    switch (s_pd.state) {
    case PD_STATE_ATTACHED:
        if (pd_fault_active() != 0U) {
            break;
        }
        if (pd_power_request_is(1U, PD_VSAFE5V_MV) == 0U) {
            s_debug_power_request_reason = PD_DEBUG_POWER_REQ_ATTACHED;
            s_debug_power_request_enable = 1U;
            pd_power_request(1U, PD_VSAFE5V_MV, PD_VSAFE5V_MA);
        }
        if (pd_power_is_ready_on() == 0U) {
            s_pd.state_timer_ms = 0U;
            if (pd_power_has_error() != 0U) {
                pd_set_state(PD_STATE_HARD_RESET);
            }
            break;
        }
        if (s_cable_state == PD_CABLE_IDLE) {
            pd_cable_start_discovery();
        }

        /* An EPR Source discovers the cable before its first Explicit
         * Contract. A completed negative result continues as SPR-only. */
        if ((s_pd.state_timer_ms >= PD_FIRST_CAP_DELAY_MS) &&
            (pd_cable_discovery_finished() != 0U)) {
            pd_set_state(PD_STATE_SEND_SOURCE_CAP);
        }
        break;

    case PD_STATE_SEND_SOURCE_CAP:
        if (pd_fault_active() != 0U) {
            if (pd_tx_is_idle() != 0U) {
                pd_set_state(PD_STATE_ATTACHED);
            }
            break;
        }
        if (pd_state_take_tx_result(&result) != 0U) {
            if (result == PD_TX_RESULT_OK) {
                s_pd.source_cap_retry = 0U;
                if (s_epr_mode_active == 0U) {
                    /* Cable discovery now precedes the first SPR Source
                     * Capabilities, so EPR/5A was already advertised. */
                    s_epr_advertise_ready = 1U;
                }
                pd_set_state(PD_STATE_WAIT_REQUEST);
            } else {
                s_pd.source_cap_retry++;
                if (s_pd.source_cap_retry >= PD_SOURCE_CAP_RETRY_MAX) {
                    s_pd.source_cap_retry = 0U;
                    if (s_pd.contract_mv != 0U) {
                        s_source_cap_refresh_pending = 0U;
                        s_source_cap_refresh_retries = 0U;
                        s_source_cap_refresh_wait_ms = 0U;
                        pd_set_state(PD_STATE_READY);
                    } else {
                        pd_set_state(PD_STATE_HARD_RESET);
                    }
                }
            }
        } else if ((pd_tx_is_idle() != 0U) &&
                   (s_tx.result == PD_TX_RESULT_NONE) &&
                   (s_state_tx_active == 0U)) {
            (void)pd_state_start_source_cap();
        }
        break;

    case PD_STATE_WAIT_REQUEST:
        if (pd_fault_active() != 0U) {
            s_source_cap_refresh_pending = 0U;
            break;
        }
        if (s_source_cap_refresh_pending != 0U) {
            if ((pd_tx_is_idle() != 0U) && (s_tx.result == PD_TX_RESULT_NONE)) {
                s_pd.source_cap_retry = 0U;
                pd_set_state(PD_STATE_SEND_SOURCE_CAP);
                break;
            }
        }
        if ((pd_5a_refresh_needed() != 0U) &&
            (s_pd.state_timer_ms >= PD_5A_REFRESH_RETRY_DELAY_MS)) {
            s_pd.source_cap_retry = 0U;
            pd_set_state(PD_STATE_SEND_SOURCE_CAP);
            break;
        }
        if ((s_pd.contract_ma != 0U) &&
            (s_source_cap_refresh_retries == 0U) &&
            (s_pd.state_timer_ms >= PD_REQUEST_TIMEOUT_MS)) {
            pd_set_state(PD_STATE_READY);
            break;
        }
        if (s_pd.state_timer_ms >= PD_REQUEST_TIMEOUT_MS) {
            pd_set_state(PD_STATE_SEND_SOURCE_CAP);
        }
        break;

    case PD_STATE_SEND_ACCEPT:
        if (pd_fault_active() != 0U) {
            pd_clear_contract_info();
            if ((pd_tx_is_idle() != 0U) && (s_tx.result == PD_TX_RESULT_NONE)) {
                pd_queue_control(DEF_TYPE_REJECT, PD_STATE_WAIT_REQUEST);
            }
            break;
        }
        if (s_pd.state_timer_ms >= PD_SEND_STATE_TIMEOUT_MS) {
            pd_debug_note_failure(PD_DEBUG_EVENT_ACCEPT_FAIL);
            pd_start_soft_reset_recovery();
            break;
        }
        if (s_pd.state_timer_ms < PD_ACCEPT_DELAY_MS) {
            break;
        }
        if (pd_state_take_tx_result(&result) != 0U) {
            if (result == PD_TX_RESULT_OK) {
                if (pd_commit_staged_request() == 0U) {
                    pd_debug_note_failure(PD_DEBUG_EVENT_ACCEPT_FAIL);
                    pd_start_soft_reset_recovery();
                    break;
                }
                pd_debug_note_event(PD_DEBUG_EVENT_ACCEPT_OK);
                s_debug_power_request_reason = PD_DEBUG_POWER_REQ_REQUEST;
                s_debug_power_request_enable = 1U;
                pd_power_request(1U, s_pd.contract_mv, s_pd.contract_ma);
                pd_debug_note_event(PD_DEBUG_EVENT_POWER_SET);
                pd_power_task(0U);
                pd_set_state(PD_STATE_APPLY_POWER);
            } else {
                pd_debug_note_failure(PD_DEBUG_EVENT_ACCEPT_FAIL);
                pd_start_soft_reset_recovery();
            }
        } else if ((pd_tx_is_idle() != 0U) &&
                   (s_tx.result == PD_TX_RESULT_NONE) &&
                   (s_state_tx_active == 0U)) {
            if (pd_state_start_control(DEF_TYPE_ACCEPT) != 0U) {
                pd_debug_note_event(PD_DEBUG_EVENT_ACCEPT_START);
            }
        }
        break;

    case PD_STATE_APPLY_POWER:
        if (pd_fault_active() != 0U) {
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        if (pd_power_is_ready_on() != 0U) {
            pd_set_state(PD_STATE_SEND_PS_RDY);
        } else if (pd_power_has_error() != 0U) {
            pd_set_state(PD_STATE_HARD_RESET);
        }
        break;

    case PD_STATE_SEND_PS_RDY:
        if (pd_fault_active() != 0U) {
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        if (s_pd.state_timer_ms >= PD_SEND_STATE_TIMEOUT_MS) {
            pd_debug_note_failure(PD_DEBUG_EVENT_PS_RDY_FAIL);
            pd_start_soft_reset_recovery();
            break;
        }
        if ((pd_tx_is_idle() != 0U) &&
            (s_tx.result == PD_TX_RESULT_NONE) &&
            (s_state_tx_active == 0U)) {
            if (pd_state_start_control(DEF_TYPE_PS_RDY) != 0U) {
                pd_debug_note_event(PD_DEBUG_EVENT_PS_RDY_START);
            }
        }
        if (pd_state_take_tx_result(&result) != 0U) {
            if (result == PD_TX_RESULT_OK) {
                pd_debug_note_event(PD_DEBUG_EVENT_PS_RDY_OK);
                pd_set_state(PD_STATE_READY);
            } else {
                pd_debug_note_failure(PD_DEBUG_EVENT_PS_RDY_FAIL);
                pd_start_soft_reset_recovery();
            }
        }
        break;

    case PD_STATE_SEND_EPR_KEEPALIVE_ACK:
        if ((s_epr_mode_active == 0U) ||
            (s_pd.state_timer_ms >= PD_SEND_STATE_TIMEOUT_MS)) {
            pd_debug_note_failure(PD_DEBUG_EVENT_KEEPALIVE_ACK_FAIL);
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        if (pd_state_take_tx_result(&result) != 0U) {
            if (result == PD_TX_RESULT_OK) {
                pd_reset_source_comm_timers();
                pd_debug_note_event(PD_DEBUG_EVENT_KEEPALIVE_ACK_OK);
                pd_set_state(PD_STATE_READY);
            } else {
                pd_debug_note_failure(PD_DEBUG_EVENT_KEEPALIVE_ACK_FAIL);
                pd_set_state(PD_STATE_HARD_RESET);
            }
        } else if ((pd_tx_is_idle() != 0U) &&
                   (s_tx.result == PD_TX_RESULT_NONE) &&
                   (s_state_tx_active == 0U)) {
            if (pd_tx_start_extended_control(PD_EXT_CTRL_EPR_KEEP_ALIVE_A, 0U) != 0U) {
                s_state_tx_active = 1U;
                pd_debug_note_event(PD_DEBUG_EVENT_KEEPALIVE_ACK_START);
            }
        }
        break;

    case PD_STATE_SEND_SOFT_RESET:
        if (s_pd.state_timer_ms >= PD_SOFT_RESET_SEND_TIMEOUT_MS) {
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        if (pd_state_take_tx_result(&result) != 0U) {
            if (result == PD_TX_RESULT_OK) {
                pd_set_state(PD_STATE_WAIT_SOFT_RESET_ACCEPT);
            } else {
                pd_set_state(PD_STATE_HARD_RESET);
            }
        } else if ((pd_tx_is_idle() != 0U) &&
                   (s_tx.result == PD_TX_RESULT_NONE) &&
                   (s_state_tx_active == 0U)) {
            (void)pd_state_start_control(DEF_TYPE_SOFT_RESET);
        }
        break;

    case PD_STATE_WAIT_SOFT_RESET_ACCEPT:
        if (s_pd.state_timer_ms >= PD_SOFT_RESET_ACCEPT_TIMEOUT_MS) {
            pd_set_state(PD_STATE_HARD_RESET);
        }
        break;

    case PD_STATE_SEND_CONTROL:
        if (s_pd.state_timer_ms >= PD_SEND_STATE_TIMEOUT_MS) {
            if (s_pd.control_msg == DEF_TYPE_WAIT) {
                pd_set_state(s_pd.next_state);
            } else {
                pd_set_state((s_pd.contract_mv != 0U) ?
                             PD_STATE_READY :
                             PD_STATE_SEND_SOURCE_CAP);
            }
            break;
        }
        if ((pd_tx_is_idle() != 0U) &&
            (s_tx.result == PD_TX_RESULT_NONE) &&
            (s_state_tx_active == 0U)) {
            (void)pd_state_start_control(s_pd.control_msg);
        }
        if (pd_state_take_tx_result(&result) != 0U) {
            if (result == PD_TX_RESULT_OK) {
                pd_set_state(s_pd.next_state);
            } else {
                if (s_pd.control_msg == DEF_TYPE_WAIT) {
                    pd_set_state(s_pd.next_state);
                } else {
                    pd_set_state((s_pd.contract_mv != 0U) ?
                                 PD_STATE_READY :
                                 PD_STATE_SEND_SOURCE_CAP);
                }
            }
        }
        break;

    case PD_STATE_HARD_RESET:
        if (s_state_tx_active == 0U) {
            pd_state_start_hard_reset();
        }
        if (pd_state_take_tx_result(&result) != 0U) {
            s_pd.hard_reset_retry++;
            s_pd.msg_id = 0U;
            s_cable_msg_id = 0U;
            s_cable2_msg_id = 0U;
            pd_clear_rx_message_ids();
            s_pd.pd_revision = 1U;
            s_cable_pd_revision = 1U;
            s_pd.source_cap_retry = 0U;
            if ((result != PD_TX_RESULT_OK) &&
                (s_pd.hard_reset_retry < PD_HARD_RESET_RETRY_MAX)) {
                break;
            }
            s_pd.hard_reset_retry = 0U;
            pd_set_state(PD_STATE_ATTACHED);
        }
        break;

    case PD_STATE_READY:
        if (pd_power_has_error() != 0U) {
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        if (pd_fault_active() != 0U) {
            s_source_cap_refresh_pending = 0U;
            s_source_cap_refresh_retries = 0U;
            s_source_cap_refresh_wait_ms = 0U;
        }
        if ((s_pd.contract_ma >= PD_5A_CURRENT_MA) &&
            (s_source_cap_refresh_pending == 0U)) {
            s_source_cap_refresh_pending = 0U;
            s_source_cap_refresh_retries = 0U;
            s_source_cap_refresh_wait_ms = 0U;
        } else if ((s_source_cap_refresh_pending == 0U) &&
                   (pd_5a_refresh_needed() != 0U)) {
            s_source_cap_refresh_wait_ms =
                (uint16_t)(s_source_cap_refresh_wait_ms + elapsed_ms);
            if (s_source_cap_refresh_wait_ms >= PD_5A_REFRESH_RETRY_DELAY_MS) {
                s_source_cap_refresh_wait_ms = 0U;
                s_source_cap_refresh_pending = 1U;
            }
        }
        if ((s_epr_advertise_ready == 0U) &&
            (s_epr_mode_active == 0U) &&
            (pd_epr_is_available() != 0U) &&
            (s_pd.contract_mv != 0U) &&
            (pd_fault_active() == 0U)) {
            s_epr_advertise_ready = 1U;
            s_source_cap_refresh_pending = 1U;
            s_source_cap_refresh_retries = 1U;
            s_source_cap_refresh_wait_ms = 0U;
        }
        if (s_source_cap_refresh_pending != 0U) {
            if ((pd_tx_is_idle() != 0U) && (s_tx.result == PD_TX_RESULT_NONE)) {
                s_pd.source_cap_retry = 0U;
                pd_set_state(PD_STATE_SEND_SOURCE_CAP);
                break;
            }
        }
        if (pd_tx_result_has_aux_owner() == 0U) {
            (void)pd_tx_take_result(&result);
        }
        s_pd.hard_reset_retry = 0U;
        break;

    default:
        break;
    }
}

/* Initialization, periodic execution, callbacks, and public status APIs. */
void PD_Init(pd_vbus_sense_cb_t vbus_sense_cb)
{
    PD_SetVbusSenseCallback(vbus_sense_cb);

    if (SystemCoreClock >= 96000000UL) {
        s_pd_tx_clk_cnt = UPD_TMR_TX_96M;
        s_pd_rx_clk_cnt = UPD_TMR_RX_96M;
    } else if (SystemCoreClock >= 48000000UL) {
        s_pd_tx_clk_cnt = UPD_TMR_TX_48M;
        s_pd_rx_clk_cnt = UPD_TMR_RX_48M;
    } else if (SystemCoreClock >= 24000000UL) {
        s_pd_tx_clk_cnt = UPD_TMR_TX_24M;
        s_pd_rx_clk_cnt = UPD_TMR_RX_24M;
    } else {
        s_pd_tx_clk_cnt = UPD_TMR_TX_12M;
        s_pd_rx_clk_cnt = UPD_TMR_RX_12M;
    }

    memcpy(s_pdos, s_default_pdos, sizeof(s_default_pdos));
    s_pdo_count = PD_SOURCE_MAX_FIXED_PDOS;

    USBPD->CONFIG = PD_DMA_EN | PD_FILT_ED;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;
    s_frontend_state = PD_FRONTEND_CLOSE_HVCP;
    pd_reset_context(1U);
}

/** @brief Finish processing the current RX packet and clear its pending flag. */
static void pd_finish_current_rx(void)
{
    uint8_t i;

    __disable_irq();
    s_rx_msg_pending = 0U;
    if (s_rx_msg_queued != 0U) {
        for (i = 0U; i < s_rx_msg_queued_len; i++) {
            s_pd_rx_buf[i] = s_pd_rx_queued_buf[i];
        }
        s_rx_msg_len = s_rx_msg_queued_len;
        s_rx_sop = s_rx_msg_queued_sop;
        s_rx_msg_queued = 0U;
        s_rx_msg_queued_len = 0U;
        s_rx_msg_pending = 1U;
    }
    __enable_irq();
}

/** @brief Replay a deferred Request after the associated transition is ready. */
static void pd_handle_deferred_request(void)
{
    uint8_t i;

    if ((s_deferred_request_valid == 0U) ||
        ((s_pd.state != PD_STATE_READY) &&
         (s_pd.state != PD_STATE_WAIT_REQUEST)) ||
        (s_rx_msg_pending != 0U) ||
        (s_goodcrc_tx_pending != 0U) ||
        (s_goodcrc_tx_active != 0U) ||
        (pd_tx_is_idle() == 0U) ||
        (s_tx.result != PD_TX_RESULT_NONE)) {
        return;
    }

    __disable_irq();
    if ((s_rx_msg_pending != 0U) ||
        (s_goodcrc_tx_pending != 0U) ||
        (s_goodcrc_tx_active != 0U) ||
        (pd_tx_is_idle() == 0U) ||
        (s_tx.result != PD_TX_RESULT_NONE)) {
        __enable_irq();
        return;
    }
    for (i = 0U; i < s_deferred_request_len; i++) {
        s_pd_rx_buf[i] = s_deferred_request_buf[i];
    }
    s_rx_msg_len = s_deferred_request_len;
    s_rx_sop = s_deferred_request_sop;
    s_rx_msg_pending = 1U;
    pd_clear_deferred_request();
    __enable_irq();

    s_replaying_deferred_request = 1U;
    pd_handle_message();
    s_replaying_deferred_request = 0U;
    pd_finish_current_rx();
    if (s_tx.state == PD_TX_IDLE) {
        pd_set_phy_rx_mode();
    }
}

/** @brief Advance source communication and keep-alive timeout handling. */
static void pd_source_comm_timeout_task(uint8_t elapsed_ms)
{
    uint32_t next_ms;

    if ((s_pd.connected == 0U) || (s_pd.state != PD_STATE_READY)) {
        pd_reset_source_comm_timers();
        return;
    }

    if (s_epr_mode_active != 0U) {
        next_ms = (uint32_t)s_source_epr_keepalive_ms + elapsed_ms;
        s_source_epr_keepalive_ms = (next_ms > 0xFFFFUL) ?
                                    0xFFFFU : (uint16_t)next_ms;
        if (s_source_epr_keepalive_ms >= PD_SOURCE_EPR_KEEPALIVE_TIMEOUT_MS) {
            pd_debug_note_failure(PD_DEBUG_EVENT_EPR_COMM_TIMEOUT);
            pd_set_state(PD_STATE_HARD_RESET);
            return;
        }
    } else {
        s_source_epr_keepalive_ms = 0U;
    }

    if (s_pps_active != 0U) {
        next_ms = (uint32_t)s_source_pps_comm_ms + elapsed_ms;
        s_source_pps_comm_ms = (next_ms > 0xFFFFUL) ?
                               0xFFFFU : (uint16_t)next_ms;
        if (s_source_pps_comm_ms >= PD_SOURCE_PPS_COMM_TIMEOUT_MS) {
            pd_debug_note_failure(PD_DEBUG_EVENT_PPS_COMM_TIMEOUT);
            pd_set_state(PD_STATE_HARD_RESET);
        }
    } else {
        s_source_pps_comm_ms = 0U;
    }
}

void PD_Task(uint8_t elapsed_ms)
{
    if (s_pd.state == PD_STATE_DISABLED) {
        return;
    }

    if (s_pd_reset_pending != 0U) {
        s_pd_reset_pending = 0U;
        pd_reset_context(1U);
    }

    pd_goodcrc_watchdog_task();
    pd_power_task(elapsed_ms);
    pd_frontend_task(elapsed_ms);
    pd_tx_task(elapsed_ms);
    if (pd_frontend_is_ready() != 0U) {
        pd_connection_task(elapsed_ms);
    }

    if (s_rx_msg_pending != 0U) {
        pd_cable_task(0U);
        pd_ext_response_task(0U);
        pd_alert_task();
        pd_revision_query_task(0U);
        if ((s_goodcrc_tx_pending == 0U) &&
            (s_goodcrc_tx_active == 0U) &&
            (pd_tx_is_idle() != 0U) &&
            (s_tx.result == PD_TX_RESULT_NONE)) {
            /* Keep the processing buffer and the GoodCRC/response handoff
             * atomic.  The policy handler is non-blocking. */
            __disable_irq();
            pd_handle_message();
            __enable_irq();
            pd_finish_current_rx();
            if (s_tx.state == PD_TX_IDLE) {
                pd_set_phy_rx_mode();
            }
        }
    }

    pd_cable_task(elapsed_ms);
    pd_ext_response_task(elapsed_ms);
    pd_alert_task();
    pd_epr_source_cap_task();

    if (s_pd.connected != 0U) {
        pd_process_state(elapsed_ms);
    }
    pd_handle_deferred_request();
    pd_revision_query_task(elapsed_ms);
    pd_source_comm_timeout_task(elapsed_ms);
}

void PD_TickMs(uint8_t elapsed_ms)
{
    uint32_t tick = (uint32_t)s_tick_ms + elapsed_ms;
    uint16_t watchdog_age;

    s_tick_ms = (tick > 0xFFFFU) ? 0xFFFFU : (uint16_t)tick;

    /* GoodCRC starts asynchronously in the USBPD IRQ.  Age it here using
     * real timer ticks so a delayed main-loop pass cannot charge old elapsed
     * time to a response that has only just started. */
    if ((s_goodcrc_tx_pending != 0U) || (s_goodcrc_tx_active != 0U)) {
        watchdog_age = (uint16_t)s_goodcrc_watchdog_ms + elapsed_ms;
        s_goodcrc_watchdog_ms = (watchdog_age > 0xFFU) ?
                                0xFFU : (uint8_t)watchdog_age;
    }
}

void PD_Tick1ms(void)
{
    PD_TickMs(1U);
}

void PD_TaskFromTick(void)
{
    uint16_t elapsed;

    __disable_irq();
    elapsed = s_tick_ms;
    s_tick_ms = 0U;
    __enable_irq();

    if (elapsed != 0U) {
        PD_Task((uint8_t)(elapsed > 255U ? 255U : elapsed));
    }
}

void PD_SetPowerControlCallback(pd_power_control_cb_t cb)
{
    s_power_cb = cb;
}

void PD_SetVbusSenseCallback(pd_vbus_sense_cb_t cb)
{
    s_vbus_sense_cb = cb;
}

uint16_t pd_sense_vbus_mv(void)
{
    if (s_vbus_sense_cb == 0) {
        return 0U;
    }

    return s_vbus_sense_cb();
}

uint8_t PD_SetFixedPdos(const pd_fixed_pdo_t *pdos, uint8_t count)
{
    uint8_t i;

    if ((pdos == 0) || (count == 0U) || (count > PD_SOURCE_MAX_FIXED_PDOS)) {
        return 0U;
    }

    for (i = 0U; i < count; i++) {
        if ((pdos[i].mv < 5000U) ||
            (pdos[i].mv > 20000U) ||
            (pdos[i].ma == 0U) ||
            (pdos[i].ma > pd_fixed_pdo_max_ma(pdos[i].mv))) {
            return 0U;
        }
    }

    for (i = 0U; i < count; i++) {
        s_pdos[i] = pdos[i];
    }
    s_pdo_count = count;
    return 1U;
}


void PD_ReportFault(uint16_t fault_bits)
{
    if (fault_bits == 0U) {
        return;
    }

    PD_UpdateFaults((uint16_t)(s_fault_bits | fault_bits));
}


void PD_UpdateFaults(uint16_t fault_bits)
{
    uint16_t old_bits = s_fault_bits;
    uint16_t new_bits = (uint16_t)(fault_bits & (uint16_t)(~old_bits));

    if (old_bits == fault_bits) {
        return;
    }

    s_fault_bits = fault_bits;
    if (fault_bits == 0U) {
        s_fault_alert_pending = 0U;
        s_fault_alert_sent = 0U;
        s_fault_alert_tx_bits = 0U;
        if (old_bits != 0U) {
            s_fault_recovery_pending = 1U;
        }
        return;
    }

    if ((new_bits != 0U) || (s_fault_alert_sent == 0U)) {
        s_fault_alert_pending = 1U;
        s_fault_alert_sent = 0U;
    }

    if (pd_active_contract_is_epr() != 0U) {
        pd_epr_exit_to_spr_recovery();
    }

    if (s_power.target_enable != 0U) {
        s_debug_power_request_reason = PD_DEBUG_POWER_REQ_FAULT_OFF;
        s_debug_power_request_enable = 0U;
        pd_power_request(0U, 0U, 0U);
    }
}


void PD_GetStatus(pd_status_t *status)
{
    if (status == 0) {
        return;
    }

    status->state = s_pd.state;
    status->connected = s_pd.connected;
    status->cc = s_pd.cc;
    status->pd_revision = s_pd.pd_revision ? DEF_PD_REVISION_30 : DEF_PD_REVISION_20;
    status->partner_revision = s_partner_revision;
    status->selected_pdo = s_pd.selected_pdo;
    status->power_state = (uint8_t)s_power.state;
    status->power_target_mv = s_power.target_mv;
    status->last_msg_type = s_last_msg_type;
    status->control_msg = s_pd.control_msg;
    status->last_request_pdo = s_last_request_pdo;
    status->last_num_do = s_last_num_do;
    status->last_extended = s_last_extended;
    status->tx_state = (uint8_t)s_tx.state;
    status->tx_result = (uint8_t)s_tx.result;
    status->tx_try_count = s_tx.try_count;
    status->rx_msg_len = s_rx_msg_len;
    status->last_rdo_pos = s_last_rdo_pos;
    status->source_cap_retry = s_pd.source_cap_retry;
    status->hard_reset_retry = s_pd.hard_reset_retry;
    status->contract_mv = s_pd.contract_mv;
    status->contract_ma = s_pd.contract_ma;
    status->last_request_mv = s_last_request_mv;
    status->last_request_ma = s_last_request_ma;
    status->source_current_ma = pd_current_limit_ma();
    status->cable_max_vbus_mv = s_cable_max_vbus_mv;
    status->last_request_is_pps = s_last_request_is_pps;
    status->last_request_is_epr = s_last_request_is_epr;
    status->pps_active = s_pps_active;
    status->epr_mode_active = s_epr_mode_active;
    status->active_supply_type = s_active_supply_type;
    status->epr_available = pd_epr_is_available();
    status->epr_last_rx_action = s_epr_last_rx_action;
    status->epr_last_tx_action = s_epr_last_tx_action;
    status->epr_last_tx_data = s_epr_last_tx_data;
    status->epr_fail_reason = s_epr_fail_reason;
    status->pps_min_mv = PD_PPS_MIN_MV;
    status->pps_max_mv = PD_PPS_MAX_MV;
    status->pps_max_ma = pd_current_limit_ma();
    status->fault_bits = s_fault_bits;
    status->last_rdo = s_last_rdo;
    status->supports_5a = s_supports_5a;
    status->fault_alert_pending = s_fault_alert_pending;
    status->fault_alert_sent = s_fault_alert_sent;
    status->cable_state = (uint8_t)s_cable_state;
    status->cable_retry = s_cable_retry;
    status->cable_pd_revision = s_cable_pd_revision;
    status->cable_tx_msg_id = s_cable_msg_id;
    status->cable_epr_capable = s_cable_epr_capable;
    status->last_vdm_sop = s_last_vdm_sop;
    status->last_vdm_cmd = s_last_vdm_cmd;
    status->last_vdm_cmdt = s_last_vdm_cmdt;
    status->last_vdm_header = s_last_vdm_header;
    status->last_cable_vdo = s_last_cable_vdo;
    status->debug_last_event = s_debug_last_event;
    status->debug_last_failure = s_debug_last_failure;
    status->debug_rx_msg_id = s_debug_rx_msg_id;
    status->debug_rx_msg_id_valid = s_debug_rx_msg_id_valid;
    status->debug_rx_sop = s_debug_rx_sop;
    status->debug_deferred_request_valid = s_deferred_request_valid;
    status->debug_epr_keepalive_pending = s_epr_keepalive_pending;
    status->debug_epr_source_cap_pending = s_epr_source_cap_pending;
    status->debug_state_tx_active = s_state_tx_active;
    status->debug_epr_chunk_phase = s_debug_epr_chunk_phase;
    status->debug_goodcrc_recover_count = s_debug_goodcrc_recover_count;
    status->debug_goodcrc_fail_msg_type = s_debug_goodcrc_fail_msg_type;
    status->debug_goodcrc_fail_msg_id = s_debug_goodcrc_fail_msg_id;
    status->debug_goodcrc_fail_stage = s_debug_goodcrc_fail_stage;
    status->debug_rx_hard_reset_count = s_debug_rx_hard_reset_count;
    status->debug_rx_hard_reset_state = s_debug_rx_hard_reset_state;
    status->debug_rx_hard_reset_power_state = s_debug_rx_hard_reset_power_state;
    status->debug_epr_entry_fail_reason = s_debug_epr_entry_fail_reason;
    status->debug_epr_entry_fail_state = s_debug_epr_entry_fail_state;
    status->debug_event_seq = s_debug_event_seq;
    status->debug_request_count = s_debug_request_count;
    status->debug_accept_ok_count = s_debug_accept_ok_count;
    status->debug_accept_fail_count = s_debug_accept_fail_count;
    status->debug_power_ready_count = s_debug_power_ready_count;
    status->debug_power_error_count = s_debug_power_error_count;
    status->debug_ps_rdy_ok_count = s_debug_ps_rdy_ok_count;
    status->debug_ps_rdy_fail_count = s_debug_ps_rdy_fail_count;
    status->debug_soft_reset_count = s_debug_soft_reset_count;
    status->debug_hard_reset_count = s_debug_hard_reset_count;
    status->debug_power_output_mv = s_power.output_mv;
    status->debug_power_state_elapsed_ms = s_power.state_elapsed_ms;
    status->debug_power_request_reason = s_debug_power_request_reason;
    status->debug_power_request_enable = s_debug_power_request_enable;
    status->debug_power_request_count = s_debug_power_request_count;
}
