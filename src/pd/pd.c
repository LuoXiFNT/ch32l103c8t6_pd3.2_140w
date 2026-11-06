/**
 * @file pd.c
 * @brief Core USB Power Delivery context, message dispatch, and public API.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#include "pd_internal.h"

/* Persistent protocol context, request staging, diagnostics, and task flow. */

__attribute__((aligned(4))) uint8_t s_pd_rx_buf[PD_RX_BUF_SIZE];
__attribute__((aligned(4))) uint8_t s_pd_rx_dma_buf[PD_RX_BUF_SIZE];
/* Received-frame ring. The PHY interrupt appends at the head, the policy layer consumes at
 * the tail. A single pending slot was not enough: when a second frame arrived before the
 * first one had been handled it was discarded, and losing an EPR_Keep_Alive made the Source
 * believe the CC path had gone silent. */
__attribute__((aligned(4))) uint8_t s_pd_rx_ring_buf[PD_RX_RING_LEN][PD_RX_BUF_SIZE];
uint8_t s_rx_ring_len[PD_RX_RING_LEN];
uint8_t s_rx_ring_sop[PD_RX_RING_LEN];
volatile uint8_t s_rx_ring_head;
volatile uint8_t s_rx_ring_tail;
__attribute__((aligned(4))) uint8_t s_pd_tx_buf[PD_TX_BUF_SIZE];
uint8_t s_pd_goodcrc_buf[PD_GOODCRC_LEN];

static const pd_fixed_pdo_t s_default_pdos[PD_SOURCE_MAX_FIXED_PDOS] = {
    { 5000u,  PD_DEFAULT_CURRENT_MA },
    { 9000u,  PD_DEFAULT_CURRENT_MA },
    { 12000u, PD_DEFAULT_CURRENT_MA },
    { 15000u, PD_DEFAULT_CURRENT_MA },
    { 20000u, PD_5A_CURRENT_MA }
};

pd_fixed_pdo_t s_pdos[PD_SOURCE_MAX_FIXED_PDOS];
uint8_t s_pdo_count;
volatile bool s_rx_msg_pending;
volatile uint8_t s_rx_msg_len;
volatile uint8_t s_rx_sop;
volatile uint16_t s_tick_ms;
volatile bool s_goodcrc_tx_active;
volatile bool s_goodcrc_tx_pending;
volatile uint8_t s_goodcrc_watchdog_ms;
volatile uint8_t s_goodcrc_sop;
pd_context_t s_pd;
pd_cc_detect_t s_cc;
pd_tx_context_t s_tx;
pd_power_context_t s_power;
volatile uint8_t s_debug_power_request_reason;
volatile bool s_debug_power_request_enable;
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
static pd_vbus_sense_cb_t s_vbus_sense_cb;
uint16_t s_frontend_wait_ms;
uint16_t s_cable_wait_ms;
uint8_t s_cable_msg_id;
uint8_t s_cable2_msg_id;
uint8_t s_cable_pd_revision;
uint8_t s_last_msg_type;
uint8_t s_last_num_do;
bool s_last_extended;
uint8_t s_last_request_pdo;
uint8_t s_last_rdo_pos;
uint32_t s_last_rdo;
uint32_t s_last_spr_rdo;
uint16_t s_last_request_mv;
uint16_t s_last_request_ma;
bool s_last_request_is_pps;
bool s_last_request_is_epr;
bool s_pps_active;
bool s_epr_mode_active;
uint8_t s_active_supply_type;
bool s_epr_mode_tx_active;
bool s_epr_source_cap_pending;
bool s_epr_source_cap_tx_active;
uint8_t s_epr_source_cap_retry;
bool s_epr_advertise_ready;
uint8_t s_epr_last_rx_action;
uint8_t s_epr_last_tx_action;
uint8_t s_epr_last_tx_data;
uint8_t s_epr_fail_reason;
bool s_epr_enter_wait_active;
uint16_t s_epr_enter_wait_ms;
bool s_supports_5a;
bool s_source_cap_refresh_pending;
uint8_t s_source_cap_refresh_retries;
uint16_t s_source_cap_refresh_wait_ms;
uint16_t s_cable_current_ma;
uint16_t s_cable_max_vbus_mv;
bool s_cable_epr_capable;
uint32_t s_last_vdm_header;
uint32_t s_last_cable_vdo;
uint8_t s_last_vdm_sop;
uint8_t s_last_vdm_cmd;
uint8_t s_last_vdm_cmdt;
uint8_t s_cable_retry;
volatile bool s_pd_reset_pending;
uint16_t s_fault_bits;
bool s_fault_alert_pending;
bool s_fault_alert_sent;
bool s_fault_alert_tx_active;
uint16_t s_fault_alert_tx_bits;
bool s_ext_response_tx_active;
uint8_t s_partner_revision;
static bool s_state_tx_active;
static bool s_fault_recovery_pending;
static bool s_epr_keepalive_pending;
/* EPR KeepAlive acknowledgement owed to the Sink. A KeepAlive that arrives while an
 * Accept / power transition / PS_RDY is running is noted here instead of replacing the
 * policy state, and is answered as soon as that sequence has finished. */
static bool s_epr_keepalive_ack_pending;
/* Policy state to return to after an acknowledgement that was injected into a running
 * Accept / power transition / PS_RDY sequence; 0 when there is nothing to resume. */
static uint8_t s_epr_resume_state;
static uint16_t s_epr_resume_ms;
static bool s_pd_revision_locked;
typedef enum {
    PD_REVISION_QUERY_IDLE = 0u,
    PD_REVISION_QUERY_DELAY,
    PD_REVISION_QUERY_TX,
    PD_REVISION_QUERY_WAIT,
    PD_REVISION_QUERY_DONE
} pd_revision_query_state_t;
static pd_revision_query_state_t s_revision_query_state;
static bool s_revision_query_tx_active;
static uint16_t s_revision_query_wait_ms;
static bool s_staged_request_valid;
static uint8_t s_staged_request_pdo;
static bool s_staged_request_is_pps;
static bool s_staged_request_is_epr;
static uint8_t s_staged_request_supply_type;
static uint16_t s_staged_request_mv;
static uint16_t s_staged_request_ma;
static uint32_t s_staged_request_rdo;
static uint32_t s_staged_request_spr_rdo;

static bool s_deferred_request_valid;
static uint8_t s_deferred_request_len;
static uint8_t s_deferred_request_sop;
static uint8_t s_deferred_request_buf[PD_RX_BUF_SIZE];
static bool s_replaying_deferred_request;
static uint8_t s_rx_last_msg_id[3];
static uint8_t s_rx_last_msg_id_valid[3];
static uint8_t s_debug_last_event;
static uint8_t s_debug_last_failure;
static uint8_t s_debug_rx_msg_id;
static bool s_debug_rx_msg_id_valid;
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
/* Communication bookkeeping used by the diagnostic page to tell a silent Port Partner
 * apart from a receive path that stopped processing frames. */
static uint16_t s_debug_keepalive_rx_count;
static uint16_t s_debug_rx_msg_count;
static uint16_t s_debug_epr_silence_max_ms;
/* Consecutive EPR silence windows already tolerated. Section 7.30.5.2 makes the Hard Reset
 * after tSourceEPRKeepAlive a Shall, so PD_EPR_SILENCE_STRIKES_TOLERATED is 0u and the first
 * silent window resets the port; any received frame clears the strike again. The counter is
 * kept so a non-zero tolerance can be restored on the bench without touching the timer. */
static uint8_t s_epr_silence_strikes;
uint16_t s_debug_rx_frame_count;
uint16_t s_debug_rx_drop_count;
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
    s_debug_epr_chunk_phase = 0u;
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
    s_debug_rx_msg_id = 0u;
    s_debug_rx_msg_id_valid = false;
    s_debug_rx_sop = PD_RX_SOP0;
    s_debug_event_seq = 0u;
    s_debug_request_count = 0u;
    s_debug_accept_ok_count = 0u;
    s_debug_accept_fail_count = 0u;
    s_debug_power_ready_count = 0u;
    s_debug_power_error_count = 0u;
    s_debug_ps_rdy_ok_count = 0u;
    s_debug_ps_rdy_fail_count = 0u;
    s_debug_soft_reset_count = 0u;
    s_debug_hard_reset_count = 0u;
    s_debug_epr_chunk_phase = 0u;
    s_debug_epr_entry_fail_reason = PD_EPR_FAIL_NONE;
    s_debug_epr_entry_fail_state = 0u;
}

void pd_reset_source_comm_timers(void)
{
    s_source_epr_keepalive_ms = 0u;
    s_source_pps_comm_ms = 0u;
    s_epr_silence_strikes = 0u;
}

/* Deferred and staged Request handling used across policy transitions. */
/** @brief Discard the deferred Request message, if any. */
static void pd_clear_deferred_request(void)
{
    s_deferred_request_valid = false;
    s_deferred_request_len = 0u;
    s_deferred_request_sop = PD_RX_SOP0;
}

/** @brief Save the current Request message for replay after a transition. */
static void pd_defer_current_request(void)
{
    uint8_t i;

    if ((s_rx_msg_len < 10u) || (s_rx_msg_len > PD_RX_BUF_SIZE)) {
        return;
    }
    for (i = 0u; i < s_rx_msg_len; i++) {
        s_deferred_request_buf[i] = s_pd_rx_buf[i];
    }
    s_deferred_request_len = s_rx_msg_len;
    s_deferred_request_sop = s_rx_sop;
    s_deferred_request_valid = true;
    pd_debug_note_event(PD_DEBUG_EVENT_REQUEST_DEFERRED);
}

/** @brief Map an RX SOP identifier to an internal message-ID slot. */
static uint8_t pd_rx_sop_index(uint8_t sop)
{
    if (sop == PD_RX_SOP1_HRST) {
        return 1u;
    }
    if (sop == PD_RX_SOP2_CRST) {
        return 2u;
    }
    return 0u;
}

/** @brief Clear duplicate-detection message IDs for all SOP types. */
static void pd_clear_rx_message_ids(void)
{
    uint8_t i;

    for (i = 0u; i < 3u; i++) {
        s_rx_last_msg_id[i] = 0u;
        s_rx_last_msg_id_valid[i] = 0u;
    }
}

/** @brief Check whether the current RX message repeats its previous ID. */
static bool pd_rx_message_is_duplicate(uint8_t msg_type)
{
    uint8_t index;
    uint8_t msg_id;

    if (msg_type == DEF_TYPE_SOFT_RESET) {
        return false;
    }

    index = pd_rx_sop_index(s_rx_sop);
    msg_id = (uint8_t)((s_pd_rx_buf[1] >> 1u) & 0x07u);
    if ((s_rx_last_msg_id_valid[index] != 0u) &&
        (s_rx_last_msg_id[index] == msg_id)) {
        return true;
    }
    s_rx_last_msg_id[index] = msg_id;
    s_rx_last_msg_id_valid[index] = 1u;
    return false;
}

/** @brief Clear the staged Request values waiting for policy approval. */
static void pd_clear_staged_request(void)
{
    s_staged_request_valid = false;
    s_staged_request_pdo = 0u;
    s_staged_request_is_pps = false;
    s_staged_request_is_epr = false;
    s_staged_request_supply_type = PD_SUPPLY_TYPE_NONE;
    s_staged_request_mv = 0u;
    s_staged_request_ma = 0u;
    s_staged_request_rdo = 0ul;
    s_staged_request_spr_rdo = 0ul;
}

/** @brief Store a validated Request until its power transition completes. */
static void pd_stage_request(uint16_t mv, uint16_t ma, uint8_t pdo_index)
{
    s_staged_request_valid = true;
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
static bool pd_commit_staged_request(void)
{
    if (!s_staged_request_valid) {
        return false;
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
    return true;
}

/** @brief Check whether the active contract uses an EPR supply type. */
static bool pd_active_contract_is_epr(void)
{
    return ((s_epr_mode_active) || (s_last_request_is_epr) ||
            (s_active_supply_type == PD_SUPPLY_TYPE_EPR_FIXED) ||
            (s_active_supply_type == PD_SUPPLY_TYPE_EPR_AVS));
}

void pd_epr_exit_to_spr_recovery(void)
{
    if (s_epr_mode_tx_active) {
        s_ext_response_tx_active = false;
    }
    s_epr_keepalive_pending = false;
    s_epr_keepalive_ack_pending = false;
    s_epr_resume_state = 0u;
    s_epr_resume_ms = 0u;
    s_epr_mode_active = false;
    s_epr_mode_tx_active = false;
    s_epr_source_cap_pending = false;
    s_epr_source_cap_tx_active = false;
    s_epr_source_cap_retry = 0u;
    s_epr_enter_wait_active = false;
    s_epr_enter_wait_ms = 0u;
    s_last_request_is_epr = false;
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
static bool pd_is_epr_keepalive_request(uint8_t msg_type,
                                           uint8_t extended,
                                           uint8_t num_do)
{
    uint16_t ext_header;
    uint16_t data_len;
    uint8_t chunk_number;

    if ((msg_type != PD_EXT_EXTENDED_CONTROL) ||
        (extended == 0u) ||
        (num_do == 0u) ||
        (s_rx_msg_len < 10u)) {
        return false;
    }

    ext_header = (uint16_t)s_pd_rx_buf[2] |
                 ((uint16_t)s_pd_rx_buf[3] << 8u);
    data_len = (uint16_t)(ext_header & PD_EXT_HEADER_DATA_SIZE_MASK);
    chunk_number = (uint8_t)((ext_header >> PD_EXT_HEADER_CHUNK_NUM_SHIFT) &
                             PD_EXT_HEADER_CHUNK_NUM_MASK);
    if (((ext_header & PD_EXT_HEADER_REQUEST_CHUNK) != 0u) ||
        (chunk_number != 0u) ||
        (data_len == 0u) ||
        (data_len > PD_EXT_CONTROL_DATA_LEN)) {
        return false;
    }

    if ((s_pd_rx_buf[4] == PD_EXT_CTRL_EPR_KEEP_ALIVE) &&
        ((data_len < PD_EXT_CONTROL_DATA_LEN) || (s_pd_rx_buf[5] == 0u))) {
        return true;
    }
    return false;
}

/** @brief Check whether an auxiliary policy owner has a pending TX result. */
static bool pd_tx_result_has_aux_owner(void)
{
    if ((s_ext_response_tx_active) ||
        (s_fault_alert_tx_active) ||
        (s_revision_query_tx_active)) {
        return true;
    }
    if ((s_cable_state == PD_CABLE_WAIT_DISCOVER_ID) &&
        (s_tx.sop == UPD_SOP1)) {
        return true;
    }
    return false;
}

/** @brief Clear state-owned transmit ownership and result context. */
static void pd_clear_state_tx_context(void)
{
    s_state_tx_active = false;
    if (!pd_tx_result_has_aux_owner()) {
        s_tx.result = PD_TX_RESULT_NONE;
    }
}

/** @brief Consume a transmit result owned by the active PD state. */
static bool pd_state_take_tx_result(pd_tx_result_t *result)
{
    pd_tx_result_t dropped;

    if (!s_state_tx_active) {
        if ((s_tx.result != PD_TX_RESULT_NONE) &&
            (!pd_tx_result_has_aux_owner())) {
            (void)pd_tx_take_result(&dropped);
        }
        return false;
    }

    if (pd_tx_take_result(result) == 0u) {
        return false;
    }
    s_state_tx_active = false;
    return true;
}

/** @brief Start a control message owned by the active policy state. */
static bool pd_state_start_control(uint8_t msg_type)
{
    if (!pd_tx_start_control(msg_type)) {
        return false;
    }
    s_state_tx_active = true;
    return true;
}

/** @brief Start a hard-reset sequence owned by the active policy state. */
static void pd_state_start_hard_reset(void)
{
    if ((!pd_tx_is_idle()) || (s_tx.result != PD_TX_RESULT_NONE)) {
        return;
    }
    pd_tx_start_hard_reset();
    if (!pd_tx_is_idle()) {
        s_state_tx_active = true;
    }
}

/** @brief Reset the partner revision-query state machine. */
static void pd_revision_query_reset(void)
{
    s_partner_revision = PD_PARTNER_REVISION_UNKNOWN;
    s_revision_query_state = PD_REVISION_QUERY_IDLE;
    s_revision_query_tx_active = false;
    s_revision_query_wait_ms = 0u;
}

/** @brief Complete a revision query with the negotiated revision value. */
static void pd_revision_query_finish(uint8_t revision)
{
    bool disable_spr_avs =
        ((revision == PD_PARTNER_REVISION_30) ||
         (revision == PD_PARTNER_REVISION_31)) &&
        ((s_partner_revision == PD_PARTNER_REVISION_UNKNOWN) ||
         (s_partner_revision == PD_PARTNER_REVISION_3X));

    s_partner_revision = revision;
    s_revision_query_state = PD_REVISION_QUERY_DONE;
    s_revision_query_tx_active = false;
    s_revision_query_wait_ms = 0u;

    if (disable_spr_avs) {
        s_source_cap_refresh_pending = true;
        s_source_cap_refresh_retries = 1u;
        s_source_cap_refresh_wait_ms = 0u;
    }
}

/** @brief Complete a revision query when the partner does not respond. */
static void pd_revision_query_finish_unresolved(void)
{
    s_revision_query_state = PD_REVISION_QUERY_DONE;
    s_revision_query_tx_active = false;
    s_revision_query_wait_ms = 0u;
}

/** @brief Process a message received while querying partner revision support. */
static bool pd_revision_query_handle_message(uint8_t msg_type,
                                                uint8_t num_do,
                                                uint8_t extended)
{
    uint32_t rmdo;
    uint8_t major;
    uint8_t minor;

    if ((s_rx_sop != PD_RX_SOP0) ||
        (s_revision_query_state != PD_REVISION_QUERY_WAIT) ||
        (extended != 0u)) {
        return false;
    }

    if ((msg_type == DEF_TYPE_REVISION) && (num_do == 1u)) {
        rmdo = pd_get_u32_le(&s_pd_rx_buf[2]);
        major = (uint8_t)((rmdo & PD_RMDO_REV_MAJOR_MASK) >>
                          PD_RMDO_REV_MAJOR_SHIFT);
        minor = (uint8_t)((rmdo & PD_RMDO_REV_MINOR_MASK) >>
                          PD_RMDO_REV_MINOR_SHIFT);
        if ((major > 3u) || ((major == 3u) && (minor >= 2u))) {
            pd_revision_query_finish(PD_PARTNER_REVISION_32);
        } else if ((major == 3u) && (minor == 1u)) {
            pd_revision_query_finish(PD_PARTNER_REVISION_31);
        } else {
            pd_revision_query_finish(PD_PARTNER_REVISION_30);
        }
        return true;
    }

    if ((msg_type == DEF_TYPE_NOT_SUPPORT) && (num_do == 0u)) {
        pd_revision_query_finish(PD_PARTNER_REVISION_30);
        return true;
    }
    return false;
}

/** @brief Advance the partner revision-query timeout and retry state. */
static void pd_revision_query_task(uint8_t elapsed_ms)
{
    pd_tx_result_t result;
    uint32_t next_ms;

    if (s_revision_query_state == PD_REVISION_QUERY_DELAY) {
        if ((!s_pd.connected) ||
            (s_pd.contract_mv == 0u) ||
            (s_pd.state != PD_STATE_READY)) {
            return;
        }
        next_ms = (uint32_t)s_revision_query_wait_ms + elapsed_ms;
        s_revision_query_wait_ms = (next_ms > 0xFFFFul) ?
                                   0xFFFFu : (uint16_t)next_ms;
        if ((s_revision_query_wait_ms < PD_REVISION_QUERY_DELAY_MS) ||
            (s_rx_msg_pending) ||
            (s_goodcrc_tx_pending) ||
            (s_goodcrc_tx_active) ||
            (s_ext_response_tx_active) ||
            ((s_epr_mode_active) &&
             (s_debug_epr_chunk_phase >= 1u) &&
             (s_debug_epr_chunk_phase < 5u)) ||
            (s_fault_alert_tx_active) ||
            (s_source_cap_refresh_pending) ||
            (!pd_tx_is_idle()) ||
            (s_tx.result != PD_TX_RESULT_NONE)) {
            return;
        }
        if (pd_tx_start_control(DEF_TYPE_GET_REVISION)) {
            s_revision_query_state = PD_REVISION_QUERY_TX;
            s_revision_query_tx_active = true;
            s_revision_query_wait_ms = 0u;
        }
        return;
    }

    if (s_revision_query_state == PD_REVISION_QUERY_TX) {
        if (pd_tx_take_result(&result) == 0u) {
            return;
        }
        s_revision_query_tx_active = false;
        if (result == PD_TX_RESULT_OK) {
            s_revision_query_state = PD_REVISION_QUERY_WAIT;
            s_revision_query_wait_ms = 0u;
        } else {
            pd_revision_query_finish_unresolved();
        }
        return;
    }

    if (s_revision_query_state == PD_REVISION_QUERY_WAIT) {
        next_ms = (uint32_t)s_revision_query_wait_ms + elapsed_ms;
        s_revision_query_wait_ms = (next_ms > 0xFFFFul) ?
                                   0xFFFFu : (uint16_t)next_ms;
        if (s_revision_query_wait_ms >= PD_SENDER_RESPONSE_MAX_MS) {
            pd_revision_query_finish_unresolved();
        }
    }
}

void pd_set_state(pd_state_t state)
{
    pd_state_t old_state = s_pd.state;

    s_pd.state = state;
    s_pd.state_timer_ms = 0u;

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
        s_pd_revision_locked = false;
        pd_revision_query_reset();
        pd_clear_contract_info();
        pd_clear_deferred_request();
        s_epr_mode_active = false;
        s_epr_mode_tx_active = false;
        s_epr_source_cap_pending = false;
        s_epr_source_cap_tx_active = false;
        s_epr_source_cap_retry = 0u;
        s_epr_advertise_ready = false;
        s_epr_keepalive_pending = false;
        s_epr_keepalive_ack_pending = false;
        s_epr_resume_state = 0u;
        s_epr_resume_ms = 0u;
        s_epr_last_rx_action = 0u;
        s_epr_last_tx_action = 0u;
        s_epr_last_tx_data = 0u;
        s_epr_fail_reason = PD_EPR_FAIL_NONE;
        s_epr_enter_wait_active = false;
        s_epr_enter_wait_ms = 0u;
        s_source_cap_refresh_pending = false;
        s_source_cap_refresh_retries = 0u;
        s_source_cap_refresh_wait_ms = 0u;
        s_ext_response_tx_active = false;
        s_fault_alert_tx_active = false;
        pd_clear_staged_request();
    }

    if (state == PD_STATE_HARD_RESET) {
        /* A Hard Reset resets Protocol Layer MessageID counters in both
         * directions.  Otherwise the Sink's first post-reset MessageID 0
         * frame can be discarded as a duplicate of pre-reset traffic. */
        pd_clear_rx_message_ids();
        /* SOP and SOP' revision detection both restart after Hard Reset. */
        s_pd.pd_revision = 1u;
        s_cable_pd_revision = 1u;
        s_pd_revision_locked = false;
        pd_revision_query_reset();
        if (pd_active_contract_is_epr()) {
            pd_epr_exit_to_spr_recovery();
        }
        s_epr_mode_active = false;
        s_epr_mode_tx_active = false;
        s_epr_source_cap_pending = false;
        s_epr_source_cap_tx_active = false;
        s_epr_source_cap_retry = 0u;
        s_epr_keepalive_pending = false;
        s_epr_keepalive_ack_pending = false;
        s_epr_resume_state = 0u;
        s_epr_resume_ms = 0u;
        s_epr_enter_wait_active = false;
        s_epr_enter_wait_ms = 0u;
        s_ext_response_tx_active = false;
        s_fault_alert_tx_active = false;
        pd_clear_staged_request();
        pd_clear_deferred_request();
        pd_cable_reset();
        s_debug_power_request_reason = PD_DEBUG_POWER_REQ_HARD_RESET;
        s_debug_power_request_enable = true;
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
    s_pd.contract_mv = 0u;
    s_pd.contract_ma = 0u;
    s_pd.selected_pdo = 0u;
    s_last_request_pdo = 0u;
    s_last_request_mv = 0u;
    s_last_request_ma = 0u;
    s_last_request_is_pps = false;
    s_last_request_is_epr = false;
    s_pps_active = false;
    s_active_supply_type = PD_SUPPLY_TYPE_NONE;
    s_last_rdo = 0ul;
    s_last_spr_rdo = 0ul;
}

void pd_reset_context(bool disable_power)
{
    memset(&s_pd, 0, sizeof(s_pd));
    memset(&s_tx, 0, sizeof(s_tx));
    memset(&s_cc, 0, sizeof(s_cc));
    memset(&s_power, 0, sizeof(s_power));
    pd_debug_reset();
    s_last_msg_type = 0u;
    s_last_num_do = 0u;
    s_last_extended = false;
    s_last_request_pdo = 0u;
    s_last_rdo_pos = 0u;
    s_last_rdo = 0ul;
    s_last_spr_rdo = 0ul;
    s_last_request_mv = 0u;
    s_last_request_ma = 0u;
    s_last_request_is_pps = false;
    s_last_request_is_epr = false;
    s_pps_active = false;
    s_epr_mode_active = false;
    s_active_supply_type = PD_SUPPLY_TYPE_NONE;
    s_epr_mode_tx_active = false;
    s_epr_source_cap_pending = false;
    s_epr_source_cap_tx_active = false;
    s_epr_source_cap_retry = 0u;
    s_epr_advertise_ready = false;
    s_epr_keepalive_pending = false;
    s_epr_keepalive_ack_pending = false;
    s_epr_resume_state = 0u;
    s_epr_resume_ms = 0u;
    s_epr_last_rx_action = 0u;
    s_epr_last_tx_action = 0u;
    s_epr_last_tx_data = 0u;
    s_epr_fail_reason = PD_EPR_FAIL_NONE;
    s_epr_enter_wait_active = false;
    s_epr_enter_wait_ms = 0u;
    s_source_cap_refresh_pending = false;
    s_source_cap_refresh_retries = 0u;
    s_source_cap_refresh_wait_ms = 0u;
    s_rx_msg_pending = false;
    s_rx_msg_len = 0u;
    s_rx_sop = PD_RX_SOP0;
    s_rx_ring_head = 0u;
    s_rx_ring_tail = 0u;
    s_cable_epr_capable = false;
    s_goodcrc_tx_active = false;
    s_goodcrc_tx_pending = false;
    s_goodcrc_watchdog_ms = 0u;
    s_goodcrc_sop = UPD_SOP0;
    s_pd_reset_pending = false;
    s_ext_response_tx_active = false;
    s_state_tx_active = false;
    s_replaying_deferred_request = false;
    s_fault_recovery_pending = false;
    s_pd_revision_locked = false;
    pd_revision_query_reset();
    pd_reset_source_comm_timers();
    pd_clear_staged_request();
    s_fault_bits = 0u;
    s_fault_alert_pending = false;
    s_fault_alert_sent = false;
    s_fault_alert_tx_active = false;
    s_fault_alert_tx_bits = 0u;
    s_pd.state = PD_STATE_DISCONNECTED;
    pd_clear_deferred_request();
    pd_clear_rx_message_ids();
    s_pd.pd_revision = 1u;
    s_cable_pd_revision = 1u;
    pd_phy_reset_cc();
    pd_power_abort();
    pd_frontend_restart();
    pd_cable_reset();
    pd_goodcrc_timer_stop();
    if (disable_power) {
        s_debug_power_request_reason = PD_DEBUG_POWER_REQ_FAULT_OFF;
        s_debug_power_request_enable = false;
        pd_power_request(false, 0u, 0u);
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
    s_rx_msg_pending = false;
    s_rx_msg_len = 0u;
    s_rx_ring_head = 0u;
    s_rx_ring_tail = 0u;
    __enable_irq();

    pd_clear_staged_request();
    pd_clear_deferred_request();
    pd_clear_rx_message_ids();
    s_pd.msg_id = 0u;
    pd_set_state(PD_STATE_SEND_SOFT_RESET);
}

/** @brief Check whether a message should be ignored during power transition. */
static bool pd_ignore_message_during_transition(uint8_t msg_type,
                                                   uint8_t num_do)
{
    uint8_t epr_action;

    if ((msg_type == DEF_TYPE_GOODCRC) || (msg_type == DEF_TYPE_SOFT_RESET)) {
        return false;
    }

    /* RX policy runs before the main state consumes the preceding TX result.
     * Serialize a fast EPR Enter behind an in-flight Source AMS instead of
     * rejecting it merely because the software state has not reached READY. */
    if ((msg_type == DEF_TYPE_EPR_MODE) &&
        (num_do == 1u) &&
        (s_rx_msg_len >= 10u)) {
        epr_action = (uint8_t)((pd_get_u32_le(&s_pd_rx_buf[2]) >>
                                PD_EPR_MODE_ACTION_SHIFT) &
                               PD_EPR_MODE_FIELD_MASK);
        if ((epr_action == PD_EPR_MODE_ACTION_ENTER) &&
            (s_pd.contract_mv != 0u) &&
            (s_state_tx_active) &&
            (pd_tx_is_idle()) &&
            (s_tx.result == PD_TX_RESULT_OK) &&
            ((s_pd.state == PD_STATE_SEND_SOURCE_CAP) ||
             (s_pd.state == PD_STATE_SEND_PS_RDY) ||
             (s_pd.state == PD_STATE_SEND_CONTROL))) {
            pd_defer_current_request();
            return true;
        }
    }
    if (!pd_is_power_transition_state()) {
        return false;
    }
    if (((msg_type == DEF_TYPE_REQUEST) && (num_do == 1u)) ||
        ((msg_type == DEF_TYPE_EPR_REQUEST) && (num_do == 2u))) {
        /* Preserve the newest Request without starting another AMS until the
         * current Accept -> power transition -> PS_RDY sequence completes. */
        pd_defer_current_request();
    }
    return true;
}

/** @brief Check whether the configured source caps contain a 5 A PDO. */
static bool pd_has_5a_pdo(void)
{
    uint8_t i;

    for (i = 0u; i < s_pdo_count; i++) {
        if (s_pdos[i].ma >= PD_5A_CURRENT_MA) {
            return true;
        }
    }
    if (pd_current_limit_ma() >= PD_5A_CURRENT_MA) {
        return true;
    }
    return false;
}

/** @brief Return the configured fixed-PDO current limit at a voltage. */
static uint16_t pd_fixed_pdo_max_ma(uint16_t mv)
{
    return (mv >= 20000u) ? PD_5A_CURRENT_MA : PD_DEFAULT_CURRENT_MA;
}

/** @brief Check whether a newly discovered 5 A cable needs cap refresh. */
static bool pd_5a_refresh_needed(void)
{
    return ((s_supports_5a) && (s_cable_current_ma >= PD_5A_CURRENT_MA) &&
            (s_pd.contract_ma != 0u) && (s_pd.contract_ma < PD_5A_CURRENT_MA) &&
            (s_source_cap_refresh_retries != 0u) && pd_has_5a_pdo());
}

/** @brief Record that a source-capability refresh transmission has started. */
static void pd_source_cap_refresh_started(void)
{
    if (s_source_cap_refresh_pending) {
        s_source_cap_refresh_pending = false;
        s_source_cap_refresh_wait_ms = 0u;
        if (s_source_cap_refresh_retries != 0u) {
            s_source_cap_refresh_retries--;
        }
    }
}

/** @brief Start the source-capability message selected by the policy state. */
static bool pd_state_start_source_cap(void)
{
    uint8_t started;

    if (s_epr_mode_active) {
        started = pd_tx_start_epr_source_cap_ext();
    } else {
        started = pd_tx_start_source_cap();
    }
    if (started == 0u) {
        return false;
    }
    s_state_tx_active = true;
    if (s_epr_mode_active) {
        s_epr_source_cap_pending = false;
        s_epr_source_cap_retry = 0u;
    }
    pd_source_cap_refresh_started();
    return true;
}

bool pd_is_power_transition_state(void)
{
    return ((s_pd.state == PD_STATE_SEND_ACCEPT) ||
            (s_pd.state == PD_STATE_APPLY_POWER) ||
            (s_pd.state == PD_STATE_SEND_PS_RDY));
}

/** @brief Start one extended response chunk owned by the policy state. */
static bool pd_tx_extended_response(uint8_t msg_type, uint8_t chunk_number)
{
    pd_tx_result_t old_result;

    if ((s_pd.pd_revision == 0u) ||
        (!pd_tx_is_idle())) {
        return false;
    }
    if (s_tx.result != PD_TX_RESULT_NONE) {
        (void)pd_tx_take_result(&old_result);
    }

    if (msg_type == DEF_TYPE_SRC_CAP) {
        if (pd_tx_start_source_cap_ext_chunk(chunk_number)) {
            s_ext_response_tx_active = true;
            return true;
        }
        return false;
    }
    if (msg_type == DEF_TYPE_GET_STATUS_R) {
        if (pd_tx_start_status_chunk(chunk_number)) {
            s_ext_response_tx_active = true;
            return true;
        }
        return false;
    }
    if (msg_type == PD_EXT_EPR_SOURCE_CAP) {
        if (pd_tx_start_epr_source_cap_ext_chunk(chunk_number)) {
            s_epr_source_cap_tx_active = true;
            s_ext_response_tx_active = true;
            return true;
        }
        return false;
    }
    return false;
}

/** @brief Start the partner revision response for the active query. */
static bool pd_tx_revision_response(void)
{
    pd_tx_result_t old_result;

    if ((s_pd.pd_revision == 0u) ||
        (!pd_tx_is_idle())) {
        return false;
    }
    if (s_tx.result != PD_TX_RESULT_NONE) {
        (void)pd_tx_take_result(&old_result);
    }

    if (pd_tx_start_revision()) {
        s_ext_response_tx_active = true;
        return true;
    }
    return false;
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

    if (pd_is_power_transition_state()) {
        return;
    }
    if ((num_do == 0u) || (s_rx_msg_len < 6u)) {
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
        return;
    }

    ext_header = (uint16_t)s_pd_rx_buf[2] |
                 ((uint16_t)s_pd_rx_buf[3] << 8u);
    data_len = (uint16_t)(ext_header & PD_EXT_HEADER_DATA_SIZE_MASK);
    chunk_number = (uint8_t)((ext_header >> PD_EXT_HEADER_CHUNK_NUM_SHIFT) &
                             PD_EXT_HEADER_CHUNK_NUM_MASK);

    if ((ext_header & PD_EXT_HEADER_REQUEST_CHUNK) != 0u) {
        if ((msg_type == PD_EXT_EPR_SOURCE_CAP) &&
            ((ext_header & PD_EXT_HEADER_CHUNKED) != 0u) &&
            (data_len == 0u)) {
            pd_debug_note_epr_chunk_phase(3u);
        }
        if (pd_tx_extended_response(msg_type, chunk_number) != 0u) {
            return;
        }
    }

    if ((msg_type == PD_EXT_EXTENDED_CONTROL) &&
        (chunk_number == 0u) &&
        (data_len >= 1u) &&
        (data_len <= PD_EXT_CONTROL_DATA_LEN) &&
        (s_rx_msg_len >= 10u)) {
        ext_ctrl_type = s_pd_rx_buf[4];
        ext_ctrl_data = (data_len >= PD_EXT_CONTROL_DATA_LEN) ? s_pd_rx_buf[5] : 0u;
        if ((ext_ctrl_type == PD_EXT_CTRL_EPR_GET_SRC_CAP) &&
            (ext_ctrl_data == 0u)) {
            if (pd_tx_extended_response(PD_EXT_EPR_SOURCE_CAP, 0u) != 0u) {
                return;
            }
        } else if ((ext_ctrl_type == PD_EXT_CTRL_EPR_KEEP_ALIVE) &&
                   (ext_ctrl_data == 0u) &&
                   (s_epr_mode_active)) {
            if (s_tx.result != PD_TX_RESULT_NONE) {
                (void)pd_tx_take_result(&old_result);
            }
            if (pd_tx_start_extended_control(PD_EXT_CTRL_EPR_KEEP_ALIVE_A, 0u)) {
                s_ext_response_tx_active = true;
                return;
            }
        }
        s_epr_fail_reason = (uint8_t)(20u + ext_ctrl_type);
    }

    pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
}

/* Incoming message dispatch and main policy state machine. */
static void pd_handle_message(void)
{
    uint8_t msg_type = (uint8_t)(s_pd_rx_buf[0] & 0x1Fu);
    uint8_t num_do = (uint8_t)((s_pd_rx_buf[1] >> 4u) & 0x07u);
    uint8_t extended = (uint8_t)((s_pd_rx_buf[1] & 0x80u) != 0u);
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
        s_cable_pd_revision = ((s_pd_rx_buf[0] & 0xC0u) == 0x80u) ? 1u : 0u;
    } else if ((!s_pd_revision_locked) &&
               (extended == 0u) &&
               (msg_type == DEF_TYPE_REQUEST) &&
               (num_do == 1u)) {
        uint8_t header_revision = (uint8_t)(s_pd_rx_buf[0] & 0xC0u);

        /* USB PD 6.1.3.1: the Sink's initial Request selects PD2.0 or PD3.x. */
        if (header_revision == 0x80u) {
            s_pd.pd_revision = 1u;
            s_pd_revision_locked = true;
            s_partner_revision = PD_PARTNER_REVISION_3X;
            s_revision_query_state = PD_REVISION_QUERY_DELAY;
            s_revision_query_wait_ms = 0u;
        } else if ((header_revision == 0x40u) ||
                   (header_revision == 0x00u)) {
            s_pd.pd_revision = 0u;
            s_pd_revision_locked = true;
            s_partner_revision = PD_PARTNER_REVISION_20;
            s_revision_query_state = PD_REVISION_QUERY_DONE;
        }
    }

    if (s_debug_rx_msg_count < 0xFFFFu) {
        s_debug_rx_msg_count++;
    }

    if ((!s_replaying_deferred_request) &&
        (pd_rx_message_is_duplicate(msg_type) != 0u)) {
        /* A duplicate frame still proves that the Port Partner is alive. Section 7.30.5.2
         * lets the Source Hard Reset when there is no traffic for tSourceEPRKeepAlive, and
         * a Sink whose EPR_Keep_Alive_Ack was lost retransmits the same KeepAlive with an
         * unchanged MessageID. Dropping those retransmissions before the timer was
         * restarted made the Source believe the link had gone silent and Hard Reset every
         * tSourceEPRKeepAlive, which is exactly the reset loop seen with a 28 V load. */
        pd_reset_source_comm_timers();
        return;
    }

    if ((s_rx_sop == PD_RX_SOP1_HRST) || (s_rx_sop == PD_RX_SOP2_CRST)) {
        if ((msg_type == DEF_TYPE_VENDOR_DEFINED) && (num_do != 0u)) {
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

    s_debug_rx_msg_id = (uint8_t)((s_pd_rx_buf[1] >> 1u) & 0x07u);
    s_debug_rx_msg_id_valid = true;
    s_debug_rx_sop = s_rx_sop;
    if ((extended == 0u) && (msg_type == DEF_TYPE_REQUEST) && (num_do == 1u)) {
        pd_debug_note_event(PD_DEBUG_EVENT_RX_REQUEST);
    } else if ((extended == 0u) &&
               (msg_type == DEF_TYPE_EPR_REQUEST) &&
               (num_do == 2u)) {
        pd_debug_note_event(PD_DEBUG_EVENT_RX_EPR_REQUEST);
    }

    if (s_pd.state == PD_STATE_SEND_SOFT_RESET) {
        return;
    }

    if (pd_revision_query_handle_message(msg_type, num_do, extended) != 0u) {
        return;
    }

    if (pd_is_epr_keepalive_request(msg_type, extended, num_do) != 0u) {
        if (s_epr_mode_active) {
            if (s_debug_keepalive_rx_count < 0xFFFFu) {
                s_debug_keepalive_rx_count++;
            }
            pd_reset_source_comm_timers();
            pd_debug_note_event(PD_DEBUG_EVENT_KEEPALIVE_RX);
            s_epr_keepalive_pending = false;
            if (pd_is_power_transition_state()) {
                if ((s_state_tx_active) || (s_tx.result != PD_TX_RESULT_NONE)) {
                    /* A frame of the running sequence is on the wire, so the acknowledgement
                     * cannot start yet. PD_STATE_READY sends it once the sequence ends. */
                    s_epr_keepalive_ack_pending = true;
                    return;
                }
                /* Nothing in flight, so the acknowledgement can go out right away (the Sink
                 * expects it within tSenderResponse) and the interrupted sequence is resumed
                 * after it. Deferring instead would leave a long EPR transition without an
                 * answer, and the Sink would drop out of EPR Mode while VBUS stays up. */
                s_epr_resume_state = (uint8_t)s_pd.state;
                s_epr_resume_ms = s_pd.state_timer_ms;
                s_epr_keepalive_ack_pending = false;
                pd_set_state(PD_STATE_SEND_EPR_KEEPALIVE_ACK);
                return;
            }
            s_epr_resume_state = 0u;
            s_epr_resume_ms = 0u;
            s_epr_keepalive_ack_pending = false;
            pd_set_state(PD_STATE_SEND_EPR_KEEPALIVE_ACK);
            return;
        }
    }

    if (pd_ignore_message_during_transition(msg_type, num_do) != 0u) {
        return;
    }

    if (s_pd.state == PD_STATE_WAIT_SOFT_RESET_ACCEPT) {
        if ((extended == 0u) &&
            (msg_type == DEF_TYPE_ACCEPT) &&
            (num_do == 0u)) {
            pd_set_state(PD_STATE_SEND_SOURCE_CAP);
        } else {
            pd_set_state(PD_STATE_HARD_RESET);
        }
        return;
    }

    if (extended != 0u) {
        if (msg_type != DEF_TYPE_GOODCRC) {
            pd_handle_extended_message(msg_type, num_do);
        }
    } else if ((msg_type == DEF_TYPE_VENDOR_DEFINED) && (num_do != 0u)) {
        pd_vdm_handle_message(num_do);
    } else if ((msg_type == DEF_TYPE_BIST) && (num_do != 0u)) {
        return;
    } else if ((msg_type == DEF_TYPE_REQUEST) && (num_do == 1u)) {
        if (s_epr_mode_active) {
            pd_set_state(PD_STATE_HARD_RESET);
            return;
        }
        old_pps_active = s_pps_active;
        old_active_supply_type = s_active_supply_type;
        old_last_spr_rdo = s_last_spr_rdo;
        if (pd_validate_request(&mv, &ma, &pdo_index)) {
            pd_stage_request(mv, ma, pdo_index);
            s_pps_active = old_pps_active;
            s_active_supply_type = old_active_supply_type;
            s_last_spr_rdo = old_last_spr_rdo;
            s_pd.control_msg = 0u;
            pd_set_state(PD_STATE_SEND_ACCEPT);
        } else {
            s_pps_active = old_pps_active;
            s_active_supply_type = old_active_supply_type;
            s_last_spr_rdo = old_last_spr_rdo;
            s_last_request_pdo = 0u;
            s_last_request_mv = 0u;
            s_last_request_ma = 0u;
            s_last_request_is_pps = false;
            pd_queue_control(DEF_TYPE_REJECT,
                             (pd_fault_active()) ? s_pd.state : PD_STATE_SEND_SOURCE_CAP);
        }
    } else if ((msg_type == DEF_TYPE_EPR_MODE) && (num_do == 1u)) {
        if (pd_handle_epr_mode_request()) {
            s_pd.control_msg = 0u;
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type == DEF_TYPE_EPR_REQUEST) {
        if (!s_epr_mode_active) {
            pd_set_state(PD_STATE_HARD_RESET);
            return;
        }
        old_pps_active = s_pps_active;
        old_active_supply_type = s_active_supply_type;
        old_last_spr_rdo = s_last_spr_rdo;
        if ((num_do == 2u) &&
            (pd_validate_epr_request(&mv, &ma, &pdo_index))) {
            pd_stage_request(mv, ma, pdo_index);
            s_pps_active = old_pps_active;
            s_active_supply_type = old_active_supply_type;
            s_last_spr_rdo = old_last_spr_rdo;
            s_pd.control_msg = 0u;
            pd_set_state(PD_STATE_SEND_ACCEPT);
        } else {
            s_pps_active = old_pps_active;
            s_active_supply_type = old_active_supply_type;
            s_last_spr_rdo = old_last_spr_rdo;
            s_last_request_pdo = 0u;
            s_last_request_mv = 0u;
            s_last_request_ma = 0u;
            s_last_request_is_epr = false;
            if (s_epr_fail_reason == PD_EPR_FAIL_NONE) {
                s_epr_fail_reason = PD_EPR_FAIL_BAD_REQUEST;
            }
            pd_queue_control(DEF_TYPE_REJECT, s_pd.state);
        }
    } else if (msg_type == DEF_TYPE_GET_SRC_CAP) {
        if (pd_is_power_transition_state()) {
            return;
        }
        if (s_epr_mode_active) {
            if (pd_tx_start_source_cap()) {
                s_ext_response_tx_active = true;
                return;
            }
            pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
            return;
        }
        s_pd.control_msg = 0u;
        s_pd.source_cap_retry = 0u;
        pd_set_state(PD_STATE_SEND_SOURCE_CAP);
    } else if (msg_type == DEF_TYPE_GET_SRC_CAP_EX) {
        if (pd_is_power_transition_state()) {
            return;
        }
        if (pd_tx_extended_response(DEF_TYPE_SRC_CAP, 0u) != 0u) {
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type == DEF_TYPE_SOFT_RESET) {
        pd_clear_staged_request();
        pd_clear_deferred_request();
        s_pd.msg_id = 0u;
        pd_clear_rx_message_ids();
        pd_queue_control(DEF_TYPE_ACCEPT, PD_STATE_SEND_SOURCE_CAP);
    } else if (msg_type == DEF_TYPE_GET_STATUS) {
        if (pd_is_power_transition_state()) {
            return;
        }
        if (pd_tx_extended_response(DEF_TYPE_GET_STATUS_R, 0u) != 0u) {
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type == DEF_TYPE_GET_PPS_STATUS) {
        if (pd_is_power_transition_state()) {
            return;
        }
        if (pd_tx_start_pps_status()) {
            s_ext_response_tx_active = true;
            return;
        }
        pd_queue_control(DEF_TYPE_NOT_SUPPORT, s_pd.state);
    } else if (msg_type == DEF_TYPE_GET_REVISION) {
        if (pd_tx_revision_response()) {
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
        if (pd_is_power_transition_state()) {
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
    if ((pd_fault_active()) && (s_power.target_enable)) {
        s_debug_power_request_reason = PD_DEBUG_POWER_REQ_FAULT_OFF;
        s_debug_power_request_enable = false;
        pd_power_request(false, 0u, 0u);
    }
    if ((s_fault_recovery_pending) && (!pd_fault_active())) {
        if ((s_power.target_enable) ||
            (s_power.state == PD_POWER_ERROR)) {
            s_debug_power_request_reason = PD_DEBUG_POWER_REQ_FAULT_REC;
            s_debug_power_request_enable = false;
            pd_power_request(false, 0u, 0u);
            return;
        }
        if ((s_power.state != PD_POWER_READY_OFF) &&
            (s_power.state != PD_POWER_OFF)) {
            return;
        }
        s_fault_recovery_pending = false;
        if (s_pd.connected) {
            s_pd.source_cap_retry = 0u;
            pd_set_state(PD_STATE_ATTACHED);
        }
    }

    switch (s_pd.state) {
    case PD_STATE_ATTACHED:
        if (pd_fault_active()) {
            break;
        }
        if (!pd_power_request_is(1u, PD_VSAFE5V_MV)) {
            s_debug_power_request_reason = PD_DEBUG_POWER_REQ_ATTACHED;
            s_debug_power_request_enable = true;
            pd_power_request(true, PD_VSAFE5V_MV, PD_VSAFE5V_MA);
        }
        if (!pd_power_is_ready_on()) {
            s_pd.state_timer_ms = 0u;
            if (pd_power_has_error()) {
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
            (pd_cable_discovery_finished())) {
            pd_set_state(PD_STATE_SEND_SOURCE_CAP);
        }
        break;

    case PD_STATE_SEND_SOURCE_CAP:
        if (pd_fault_active()) {
            if (pd_tx_is_idle()) {
                pd_set_state(PD_STATE_ATTACHED);
            }
            break;
        }
        if (pd_state_take_tx_result(&result) != 0u) {
            if (result == PD_TX_RESULT_OK) {
                s_pd.source_cap_retry = 0u;
                if (!s_epr_mode_active) {
                    /* Cable discovery now precedes the first SPR Source
                     * Capabilities, so EPR/5A was already advertised. */
                    s_epr_advertise_ready = true;
                }
                pd_set_state(PD_STATE_WAIT_REQUEST);
            } else {
                s_pd.source_cap_retry++;
                if (s_pd.source_cap_retry >= PD_SOURCE_CAP_RETRY_MAX) {
                    s_pd.source_cap_retry = 0u;
                    if (s_pd.contract_mv != 0u) {
                        s_source_cap_refresh_pending = false;
                        s_source_cap_refresh_retries = 0u;
                        s_source_cap_refresh_wait_ms = 0u;
                        pd_set_state(PD_STATE_READY);
                    } else {
                        pd_set_state(PD_STATE_HARD_RESET);
                    }
                }
            }
        } else if ((pd_tx_is_idle()) &&
                   (s_tx.result == PD_TX_RESULT_NONE) &&
                   (!s_state_tx_active)) {
            (void)pd_state_start_source_cap();
        }
        break;

    case PD_STATE_WAIT_REQUEST:
        if (pd_fault_active()) {
            s_source_cap_refresh_pending = false;
            break;
        }
        if (s_source_cap_refresh_pending) {
            if ((pd_tx_is_idle()) && (s_tx.result == PD_TX_RESULT_NONE)) {
                s_pd.source_cap_retry = 0u;
                pd_set_state(PD_STATE_SEND_SOURCE_CAP);
                break;
            }
        }
        if ((pd_5a_refresh_needed()) &&
            (s_pd.state_timer_ms >= PD_5A_REFRESH_RETRY_DELAY_MS)) {
            s_pd.source_cap_retry = 0u;
            pd_set_state(PD_STATE_SEND_SOURCE_CAP);
            break;
        }
        if ((s_pd.contract_ma != 0u) &&
            (s_source_cap_refresh_retries == 0u) &&
            (s_pd.state_timer_ms >= PD_REQUEST_TIMEOUT_MS)) {
            pd_set_state(PD_STATE_READY);
            break;
        }
        if (s_pd.state_timer_ms >= PD_REQUEST_TIMEOUT_MS) {
            pd_set_state(PD_STATE_SEND_SOURCE_CAP);
        }
        break;

    case PD_STATE_SEND_ACCEPT:
        if (pd_fault_active()) {
            pd_clear_contract_info();
            if ((pd_tx_is_idle()) && (s_tx.result == PD_TX_RESULT_NONE)) {
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
        if (pd_state_take_tx_result(&result) != 0u) {
            if (result == PD_TX_RESULT_OK) {
                if (!pd_commit_staged_request()) {
                    pd_debug_note_failure(PD_DEBUG_EVENT_ACCEPT_FAIL);
                    pd_start_soft_reset_recovery();
                    break;
                }
                pd_debug_note_event(PD_DEBUG_EVENT_ACCEPT_OK);
                s_debug_power_request_reason = PD_DEBUG_POWER_REQ_REQUEST;
                s_debug_power_request_enable = true;
                pd_power_request(true, s_pd.contract_mv, s_pd.contract_ma);
                pd_debug_note_event(PD_DEBUG_EVENT_POWER_SET);
                pd_power_task(0u);
                pd_set_state(PD_STATE_APPLY_POWER);
            } else {
                pd_debug_note_failure(PD_DEBUG_EVENT_ACCEPT_FAIL);
                pd_start_soft_reset_recovery();
            }
        } else if ((pd_tx_is_idle()) &&
                   (s_tx.result == PD_TX_RESULT_NONE) &&
                   (!s_state_tx_active)) {
            if (pd_state_start_control(DEF_TYPE_ACCEPT) != 0u) {
                pd_debug_note_event(PD_DEBUG_EVENT_ACCEPT_START);
            }
        }
        break;

    case PD_STATE_APPLY_POWER:
        if (pd_fault_active()) {
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        if (pd_power_is_ready_on()) {
            pd_set_state(PD_STATE_SEND_PS_RDY);
        } else if (pd_power_has_error()) {
            pd_set_state(PD_STATE_HARD_RESET);
        }
        break;

    case PD_STATE_SEND_PS_RDY:
        if (pd_fault_active()) {
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        if (s_pd.state_timer_ms >= PD_SEND_STATE_TIMEOUT_MS) {
            pd_debug_note_failure(PD_DEBUG_EVENT_PS_RDY_FAIL);
            pd_start_soft_reset_recovery();
            break;
        }
        if ((pd_tx_is_idle()) &&
            (s_tx.result == PD_TX_RESULT_NONE) &&
            (!s_state_tx_active)) {
            if (pd_state_start_control(DEF_TYPE_PS_RDY) != 0u) {
                pd_debug_note_event(PD_DEBUG_EVENT_PS_RDY_START);
            }
        }
        if (pd_state_take_tx_result(&result) != 0u) {
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
        if ((!s_epr_mode_active) ||
            (s_pd.state_timer_ms >= PD_SEND_STATE_TIMEOUT_MS)) {
            pd_debug_note_failure(PD_DEBUG_EVENT_KEEPALIVE_ACK_FAIL);
            s_epr_resume_state = 0u;
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        if (pd_state_take_tx_result(&result) != 0u) {
            if (result == PD_TX_RESULT_OK) {
                pd_reset_source_comm_timers();
                pd_debug_note_event(PD_DEBUG_EVENT_KEEPALIVE_ACK_OK);
                if (s_epr_resume_state != 0u) {
                    /* Go back to the Accept / power transition / PS_RDY that this
                     * acknowledgement was injected into, with its timer restored. */
                    pd_state_t resume = (pd_state_t)s_epr_resume_state;
                    uint16_t resume_ms = s_epr_resume_ms;

                    s_epr_resume_state = 0u;
                    s_epr_resume_ms = 0u;
                    pd_set_state(resume);
                    s_pd.state_timer_ms = resume_ms;
                } else {
                    pd_set_state(PD_STATE_READY);
                }
            } else {
                pd_debug_note_failure(PD_DEBUG_EVENT_KEEPALIVE_ACK_FAIL);
                s_epr_resume_state = 0u;
                pd_set_state(PD_STATE_HARD_RESET);
            }
        } else if ((pd_tx_is_idle()) &&
                   (s_tx.result == PD_TX_RESULT_NONE) &&
                   (!s_state_tx_active)) {
            if (pd_tx_start_extended_control(PD_EXT_CTRL_EPR_KEEP_ALIVE_A, 0u)) {
                s_state_tx_active = true;
                pd_debug_note_event(PD_DEBUG_EVENT_KEEPALIVE_ACK_START);
            }
        }
        break;

    case PD_STATE_SEND_SOFT_RESET:
        if (s_pd.state_timer_ms >= PD_SOFT_RESET_SEND_TIMEOUT_MS) {
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        if (pd_state_take_tx_result(&result) != 0u) {
            if (result == PD_TX_RESULT_OK) {
                pd_set_state(PD_STATE_WAIT_SOFT_RESET_ACCEPT);
            } else {
                pd_set_state(PD_STATE_HARD_RESET);
            }
        } else if ((pd_tx_is_idle()) &&
                   (s_tx.result == PD_TX_RESULT_NONE) &&
                   (!s_state_tx_active)) {
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
                pd_set_state((s_pd.contract_mv != 0u) ?
                             PD_STATE_READY :
                             PD_STATE_SEND_SOURCE_CAP);
            }
            break;
        }
        if ((pd_tx_is_idle()) &&
            (s_tx.result == PD_TX_RESULT_NONE) &&
            (!s_state_tx_active)) {
            (void)pd_state_start_control(s_pd.control_msg);
        }
        if (pd_state_take_tx_result(&result) != 0u) {
            if (result == PD_TX_RESULT_OK) {
                pd_set_state(s_pd.next_state);
            } else {
                if (s_pd.control_msg == DEF_TYPE_WAIT) {
                    pd_set_state(s_pd.next_state);
                } else {
                    pd_set_state((s_pd.contract_mv != 0u) ?
                                 PD_STATE_READY :
                                 PD_STATE_SEND_SOURCE_CAP);
                }
            }
        }
        break;

    case PD_STATE_HARD_RESET:
        if (!s_state_tx_active) {
            pd_state_start_hard_reset();
        }
        if (pd_state_take_tx_result(&result) != 0u) {
            s_pd.hard_reset_retry++;
            s_pd.msg_id = 0u;
            s_cable_msg_id = 0u;
            s_cable2_msg_id = 0u;
            pd_clear_rx_message_ids();
            s_pd.pd_revision = 1u;
            s_cable_pd_revision = 1u;
            s_pd.source_cap_retry = 0u;
            if ((result != PD_TX_RESULT_OK) &&
                (s_pd.hard_reset_retry < PD_HARD_RESET_RETRY_MAX)) {
                break;
            }
            s_pd.hard_reset_retry = 0u;
            pd_set_state(PD_STATE_ATTACHED);
        }
        break;

    case PD_STATE_READY:
        if (pd_power_has_error()) {
            pd_set_state(PD_STATE_HARD_RESET);
            break;
        }
        /* A KeepAlive that arrived while the previous Accept / power transition / PS_RDY
         * was running is answered now, before any new exchange starts. */
        if ((s_epr_keepalive_ack_pending) &&
            (s_epr_mode_active) &&
            (pd_tx_is_idle()) &&
            (s_tx.result == PD_TX_RESULT_NONE) &&
            (!s_state_tx_active)) {
            s_epr_keepalive_ack_pending = false;
            pd_set_state(PD_STATE_SEND_EPR_KEEPALIVE_ACK);
            break;
        }
        if (pd_fault_active()) {
            s_source_cap_refresh_pending = false;
            s_source_cap_refresh_retries = 0u;
            s_source_cap_refresh_wait_ms = 0u;
        }
        if ((s_pd.contract_ma >= PD_5A_CURRENT_MA) &&
            (!s_source_cap_refresh_pending)) {
            s_source_cap_refresh_pending = false;
            s_source_cap_refresh_retries = 0u;
            s_source_cap_refresh_wait_ms = 0u;
        } else if ((!s_source_cap_refresh_pending) &&
                   (pd_5a_refresh_needed())) {
            s_source_cap_refresh_wait_ms =
                (uint16_t)(s_source_cap_refresh_wait_ms + elapsed_ms);
            if (s_source_cap_refresh_wait_ms >= PD_5A_REFRESH_RETRY_DELAY_MS) {
                s_source_cap_refresh_wait_ms = 0u;
                s_source_cap_refresh_pending = true;
            }
        }
        if ((!s_epr_advertise_ready) &&
            (!s_epr_mode_active) &&
            (pd_epr_is_available()) &&
            (s_pd.contract_mv != 0u) &&
            (!pd_fault_active())) {
            s_epr_advertise_ready = true;
            s_source_cap_refresh_pending = true;
            s_source_cap_refresh_retries = 1u;
            s_source_cap_refresh_wait_ms = 0u;
        }
        if (s_source_cap_refresh_pending) {
            if ((pd_tx_is_idle()) && (s_tx.result == PD_TX_RESULT_NONE)) {
                s_pd.source_cap_retry = 0u;
                pd_set_state(PD_STATE_SEND_SOURCE_CAP);
                break;
            }
        }
        if (!pd_tx_result_has_aux_owner()) {
            (void)pd_tx_take_result(&result);
        }
        s_pd.hard_reset_retry = 0u;
        break;

    default:
        break;
    }
}

/* Initialization, periodic execution, callbacks, and public status APIs. */
void PD_Init(pd_vbus_sense_cb_t vbus_sense_cb)
{
    PD_SetVbusSenseCallback(vbus_sense_cb);

    memcpy(s_pdos, s_default_pdos, sizeof(s_default_pdos));
    s_pdo_count = PD_SOURCE_MAX_FIXED_PDOS;
    pd_phy_init();
    s_frontend_state = PD_FRONTEND_CLOSE_HVCP;
    pd_reset_context(true);
}

/** @brief Finish processing the current RX packet and clear its pending flag. */
static void pd_finish_current_rx(void)
{
    uint8_t i;
    uint8_t len;

    __disable_irq();
    s_rx_msg_pending = false;
    if (s_rx_ring_tail != s_rx_ring_head) {
        len = s_rx_ring_len[s_rx_ring_tail];
        for (i = 0u; i < len; i++) {
            s_pd_rx_buf[i] = s_pd_rx_ring_buf[s_rx_ring_tail][i];
        }
        s_rx_msg_len = len;
        s_rx_sop = s_rx_ring_sop[s_rx_ring_tail];
        s_rx_ring_tail = (uint8_t)((s_rx_ring_tail + 1u) % PD_RX_RING_LEN);
        s_rx_msg_pending = true;
    }
    __enable_irq();
}

/** @brief Replay a deferred Request after the associated transition is ready. */
static void pd_handle_deferred_request(void)
{
    uint8_t i;

    if ((!s_deferred_request_valid) ||
        ((s_pd.state != PD_STATE_READY) &&
         (s_pd.state != PD_STATE_WAIT_REQUEST)) ||
        (s_rx_msg_pending) ||
        (s_goodcrc_tx_pending) ||
        (s_goodcrc_tx_active) ||
        (!pd_tx_is_idle()) ||
        (s_tx.result != PD_TX_RESULT_NONE)) {
        return;
    }

    __disable_irq();
    if ((s_rx_msg_pending) ||
        (s_goodcrc_tx_pending) ||
        (s_goodcrc_tx_active) ||
        (!pd_tx_is_idle()) ||
        (s_tx.result != PD_TX_RESULT_NONE)) {
        __enable_irq();
        return;
    }
    for (i = 0u; i < s_deferred_request_len; i++) {
        s_pd_rx_buf[i] = s_deferred_request_buf[i];
    }
    s_rx_msg_len = s_deferred_request_len;
    s_rx_sop = s_deferred_request_sop;
    s_rx_msg_pending = true;
    pd_clear_deferred_request();
    __enable_irq();

    s_replaying_deferred_request = true;
    pd_handle_message();
    s_replaying_deferred_request = false;
    pd_finish_current_rx();
    if (s_tx.state == PD_TX_IDLE) {
        pd_set_phy_rx_mode();
    }
}

/** @brief Advance source communication and keep-alive timeout handling. */
static void pd_source_comm_timeout_task(uint8_t elapsed_ms)
{
    uint32_t next_ms;

    if ((!s_pd.connected) || (s_pd.state != PD_STATE_READY)) {
        pd_reset_source_comm_timers();
        return;
    }

    if (s_epr_mode_active) {
        next_ms = (uint32_t)s_source_epr_keepalive_ms + elapsed_ms;
        s_source_epr_keepalive_ms = (next_ms > 0xFFFFul) ?
                                    0xFFFFu : (uint16_t)next_ms;
        if (s_source_epr_keepalive_ms > s_debug_epr_silence_max_ms) {
            s_debug_epr_silence_max_ms = s_source_epr_keepalive_ms;
        }
        if (s_source_epr_keepalive_ms >= PD_SOURCE_EPR_KEEPALIVE_TIMEOUT_MS) {
            pd_debug_note_failure(PD_DEBUG_EVENT_EPR_COMM_TIMEOUT);
            if (s_epr_silence_strikes < PD_EPR_SILENCE_STRIKES_TOLERATED) {
                /* Only reached when the tolerance knob is non-zero: restart the silent window
                 * and keep the contract instead of resetting on the first miss. */
                s_epr_silence_strikes++;
                s_source_epr_keepalive_ms = 0u;
                return;
            }
            s_epr_silence_strikes = 0u;
            pd_set_state(PD_STATE_HARD_RESET);
            return;
        }
    } else {
        s_source_epr_keepalive_ms = 0u;
    }

    if (s_pps_active) {
        next_ms = (uint32_t)s_source_pps_comm_ms + elapsed_ms;
        s_source_pps_comm_ms = (next_ms > 0xFFFFul) ?
                               0xFFFFu : (uint16_t)next_ms;
        if (s_source_pps_comm_ms >= PD_SOURCE_PPS_COMM_TIMEOUT_MS) {
            pd_debug_note_failure(PD_DEBUG_EVENT_PPS_COMM_TIMEOUT);
            pd_set_state(PD_STATE_HARD_RESET);
        }
    } else {
        s_source_pps_comm_ms = 0u;
    }
}

void PD_Task(uint8_t elapsed_ms)
{
    if (s_pd.state == PD_STATE_DISABLED) {
        return;
    }

    if (s_pd_reset_pending) {
        s_pd_reset_pending = false;
        pd_reset_context(true);
    }

    pd_goodcrc_watchdog_task();
    pd_power_task(elapsed_ms);
    pd_frontend_task(elapsed_ms);
    pd_tx_task(elapsed_ms);
    if (pd_frontend_is_ready()) {
        pd_connection_task(elapsed_ms);
    }

    if (s_rx_msg_pending) {
        pd_cable_task(0u);
        pd_ext_response_task(0u);
        pd_alert_task();
        pd_revision_query_task(0u);
        if ((!s_goodcrc_tx_pending) &&
            (!s_goodcrc_tx_active) &&
            (pd_tx_is_idle()) &&
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

    if (s_pd.connected) {
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

    s_tick_ms = (tick > 0xFFFFu) ? 0xFFFFu : (uint16_t)tick;

    /* GoodCRC starts asynchronously in the USBPD IRQ.  Age it here using
     * real timer ticks so a delayed main-loop pass cannot charge old elapsed
     * time to a response that has only just started. */
    if ((s_goodcrc_tx_pending) || (s_goodcrc_tx_active)) {
        watchdog_age = (uint16_t)s_goodcrc_watchdog_ms + elapsed_ms;
        s_goodcrc_watchdog_ms = (watchdog_age > 0xFFu) ?
                                0xFFu : (uint8_t)watchdog_age;
    }
}

void PD_Tick1ms(void)
{
    PD_TickMs(1u);
}

void PD_TaskFromTick(void)
{
    uint16_t elapsed;

    __disable_irq();
    elapsed = s_tick_ms;
    s_tick_ms = 0u;
    __enable_irq();

    if (elapsed != 0u) {
        PD_Task((uint8_t)(elapsed > 255u ? 255u : elapsed));
    }
}

void PD_SetVbusSenseCallback(pd_vbus_sense_cb_t cb)
{
    s_vbus_sense_cb = cb;
}

uint16_t pd_sense_vbus_mv(void)
{
    if (s_vbus_sense_cb == 0) {
        return 0u;
    }

    return s_vbus_sense_cb();
}

bool PD_SetFixedPdos(const pd_fixed_pdo_t *pdos, uint8_t count)
{
    uint8_t i;

    if ((pdos == 0) || (count == 0u) || (count > PD_SOURCE_MAX_FIXED_PDOS)) {
        return false;
    }

    for (i = 0u; i < count; i++) {
        if ((pdos[i].mv < 5000u) ||
            (pdos[i].mv > 20000u) ||
            (pdos[i].ma == 0u) ||
            (pdos[i].ma > pd_fixed_pdo_max_ma(pdos[i].mv))) {
            return false;
        }
    }

    for (i = 0u; i < count; i++) {
        s_pdos[i] = pdos[i];
    }
    s_pdo_count = count;
    return true;
}


void PD_ReportFault(uint16_t fault_bits)
{
    if (fault_bits == 0u) {
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
    if (fault_bits == 0u) {
        s_fault_alert_pending = false;
        s_fault_alert_sent = false;
        s_fault_alert_tx_bits = 0u;
        if (old_bits != 0u) {
            s_fault_recovery_pending = true;
        }
        return;
    }

    if ((new_bits != 0u) || (!s_fault_alert_sent)) {
        s_fault_alert_pending = true;
        s_fault_alert_sent = false;
    }

    if (pd_active_contract_is_epr()) {
        pd_epr_exit_to_spr_recovery();
    }

    if (s_power.target_enable) {
        s_debug_power_request_reason = PD_DEBUG_POWER_REQ_FAULT_OFF;
        s_debug_power_request_enable = false;
        pd_power_request(false, 0u, 0u);
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
    status->debug_rx_msg_count = s_debug_rx_msg_count;
    status->debug_keepalive_rx_count = s_debug_keepalive_rx_count;
    status->debug_epr_keepalive_age_ms = s_source_epr_keepalive_ms;
    status->debug_epr_silence_max_ms = s_debug_epr_silence_max_ms;
    status->debug_rx_frame_count = s_debug_rx_frame_count;
    status->debug_rx_drop_count = s_debug_rx_drop_count;
    status->debug_vbus_min_mv = pd_power_vbus_min_mv();
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
