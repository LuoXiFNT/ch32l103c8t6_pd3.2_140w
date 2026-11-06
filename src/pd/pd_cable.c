/**
 * @file pd_cable.c
 * @brief VCONN, cable discovery, and structured VDM handling.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#include "pd_internal.h"

/* VCONN sequencing, cable identity parsing, and VDM response handling. */

void pd_cable_reset(void)
{
    s_cable_state = PD_CABLE_IDLE;
    s_cable_wait_ms = 0u;
    s_cable_retry = 0u;
    s_cable_msg_id = 0u;
    s_cable2_msg_id = 0u;
    s_cable_pd_revision = 1u;
    s_supports_5a = false;
    s_epr_source_cap_pending = false;
    s_epr_source_cap_tx_active = false;
    s_epr_source_cap_retry = 0u;
    s_epr_advertise_ready = false;
    s_source_cap_refresh_pending = false;
    s_source_cap_refresh_retries = 0u;
    s_source_cap_refresh_wait_ms = 0u;
    s_cable_current_ma = PD_DEFAULT_CURRENT_MA;
    s_cable_max_vbus_mv = 20000u;
    s_cable_epr_capable = false;
    s_last_vdm_header = 0ul;
    s_last_cable_vdo = 0ul;
    s_last_vdm_sop = 0u;
    s_last_vdm_cmd = 0u;
    s_last_vdm_cmdt = 0u;
}

void pd_cable_start_discovery(void)
{
    s_cable_state = PD_CABLE_ENABLE_VCONN;
    s_cable_wait_ms = 0u;
    s_cable_retry = 0u;
    s_supports_5a = false;
    s_epr_source_cap_pending = false;
    s_epr_source_cap_tx_active = false;
    s_epr_source_cap_retry = 0u;
    s_epr_advertise_ready = false;
    s_source_cap_refresh_pending = false;
    s_source_cap_refresh_retries = 0u;
    s_source_cap_refresh_wait_ms = 0u;
    s_cable_current_ma = PD_DEFAULT_CURRENT_MA;
    s_cable_max_vbus_mv = 20000u;
    s_cable_epr_capable = false;
}

bool pd_cable_discovery_finished(void)
{
    return ((s_cable_state == PD_CABLE_DONE) ||
            (s_cable_state == PD_CABLE_FAILED) ||
            (s_cable_state == PD_CABLE_IDLE));
}


/** @brief Restore PHY routing to the partner-facing CC channel. */
static void pd_cable_restore_phy(void)
{
    pd_select_cc(s_pd.cc);
}

/** @brief Finish cable discovery and publish its capability result. */
static void pd_cable_finish(bool success)
{
    pd_tx_result_t result;
    bool became_5a = success && (s_cable_current_ma >= PD_5A_CURRENT_MA);

    if (s_tx.sop == UPD_SOP1) {
        (void)pd_tx_take_result(&result);
    }

    pd_cable_restore_phy();
    (void)pd_cc_vconn_enable();
    s_cable_state = (success) ? PD_CABLE_DONE : PD_CABLE_FAILED;
    if (!success) {
        s_supports_5a = false;
        s_epr_mode_active = false;
        s_epr_source_cap_pending = false;
        s_epr_source_cap_tx_active = false;
        s_epr_source_cap_retry = 0u;
        s_epr_advertise_ready = false;
        s_cable_epr_capable = false;
        s_source_cap_refresh_pending = false;
        s_source_cap_refresh_retries = 0u;
        s_source_cap_refresh_wait_ms = 0u;
        s_cable_current_ma = PD_DEFAULT_CURRENT_MA;
        s_cable_max_vbus_mv = 20000u;
    }
    s_cable_wait_ms = 0u;

    if (became_5a) {
        s_source_cap_refresh_pending = true;
        s_source_cap_refresh_retries = PD_5A_REFRESH_RETRY_COUNT;
        s_source_cap_refresh_wait_ms = 0u;
    }
}

/** @brief Check whether a VDM header is structured. */
static bool pd_vdm_is_structured(uint32_t vdm_header)
{
    return ((vdm_header & PD_VDM_STRUCTURED) != 0ul);
}

/** @brief Extract the command field from a VDM header. */
static uint8_t pd_vdm_cmd(uint32_t vdm_header)
{
    return (uint8_t)(vdm_header & PD_VDM_CMD_MASK);
}

/** @brief Extract the command-type field from a VDM header. */
static uint8_t pd_vdm_cmdt(uint32_t vdm_header)
{
    return (uint8_t)(vdm_header & PD_VDM_CMDT_MASK);
}

/** @brief Build and start a structured VDM response for the received SOP. */
static uint8_t pd_vdm_send_response(uint8_t rx_sop, uint32_t request_header, uint32_t cmdt)
{
    uint8_t payload[4];
    uint32_t response;

    if ((!pd_tx_is_idle()) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        (pd_is_power_transition_state())) {
        return 0u;
    }

    response = (request_header & PD_VDM_HEADER_KEEP_MASK) |
               PD_VDM_STRUCTURED |
               pd_vdm_version_bits(pd_rx_sop_to_tx_sop(rx_sop)) |
               (cmdt & PD_VDM_CMDT_MASK);
    pd_put_u32_le(payload, response);
    return pd_tx_start_sop(DEF_TYPE_VENDOR_DEFINED,
                           payload,
                           sizeof(payload),
                           pd_rx_sop_to_tx_sop(rx_sop));
}

bool pd_vdm_send_source_identity(uint8_t rx_sop, uint32_t request_header)
{
    uint8_t payload[16];
    uint32_t response;
    uint32_t id_header;
    uint32_t product_vdo;

    if ((rx_sop != PD_RX_SOP0) ||
        (!pd_tx_is_idle()) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        (pd_is_power_transition_state())) {
        return 0u;
    }

    response = (request_header & PD_VDM_HEADER_KEEP_MASK) |
               PD_VDM_STRUCTURED |
               pd_vdm_version_bits(UPD_SOP0) |
               PD_VDM_CMDT_ACK;
    id_header = (uint32_t)PD_SOURCE_EXT_VID;
    product_vdo = ((uint32_t)PD_SOURCE_EXT_PID << 16u) |
                  (uint32_t)PD_PRODUCT_BCD_DEVICE;

    pd_put_u32_le(&payload[0], response);
    pd_put_u32_le(&payload[4], id_header);
    pd_put_u32_le(&payload[8], PD_SOURCE_EXT_XID);
    pd_put_u32_le(&payload[12], product_vdo);

    return pd_tx_start_sop(DEF_TYPE_VENDOR_DEFINED,
                           payload,
                           sizeof(payload),
                           UPD_SOP0);
}

/** @brief Decode the cable current rating from its VDO. */
static uint16_t pd_cable_vdo_current_ma(uint32_t cable_vdo)
{
    switch (cable_vdo & PD_CABLE_VDO_CURRENT_MASK) {
    case PD_CABLE_VDO_CURRENT_5A:
        return PD_5A_CURRENT_MA;

    case PD_CABLE_VDO_CURRENT_3A:
    default:
        return PD_DEFAULT_CURRENT_MA;
    }
}

/** @brief Decode the maximum VBUS voltage rating from a cable VDO. */
static uint16_t pd_cable_vdo_max_vbus_mv(uint32_t cable_vdo)
{
    uint32_t max_vbus = (cable_vdo & PD_CABLE_VDO_MAX_VBUS_MASK) >>
                        PD_CABLE_VDO_MAX_VBUS_SHIFT;

    switch (max_vbus) {
    case PD_CABLE_MAX_VBUS_30V:
        return 30000u;

    case PD_CABLE_MAX_VBUS_40V:
        return 40000u;

    case PD_CABLE_MAX_VBUS_50V:
        return 50000u;

    case PD_CABLE_MAX_VBUS_20V:
    default:
        return 20000u;
    }
}

/** @brief Check whether an Identity Header describes an active or passive cable. */
static bool pd_identity_is_cable(uint32_t id_header)
{
    uint8_t product_type = (uint8_t)((id_header & PD_IDH_PRODUCT_TYPE_MASK) >>
                                     PD_IDH_PRODUCT_TYPE_SHIFT);

    return ((product_type == PD_PRODUCT_TYPE_PASSIVE_CABLE) ||
            (product_type == PD_PRODUCT_TYPE_ACTIVE_CABLE));
}

/** @brief Find the cable current capability in a Discover Identity response. */
static uint16_t pd_cable_find_current_ma(uint8_t num_do, uint32_t *found_vdo)
{
    uint32_t id_header;
    uint32_t cable_vdo;

    if (num_do <= PD_VDO_INDEX_CABLE_FIRST) {
        if (found_vdo != 0) {
            *found_vdo = 0ul;
        }
        return PD_DEFAULT_CURRENT_MA;
    }

    id_header = pd_get_u32_le(&s_pd_rx_buf[2u + ((uint16_t)PD_VDO_INDEX_ID_HEADER * 4u)]);
    if (pd_identity_is_cable(id_header) == 0u) {
        if (found_vdo != 0) {
            *found_vdo = 0ul;
        }
        return PD_DEFAULT_CURRENT_MA;
    }

    cable_vdo = pd_get_u32_le(&s_pd_rx_buf[2u + ((uint16_t)PD_VDO_INDEX_CABLE_FIRST * 4u)]);
    if (found_vdo != 0) {
        *found_vdo = cable_vdo;
    }
    return pd_cable_vdo_current_ma(cable_vdo);
}

/** @brief Parse Discover Identity data and update the cached cable properties. */
static bool pd_cable_parse_identity(uint8_t num_do)
{
    uint32_t vdm_header;
    uint32_t cable_vdo;

    if (num_do < 4u) {
        return false;
    }

    vdm_header = pd_get_u32_le(&s_pd_rx_buf[2]);
    s_last_vdm_header = vdm_header;
    s_last_vdm_sop = s_rx_sop;
    s_last_vdm_cmd = pd_vdm_cmd(vdm_header);
    s_last_vdm_cmdt = pd_vdm_cmdt(vdm_header);

    if (((vdm_header >> 16u) != PD_SID_PD) ||
        (pd_vdm_is_structured(vdm_header) == 0u) ||
        (pd_vdm_cmd(vdm_header) != DEF_VDM_DISC_IDENT) ||
        (pd_vdm_cmdt(vdm_header) != PD_VDM_CMDT_ACK)) {
        return false;
    }

    s_cable_current_ma = pd_cable_find_current_ma(num_do, &cable_vdo);
    s_last_cable_vdo = cable_vdo;
    s_cable_max_vbus_mv = (cable_vdo != 0ul) ?
                          pd_cable_vdo_max_vbus_mv(cable_vdo) :
                          20000u;
    s_cable_epr_capable = ((cable_vdo & PD_CABLE_VDO_EPR_CAPABLE) != 0ul);
    s_supports_5a = (s_cable_current_ma >= PD_5A_CURRENT_MA);
    if (!pd_epr_is_available()) {
        s_epr_mode_active = false;
        s_epr_source_cap_pending = false;
        s_epr_source_cap_tx_active = false;
        s_epr_source_cap_retry = 0u;
        s_epr_advertise_ready = false;
    }
    return true;
}


void pd_cable_handle_message(uint8_t num_do)
{
    if (s_cable_state != PD_CABLE_WAIT_DISCOVER_ID) {
        return;
    }

    if (pd_cable_parse_identity(num_do) != 0u) {
        pd_cable_finish(true);
    } else {
        pd_cable_finish(false);
    }
}


void pd_vdm_handle_message(uint8_t num_do)
{
    uint32_t vdm_header;

    if (num_do == 0u) {
        return;
    }

    vdm_header = pd_get_u32_le(&s_pd_rx_buf[2]);
    s_last_vdm_header = vdm_header;
    s_last_vdm_sop = s_rx_sop;
    s_last_vdm_cmd = pd_vdm_cmd(vdm_header);
    s_last_vdm_cmdt = pd_vdm_cmdt(vdm_header);

    if (pd_vdm_is_structured(vdm_header) == 0u) {
        return;
    }

    if (pd_vdm_cmdt(vdm_header) == PD_VDM_CMDT_INIT) {
        if ((pd_vdm_cmd(vdm_header) == DEF_VDM_DISC_IDENT) &&
            (s_rx_sop == PD_RX_SOP0)) {
            if (pd_vdm_send_source_identity(s_rx_sop, vdm_header)) {
                s_ext_response_tx_active = true;
            }
            return;
        }
        if (pd_vdm_send_response(s_rx_sop, vdm_header, PD_VDM_CMDT_NAK) != 0u) {
            s_ext_response_tx_active = true;
        }
    }
}


void pd_cable_task(uint8_t elapsed_ms)
{
    uint32_t vdm;
    uint8_t payload[4];
    pd_tx_result_t result;

    if ((!s_pd.connected) || (!pd_frontend_is_ready())) {
        return;
    }
    if ((s_pd.contract_mv == 0u) && (!pd_power_is_ready_on())) {
        return;
    }

    switch (s_cable_state) {
    case PD_CABLE_ENABLE_VCONN:
        if (pd_cc_vconn_enable() == CH211_OK) {
            s_cable_state = PD_CABLE_WAIT_VCONN;
            s_cable_wait_ms = 0u;
        } else {
            s_cable_wait_ms = (uint16_t)(s_cable_wait_ms + elapsed_ms);
            if (s_cable_wait_ms >= PD_CABLE_VCONN_TIMEOUT_MS) {
                pd_cable_finish(false);
            }
        }
        break;

    case PD_CABLE_WAIT_VCONN:
        if (pd_cc_vconn_busy()) {
            s_cable_wait_ms = (uint16_t)(s_cable_wait_ms + elapsed_ms);
            if (s_cable_wait_ms >= PD_CABLE_VCONN_TIMEOUT_MS) {
                pd_cable_finish(false);
            }
            break;
        }
        s_cable_wait_ms = (uint16_t)(s_cable_wait_ms + elapsed_ms);
        if (s_cable_wait_ms >= PD_VCONN_SETTLE_MS) {
            pd_select_cc(s_pd.cc);
            s_cable_state = PD_CABLE_SEND_DISCOVER_ID;
            s_cable_wait_ms = 0u;
        }
        break;

    case PD_CABLE_SEND_DISCOVER_ID:
        if (s_cable_retry != 0u) {
            s_cable_wait_ms = (uint16_t)(s_cable_wait_ms + elapsed_ms);
            if (s_cable_wait_ms < PD_CABLE_DISCOVER_RETRY_DELAY_MS) {
                break;
            }
        }
        if ((!s_rx_msg_pending) && (!pd_is_power_transition_state())) {
            pd_select_cc(s_pd.cc);
            vdm = pd_make_structured_vdm((uint16_t)PD_SID_PD,
                                         DEF_VDM_DISC_IDENT,
                                         PD_VDM_CMDT_INIT,
                                         UPD_SOP1);
            pd_put_u32_le(payload, vdm);
            if (pd_tx_start_sop(DEF_TYPE_VENDOR_DEFINED,
                                payload,
                                sizeof(payload),
                                UPD_SOP1)) {
                s_cable_state = PD_CABLE_WAIT_DISCOVER_ID;
                s_cable_wait_ms = 0u;
            }
        }
        break;

    case PD_CABLE_WAIT_DISCOVER_ID:
        if ((s_tx.result != PD_TX_RESULT_NONE) && (s_tx.sop != UPD_SOP1)) {
            break;
        }
        if ((s_tx.sop == UPD_SOP1) && (pd_tx_take_result(&result) != 0u)) {
            if (result != PD_TX_RESULT_OK) {
                if (++s_cable_retry < PD_CABLE_DISCOVER_RETRY_MAX) {
                    s_cable_state = PD_CABLE_SEND_DISCOVER_ID;
                } else {
                    pd_cable_finish(false);
                }
                s_cable_wait_ms = 0u;
                break;
            }
        }
        if (s_rx_msg_pending) {
            break;
        }
        s_cable_wait_ms = (uint16_t)(s_cable_wait_ms + elapsed_ms);
        if (s_cable_wait_ms >= PD_CABLE_DISCOVER_TIMEOUT_MS) {
            if (++s_cable_retry < PD_CABLE_DISCOVER_RETRY_MAX) {
                s_cable_state = PD_CABLE_SEND_DISCOVER_ID;
            } else {
                pd_cable_finish(false);
            }
            s_cable_wait_ms = 0u;
        }
        break;

    case PD_CABLE_FAILED:
        s_cable_wait_ms = (uint16_t)(s_cable_wait_ms + elapsed_ms);
        if ((s_cable_wait_ms >= PD_CABLE_REDISCOVER_MS) &&
            ((s_pd.state == PD_STATE_READY) || (s_pd.state == PD_STATE_WAIT_REQUEST)) &&
            (pd_tx_is_idle()) &&
            (s_tx.result == PD_TX_RESULT_NONE) &&
            (!s_rx_msg_pending) &&
            (!pd_is_power_transition_state())) {
            pd_cable_start_discovery();
        }
        break;

    case PD_CABLE_IDLE:
        if (((s_pd.state == PD_STATE_READY) ||
             (s_pd.state == PD_STATE_WAIT_REQUEST)) &&
            (!s_epr_mode_active) &&
            (pd_tx_is_idle()) &&
            (s_tx.result == PD_TX_RESULT_NONE) &&
            (!s_rx_msg_pending)) {
            pd_cable_start_discovery();
        }
        break;

    default:
        break;
    }
}
