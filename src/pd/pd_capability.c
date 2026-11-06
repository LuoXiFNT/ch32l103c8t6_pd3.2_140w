/**
 * @file pd_capability.c
 * @brief Source PDO/APDO construction and Request/RDO validation.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#include "pd_internal.h"

/* Advertised capabilities, extended source capabilities, and request checks. */

/* Keep request decoding on the exact PDO layout most recently put on wire. */
static uint8_t s_advertised_spr_avs;

/** @brief Return the current limit actually advertised for one fixed PDO. */
static uint16_t pd_effective_pdo_ma(const pd_fixed_pdo_t *pdo)
{
    uint16_t limit = pd_current_limit_ma();

    return (pdo->ma > limit) ? limit : pdo->ma;
}

/** @brief Check whether a PPS APDO is present in the source capabilities. */
static bool pd_pps_is_advertised(void)
{
    return (s_pd.pd_revision != 0u);
}

/** @brief Return the current limit for the fixed PDO at the requested voltage. */
static uint16_t pd_effective_fixed_current_ma(uint16_t mv)
{
    uint8_t i;

    for (i = 0u; i < s_pdo_count; i++) {
        if (s_pdos[i].mv == mv) {
            return pd_effective_pdo_ma(&s_pdos[i]);
        }
    }
    return 0u;
}

/** @brief Check whether an SPR AVS APDO is currently advertised. */
static bool pd_spr_avs_is_advertised(void)
{
    bool incompatible_partner =
        (s_partner_revision == PD_PARTNER_REVISION_20) ||
        (s_partner_revision == PD_PARTNER_REVISION_30) ||
        (s_partner_revision == PD_PARTNER_REVISION_31);

    return ((s_pd.pd_revision != 0u) && (!incompatible_partner) &&
            (pd_effective_fixed_current_ma(PD_SPR_AVS_SWITCH_MV) != 0u) &&
            (pd_effective_fixed_current_ma(PD_SPR_AVS_MAX_MV) != 0u));
}

/** @brief Check whether the active cable and policy permit SPR AVS requests. */
static uint8_t pd_spr_avs_request_is_allowed(void)
{
    return s_advertised_spr_avs;
}

bool pd_epr_is_available(void)
{
    return ((s_pd.pd_revision != 0u) && (s_cable_epr_capable) &&
            (pd_current_limit_ma() >= PD_5A_CURRENT_MA) &&
            (s_cable_max_vbus_mv >= PD_EPR_MIN_CABLE_VBUS_MV));
}

bool pd_epr_source_capable(void)
{
    return pd_epr_is_available();
}

bool pd_epr_mode_capable_is_advertised(void)
{
    /* This bit describes the Source capability.  Cable capability is
     * verified separately during the EPR Mode Entry AMS. */
    return (s_pd.pd_revision != 0u);
}

/** @brief Return the number of data objects in the current SPR source caps. */
static uint8_t pd_source_cap_object_count(void)
{
    return (uint8_t)(s_pdo_count +
                     s_advertised_spr_avs +
                     pd_pps_is_advertised());
}

/** @brief Return the 1-based EPR fixed PDO position, if present. */
static uint8_t pd_epr_fixed_object_pos(void)
{
    /* The EPR block follows whatever SPR objects were actually advertised. This used to be
     * a fixed position 8, which is only correct when both the SPR AVS APDO and the PPS APDO
     * are present; with a PD 3.1 Port Partner the SPR AVS APDO is not advertised, so the
     * EPR_Source_Capabilities carried a zero padding PDO in the middle and a strict Sink
     * rejected the whole message (the contract then looked lost while VBUS stayed up). */
    return (uint8_t)(s_pdo_count + s_advertised_spr_avs +
                     pd_pps_is_advertised() + 1u);
}

/** @brief Return the 1-based EPR AVS APDO position, if present. */
static uint8_t pd_epr_avs_object_pos(void)
{
    return (uint8_t)(pd_epr_fixed_object_pos() + 1u);
}

/** @brief Return the 1-based SPR AVS APDO position, if present. */
static uint8_t pd_spr_avs_object_pos(void)
{
    return (uint8_t)(s_pdo_count + 1u);
}

/** @brief Return the 1-based PPS APDO position, if present. */
static uint8_t pd_pps_object_pos(void)
{
    return (uint8_t)(s_pdo_count + s_advertised_spr_avs + 1u);
}

uint16_t pd_effective_pps_ma(void)
{
    uint16_t limit = pd_current_limit_ma();

    return (limit >= PD_5A_CURRENT_MA) ? PD_5A_CURRENT_MA : PD_DEFAULT_CURRENT_MA;
}

/** @brief Encode one fixed-supply PDO with the required source capability bits. */
static uint32_t pd_encode_fixed_pdo(uint16_t mv, uint16_t ma, uint8_t first_pdo)
{
    uint32_t current_10ma = (uint32_t)((ma + 5u) / 10u);
    uint32_t voltage_50mv = (uint32_t)((mv + 25u) / 50u);
    uint32_t pdo;

    if (current_10ma > 0x3FFul) current_10ma = 0x3FFul;
    if (voltage_50mv > 0x3FFul) voltage_50mv = 0x3FFul;
    pdo = (voltage_50mv << 10u) | current_10ma;
    if (first_pdo != 0u) {
        pdo |= PD_SRC_CAP_FLAGS;
        if (pd_epr_mode_capable_is_advertised()) {
            pdo |= PD_FIXED_PDO_EPR_MODE;
        }
    }
    return pdo;
}

/** @brief Encode one PPS APDO from its voltage and current limits. */
static uint32_t pd_encode_pps_apdo(uint16_t min_mv, uint16_t max_mv, uint16_t ma)
{
    uint32_t current_50ma = (uint32_t)((ma + 25u) / 50u);
    uint32_t min_100mv = (uint32_t)((min_mv + 50u) / 100u);
    uint32_t max_100mv = (uint32_t)((max_mv + 50u) / 100u);

    if (current_50ma > 0x7Ful) current_50ma = 0x7Ful;
    if (min_100mv > 0xFFul) min_100mv = 0xFFul;
    if (max_100mv > 0xFFul) max_100mv = 0xFFul;

    return PD_PDO_TYPE_AUGMENTED |
           PD_APDO_TYPE_SPR_PPS |
           (max_100mv << 17u) |
           (min_100mv << 8u) |
           current_50ma;
}

/** @brief Encode the configured SPR AVS APDO. */
static uint32_t pd_encode_spr_avs_apdo(uint16_t ma_15v, uint16_t ma_20v)
{
    uint32_t current_15v_10ma = (uint32_t)((ma_15v + 5u) / 10u);
    uint32_t current_20v_10ma = (uint32_t)((ma_20v + 5u) / 10u);

    if (current_15v_10ma > 500ul) current_15v_10ma = 500ul;
    if (current_20v_10ma > 500ul) current_20v_10ma = 500ul;

    return PD_PDO_TYPE_AUGMENTED |
           PD_APDO_TYPE_SPR_AVS |
           (current_15v_10ma << PD_SPR_AVS_APDO_15V_CUR_SHIFT) |
           current_20v_10ma;
}

/** @brief Encode one EPR AVS APDO from its voltage range and power limit. */
static uint32_t pd_encode_epr_avs_apdo(uint16_t min_mv, uint16_t max_mv, uint16_t pdp_w)
{
    uint32_t min_100mv = (uint32_t)(min_mv / 100u);
    uint32_t max_100mv = (uint32_t)(max_mv / 100u);
    uint32_t pdp = (uint32_t)pdp_w;

    if (min_100mv > 0xFFul) min_100mv = 0xFFul;
    if (max_100mv > 0x1FFul) max_100mv = 0x1FFul;
    if (pdp > 240ul) pdp = 240ul;

    return PD_PDO_TYPE_AUGMENTED |
           PD_APDO_TYPE_EPR_AVS |
           (max_100mv << 17u) |
           (min_100mv << 8u) |
           pdp;
}

bool pd_tx_start_source_cap(void)
{
    uint8_t payload[PD_SOURCE_MAX_PDOS * 4u];
    uint8_t i;

    s_advertised_spr_avs = pd_spr_avs_is_advertised();
    for (i = 0u; i < s_pdo_count; i++) {
        pd_put_u32_le(&payload[i * 4u],
                      pd_encode_fixed_pdo(s_pdos[i].mv,
                                          pd_effective_pdo_ma(&s_pdos[i]),
                                          (i == 0u) ? 1u : 0u));
    }
    if (s_advertised_spr_avs != 0u) {
        pd_put_u32_le(&payload[i * 4u],
                      pd_encode_spr_avs_apdo(
                          pd_effective_fixed_current_ma(PD_SPR_AVS_SWITCH_MV),
                          pd_effective_fixed_current_ma(PD_SPR_AVS_MAX_MV)));
        i++;
    }
    if (pd_pps_is_advertised()) {
        pd_put_u32_le(&payload[i * 4u],
                      pd_encode_pps_apdo(PD_PPS_MIN_MV,
                                         PD_PPS_MAX_MV,
                                         pd_effective_pps_ma()));
        i++;
    }
    return pd_tx_start(DEF_TYPE_SRC_CAP,
                       payload,
                       (uint8_t)(pd_source_cap_object_count() * 4u));
}

/** @brief Fill the EPR source-capability payload and return its object count. */
static uint8_t pd_fill_epr_source_cap(uint8_t *payload)
{
    uint8_t i;

    memset(payload, 0, PD_EPR_SOURCE_MAX_PDOS * 4u);
    for (i = 0u; i < s_pdo_count; i++) {
        pd_put_u32_le(&payload[i * 4u],
                      pd_encode_fixed_pdo(s_pdos[i].mv,
                                          pd_effective_pdo_ma(&s_pdos[i]),
                                          (i == 0u) ? 1u : 0u));
    }
    if (s_advertised_spr_avs != 0u) {
        pd_put_u32_le(&payload[i * 4u],
                      pd_encode_spr_avs_apdo(
                          pd_effective_fixed_current_ma(PD_SPR_AVS_SWITCH_MV),
                          pd_effective_fixed_current_ma(PD_SPR_AVS_MAX_MV)));
        i++;
    }
    if (pd_pps_is_advertised()) {
        pd_put_u32_le(&payload[i * 4u],
                      pd_encode_pps_apdo(PD_PPS_MIN_MV,
                                         PD_PPS_MAX_MV,
                                         pd_effective_pps_ma()));
        i++;
    }
    i = (uint8_t)(pd_epr_fixed_object_pos() - 1u);
    pd_put_u32_le(&payload[i * 4u],
                  pd_encode_fixed_pdo(PD_EPR_FIXED_MV,
                                      PD_EPR_FIXED_MA,
                                      0u));
    i++;
    pd_put_u32_le(&payload[i * 4u],
                  pd_encode_epr_avs_apdo(PD_EPR_AVS_MIN_MV,
                                         PD_EPR_AVS_MAX_MV,
                                         PD_EPR_AVS_PDP_W));
    i++;
    return i;
}

/** @brief Fill the SPR extended source-capability data block. */
static void pd_fill_source_cap_ext(uint8_t *payload)
{
    memset(payload, 0, PD_EXT_SOURCE_CAP_DATA_LEN);
    pd_put_u16_le(&payload[0], PD_SOURCE_EXT_VID);
    pd_put_u16_le(&payload[2], PD_SOURCE_EXT_PID);
    pd_put_u32_le(&payload[4], PD_SOURCE_EXT_XID);
    payload[8] = PD_SOURCE_EXT_FW_VERSION;
    payload[9] = PD_SOURCE_EXT_HW_VERSION;
    payload[21] = (uint8_t)(PD_SOURCE_EXT_INPUT_EXTERNAL | PD_SOURCE_EXT_INPUT_DC);
    payload[23] = 100u;
    payload[24] = (PD_SOURCE_DEFAULT_POWER_W > 240u) ? 240u : PD_SOURCE_DEFAULT_POWER_W;
}

bool pd_tx_start_source_cap_ext_chunk(uint8_t chunk_number)
{
    uint8_t payload[PD_EXT_SOURCE_CAP_DATA_LEN];

    pd_fill_source_cap_ext(payload);
    return pd_tx_start_extended_chunk(DEF_TYPE_SRC_CAP,
                                      payload,
                                      PD_EXT_SOURCE_CAP_DATA_LEN,
                                      chunk_number);
}

bool pd_tx_start_source_cap_ext(void)
{
    return pd_tx_start_source_cap_ext_chunk(0u);
}

bool pd_tx_start_epr_source_cap_ext_chunk(uint8_t chunk_number)
{
    uint8_t payload[PD_EPR_SOURCE_MAX_PDOS * 4u];
    uint8_t pdo_count;

    if (!pd_epr_source_capable()) {
        return false;
    }

    if (chunk_number == 0u) {
        s_advertised_spr_avs = pd_spr_avs_is_advertised();
    }
    pdo_count = pd_fill_epr_source_cap(payload);
    if (pd_tx_start_extended_chunk(PD_EXT_EPR_SOURCE_CAP,
                                   payload,
                                   (uint16_t)pdo_count * 4u,
                                   chunk_number) == 0u) {
        return false;
    }
    pd_debug_note_epr_chunk_phase((chunk_number == 0u) ? 1u : 4u);
    return true;
}

bool pd_tx_start_epr_source_cap_ext(void)
{
    return pd_tx_start_epr_source_cap_ext_chunk(0u);
}

/** @brief Decode and clamp the requested current of a fixed-supply RDO. */
static uint16_t pd_fixed_request_current_ma(uint32_t rdo, uint16_t pdo_ma)
{
    uint16_t op_ma;
    uint16_t max_ma;

    op_ma = (uint16_t)((rdo & PD_RDO_OPERATING_CUR_MASK) * 10u);
    max_ma = (uint16_t)(((rdo & PD_RDO_MAX_CUR_MASK) >> PD_RDO_MAX_CUR_SHIFT) * 10u);
    if (max_ma == 0u) {
        max_ma = op_ma;
    }
    (void)max_ma;
    if (op_ma == 0u) {
        op_ma = pdo_ma;
    }
    if (op_ma > pdo_ma) {
        op_ma = pdo_ma;
    }
    return op_ma;
}

/** @brief Calculate the current available from the SPR AVS power limit. */
static uint16_t pd_spr_avs_max_current_ma(uint16_t mv)
{
    if (mv < PD_SPR_AVS_SWITCH_MV) {
        return pd_effective_fixed_current_ma(PD_SPR_AVS_SWITCH_MV);
    }
    return pd_effective_fixed_current_ma(PD_SPR_AVS_MAX_MV);
}

/** @brief Validate an SPR AVS RDO and return its requested operating point. */
static bool pd_validate_spr_avs_rdo(uint32_t rdo,
                                       uint16_t *mv,
                                       uint16_t *ma)
{
    uint32_t voltage_units;
    uint32_t req_mv;
    uint16_t op_ma;
    uint16_t max_ma;

    voltage_units = (rdo & PD_AVS_RDO_VOLTAGE_MASK) >>
                    PD_AVS_RDO_VOLTAGE_SHIFT;
    if ((voltage_units & PD_AVS_RDO_100MV_ALIGN_MASK) != 0ul) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_VOLTAGE;
        return false;
    }

    req_mv = voltage_units * PD_AVS_RDO_VOLTAGE_STEP_MV;
    if ((req_mv < PD_SPR_AVS_MIN_MV) ||
        (req_mv > PD_SPR_AVS_MAX_MV)) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_VOLTAGE;
        return false;
    }

    op_ma = (uint16_t)((rdo & PD_AVS_RDO_OP_CUR_MASK) * 50u);
    max_ma = pd_spr_avs_max_current_ma((uint16_t)req_mv);
    if ((op_ma == 0u) || (max_ma == 0u) || (op_ma > max_ma)) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_CURRENT;
        return false;
    }

    *mv = (uint16_t)req_mv;
    *ma = op_ma;
    return true;
}

/** @brief Calculate the current available from the EPR AVS power limit. */
static uint16_t pd_epr_avs_max_current_ma(uint16_t mv)
{
    uint32_t pdp_w = PD_EPR_AVS_PDP_W;
    uint32_t max_ma;

    if (mv == 0u) {
        return 0u;
    }
    if (pdp_w > 240ul) {
        pdp_w = 240ul;
    }

    max_ma = (pdp_w * 1000000ul) / (uint32_t)mv;
    if (max_ma > PD_5A_CURRENT_MA) {
        max_ma = PD_5A_CURRENT_MA;
    }
    max_ma = (max_ma / 50ul) * 50ul;
    return (uint16_t)max_ma;
}

bool pd_epr_request_is_mode_request(void)
{
    uint32_t eprmdo;
    uint8_t action;

    if (s_rx_msg_len < 10u) {
        return false;
    }

    eprmdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    action = (uint8_t)((eprmdo >> PD_EPR_MODE_ACTION_SHIFT) &
                       PD_EPR_MODE_FIELD_MASK);
    return ((action == PD_EPR_MODE_ACTION_ENTER) || (action == PD_EPR_MODE_ACTION_EXIT));
}

bool pd_epr_request_is_pdo_request(void)
{
    uint32_t rdo;
    uint8_t obj_pos;

    if ((s_rx_msg_len < 10u) ||
        (pd_epr_request_is_mode_request())) {
        return false;
    }

    rdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    obj_pos = (uint8_t)((rdo & PD_RDO_OBJECT_POS_MASK) >> PD_RDO_OBJECT_POS_SHIFT);
    return (obj_pos != 0u);
}


bool pd_handle_epr_mode_request(void)
{
    uint32_t eprmdo;
    uint8_t action;
    uint8_t fail_data = PD_EPR_MODE_FAIL_UNKNOWN;

    if (s_rx_msg_len < 10u) {
        s_epr_fail_reason = PD_EPR_FAIL_BAD_REQUEST;
        return 0u;
    }

    eprmdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    action = (uint8_t)((eprmdo >> PD_EPR_MODE_ACTION_SHIFT) &
                       PD_EPR_MODE_FIELD_MASK);
    s_epr_last_rx_action = action;

    if (action == PD_EPR_MODE_ACTION_ENTER) {
        pd_debug_begin_epr_entry();
        s_epr_last_tx_data = 0u;
    }

    if ((action != PD_EPR_MODE_ACTION_EXIT) &&
        (action != PD_EPR_MODE_ACTION_ENTER)) {
        s_epr_fail_reason = PD_EPR_FAIL_UNKNOWN_ACTION;
        return 1u;
    }

    if ((!pd_tx_is_idle()) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        (pd_is_power_transition_state())) {
        s_epr_fail_reason = PD_EPR_FAIL_BUSY;
        return 0u;
    }

    if (action == PD_EPR_MODE_ACTION_EXIT) {
        if ((s_active_supply_type == PD_SUPPLY_TYPE_EPR_FIXED) ||
            (s_active_supply_type == PD_SUPPLY_TYPE_EPR_AVS)) {
            pd_set_state(PD_STATE_HARD_RESET);
            return 1u;
        }
        s_epr_mode_active = false;
        s_epr_source_cap_pending = false;
        s_epr_source_cap_tx_active = false;
        s_epr_source_cap_retry = 0u;
        s_source_cap_refresh_pending = true;
        s_source_cap_refresh_retries = 1u;
        s_source_cap_refresh_wait_ms = 0u;
        s_epr_fail_reason = PD_EPR_FAIL_NONE;
        return 1u;
    }

    if (pd_fault_active()) {
        fail_data = PD_EPR_MODE_FAIL_UNABLE;
        s_epr_fail_reason = PD_EPR_FAIL_FAULT;
    } else if ((s_pd.state != PD_STATE_READY) &&
               (s_pd.state != PD_STATE_WAIT_REQUEST)) {
        fail_data = PD_EPR_MODE_FAIL_UNABLE;
        s_epr_fail_reason = PD_EPR_FAIL_STATE;
    } else if (s_pd.contract_mv == 0u) {
        fail_data = PD_EPR_MODE_FAIL_UNABLE;
        s_epr_fail_reason = PD_EPR_FAIL_NO_CONTRACT;
    } else if ((s_last_spr_rdo & PD_RDO_EPR_CAPABLE) == 0ul) {
        fail_data = PD_EPR_MODE_FAIL_RDO;
        s_epr_fail_reason = PD_EPR_FAIL_BAD_REQUEST;
    } else if (!pd_epr_mode_capable_is_advertised()) {
        fail_data = PD_EPR_MODE_FAIL_PDO;
        s_epr_fail_reason = PD_EPR_FAIL_UNAVAILABLE;
    }

    if (fail_data != PD_EPR_MODE_FAIL_UNKNOWN) {
        s_epr_mode_active = false;
        s_epr_source_cap_pending = false;
        s_epr_source_cap_tx_active = false;
        s_epr_source_cap_retry = 0u;
        return pd_epr_start_mode_response(PD_EPR_MODE_ACTION_FAILED, fail_data);
    }

    s_epr_mode_active = false;
    s_epr_source_cap_pending = false;
    s_epr_source_cap_tx_active = false;
    s_epr_source_cap_retry = 0u;
    s_epr_fail_reason = PD_EPR_FAIL_NONE;
    s_epr_enter_wait_ms = 0u;
    return pd_epr_start_mode_response(PD_EPR_MODE_ACTION_ENTER_ACK, 0u);
}

/** @brief Validate an EPR RDO and return its requested operating point. */
static bool pd_validate_epr_rdo(uint32_t rdo,
                                   uint32_t requested_pdo,
                                   uint16_t *mv,
                                   uint16_t *ma,
                                   uint8_t *pdo_index)
{
    uint8_t obj_pos;
    uint16_t op_ma;
    uint16_t pdo_mv;
    uint16_t pdo_ma;
    uint32_t expected_pdo;
    uint32_t pdo_mask;
    uint8_t supply_type = PD_SUPPLY_TYPE_NONE;

    s_last_rdo = rdo;
    obj_pos = (uint8_t)((rdo & PD_RDO_OBJECT_POS_MASK) >> PD_RDO_OBJECT_POS_SHIFT);
    s_last_rdo_pos = obj_pos;
    s_last_request_is_pps = false;
    s_last_request_is_epr = false;

    if (obj_pos == pd_epr_fixed_object_pos()) {
        pdo_mv = PD_EPR_FIXED_MV;
        pdo_ma = PD_EPR_FIXED_MA;
        expected_pdo = pd_encode_fixed_pdo(PD_EPR_FIXED_MV,
                                           PD_EPR_FIXED_MA,
                                           0u);
        pdo_mask = PD_PDO_TYPE_MASK |
                   PD_FIXED_PDO_VOLTAGE_MASK |
                   PD_FIXED_PDO_CURRENT_MASK;
        s_last_request_is_epr = true;
        supply_type = PD_SUPPLY_TYPE_EPR_FIXED;
    } else if (obj_pos == pd_epr_avs_object_pos()) {
        uint32_t voltage_units;
        uint32_t req_mv;
        uint16_t max_ma;

        expected_pdo = pd_encode_epr_avs_apdo(PD_EPR_AVS_MIN_MV,
                                              PD_EPR_AVS_MAX_MV,
                                              PD_EPR_AVS_PDP_W);
        pdo_mask = PD_PDO_TYPE_MASK |
                   PD_APDO_TYPE_MASK |
                   PD_AVS_APDO_PEAK_CUR_MASK |
                   PD_AVS_APDO_MAX_VOLT_MASK |
                   PD_AVS_APDO_MIN_VOLT_MASK |
                   PD_AVS_APDO_PDP_MASK;
        if (((requested_pdo ^ expected_pdo) & pdo_mask) != 0ul) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_PDO_COPY;
            return false;
        }

        voltage_units = (rdo & PD_AVS_RDO_VOLTAGE_MASK) >>
                        PD_AVS_RDO_VOLTAGE_SHIFT;
        if ((voltage_units & PD_AVS_RDO_100MV_ALIGN_MASK) != 0ul) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_VOLTAGE;
            return false;
        }

        req_mv = voltage_units * PD_AVS_RDO_VOLTAGE_STEP_MV;
        if ((req_mv < PD_EPR_AVS_MIN_MV) ||
            (req_mv > PD_EPR_AVS_MAX_MV)) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_VOLTAGE;
            return false;
        }

        op_ma = (uint16_t)((rdo & PD_AVS_RDO_OP_CUR_MASK) * 50u);
        max_ma = pd_epr_avs_max_current_ma((uint16_t)req_mv);
        if ((op_ma == 0u) || (op_ma > max_ma)) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_CURRENT;
            return false;
        }

        *mv = (uint16_t)req_mv;
        *ma = op_ma;
        *pdo_index = obj_pos;
        s_last_request_is_epr = true;
        s_active_supply_type = PD_SUPPLY_TYPE_EPR_AVS;
        return true;
    } else if ((pd_spr_avs_request_is_allowed() != 0u) &&
               (obj_pos == pd_spr_avs_object_pos())) {
        expected_pdo = pd_encode_spr_avs_apdo(
            pd_effective_fixed_current_ma(PD_SPR_AVS_SWITCH_MV),
            pd_effective_fixed_current_ma(PD_SPR_AVS_MAX_MV));
        pdo_mask = PD_PDO_TYPE_MASK |
                   PD_APDO_TYPE_MASK |
                   PD_AVS_APDO_PEAK_CUR_MASK |
                   PD_SPR_AVS_APDO_15V_CUR_MASK |
                   PD_SPR_AVS_APDO_20V_CUR_MASK;
        if (((requested_pdo ^ expected_pdo) & pdo_mask) != 0ul) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_PDO_COPY;
            return false;
        }
        if (!pd_validate_spr_avs_rdo(rdo, mv, ma)) {
            return false;
        }
        *pdo_index = obj_pos;
        s_partner_revision = PD_PARTNER_REVISION_32;
        s_active_supply_type = PD_SUPPLY_TYPE_SPR_AVS;
        return true;
    } else if ((pd_pps_is_advertised()) &&
               (obj_pos == pd_pps_object_pos())) {
        uint16_t req_mv;
        uint16_t max_ma;

        expected_pdo = pd_encode_pps_apdo(PD_PPS_MIN_MV,
                                          PD_PPS_MAX_MV,
                                          pd_effective_pps_ma());
        pdo_mask = PD_PDO_TYPE_MASK |
                   PD_APDO_TYPE_MASK |
                   PD_AVS_APDO_MAX_VOLT_MASK |
                   PD_AVS_APDO_MIN_VOLT_MASK |
                   PD_AVS_APDO_PDP_MASK;
        if (((requested_pdo ^ expected_pdo) & pdo_mask) != 0ul) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_PDO_COPY;
            return false;
        }

        req_mv = (uint16_t)(((rdo & PD_PPS_RDO_VOLTAGE_MASK) >>
                             PD_PPS_RDO_VOLTAGE_SHIFT) * 20u);
        op_ma = (uint16_t)((rdo & PD_PPS_RDO_OP_CUR_MASK) * 50u);
        max_ma = pd_effective_pps_ma();
        if ((req_mv < PD_PPS_MIN_MV) ||
            (req_mv > PD_PPS_MAX_MV) ||
            (op_ma == 0u) ||
            (op_ma > max_ma)) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_CURRENT;
            return false;
        }

        *mv = req_mv;
        *ma = op_ma;
        *pdo_index = obj_pos;
        s_last_request_is_pps = true;
        s_active_supply_type = PD_SUPPLY_TYPE_SPR_PPS;
        return true;
    } else if ((obj_pos >= 1u) && (obj_pos <= s_pdo_count)) {
        pdo_mv = s_pdos[obj_pos - 1u].mv;
        pdo_ma = pd_effective_pdo_ma(&s_pdos[obj_pos - 1u]);
        expected_pdo = pd_encode_fixed_pdo(pdo_mv,
                                           pdo_ma,
                                           (obj_pos == 1u) ? 1u : 0u);
        pdo_mask = PD_PDO_TYPE_MASK |
                   PD_FIXED_PDO_VOLTAGE_MASK |
                   PD_FIXED_PDO_CURRENT_MASK;
        supply_type = PD_SUPPLY_TYPE_SPR_FIXED;
    } else {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_OBJ;
        return false;
    }

    if (((requested_pdo ^ expected_pdo) & pdo_mask) != 0ul) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_PDO_COPY;
        return false;
    }

    op_ma = pd_fixed_request_current_ma(rdo, pdo_ma);
    if (op_ma == 0u) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_CURRENT;
        return false;
    }

    *mv = pdo_mv;
    *ma = op_ma;
    *pdo_index = obj_pos;
    s_active_supply_type = supply_type;
    return true;
}


bool pd_validate_epr_request(uint16_t *mv, uint16_t *ma, uint8_t *pdo_index)
{
    uint32_t rdo;
    uint32_t requested_pdo;
    uint8_t num_do;

    if (pd_fault_active()) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_FAULT;
        return 0u;
    }
    if (s_rx_msg_len < 10u) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_LEN;
        return 0u;
    }

    rdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    s_last_rdo = rdo;
    s_last_rdo_pos = (uint8_t)((rdo & PD_RDO_OBJECT_POS_MASK) >> PD_RDO_OBJECT_POS_SHIFT);

    if (s_rx_msg_len < 14u) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_LEN;
        return 0u;
    }
    if ((!s_epr_mode_active) || (!pd_epr_is_available())) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_MODE;
        return 0u;
    }

    num_do = (uint8_t)((s_pd_rx_buf[1] >> 4u) & 0x07u);
    if (num_do != 2u) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_NUM_DO;
        return 0u;
    }

    requested_pdo = pd_get_u32_le(&s_pd_rx_buf[6]);
    return pd_validate_epr_rdo(rdo, requested_pdo, mv, ma, pdo_index);
}


bool pd_validate_request(uint16_t *mv, uint16_t *ma, uint8_t *pdo_index)
{
    uint32_t rdo;
    uint8_t obj_pos;
    uint16_t op_ma;
    uint16_t max_ma;
    uint16_t pdo_ma;
    const pd_fixed_pdo_t *pdo;

    if (pd_fault_active()) return 0u;
    if (s_rx_msg_len < 10u) return 0u;

    rdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    s_last_rdo = rdo;
    obj_pos = (uint8_t)((rdo & PD_RDO_OBJECT_POS_MASK) >> PD_RDO_OBJECT_POS_SHIFT);
    s_last_rdo_pos = obj_pos;
    s_last_request_is_pps = false;
    s_last_request_is_epr = false;
    s_epr_fail_reason = PD_EPR_FAIL_NONE;
    if ((obj_pos == 0u) || (obj_pos > pd_source_cap_object_count())) return 0u;

    if ((pd_spr_avs_request_is_allowed() != 0u) &&
        (obj_pos == pd_spr_avs_object_pos())) {
        if (!pd_validate_spr_avs_rdo(rdo, mv, ma)) {
            return false;
        }
        *pdo_index = obj_pos;
        s_last_spr_rdo = rdo;
        s_partner_revision = PD_PARTNER_REVISION_32;
        s_active_supply_type = PD_SUPPLY_TYPE_SPR_AVS;
        return true;
    }

    if ((pd_pps_is_advertised()) && (obj_pos == pd_pps_object_pos())) {
        uint16_t req_mv;
        uint16_t op_ma_pps;
        uint16_t max_ma_pps = pd_effective_pps_ma();

        req_mv = (uint16_t)(((rdo & PD_PPS_RDO_VOLTAGE_MASK) >>
                             PD_PPS_RDO_VOLTAGE_SHIFT) * 20u);
        op_ma_pps = (uint16_t)((rdo & PD_PPS_RDO_OP_CUR_MASK) * 50u);

        if ((req_mv < PD_PPS_MIN_MV) ||
            (req_mv > PD_PPS_MAX_MV) ||
            (op_ma_pps == 0u) ||
            (op_ma_pps > max_ma_pps)) {
            return false;
        }

        *mv = req_mv;
        *ma = op_ma_pps;
        *pdo_index = obj_pos;
        s_last_request_is_pps = true;
        s_last_spr_rdo = rdo;
        s_active_supply_type = PD_SUPPLY_TYPE_SPR_PPS;
        return true;
    }

    if (obj_pos > s_pdo_count) {
        return false;
    }

    pdo = &s_pdos[obj_pos - 1u];
    pdo_ma = pd_effective_pdo_ma(pdo);
    op_ma = pd_fixed_request_current_ma(rdo, pdo_ma);
    max_ma = (uint16_t)(((rdo & PD_RDO_MAX_CUR_MASK) >> PD_RDO_MAX_CUR_SHIFT) * 10u);
    (void)max_ma;

    *mv = pdo->mv;
    *ma = op_ma;
    *pdo_index = obj_pos;
    s_last_spr_rdo = rdo;
    s_active_supply_type = PD_SUPPLY_TYPE_SPR_FIXED;
    return true;
}
