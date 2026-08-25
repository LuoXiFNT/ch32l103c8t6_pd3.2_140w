/**
 * @file pd_capability.c
 * @brief Source PDO/APDO construction and Request/RDO validation.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
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
static uint8_t pd_pps_is_advertised(void)
{
    return (s_pd.pd_revision != 0U) ? 1U : 0U;
}

/** @brief Return the current limit for the fixed PDO at the requested voltage. */
static uint16_t pd_effective_fixed_current_ma(uint16_t mv)
{
    uint8_t i;

    for (i = 0U; i < s_pdo_count; i++) {
        if (s_pdos[i].mv == mv) {
            return pd_effective_pdo_ma(&s_pdos[i]);
        }
    }
    return 0U;
}

/** @brief Check whether an SPR AVS APDO is currently advertised. */
static uint8_t pd_spr_avs_is_advertised(void)
{
    uint8_t incompatible_partner =
        ((s_partner_revision == PD_PARTNER_REVISION_20) ||
         (s_partner_revision == PD_PARTNER_REVISION_30) ||
         (s_partner_revision == PD_PARTNER_REVISION_31)) ? 1U : 0U;

    return ((s_pd.pd_revision != 0U) &&
            (incompatible_partner == 0U) &&
            (pd_effective_fixed_current_ma(PD_SPR_AVS_SWITCH_MV) != 0U) &&
            (pd_effective_fixed_current_ma(PD_SPR_AVS_MAX_MV) != 0U)) ? 1U : 0U;
}

/** @brief Check whether the active cable and policy permit SPR AVS requests. */
static uint8_t pd_spr_avs_request_is_allowed(void)
{
    return s_advertised_spr_avs;
}

uint8_t pd_epr_is_available(void)
{
    return ((s_pd.pd_revision != 0U) &&
            (s_cable_epr_capable != 0U) &&
            (pd_current_limit_ma() >= PD_5A_CURRENT_MA) &&
            (s_cable_max_vbus_mv >= PD_EPR_MIN_CABLE_VBUS_MV)) ? 1U : 0U;
}

uint8_t pd_epr_source_capable(void)
{
    return pd_epr_is_available();
}

uint8_t pd_epr_mode_capable_is_advertised(void)
{
    /* This bit describes the Source capability.  Cable capability is
     * verified separately during the EPR Mode Entry AMS. */
    return (s_pd.pd_revision != 0U) ? 1U : 0U;
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
    return 8U;
}

/** @brief Return the 1-based EPR AVS APDO position, if present. */
static uint8_t pd_epr_avs_object_pos(void)
{
    return 9U;
}

/** @brief Return the 1-based SPR AVS APDO position, if present. */
static uint8_t pd_spr_avs_object_pos(void)
{
    return (uint8_t)(s_pdo_count + 1U);
}

/** @brief Return the 1-based PPS APDO position, if present. */
static uint8_t pd_pps_object_pos(void)
{
    return (uint8_t)(s_pdo_count + s_advertised_spr_avs + 1U);
}

uint16_t pd_effective_pps_ma(void)
{
    uint16_t limit = pd_current_limit_ma();

    return (limit >= PD_5A_CURRENT_MA) ? PD_5A_CURRENT_MA : PD_DEFAULT_CURRENT_MA;
}

/** @brief Encode one fixed-supply PDO with the required source capability bits. */
static uint32_t pd_encode_fixed_pdo(uint16_t mv, uint16_t ma, uint8_t first_pdo)
{
    uint32_t current_10ma = (uint32_t)((ma + 5U) / 10U);
    uint32_t voltage_50mv = (uint32_t)((mv + 25U) / 50U);
    uint32_t pdo;

    if (current_10ma > 0x3FFUL) current_10ma = 0x3FFUL;
    if (voltage_50mv > 0x3FFUL) voltage_50mv = 0x3FFUL;
    pdo = (voltage_50mv << 10U) | current_10ma;
    if (first_pdo != 0U) {
        pdo |= PD_SRC_CAP_FLAGS;
        if (pd_epr_mode_capable_is_advertised() != 0U) {
            pdo |= PD_FIXED_PDO_EPR_MODE;
        }
    }
    return pdo;
}

/** @brief Encode one PPS APDO from its voltage and current limits. */
static uint32_t pd_encode_pps_apdo(uint16_t min_mv, uint16_t max_mv, uint16_t ma)
{
    uint32_t current_50ma = (uint32_t)((ma + 25U) / 50U);
    uint32_t min_100mv = (uint32_t)((min_mv + 50U) / 100U);
    uint32_t max_100mv = (uint32_t)((max_mv + 50U) / 100U);

    if (current_50ma > 0x7FUL) current_50ma = 0x7FUL;
    if (min_100mv > 0xFFUL) min_100mv = 0xFFUL;
    if (max_100mv > 0xFFUL) max_100mv = 0xFFUL;

    return PD_PDO_TYPE_AUGMENTED |
           PD_APDO_TYPE_SPR_PPS |
           (max_100mv << 17U) |
           (min_100mv << 8U) |
           current_50ma;
}

/** @brief Encode the configured SPR AVS APDO. */
static uint32_t pd_encode_spr_avs_apdo(uint16_t ma_15v, uint16_t ma_20v)
{
    uint32_t current_15v_10ma = (uint32_t)((ma_15v + 5U) / 10U);
    uint32_t current_20v_10ma = (uint32_t)((ma_20v + 5U) / 10U);

    if (current_15v_10ma > 500UL) current_15v_10ma = 500UL;
    if (current_20v_10ma > 500UL) current_20v_10ma = 500UL;

    return PD_PDO_TYPE_AUGMENTED |
           PD_APDO_TYPE_SPR_AVS |
           (current_15v_10ma << PD_SPR_AVS_APDO_15V_CUR_SHIFT) |
           current_20v_10ma;
}

/** @brief Encode one EPR AVS APDO from its voltage range and power limit. */
static uint32_t pd_encode_epr_avs_apdo(uint16_t min_mv, uint16_t max_mv, uint16_t pdp_w)
{
    uint32_t min_100mv = (uint32_t)(min_mv / 100U);
    uint32_t max_100mv = (uint32_t)(max_mv / 100U);
    uint32_t pdp = (uint32_t)pdp_w;

    if (min_100mv > 0xFFUL) min_100mv = 0xFFUL;
    if (max_100mv > 0x1FFUL) max_100mv = 0x1FFUL;
    if (pdp > 240UL) pdp = 240UL;

    return PD_PDO_TYPE_AUGMENTED |
           PD_APDO_TYPE_EPR_AVS |
           (max_100mv << 17U) |
           (min_100mv << 8U) |
           pdp;
}

uint8_t pd_tx_start_source_cap(void)
{
    uint8_t payload[PD_SOURCE_MAX_PDOS * 4U];
    uint8_t i;

    s_advertised_spr_avs = pd_spr_avs_is_advertised();
    for (i = 0U; i < s_pdo_count; i++) {
        pd_put_u32_le(&payload[i * 4U],
                      pd_encode_fixed_pdo(s_pdos[i].mv,
                                          pd_effective_pdo_ma(&s_pdos[i]),
                                          (i == 0U) ? 1U : 0U));
    }
    if (s_advertised_spr_avs != 0U) {
        pd_put_u32_le(&payload[i * 4U],
                      pd_encode_spr_avs_apdo(
                          pd_effective_fixed_current_ma(PD_SPR_AVS_SWITCH_MV),
                          pd_effective_fixed_current_ma(PD_SPR_AVS_MAX_MV)));
        i++;
    }
    if (pd_pps_is_advertised() != 0U) {
        pd_put_u32_le(&payload[i * 4U],
                      pd_encode_pps_apdo(PD_PPS_MIN_MV,
                                         PD_PPS_MAX_MV,
                                         pd_effective_pps_ma()));
        i++;
    }
    return pd_tx_start(DEF_TYPE_SRC_CAP,
                       payload,
                       (uint8_t)(pd_source_cap_object_count() * 4U));
}

/** @brief Fill the EPR source-capability payload and return its object count. */
static uint8_t pd_fill_epr_source_cap(uint8_t *payload)
{
    uint8_t i;

    memset(payload, 0, PD_EPR_SOURCE_MAX_PDOS * 4U);
    for (i = 0U; i < s_pdo_count; i++) {
        pd_put_u32_le(&payload[i * 4U],
                      pd_encode_fixed_pdo(s_pdos[i].mv,
                                          pd_effective_pdo_ma(&s_pdos[i]),
                                          (i == 0U) ? 1U : 0U));
    }
    if (s_advertised_spr_avs != 0U) {
        pd_put_u32_le(&payload[i * 4U],
                      pd_encode_spr_avs_apdo(
                          pd_effective_fixed_current_ma(PD_SPR_AVS_SWITCH_MV),
                          pd_effective_fixed_current_ma(PD_SPR_AVS_MAX_MV)));
        i++;
    }
    if (pd_pps_is_advertised() != 0U) {
        pd_put_u32_le(&payload[i * 4U],
                      pd_encode_pps_apdo(PD_PPS_MIN_MV,
                                         PD_PPS_MAX_MV,
                                         pd_effective_pps_ma()));
        i++;
    }
    i = (uint8_t)(pd_epr_fixed_object_pos() - 1U);
    pd_put_u32_le(&payload[i * 4U],
                  pd_encode_fixed_pdo(PD_EPR_FIXED_MV,
                                      PD_EPR_FIXED_MA,
                                      0U));
    i++;
    pd_put_u32_le(&payload[i * 4U],
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
    payload[23] = 100U;
    payload[24] = (PD_SOURCE_DEFAULT_POWER_W > 240U) ? 240U : PD_SOURCE_DEFAULT_POWER_W;
}

uint8_t pd_tx_start_source_cap_ext_chunk(uint8_t chunk_number)
{
    uint8_t payload[PD_EXT_SOURCE_CAP_DATA_LEN];

    pd_fill_source_cap_ext(payload);
    return pd_tx_start_extended_chunk(DEF_TYPE_SRC_CAP,
                                      payload,
                                      PD_EXT_SOURCE_CAP_DATA_LEN,
                                      chunk_number);
}

uint8_t pd_tx_start_source_cap_ext(void)
{
    return pd_tx_start_source_cap_ext_chunk(0U);
}

uint8_t pd_tx_start_epr_source_cap_ext_chunk(uint8_t chunk_number)
{
    uint8_t payload[PD_EPR_SOURCE_MAX_PDOS * 4U];
    uint8_t pdo_count;

    if (pd_epr_source_capable() == 0U) {
        return 0U;
    }

    if (chunk_number == 0U) {
        s_advertised_spr_avs = pd_spr_avs_is_advertised();
    }
    pdo_count = pd_fill_epr_source_cap(payload);
    if (pd_tx_start_extended_chunk(PD_EXT_EPR_SOURCE_CAP,
                                   payload,
                                   (uint16_t)pdo_count * 4U,
                                   chunk_number) == 0U) {
        return 0U;
    }
    pd_debug_note_epr_chunk_phase((chunk_number == 0U) ? 1U : 4U);
    return 1U;
}

uint8_t pd_tx_start_epr_source_cap_ext(void)
{
    return pd_tx_start_epr_source_cap_ext_chunk(0U);
}

/** @brief Decode and clamp the requested current of a fixed-supply RDO. */
static uint16_t pd_fixed_request_current_ma(uint32_t rdo, uint16_t pdo_ma)
{
    uint16_t op_ma;
    uint16_t max_ma;

    op_ma = (uint16_t)((rdo & PD_RDO_OPERATING_CUR_MASK) * 10U);
    max_ma = (uint16_t)(((rdo & PD_RDO_MAX_CUR_MASK) >> PD_RDO_MAX_CUR_SHIFT) * 10U);
    if (max_ma == 0U) {
        max_ma = op_ma;
    }
    (void)max_ma;
    if (op_ma == 0U) {
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
static uint8_t pd_validate_spr_avs_rdo(uint32_t rdo,
                                       uint16_t *mv,
                                       uint16_t *ma)
{
    uint32_t voltage_units;
    uint32_t req_mv;
    uint16_t op_ma;
    uint16_t max_ma;

    voltage_units = (rdo & PD_AVS_RDO_VOLTAGE_MASK) >>
                    PD_AVS_RDO_VOLTAGE_SHIFT;
    if ((voltage_units & PD_AVS_RDO_100MV_ALIGN_MASK) != 0UL) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_VOLTAGE;
        return 0U;
    }

    req_mv = voltage_units * PD_AVS_RDO_VOLTAGE_STEP_MV;
    if ((req_mv < PD_SPR_AVS_MIN_MV) ||
        (req_mv > PD_SPR_AVS_MAX_MV)) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_VOLTAGE;
        return 0U;
    }

    op_ma = (uint16_t)((rdo & PD_AVS_RDO_OP_CUR_MASK) * 50U);
    max_ma = pd_spr_avs_max_current_ma((uint16_t)req_mv);
    if ((op_ma == 0U) || (max_ma == 0U) || (op_ma > max_ma)) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_CURRENT;
        return 0U;
    }

    *mv = (uint16_t)req_mv;
    *ma = op_ma;
    return 1U;
}

/** @brief Calculate the current available from the EPR AVS power limit. */
static uint16_t pd_epr_avs_max_current_ma(uint16_t mv)
{
    uint32_t pdp_w = PD_EPR_AVS_PDP_W;
    uint32_t max_ma;

    if (mv == 0U) {
        return 0U;
    }
    if (pdp_w > 240UL) {
        pdp_w = 240UL;
    }

    max_ma = (pdp_w * 1000000UL) / (uint32_t)mv;
    if (max_ma > PD_5A_CURRENT_MA) {
        max_ma = PD_5A_CURRENT_MA;
    }
    max_ma = (max_ma / 50UL) * 50UL;
    return (uint16_t)max_ma;
}

uint8_t pd_epr_request_is_mode_request(void)
{
    uint32_t eprmdo;
    uint8_t action;

    if (s_rx_msg_len < 10U) {
        return 0U;
    }

    eprmdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    action = (uint8_t)((eprmdo >> PD_EPR_MODE_ACTION_SHIFT) &
                       PD_EPR_MODE_FIELD_MASK);
    return ((action == PD_EPR_MODE_ACTION_ENTER) ||
            (action == PD_EPR_MODE_ACTION_EXIT)) ? 1U : 0U;
}

uint8_t pd_epr_request_is_pdo_request(void)
{
    uint32_t rdo;
    uint8_t obj_pos;

    if ((s_rx_msg_len < 10U) ||
        (pd_epr_request_is_mode_request() != 0U)) {
        return 0U;
    }

    rdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    obj_pos = (uint8_t)((rdo & PD_RDO_OBJECT_POS_MASK) >> PD_RDO_OBJECT_POS_SHIFT);
    return (obj_pos != 0U) ? 1U : 0U;
}


uint8_t pd_handle_epr_mode_request(void)
{
    uint32_t eprmdo;
    uint8_t action;
    uint8_t fail_data = PD_EPR_MODE_FAIL_UNKNOWN;

    if (s_rx_msg_len < 10U) {
        s_epr_fail_reason = PD_EPR_FAIL_BAD_REQUEST;
        return 0U;
    }

    eprmdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    action = (uint8_t)((eprmdo >> PD_EPR_MODE_ACTION_SHIFT) &
                       PD_EPR_MODE_FIELD_MASK);
    s_epr_last_rx_action = action;

    if (action == PD_EPR_MODE_ACTION_ENTER) {
        pd_debug_begin_epr_entry();
        s_epr_last_tx_data = 0U;
    }

    if ((action != PD_EPR_MODE_ACTION_EXIT) &&
        (action != PD_EPR_MODE_ACTION_ENTER)) {
        s_epr_fail_reason = PD_EPR_FAIL_UNKNOWN_ACTION;
        return 1U;
    }

    if ((pd_tx_is_idle() == 0U) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        (pd_is_power_transition_state() != 0U)) {
        s_epr_fail_reason = PD_EPR_FAIL_BUSY;
        return 0U;
    }

    if (action == PD_EPR_MODE_ACTION_EXIT) {
        if ((s_active_supply_type == PD_SUPPLY_TYPE_EPR_FIXED) ||
            (s_active_supply_type == PD_SUPPLY_TYPE_EPR_AVS)) {
            pd_set_state(PD_STATE_HARD_RESET);
            return 1U;
        }
        s_epr_mode_active = 0U;
        s_epr_source_cap_pending = 0U;
        s_epr_source_cap_tx_active = 0U;
        s_epr_source_cap_retry = 0U;
        s_source_cap_refresh_pending = 1U;
        s_source_cap_refresh_retries = 1U;
        s_source_cap_refresh_wait_ms = 0U;
        s_epr_fail_reason = PD_EPR_FAIL_NONE;
        return 1U;
    }

    if (pd_fault_active() != 0U) {
        fail_data = PD_EPR_MODE_FAIL_UNABLE;
        s_epr_fail_reason = PD_EPR_FAIL_FAULT;
    } else if ((s_pd.state != PD_STATE_READY) &&
               (s_pd.state != PD_STATE_WAIT_REQUEST)) {
        fail_data = PD_EPR_MODE_FAIL_UNABLE;
        s_epr_fail_reason = PD_EPR_FAIL_STATE;
    } else if (s_pd.contract_mv == 0U) {
        fail_data = PD_EPR_MODE_FAIL_UNABLE;
        s_epr_fail_reason = PD_EPR_FAIL_NO_CONTRACT;
    } else if ((s_last_spr_rdo & PD_RDO_EPR_CAPABLE) == 0UL) {
        fail_data = PD_EPR_MODE_FAIL_RDO;
        s_epr_fail_reason = PD_EPR_FAIL_BAD_REQUEST;
    } else if (pd_epr_mode_capable_is_advertised() == 0U) {
        fail_data = PD_EPR_MODE_FAIL_PDO;
        s_epr_fail_reason = PD_EPR_FAIL_UNAVAILABLE;
    }

    if (fail_data != PD_EPR_MODE_FAIL_UNKNOWN) {
        s_epr_mode_active = 0U;
        s_epr_source_cap_pending = 0U;
        s_epr_source_cap_tx_active = 0U;
        s_epr_source_cap_retry = 0U;
        return pd_epr_start_mode_response(PD_EPR_MODE_ACTION_FAILED, fail_data);
    }

    s_epr_mode_active = 0U;
    s_epr_source_cap_pending = 0U;
    s_epr_source_cap_tx_active = 0U;
    s_epr_source_cap_retry = 0U;
    s_epr_fail_reason = PD_EPR_FAIL_NONE;
    s_epr_enter_wait_ms = 0U;
    return pd_epr_start_mode_response(PD_EPR_MODE_ACTION_ENTER_ACK, 0U);
}

/** @brief Validate an EPR RDO and return its requested operating point. */
static uint8_t pd_validate_epr_rdo(uint32_t rdo,
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
    s_last_request_is_pps = 0U;
    s_last_request_is_epr = 0U;

    if (obj_pos == pd_epr_fixed_object_pos()) {
        pdo_mv = PD_EPR_FIXED_MV;
        pdo_ma = PD_EPR_FIXED_MA;
        expected_pdo = pd_encode_fixed_pdo(PD_EPR_FIXED_MV,
                                           PD_EPR_FIXED_MA,
                                           0U);
        pdo_mask = PD_PDO_TYPE_MASK |
                   PD_FIXED_PDO_VOLTAGE_MASK |
                   PD_FIXED_PDO_CURRENT_MASK;
        s_last_request_is_epr = 1U;
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
        if (((requested_pdo ^ expected_pdo) & pdo_mask) != 0UL) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_PDO_COPY;
            return 0U;
        }

        voltage_units = (rdo & PD_AVS_RDO_VOLTAGE_MASK) >>
                        PD_AVS_RDO_VOLTAGE_SHIFT;
        if ((voltage_units & PD_AVS_RDO_100MV_ALIGN_MASK) != 0UL) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_VOLTAGE;
            return 0U;
        }

        req_mv = voltage_units * PD_AVS_RDO_VOLTAGE_STEP_MV;
        if ((req_mv < PD_EPR_AVS_MIN_MV) ||
            (req_mv > PD_EPR_AVS_MAX_MV)) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_VOLTAGE;
            return 0U;
        }

        op_ma = (uint16_t)((rdo & PD_AVS_RDO_OP_CUR_MASK) * 50U);
        max_ma = pd_epr_avs_max_current_ma((uint16_t)req_mv);
        if ((op_ma == 0U) || (op_ma > max_ma)) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_CURRENT;
            return 0U;
        }

        *mv = (uint16_t)req_mv;
        *ma = op_ma;
        *pdo_index = obj_pos;
        s_last_request_is_epr = 1U;
        s_active_supply_type = PD_SUPPLY_TYPE_EPR_AVS;
        return 1U;
    } else if ((pd_spr_avs_request_is_allowed() != 0U) &&
               (obj_pos == pd_spr_avs_object_pos())) {
        expected_pdo = pd_encode_spr_avs_apdo(
            pd_effective_fixed_current_ma(PD_SPR_AVS_SWITCH_MV),
            pd_effective_fixed_current_ma(PD_SPR_AVS_MAX_MV));
        pdo_mask = PD_PDO_TYPE_MASK |
                   PD_APDO_TYPE_MASK |
                   PD_AVS_APDO_PEAK_CUR_MASK |
                   PD_SPR_AVS_APDO_15V_CUR_MASK |
                   PD_SPR_AVS_APDO_20V_CUR_MASK;
        if (((requested_pdo ^ expected_pdo) & pdo_mask) != 0UL) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_PDO_COPY;
            return 0U;
        }
        if (pd_validate_spr_avs_rdo(rdo, mv, ma) == 0U) {
            return 0U;
        }
        *pdo_index = obj_pos;
        s_partner_revision = PD_PARTNER_REVISION_32;
        s_active_supply_type = PD_SUPPLY_TYPE_SPR_AVS;
        return 1U;
    } else if ((pd_pps_is_advertised() != 0U) &&
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
        if (((requested_pdo ^ expected_pdo) & pdo_mask) != 0UL) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_PDO_COPY;
            return 0U;
        }

        req_mv = (uint16_t)(((rdo & PD_PPS_RDO_VOLTAGE_MASK) >>
                             PD_PPS_RDO_VOLTAGE_SHIFT) * 20U);
        op_ma = (uint16_t)((rdo & PD_PPS_RDO_OP_CUR_MASK) * 50U);
        max_ma = pd_effective_pps_ma();
        if ((req_mv < PD_PPS_MIN_MV) ||
            (req_mv > PD_PPS_MAX_MV) ||
            (op_ma == 0U) ||
            (op_ma > max_ma)) {
            s_epr_fail_reason = PD_EPR_FAIL_REQ_CURRENT;
            return 0U;
        }

        *mv = req_mv;
        *ma = op_ma;
        *pdo_index = obj_pos;
        s_last_request_is_pps = 1U;
        s_active_supply_type = PD_SUPPLY_TYPE_SPR_PPS;
        return 1U;
    } else if ((obj_pos >= 1U) && (obj_pos <= s_pdo_count)) {
        pdo_mv = s_pdos[obj_pos - 1U].mv;
        pdo_ma = pd_effective_pdo_ma(&s_pdos[obj_pos - 1U]);
        expected_pdo = pd_encode_fixed_pdo(pdo_mv,
                                           pdo_ma,
                                           (obj_pos == 1U) ? 1U : 0U);
        pdo_mask = PD_PDO_TYPE_MASK |
                   PD_FIXED_PDO_VOLTAGE_MASK |
                   PD_FIXED_PDO_CURRENT_MASK;
        supply_type = PD_SUPPLY_TYPE_SPR_FIXED;
    } else {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_OBJ;
        return 0U;
    }

    if (((requested_pdo ^ expected_pdo) & pdo_mask) != 0UL) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_PDO_COPY;
        return 0U;
    }

    op_ma = pd_fixed_request_current_ma(rdo, pdo_ma);
    if (op_ma == 0U) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_CURRENT;
        return 0U;
    }

    *mv = pdo_mv;
    *ma = op_ma;
    *pdo_index = obj_pos;
    s_active_supply_type = supply_type;
    return 1U;
}


uint8_t pd_validate_epr_request(uint16_t *mv, uint16_t *ma, uint8_t *pdo_index)
{
    uint32_t rdo;
    uint32_t requested_pdo;
    uint8_t num_do;

    if (pd_fault_active() != 0U) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_FAULT;
        return 0U;
    }
    if (s_rx_msg_len < 10U) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_LEN;
        return 0U;
    }

    rdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    s_last_rdo = rdo;
    s_last_rdo_pos = (uint8_t)((rdo & PD_RDO_OBJECT_POS_MASK) >> PD_RDO_OBJECT_POS_SHIFT);

    if (s_rx_msg_len < 14U) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_LEN;
        return 0U;
    }
    if ((s_epr_mode_active == 0U) || (pd_epr_is_available() == 0U)) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_MODE;
        return 0U;
    }

    num_do = (uint8_t)((s_pd_rx_buf[1] >> 4U) & 0x07U);
    if (num_do != 2U) {
        s_epr_fail_reason = PD_EPR_FAIL_REQ_NUM_DO;
        return 0U;
    }

    requested_pdo = pd_get_u32_le(&s_pd_rx_buf[6]);
    return pd_validate_epr_rdo(rdo, requested_pdo, mv, ma, pdo_index);
}


uint8_t pd_validate_request(uint16_t *mv, uint16_t *ma, uint8_t *pdo_index)
{
    uint32_t rdo;
    uint8_t obj_pos;
    uint16_t op_ma;
    uint16_t max_ma;
    uint16_t pdo_ma;
    const pd_fixed_pdo_t *pdo;

    if (pd_fault_active() != 0U) return 0U;
    if (s_rx_msg_len < 10U) return 0U;

    rdo = pd_get_u32_le(&s_pd_rx_buf[2]);
    s_last_rdo = rdo;
    obj_pos = (uint8_t)((rdo & PD_RDO_OBJECT_POS_MASK) >> PD_RDO_OBJECT_POS_SHIFT);
    s_last_rdo_pos = obj_pos;
    s_last_request_is_pps = 0U;
    s_last_request_is_epr = 0U;
    s_epr_fail_reason = PD_EPR_FAIL_NONE;
    if ((obj_pos == 0U) || (obj_pos > pd_source_cap_object_count())) return 0U;

    if ((pd_spr_avs_request_is_allowed() != 0U) &&
        (obj_pos == pd_spr_avs_object_pos())) {
        if (pd_validate_spr_avs_rdo(rdo, mv, ma) == 0U) {
            return 0U;
        }
        *pdo_index = obj_pos;
        s_last_spr_rdo = rdo;
        s_partner_revision = PD_PARTNER_REVISION_32;
        s_active_supply_type = PD_SUPPLY_TYPE_SPR_AVS;
        return 1U;
    }

    if ((pd_pps_is_advertised() != 0U) && (obj_pos == pd_pps_object_pos())) {
        uint16_t req_mv;
        uint16_t op_ma_pps;
        uint16_t max_ma_pps = pd_effective_pps_ma();

        req_mv = (uint16_t)(((rdo & PD_PPS_RDO_VOLTAGE_MASK) >>
                             PD_PPS_RDO_VOLTAGE_SHIFT) * 20U);
        op_ma_pps = (uint16_t)((rdo & PD_PPS_RDO_OP_CUR_MASK) * 50U);

        if ((req_mv < PD_PPS_MIN_MV) ||
            (req_mv > PD_PPS_MAX_MV) ||
            (op_ma_pps == 0U) ||
            (op_ma_pps > max_ma_pps)) {
            return 0U;
        }

        *mv = req_mv;
        *ma = op_ma_pps;
        *pdo_index = obj_pos;
        s_last_request_is_pps = 1U;
        s_last_spr_rdo = rdo;
        s_active_supply_type = PD_SUPPLY_TYPE_SPR_PPS;
        return 1U;
    }

    if (obj_pos > s_pdo_count) {
        return 0U;
    }

    pdo = &s_pdos[obj_pos - 1U];
    pdo_ma = pd_effective_pdo_ma(pdo);
    op_ma = pd_fixed_request_current_ma(rdo, pdo_ma);
    max_ma = (uint16_t)(((rdo & PD_RDO_MAX_CUR_MASK) >> PD_RDO_MAX_CUR_SHIFT) * 10U);
    (void)max_ma;

    *mv = pdo->mv;
    *ma = op_ma;
    *pdo_index = obj_pos;
    s_last_spr_rdo = rdo;
    s_active_supply_type = PD_SUPPLY_TYPE_SPR_FIXED;
    return 1U;
}
