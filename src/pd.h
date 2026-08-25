/**
 * @file pd.h
 * @brief Public USB Power Delivery source-controller API and status types.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef __PD_H
#define __PD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PD_SOURCE_MAX_FIXED_PDOS  5U
#define PD_SOURCE_MAX_PDOS        7U
#define PD_EPR_SOURCE_MAX_PDOS    9U
#define PD_SOURCE_DEFAULT_POWER_W 140U

#define PD_FAULT_MP2980_OCP       (1U << 0)
#define PD_FAULT_MP2980_OTP       (1U << 1)
#define PD_FAULT_MP2980_OVP       (1U << 2)
#define PD_FAULT_MP2980_PG        (1U << 3)
#define PD_FAULT_MP2980_INT       (1U << 4)
#define PD_FAULT_CH211_OVP        (1U << 5)
#define PD_FAULT_CH211_OTP        (1U << 6)
#define PD_FAULT_CH211_PG         (1U << 7)
#define PD_FAULT_CH211_INT        (1U << 8)
#define PD_FAULT_LM5069_PG        (1U << 9)
#define PD_FAULT_LMR33630_PG      (1U << 10)
#define PD_FAULT_INA226_C_ALERT   (1U << 11)
#define PD_FAULT_INA226_A_ALERT   (1U << 12)
#define PD_FAULT_CH217K_FLAG      (1U << 13)

typedef enum {
    PD_STATE_DISABLED = 0,
    PD_STATE_DISCONNECTED,
    PD_STATE_ATTACHED,
    PD_STATE_SEND_SOURCE_CAP,
    PD_STATE_WAIT_REQUEST,
    PD_STATE_APPLY_POWER,
    PD_STATE_SEND_ACCEPT,
    PD_STATE_SEND_PS_RDY,
    PD_STATE_SEND_CONTROL,
    PD_STATE_READY,
    PD_STATE_SEND_SOFT_RESET,
    PD_STATE_WAIT_SOFT_RESET_ACCEPT,
    PD_STATE_HARD_RESET,
    PD_STATE_SEND_EPR_KEEPALIVE_ACK
} pd_state_t;

typedef enum {
    PD_SUPPLY_TYPE_NONE = 0U,
    PD_SUPPLY_TYPE_SPR_FIXED,
    PD_SUPPLY_TYPE_EPR_FIXED,
    PD_SUPPLY_TYPE_SPR_PPS,
    PD_SUPPLY_TYPE_SPR_AVS,
    PD_SUPPLY_TYPE_EPR_AVS
} pd_supply_type_t;

typedef enum {
    PD_PARTNER_REVISION_UNKNOWN = 0U,
    PD_PARTNER_REVISION_20,
    PD_PARTNER_REVISION_3X,
    PD_PARTNER_REVISION_30,
    PD_PARTNER_REVISION_31,
    PD_PARTNER_REVISION_32
} pd_partner_revision_t;

/* Persistent trace codes used by the on-device PD diagnostic page. */
typedef enum {
    PD_DEBUG_EVENT_NONE = 0U,
    PD_DEBUG_EVENT_RX_REQUEST,
    PD_DEBUG_EVENT_RX_EPR_REQUEST,
    PD_DEBUG_EVENT_REQUEST_DEFERRED,
    PD_DEBUG_EVENT_ACCEPT_START,
    PD_DEBUG_EVENT_ACCEPT_OK,
    PD_DEBUG_EVENT_ACCEPT_FAIL,
    PD_DEBUG_EVENT_POWER_SET,
    PD_DEBUG_EVENT_POWER_READY,
    PD_DEBUG_EVENT_POWER_ERROR,
    PD_DEBUG_EVENT_PS_RDY_START,
    PD_DEBUG_EVENT_PS_RDY_OK,
    PD_DEBUG_EVENT_PS_RDY_FAIL,
    PD_DEBUG_EVENT_SOFT_RESET,
    PD_DEBUG_EVENT_HARD_RESET,
    PD_DEBUG_EVENT_KEEPALIVE_RX,
    PD_DEBUG_EVENT_KEEPALIVE_ACK_START,
    PD_DEBUG_EVENT_KEEPALIVE_ACK_OK,
    PD_DEBUG_EVENT_KEEPALIVE_ACK_FAIL,
    PD_DEBUG_EVENT_EPR_COMM_TIMEOUT,
    PD_DEBUG_EVENT_PPS_COMM_TIMEOUT,
    PD_DEBUG_EVENT_EPR_MODE_ENTER
} pd_debug_event_t;

typedef struct {
    uint16_t mv;
    uint16_t ma;
} pd_fixed_pdo_t;

typedef struct {
    pd_state_t state;
    uint8_t connected;
    uint8_t cc;
    uint8_t pd_revision;
    uint8_t partner_revision;
    uint8_t selected_pdo;
    uint8_t power_state;
    uint8_t last_msg_type;
    uint8_t control_msg;
    uint8_t last_request_pdo;
    uint8_t last_num_do;
    uint8_t last_extended;
    uint8_t tx_state;
    uint8_t tx_result;
    uint8_t tx_try_count;
    uint8_t rx_msg_len;
    uint8_t last_rdo_pos;
    uint8_t source_cap_retry;
    uint8_t hard_reset_retry;
    uint16_t contract_mv;
    uint16_t contract_ma;
    uint16_t power_target_mv;
    uint16_t last_request_mv;
    uint16_t last_request_ma;
    uint16_t source_current_ma;
    uint16_t cable_max_vbus_mv;
    uint8_t last_request_is_pps;
    uint8_t last_request_is_epr;
    uint8_t pps_active;
    uint8_t epr_mode_active;
    uint8_t active_supply_type;
    uint8_t epr_available;
    uint8_t epr_last_rx_action;
    uint8_t epr_last_tx_action;
    uint8_t epr_last_tx_data;
    uint8_t epr_fail_reason;
    uint16_t pps_min_mv;
    uint16_t pps_max_mv;
    uint16_t pps_max_ma;
    uint16_t fault_bits;
    uint32_t last_rdo;
    uint8_t supports_5a;
    uint8_t fault_alert_pending;
    uint8_t fault_alert_sent;
    uint8_t cable_state;
    uint8_t cable_retry;
    uint8_t cable_pd_revision;
    uint8_t cable_tx_msg_id;
    uint8_t cable_epr_capable;
    uint8_t last_vdm_sop;
    uint8_t last_vdm_cmd;
    uint8_t last_vdm_cmdt;
    uint32_t last_vdm_header;
    uint32_t last_cable_vdo;
    uint8_t debug_last_event;
    uint8_t debug_last_failure;
    uint8_t debug_rx_msg_id;
    uint8_t debug_rx_msg_id_valid;
    uint8_t debug_rx_sop;
    uint8_t debug_deferred_request_valid;
    uint8_t debug_epr_keepalive_pending;
    uint8_t debug_epr_source_cap_pending;
    uint8_t debug_state_tx_active;
    uint8_t debug_epr_chunk_phase;
    uint8_t debug_goodcrc_recover_count;
    uint8_t debug_goodcrc_fail_msg_type;
    uint8_t debug_goodcrc_fail_msg_id;
    uint8_t debug_goodcrc_fail_stage;
    uint8_t debug_rx_hard_reset_count;
    uint8_t debug_rx_hard_reset_state;
    uint8_t debug_rx_hard_reset_power_state;
    uint8_t debug_epr_entry_fail_reason;
    uint8_t debug_epr_entry_fail_state;
    uint16_t debug_event_seq;
    uint16_t debug_request_count;
    uint16_t debug_accept_ok_count;
    uint16_t debug_accept_fail_count;
    uint16_t debug_power_ready_count;
    uint16_t debug_power_error_count;
    uint16_t debug_ps_rdy_ok_count;
    uint16_t debug_ps_rdy_fail_count;
    uint16_t debug_soft_reset_count;
    uint16_t debug_hard_reset_count;
    uint16_t debug_power_output_mv;
    uint16_t debug_power_state_elapsed_ms;
    uint8_t debug_power_request_reason;
    uint8_t debug_power_request_enable;
    uint8_t debug_power_request_count;
} pd_status_t;

/** @brief Callback used by PD policy to apply a VBUS power request. */
typedef uint8_t (*pd_power_control_cb_t)(uint16_t mv, uint16_t ma, uint8_t enable);
/** @brief Callback used by PD policy to measure VBUS in millivolts. */
typedef uint16_t (*pd_vbus_sense_cb_t)(void);

/** @brief Initialize the USB-PD policy and PHY state machines. */
void PD_Init(pd_vbus_sense_cb_t vbus_sense_cb);
/** @brief Run one cooperative USB-PD task slice. */
void PD_Task(uint8_t elapsed_ms);
/** @brief Advance PD timers by the elapsed millisecond count. */
void PD_TickMs(uint8_t elapsed_ms);
/** @brief Advance PD timers by exactly one millisecond. */
void PD_Tick1ms(void);
/** @brief Run one task slice using timer time accumulated by interrupts. */
void PD_TaskFromTick(void);

/** @brief Register the platform VBUS power-control callback. */
void PD_SetPowerControlCallback(pd_power_control_cb_t cb);
/** @brief Register the platform VBUS measurement callback. */
void PD_SetVbusSenseCallback(pd_vbus_sense_cb_t cb);
/** @brief Replace the fixed PDO table advertised by the source. */
uint8_t PD_SetFixedPdos(const pd_fixed_pdo_t *pdos, uint8_t count);
/** @brief Copy the current connection, contract, and diagnostic status. */
void PD_GetStatus(pd_status_t *status);
/** @brief Replace the latched PD fault bit set. */
void PD_UpdateFaults(uint16_t fault_bits);
/** @brief Report additional fault bits to the PD policy. */
void PD_ReportFault(uint16_t fault_bits);

#ifdef __cplusplus
}
#endif

#endif
