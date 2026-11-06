/**
 * @file pd.h
 * @brief Public USB Power Delivery source-controller API and status types.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#ifndef __PD_H
#define __PD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PD_SOURCE_MAX_FIXED_PDOS  5u
#define PD_SOURCE_MAX_PDOS        7u
#define PD_EPR_SOURCE_MAX_PDOS    9u
#define PD_SOURCE_DEFAULT_POWER_W 140u

/** @name Latched fault bits, one per monitored source */
/** @{ */
#define PD_FAULT_MP2980_OCP       (1u << 0)   /**< Output regulator over-current. */
#define PD_FAULT_MP2980_OTP       (1u << 1)
#define PD_FAULT_MP2980_OVP       (1u << 2)
#define PD_FAULT_MP2980_PG        (1u << 3)
#define PD_FAULT_MP2980_INT       (1u << 4)
#define PD_FAULT_CH211_OVP        (1u << 5)
#define PD_FAULT_CH211_OTP        (1u << 6)
#define PD_FAULT_CH211_PG         (1u << 7)
#define PD_FAULT_CH211_INT        (1u << 8)
#define PD_FAULT_LM5069_PG        (1u << 9)
#define PD_FAULT_LMR33630_PG      (1u << 10)
#define PD_FAULT_INA226_C_ALERT   (1u << 11)
#define PD_FAULT_INA226_A_ALERT   (1u << 12)
#define PD_FAULT_CH217K_FLAG      (1u << 13)
/** @} */

/** Policy state of the source port, in the order reported on the diagnostic page. */
typedef enum {
    PD_STATE_DISABLED = 0,          /**< Port off; nothing is driven. */
    PD_STATE_DISCONNECTED,          /**< No partner attached. */
    PD_STATE_ATTACHED,              /**< Partner attached, vSafe5V being established. */
    PD_STATE_SEND_SOURCE_CAP,       /**< Advertising Source Capabilities. */
    PD_STATE_WAIT_REQUEST,          /**< Capabilities sent, waiting for a Request. */
    PD_STATE_APPLY_POWER,           /**< Accept acknowledged, moving VBUS to the contract. */
    PD_STATE_SEND_ACCEPT,           /**< Request accepted, Accept being transmitted. */
    PD_STATE_SEND_PS_RDY,           /**< VBUS in range, declaring the supply ready. */
    PD_STATE_SEND_CONTROL,          /**< Transmitting one queued control message. */
    PD_STATE_READY,                 /**< Explicit contract in force. */
    PD_STATE_SEND_SOFT_RESET,       /**< Protocol error recovery: Soft Reset out. */
    PD_STATE_WAIT_SOFT_RESET_ACCEPT,/**< Soft Reset sent, waiting for Accept. */
    PD_STATE_HARD_RESET,            /**< Reset signalling and restart. */
    PD_STATE_SEND_EPR_KEEPALIVE_ACK /**< Answering an EPR KeepAlive out of sequence. */
} pd_state_t;

/** Supply type of the contract in force. */
typedef enum {
    PD_SUPPLY_TYPE_NONE = 0u,       /**< No contract. */
    PD_SUPPLY_TYPE_SPR_FIXED,       /**< Fixed PDO in SPR mode. */
    PD_SUPPLY_TYPE_EPR_FIXED,       /**< Fixed PDO in EPR mode. */
    PD_SUPPLY_TYPE_SPR_PPS,         /**< Programmable power supply APDO. */
    PD_SUPPLY_TYPE_SPR_AVS,         /**< Adjustable voltage supply in SPR mode. */
    PD_SUPPLY_TYPE_EPR_AVS          /**< Adjustable voltage supply in EPR mode. */
} pd_supply_type_t;

/** USB-PD revision reported by, or assumed for, the port partner. */
typedef enum {
    PD_PARTNER_REVISION_UNKNOWN = 0u,   /**< Not queried yet. */
    PD_PARTNER_REVISION_20,             /**< PD 2.0 contract. */
    PD_PARTNER_REVISION_3X,             /**< PD 3.x, exact minor revision unknown. */
    PD_PARTNER_REVISION_30,             /**< PD 3.0. */
    PD_PARTNER_REVISION_31,             /**< PD 3.1. */
    PD_PARTNER_REVISION_32              /**< PD 3.2. */
} pd_partner_revision_t;

/* Persistent trace codes used by the on-device PD diagnostic page. */
typedef enum {
    PD_DEBUG_EVENT_NONE = 0u,           /**< No event recorded. */
    PD_DEBUG_EVENT_RX_REQUEST,          /**< Request received. */
    PD_DEBUG_EVENT_RX_EPR_REQUEST,      /**< EPR Request received. */
    PD_DEBUG_EVENT_REQUEST_DEFERRED,    /**< Request deferred until the AMS finished. */
    PD_DEBUG_EVENT_ACCEPT_START,        /**< Accept transmission started. */
    PD_DEBUG_EVENT_ACCEPT_OK,           /**< Accept acknowledged. */
    PD_DEBUG_EVENT_ACCEPT_FAIL,         /**< Accept not acknowledged. */
    PD_DEBUG_EVENT_POWER_SET,           /**< Output voltage requested. */
    PD_DEBUG_EVENT_POWER_READY,         /**< Output reached the requested voltage. */
    PD_DEBUG_EVENT_POWER_ERROR,         /**< Output sequencing failed. */
    PD_DEBUG_EVENT_PS_RDY_START,        /**< PS_RDY transmission started. */
    PD_DEBUG_EVENT_PS_RDY_OK,           /**< PS_RDY acknowledged. */
    PD_DEBUG_EVENT_PS_RDY_FAIL,         /**< PS_RDY not acknowledged. */
    PD_DEBUG_EVENT_SOFT_RESET,          /**< Soft Reset issued. */
    PD_DEBUG_EVENT_HARD_RESET,          /**< Hard Reset issued. */
    PD_DEBUG_EVENT_KEEPALIVE_RX,        /**< EPR KeepAlive received. */
    PD_DEBUG_EVENT_KEEPALIVE_ACK_START, /**< KeepAlive Ack transmission started. */
    PD_DEBUG_EVENT_KEEPALIVE_ACK_OK,    /**< KeepAlive Ack acknowledged. */
    PD_DEBUG_EVENT_KEEPALIVE_ACK_FAIL,  /**< KeepAlive Ack not acknowledged. */
    PD_DEBUG_EVENT_EPR_COMM_TIMEOUT,    /**< No EPR traffic within tSourceEPRKeepAlive. */
    PD_DEBUG_EVENT_PPS_COMM_TIMEOUT,    /**< No PPS traffic within tPPSTimeout. */
    PD_DEBUG_EVENT_EPR_MODE_ENTER        /**< EPR Mode entered. */
} pd_debug_event_t;

/** One fixed Source Capability the board can advertise. */
typedef struct {
    uint16_t mv;                    /**< Contract voltage. */
    uint16_t ma;                    /**< Current the contract guarantees. */
} pd_fixed_pdo_t;

/**
 * @brief Snapshot of the port for the display and fault services.
 *
 * Filled by PD_GetStatus(). Fields are grouped as: link and contract state, transmit and
 * receive bookkeeping, EPR/PPS details, fault reporting, then the diagnostic counters the
 * on-device debug page reads.
 */
typedef struct {
    pd_state_t state;                   /**< Policy state. */
    bool connected;                     /**< Partner attached. */
    uint8_t cc;                         /**< CC line carrying the contract: 1 or 2. */
    uint8_t pd_revision;                /**< Revision in force: 0 = PD 2.0, 1 = PD 3.x. */
    uint8_t partner_revision;           /**< Partner revision, see pd_partner_revision_t. */
    uint8_t selected_pdo;               /**< PDO index of the active contract. */
    uint8_t power_state;                /**< Power sequencer state, see pd_power_state_t. */
    uint8_t last_msg_type;              /**< Type of the last received message. */
    uint8_t control_msg;                /**< Control message being transmitted, if any. */
    uint8_t last_request_pdo;           /**< PDO index the partner asked for. */
    uint8_t last_num_do;                /**< Data objects in the last received message. */
    bool last_extended;                 /**< Last received message was extended. */
    uint8_t tx_state;                   /**< Transmit state, see pd_tx_state_t. */
    uint8_t tx_result;                  /**< Result of the last transfer, see pd_tx_result_t. */
    uint8_t tx_try_count;               /**< Transmissions used by the current message. */
    uint8_t rx_msg_len;                 /**< Length of the message being processed. */
    uint8_t last_rdo_pos;               /**< Object position in the last RDO. */
    uint8_t source_cap_retry;           /**< Capability re-advertisements used. */
    uint8_t hard_reset_retry;           /**< Hard Reset repeats used. */
    uint16_t contract_mv;               /**< Contract voltage, 0 when there is no contract. */
    uint16_t contract_ma;               /**< Contract current. */
    uint16_t power_target_mv;           /**< Voltage the sequencer is moving to. */
    uint16_t last_request_mv;           /**< Voltage of the partner's last Request. */
    uint16_t last_request_ma;           /**< Current of the partner's last Request. */
    uint16_t source_current_ma;         /**< Current the cable and PDOs allow. */
    uint16_t cable_max_vbus_mv;         /**< Highest voltage the cable is rated for. */
    bool last_request_is_pps;           /**< The last Request selected a PPS APDO. */
    bool last_request_is_epr;           /**< The last Request was an EPR Request. */
    bool pps_active;                    /**< A PPS contract is in force. */
    bool epr_mode_active;               /**< EPR mode is in force. */
    uint8_t active_supply_type;         /**< Supply type of the contract in force. */
    bool epr_available;                 /**< EPR is possible with this partner and cable. */
    uint8_t epr_last_rx_action;         /**< Action of the last EPR Mode message received. */
    uint8_t epr_last_tx_action;         /**< Action of the last EPR Mode message sent. */
    uint8_t epr_last_tx_data;           /**< Data field of the last EPR Mode message sent. */
    uint8_t epr_fail_reason;            /**< Why the last EPR entry attempt stopped. */
    uint16_t pps_min_mv;                /**< Lower edge of the advertised PPS range. */
    uint16_t pps_max_mv;                /**< Upper edge of the advertised PPS range. */
    uint16_t pps_max_ma;                /**< Current limit of the advertised PPS range. */
    uint16_t fault_bits;                /**< Latched PD_FAULT_* bits. */
    uint32_t last_rdo;                  /**< Last Request Data Object received. */
    bool supports_5a;                   /**< The cable is rated for 5 A. */
    bool fault_alert_pending;           /**< An Alert message is owed to the partner. */
    bool fault_alert_sent;              /**< The current Alert has been acknowledged. */
    uint8_t cable_state;                /**< Cable discovery state, see pd_cable_state_t. */
    uint8_t cable_retry;                /**< Discover Identity attempts used. */
    uint8_t cable_pd_revision;          /**< Revision reported by the cable plug. */
    uint8_t cable_tx_msg_id;            /**< MessageID used towards the cable plug. */
    bool cable_epr_capable;             /**< The cable plug reported EPR capability. */
    uint8_t last_vdm_sop;               /**< SOP of the last VDM handled. */
    uint8_t last_vdm_cmd;               /**< Command of the last VDM handled. */
    uint8_t last_vdm_cmdt;              /**< Command type of the last VDM handled. */
    uint32_t last_vdm_header;           /**< Header of the last VDM handled. */
    uint32_t last_cable_vdo;            /**< Identity VDO reported by the cable plug. */
    uint8_t debug_last_event;           /**< Last pd_debug_event_t recorded. */
    uint8_t debug_last_failure;         /**< Last failure code recorded. */
    uint8_t debug_rx_msg_id;            /**< MessageID of the last received message. */
    bool debug_rx_msg_id_valid;         /**< debug_rx_msg_id has been captured. */
    uint8_t debug_rx_sop;               /**< SOP of the last received message. */
    bool debug_deferred_request_valid;  /**< A Request is being replayed later. */
    bool debug_epr_keepalive_pending;   /**< An EPR KeepAlive is waiting to be answered. */
    bool debug_epr_source_cap_pending;  /**< EPR Source Capabilities are queued. */
    bool debug_state_tx_active;         /**< The policy state owns a transfer. */
    uint8_t debug_epr_chunk_phase;      /**< Chunk phase of the EPR capability exchange. */
    uint8_t debug_goodcrc_recover_count;/**< Acknowledgement watchdog recoveries. */
    uint8_t debug_goodcrc_fail_msg_type;/**< Message type whose acknowledgement stalled. */
    uint8_t debug_goodcrc_fail_msg_id;  /**< MessageID whose acknowledgement stalled. */
    uint8_t debug_goodcrc_fail_stage;   /**< Stage the acknowledgement stalled in. */
    uint8_t debug_rx_hard_reset_count;  /**< Hard Resets received. */
    uint8_t debug_rx_hard_reset_state;  /**< Policy state when the last one arrived. */
    uint8_t debug_rx_hard_reset_power_state; /**< Power state when the last one arrived. */
    uint8_t debug_epr_entry_fail_reason;/**< Reason the last EPR entry failed. */
    uint8_t debug_epr_entry_fail_state; /**< Policy state when it failed. */
    uint16_t debug_event_seq;           /**< Increments on every recorded event. */
    uint16_t debug_request_count;       /**< Requests received. */
    uint16_t debug_accept_ok_count;     /**< Accept transmitted successfully. */
    uint16_t debug_accept_fail_count;   /**< Accept transmission failures. */
    uint16_t debug_power_ready_count;   /**< Supply reached the requested voltage. */
    uint16_t debug_power_error_count;   /**< Power sequencer errors. */
    uint16_t debug_ps_rdy_ok_count;     /**< PS_RDY transmitted successfully. */
    uint16_t debug_ps_rdy_fail_count;   /**< PS_RDY transmission failures. */
    uint16_t debug_soft_reset_count;    /**< Soft Resets issued. */
    uint16_t debug_hard_reset_count;    /**< Hard Resets issued. */
    uint16_t debug_rx_msg_count;        /**< Frames handed to the policy layer. */
    uint16_t debug_keepalive_rx_count;  /**< EPR KeepAlives received. */
    uint16_t debug_epr_keepalive_age_ms;/**< Time since the last keepalive traffic. */
    uint16_t debug_epr_silence_max_ms;  /**< Longest keepalive silence seen. */
    uint16_t debug_rx_frame_count;      /**< Frames seen by the PHY. */
    uint16_t debug_rx_drop_count;       /**< Frames dropped before reaching the policy layer. */
    uint16_t debug_vbus_min_mv;         /**< Deepest output sag of the last window. */
    uint16_t debug_power_output_mv;     /**< Output voltage the sequencer assumes. */
    uint16_t debug_power_state_elapsed_ms; /**< Time spent in the current power state. */
    uint8_t debug_power_request_reason; /**< Why power was last requested, see PD_DEBUG_POWER_REQ_*. */
    uint8_t debug_power_request_enable; /**< Last power request enabled the output. */
    uint8_t debug_power_request_count;  /**< Power requests issued. */
} pd_status_t;

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

/** @brief Register the platform VBUS measurement callback. */
void PD_SetVbusSenseCallback(pd_vbus_sense_cb_t cb);
/** @brief Replace the fixed PDO table advertised by the source. */
bool PD_SetFixedPdos(const pd_fixed_pdo_t *pdos, uint8_t count);
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
