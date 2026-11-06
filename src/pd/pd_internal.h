/**
 * @file pd_internal.h
 * @brief Internal state, constants, and cross-module declarations for PD.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#ifndef __PD_INTERNAL_H
#define __PD_INTERNAL_H

#include "pd.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "debug.h"
#include "ch32l103_usbpd.h"
#include "ch211_i2c_dma_port.h"
#include "mp2980_i2c_dma_port.h"

/** @name Frame buffers and retry limits */
/** @{ */
#define PD_RX_BUF_SIZE               34u    /**< Receive buffer: header, 7 objects and CRC. */
/** Number of received frames buffered between the PHY interrupt and the policy layer. */
#define PD_RX_RING_LEN               4u
#define PD_TX_BUF_SIZE               34u    /**< Transmit buffer, same size as the receive one. */
#define PD_GOODCRC_LEN               2u     /**< A GoodCRC carries only its header. */
#define PD_SOURCE_CAP_RETRY_MAX      3u     /**< Capability re-advertisements before a reset. */
#define PD_TX_RETRY_MAX              3u     /**< nRetryCount + 1: one attempt and two retries. */
#define PD_HARD_RESET_RETRY_MAX      3u     /**< Hard Reset repeats before the port is dropped. */
/** @} */

/** @name Protocol timers */
/** @{ */
/** tTypeCSendSourceCap: delay from attach to the first Source Capabilities. */
#define PD_FIRST_CAP_DELAY_MS        120u
/** How long a Request may be awaited before the capabilities are re-advertised. */
#define PD_REQUEST_TIMEOUT_MS        150u
/* CRCReceiveTimer / tReceive, section 7.31.1: how long the sender of a Message waits for the
 * GoodCRC before retrying. The specification range is 900 us to 1100 us and the timer starts
 * when the last bit of the Message EOP has been transmitted. The 140 W Sink on this board
 * answers in 2 ms to 5 ms, so clamping the timer to the specification turned every slow reply
 * into three transmissions of the same MessageID and ended in a Hard Reset. The timer only
 * decides when we retransmit and the partner cannot observe it, so 6 ms is used deliberately. */
#define PD_GOODCRC_TIMEOUT_US        6000u
/** Delay before the Accept transfer starts, so the partner can prepare. */
#define PD_ACCEPT_DELAY_MS           2u
/** Upper bound for every state that transmits a single message. */
#define PD_SEND_STATE_TIMEOUT_MS     250u
/** How long the Soft Reset itself may take to leave the wire. */
#define PD_SOFT_RESET_SEND_TIMEOUT_MS 50u
/** How long the partner has to answer a Soft Reset with Accept. */
#define PD_SOFT_RESET_ACCEPT_TIMEOUT_MS 50u
/** tSenderResponse: how long the partner's Revision reply is awaited. */
#define PD_SENDER_RESPONSE_MAX_MS     50u
/** Pause after the contract is in force before asking for the revision. */
#define PD_REVISION_QUERY_DELAY_MS    20u
/** @} */

/** @name Power sequencing */
/** @{ */
#define PD_POWER_SETTLE_MARGIN_MS    60u    /**< Extra settle time per fixed-PDO step. */
#define PD_POWER_SETTLE_MIN_MS       80u    /**< Floor for the settle time. */
#define PD_APDO_STEP_SETTLE_MARGIN_MS 15u   /**< Extra settle time per small APDO step. */
#define PD_APDO_STEP_SETTLE_MIN_MS   25u    /**< Floor for a small APDO step. */
#define PD_PPS_LARGE_SETTLE_MARGIN_MS 45u   /**< Extra settle time for a large PPS step. */
#define PD_PPS_SMALL_STEP_MAX_MV     500u   /**< PPS step counted as small. */
#define PD_AVS_SMALL_STEP_MAX_MV     1000u  /**< AVS step counted as small. */
#define PD_POWER_DOWNSHIFT_MIN_MV    1000u  /**< Drop above which the bus is discharged. */
#define PD_POWER_VBUS_READY_MARGIN_MV 500u  /**< Tolerance for declaring a target reached. */
#define PD_POWER_VBUS_TARGET_TOLERANCE_MV 500u /**< Same tolerance for the settled check. */
#define PD_POWER_SLEW_MV_PER_MS      75u    /**< Regulator slew used for settle estimates. */
#define PD_POWER_TIMEOUT_MS          500u   /**< Timeout of one power step. */
#define PD_POWER_EXTRA_TIMEOUT_MS    500u   /**< Additional timeout for large steps. */
#define PD_POWER_STATE_TIMEOUT_MS    2000u  /**< Watchdog over the whole sequence. */
/* Discharge window after the output is switched off. Deliberately longer than the data
 * sheet minimum: a short discharge leaves charge on the bus and the following attach sees
 * a slowly falling VBUS, which the partner misreads as an unstable source. This is local
 * power sequencing rather than a timer defined by the PD specification, so the margin can
 * be chosen freely and does not affect protocol operation. */
#define PD_POWER_VBUS_DISCHARGE_MS   2000u
/** Window over which the deepest output sag is reported on the diagnostic page. */
#define PD_VBUS_MIN_WINDOW_MS        2000u
#define PD_FRONTEND_STATE_TIMEOUT_MS 500u   /**< CH211 front-end step timeout. */
/** @} */

/** @name EPR supervision */
/** @{ */
#define PD_TX_PHY_TIMEOUT_MS         5u     /**< Transmit completion timeout. */
#define PD_GOODCRC_STUCK_TIMEOUT_MS  4u     /**< Acknowledgement watchdog. */
/** tSourceEPRKeepAlive: traffic gap above which the port is hard reset. */
#define PD_SOURCE_EPR_KEEPALIVE_TIMEOUT_MS 1000u
/* Silent windows tolerated before the mandated EPR Hard Reset. 0 is strict compliance:
 * section 7.30.5.2 requires a Hard Reset after one tSourceEPRKeepAlive without traffic.
 * Raise it only for bench debugging of a link that loses frames. */
#define PD_EPR_SILENCE_STRIKES_TOLERATED 0u
/** tPPSTimeout: silence timeout of a PPS contract. */
#define PD_SOURCE_PPS_COMM_TIMEOUT_MS 13500u
/** @} */
/** @name Reason recorded for the last output power request */
/** @{ */
#define PD_DEBUG_POWER_REQ_NONE       0u    /**< No request yet. */
#define PD_DEBUG_POWER_REQ_ATTACHED   1u    /**< vSafe5V after attach. */
#define PD_DEBUG_POWER_REQ_REQUEST    2u    /**< Contract requested by the partner. */
#define PD_DEBUG_POWER_REQ_FAULT_OFF  3u    /**< Output removed because of a fault. */
#define PD_DEBUG_POWER_REQ_HARD_RESET 4u    /**< Hard Reset sequence. */
#define PD_DEBUG_POWER_REQ_FAULT_REC  5u    /**< Recovery after the fault cleared. */
/** @} */
/** @name Cable discovery timings */
/** @{ */
/** tVconnSourceOn: settle time after VCONN is applied before identity traffic. */
#define PD_VCONN_SETTLE_MS           50u
#define PD_CABLE_VCONN_TIMEOUT_MS     500u
#define PD_CABLE_DISCOVER_TIMEOUT_MS 50u
#define PD_CABLE_DISCOVER_RETRY_DELAY_MS 10u
#define PD_CABLE_DISCOVER_RETRY_MAX  10u
#define PD_CABLE_REDISCOVER_MS       150u
#define PD_EPR_ENTER_CABLE_WAIT_MS   550u
#define PD_EPR_SOURCE_CAP_RETRY_MAX  3u
#define PD_5A_REFRESH_RETRY_COUNT    1u
#define PD_5A_REFRESH_RETRY_DELAY_MS 80u
/** @} */
/** @name Contract voltage and current constants */
/** @{ */
/** vSafe5V: the default contract every source offers after attach. */
#define PD_VSAFE5V_MV                5000u
#define PD_DEFAULT_CURRENT_MA        3000u
#define PD_5A_CURRENT_MA             5000u
#define PD_VSAFE5V_MA                PD_DEFAULT_CURRENT_MA
#define PD_PPS_MIN_MV                5000u
#define PD_PPS_MAX_MV                21000u
#define PD_PPS_STATUS_DATA_LEN       4u
#define PD_SPR_AVS_MIN_MV            9000u
#define PD_SPR_AVS_SWITCH_MV         15000u
#define PD_SPR_AVS_MAX_MV            20000u
#define PD_EPR_FIXED_MV              28000u
#define PD_EPR_FIXED_MA              PD_5A_CURRENT_MA
#define PD_EPR_AVS_MIN_MV            15000u
#define PD_EPR_AVS_MAX_MV            PD_EPR_FIXED_MV
#define PD_EPR_AVS_PDP_W             PD_SOURCE_DEFAULT_POWER_W
#define PD_EPR_MIN_CABLE_VBUS_MV     50000u
/** @} */
/** @name Fixed PDO capability bits (section 6.4.1.2.3) */
/** @{ */
#define PD_FIXED_PDO_DUAL_ROLE_POWER 0x20000000ul /**< Dual-role power capable. */
#define PD_FIXED_PDO_USB_SUSPEND     0x10000000ul
#define PD_FIXED_PDO_UNCONSTRAINED   0x08000000ul
#define PD_FIXED_PDO_USB_COMM        0x04000000ul
#define PD_FIXED_PDO_DUAL_ROLE_DATA  0x02000000ul
#define PD_FIXED_PDO_UNCHUNKED_EXT   0x01000000ul
#define PD_FIXED_PDO_EPR_MODE        0x00800000ul /**< EPR mode capable. */
/** @} */
/* Capability bits carried by the first (vSafe5V) Source Capability PDO. This port is a
 * wall-powered supply, so Unconstrained Power is declared and the Sink may plan for the
 * full negotiated power instead of assuming the Source is limited by its own consumption.
 * USB Suspend and USB Communications stay clear because the port carries no USB data path,
 * and Dual-Role Data stays clear because the port has no data role to swap. */
#define PD_SRC_CAP_FLAGS             (PD_FIXED_PDO_UNCONSTRAINED)
/** @name CC debounce and frame length limits */
/** @{ */
/** Consecutive detections before an attach is accepted. */
#define PD_CC_DEBOUNCE_ATTACH        3u
#define PD_CC_DEBOUNCE_DETACH        5u
#define PD_CONNECTION_CHECK_MS       120u
#define PD_GOODCRC_RX_LEN            6u
#define PD_EXT_CHUNK_DATA_MAX        26u
#define PD_EXT_SOURCE_CAP_DATA_LEN   25u
#define PD_EXT_STATUS_DATA_LEN       6u
/** @} */
/** @name Extended message types and extended header fields (section 6.5) */
/** @{ */
#define PD_EXT_PPS_STATUS            0x0Cu
#define PD_EXT_EXTENDED_CONTROL      0x10u
#define PD_EXT_EPR_SOURCE_CAP        0x11u
#define PD_EXT_CONTROL_DATA_LEN      2u
#define PD_EXT_CTRL_EPR_GET_SRC_CAP  0x01u
#define PD_EXT_CTRL_EPR_GET_SNK_CAP  0x02u
#define PD_EXT_CTRL_EPR_KEEP_ALIVE   0x03u
#define PD_EXT_CTRL_EPR_KEEP_ALIVE_A 0x04u
#define PD_EXT_HEADER_DATA_SIZE_MASK 0x01FFu
#define PD_EXT_HEADER_REQUEST_CHUNK  0x0400u
#define PD_EXT_HEADER_CHUNK_NUM_SHIFT 11u
#define PD_EXT_HEADER_CHUNK_NUM_MASK 0x0Fu
#define PD_EXT_HEADER_CHUNKED        0x8000u
/** @} */
/** @name Alert Data Object bits and Status message bits (section 6.5.3) */
/** @{ */
#define PD_ALERT_ADO_FIXED_BATT      0x80000000ul
#define PD_ALERT_ADO_HOT_SWAP_BATT   0x40000000ul
#define PD_ALERT_ADO_POWER_STATE     0x20000000ul
#define PD_ALERT_ADO_OPERATING_COND  0x10000000ul
#define PD_ALERT_ADO_SOURCE_INPUT    0x08000000ul
#define PD_ALERT_ADO_OVP             0x04000000ul
#define PD_ALERT_ADO_OCP             0x02000000ul
#define PD_ALERT_ADO_OTP             0x01000000ul
#define PD_STATUS_EVENT_OCP          0x01u
#define PD_STATUS_EVENT_OTP          0x02u
#define PD_STATUS_EVENT_OVP          0x04u
#define PD_STATUS_EVENT_SOURCE_INPUT 0x08u
#define PD_STATUS_EVENT_OPERATING    0x10u
#define PD_STATUS_EVENT_POWER        0x20u
#define PD_STATUS_INPUT_EXTERNAL     0x01u
#define PD_STATUS_INPUT_DC           0x02u
#define PD_STATUS_TEMP_NORMAL        0x00u
#define PD_STATUS_TEMP_OVER          0x02u
#define PD_SOURCE_EXT_INPUT_EXTERNAL 0x01u
#define PD_SOURCE_EXT_INPUT_DC       0x02u
/** @} */
/** @name Source identity constants */
/** @{ */
#define PD_PRODUCT_BCD_DEVICE        0x0100u
#define PD_REVISION_DATA_OBJECT      0x32120000ul
#define PD_RMDO_REV_MAJOR_MASK       0xF0000000ul
#define PD_RMDO_REV_MAJOR_SHIFT      28u
#define PD_RMDO_REV_MINOR_MASK       0x0F000000ul
#define PD_RMDO_REV_MINOR_SHIFT      24u
/** @} */

#ifndef PD_SOURCE_EXT_VID
#define PD_SOURCE_EXT_VID            0x0000u
#endif
#ifndef PD_SOURCE_EXT_PID
#define PD_SOURCE_EXT_PID            0x0000u
#endif
#ifndef PD_SOURCE_EXT_XID
#define PD_SOURCE_EXT_XID            0x00000000ul
#endif
#ifndef PD_SOURCE_EXT_FW_VERSION
#define PD_SOURCE_EXT_FW_VERSION     0x01u
#endif
#ifndef PD_SOURCE_EXT_HW_VERSION
#define PD_SOURCE_EXT_HW_VERSION     0x01u
#endif

/** @name Request Data Object fields (section 6.4.2) */
/** @{ */
#define PD_RDO_OBJECT_POS_MASK       0xF0000000ul
#define PD_RDO_OBJECT_POS_SHIFT      28u
#define PD_RDO_OPERATING_CUR_MASK    0x000003FFul
#define PD_RDO_MAX_CUR_MASK          0x000FFC00ul
#define PD_RDO_MAX_CUR_SHIFT         10u
#define PD_RDO_EPR_CAPABLE           0x00400000ul
/** @} */

/** @name EPR Mode actions, data and failure codes (section 6.5.11) */
/** @{ */
#define PD_EPR_MODE_FIELD_MASK       0xFFul
#define PD_EPR_MODE_ACTION_SHIFT     24u
#define PD_EPR_MODE_DATA_SHIFT       16u
#define PD_EPR_MODE_ACTION_ENTER     0x01u
#define PD_EPR_MODE_ACTION_ENTER_ACK 0x02u
#define PD_EPR_MODE_ACTION_ENTER_OK  0x03u
#define PD_EPR_MODE_ACTION_FAILED    0x04u
#define PD_EPR_MODE_ACTION_EXIT      0x05u
#define PD_EPR_MODE_FAIL_UNKNOWN     0x00u
#define PD_EPR_MODE_FAIL_CABLE       0x01u
#define PD_EPR_MODE_FAIL_VCONN       0x02u
#define PD_EPR_MODE_FAIL_RDO         0x03u
#define PD_EPR_MODE_FAIL_UNABLE      0x04u
#define PD_EPR_MODE_FAIL_PDO         0x05u
/** @} */

/** @name Internal reasons an EPR entry attempt stopped (diagnostic only) */
/** @{ */
#define PD_EPR_FAIL_NONE             0u
#define PD_EPR_FAIL_BUSY             1u
#define PD_EPR_FAIL_FAULT            2u
#define PD_EPR_FAIL_STATE            3u
#define PD_EPR_FAIL_NO_CONTRACT      4u
#define PD_EPR_FAIL_UNAVAILABLE      5u
#define PD_EPR_FAIL_UNKNOWN_ACTION   6u
#define PD_EPR_FAIL_TX_FAIL          7u
#define PD_EPR_FAIL_BAD_REQUEST      8u
#define PD_EPR_FAIL_REQ_FAULT        9u
#define PD_EPR_FAIL_REQ_LEN          10u
#define PD_EPR_FAIL_REQ_MODE         11u
#define PD_EPR_FAIL_REQ_NUM_DO       12u
#define PD_EPR_FAIL_REQ_OBJ          13u
#define PD_EPR_FAIL_REQ_PDO_COPY     14u
#define PD_EPR_FAIL_REQ_CURRENT      15u
#define PD_EPR_FAIL_REQ_VOLTAGE      16u
#define PD_EPR_FAIL_SPR_FALLBACK     17u
/** @} */

/** @name Power Data Object fields (section 6.4.1) */
/** @{ */
#define PD_PDO_TYPE_MASK             0xC0000000ul
#define PD_PDO_TYPE_FIXED            0x00000000ul
#define PD_PDO_TYPE_AUGMENTED        0xC0000000ul
#define PD_FIXED_PDO_VOLTAGE_MASK    0x000FFC00ul
#define PD_FIXED_PDO_CURRENT_MASK    0x000003FFul
#define PD_APDO_TYPE_MASK            0x30000000ul
#define PD_APDO_TYPE_SPR_PPS         0x00000000ul
#define PD_APDO_TYPE_EPR_AVS         0x10000000ul
#define PD_APDO_TYPE_SPR_AVS         0x20000000ul
#define PD_AVS_APDO_PEAK_CUR_MASK    0x0C000000ul
#define PD_AVS_APDO_MAX_VOLT_MASK    0x03FE0000ul
#define PD_AVS_APDO_MIN_VOLT_MASK    0x0000FF00ul
#define PD_AVS_APDO_PDP_MASK         0x000000FFul
#define PD_SPR_AVS_APDO_15V_CUR_MASK 0x000FFC00ul
#define PD_SPR_AVS_APDO_20V_CUR_MASK 0x000003FFul
#define PD_SPR_AVS_APDO_15V_CUR_SHIFT 10u
#define PD_PPS_RDO_OP_CUR_MASK       0x0000007Ful
#define PD_PPS_RDO_VOLTAGE_MASK      0x000FFE00ul
#define PD_PPS_RDO_VOLTAGE_SHIFT     9u
#define PD_AVS_RDO_OP_CUR_MASK       0x0000007Ful
#define PD_AVS_RDO_VOLTAGE_MASK      0x001FFE00ul
#define PD_AVS_RDO_VOLTAGE_SHIFT     9u
#define PD_AVS_RDO_VOLTAGE_STEP_MV   25u
#define PD_AVS_RDO_100MV_ALIGN_MASK  0x03ul
/** @} */

/** @name Structured VDM header fields (section 6.4.4) */
/** @{ */
#define PD_SID_PD                    0xFF00ul
#define PD_VDM_STRUCTURED            0x00008000ul
#define PD_VDM_VERSION_20            0x00002000ul
#define PD_VDM_CMDT_MASK             0x000000C0ul
#define PD_VDM_CMDT_INIT             0x00000000ul
#define PD_VDM_CMDT_ACK              0x00000040ul
#define PD_VDM_CMDT_NAK              0x00000080ul
#define PD_VDM_CMDT_BUSY             0x000000C0ul
#define PD_VDM_CMD_MASK              0x0000001Ful
#define PD_VDM_HEADER_KEEP_MASK      0xFFFF071Ful
/** @} */
/** @name Cable VDO fields (section 6.4.4.3) */
/** @{ */
#define PD_CABLE_VDO_CURRENT_MASK    0x00000060ul
#define PD_CABLE_VDO_CURRENT_3A      0x00000020ul
#define PD_CABLE_VDO_CURRENT_5A      0x00000040ul
#define PD_CABLE_VDO_MAX_VBUS_MASK   0x00000600ul
#define PD_CABLE_VDO_MAX_VBUS_SHIFT  9u
#define PD_CABLE_VDO_EPR_CAPABLE     0x00020000ul
#define PD_CABLE_MAX_VBUS_20V        0u
#define PD_CABLE_MAX_VBUS_30V        1u
#define PD_CABLE_MAX_VBUS_40V        2u
#define PD_CABLE_MAX_VBUS_50V        3u
#define PD_IDH_PRODUCT_TYPE_MASK     0x38000000ul
#define PD_IDH_PRODUCT_TYPE_SHIFT    27u
#define PD_PRODUCT_TYPE_PASSIVE_CABLE 3u
#define PD_PRODUCT_TYPE_ACTIVE_CABLE 4u
#define PD_VDO_INDEX_HEADER          0u
#define PD_VDO_INDEX_ID_HEADER       1u
#define PD_VDO_INDEX_CERT_STAT       2u
#define PD_VDO_INDEX_PRODUCT         3u
#define PD_VDO_INDEX_CABLE_FIRST     4u
/** @} */

/** Transmit progress of the single message context. */
typedef enum {
    PD_TX_IDLE = 0,             /**< No transfer in progress. */
    PD_TX_WAIT_PHY_DONE,        /**< Frame handed to the PHY, completion pending. */
    PD_TX_WAIT_GOODCRC          /**< Frame on the wire, waiting for the acknowledgement. */
} pd_tx_state_t;

/** Outcome of a transfer, consumed by whoever started it. */
typedef enum {
    PD_TX_RESULT_NONE = 0,      /**< No result waiting. */
    PD_TX_RESULT_OK,            /**< Acknowledged by the partner. */
    PD_TX_RESULT_FAIL           /**< Retries exhausted without an acknowledgement. */
} pd_tx_result_t;

/** Power sequencer states. Each step programs one device and waits for the bus to settle. */
typedef enum {
    PD_POWER_OFF = 0,                   /**< Nothing driven. */
    PD_POWER_CLOSE_HVCP,                /**< Close the high-voltage protection switch. */
    PD_POWER_WAIT_HVCP_CLOSED,          /**< Waiting for the switch command to land. */
    PD_POWER_DISABLE_MP2980,            /**< Disable the output regulator. */
    PD_POWER_WAIT_MP2980_DISABLED,      /**< Waiting for the regulator to stop. */
    PD_POWER_ENABLE_MP2980_DISCHARGE,   /**< Bleed the regulator output. */
    PD_POWER_WAIT_MP2980_DISCHARGE_ON,  /**< Waiting for the bleed command. */
    PD_POWER_ENABLE_VBUS_DISCHARGE,     /**< Enable the VBUS discharge path. */
    PD_POWER_WAIT_VBUS_DISCHARGE_ON,    /**< Waiting for the discharge command. */
    PD_POWER_VBUS_DISCHARGE,            /**< Discharging, see PD_POWER_VBUS_DISCHARGE_MS. */
    PD_POWER_DISABLE_VBUS_DISCHARGE,    /**< Release the VBUS discharge path. */
    PD_POWER_WAIT_VBUS_DISCHARGE_OFF,   /**< Waiting for the release command. */
    PD_POWER_DISABLE_MP2980_DISCHARGE,  /**< Release the regulator bleed. */
    PD_POWER_WAIT_MP2980_DISCHARGE_OFF, /**< Waiting for the release command. */
    PD_POWER_READY_OFF,                 /**< Output is off and discharged. */
    PD_POWER_SET_MP2980,                /**< Program voltage and current limit. */
    PD_POWER_WAIT_MP2980,               /**< Waiting for the regulator to settle. */
    PD_POWER_OPEN_HVCP,                 /**< Open the protection switch to pass VBUS. */
    PD_POWER_WAIT_HVCP_OPEN,            /**< Waiting for the switch command to land. */
    PD_POWER_ENABLE_DOWNSHIFT_DISCHARGE,/**< Bleed VBUS before a large downward step. */
    PD_POWER_WAIT_DOWNSHIFT_DISCHARGE_ON,/**< Waiting for the bleed command. */
    PD_POWER_DOWNSHIFT_DISCHARGE,       /**< Bleeding towards the new target. */
    PD_POWER_DISABLE_DOWNSHIFT_DISCHARGE,/**< Release the bleed path. */
    PD_POWER_WAIT_DOWNSHIFT_DISCHARGE_OFF,/**< Waiting for the release command. */
    PD_POWER_READY_ON,                  /**< Output is on target. */
    PD_POWER_ERROR                      /**< A step failed; the policy layer decides next. */
} pd_power_state_t;

/** CH211 CC front-end setup: one command per state, one wait state per command. */
typedef enum {
    PD_FRONTEND_CLOSE_HVCP = 0,         /**< Start with the protection switch closed. */
    PD_FRONTEND_WAIT_CLOSE_HVCP,        /**< Waiting for the switch command. */
    PD_FRONTEND_CC1_RD_OFF,             /**< Remove the CC1 pull-down. */
    PD_FRONTEND_WAIT_CC1_RD_OFF,        /**< Waiting for the command. */
    PD_FRONTEND_CC2_RD_OFF,             /**< Remove the CC2 pull-down. */
    PD_FRONTEND_WAIT_CC2_RD_OFF,        /**< Waiting for the command. */
    PD_FRONTEND_CC1_VCONN_OFF,          /**< Stop sourcing VCONN on CC1. */
    PD_FRONTEND_WAIT_CC1_VCONN_OFF,     /**< Waiting for the command. */
    PD_FRONTEND_CC2_VCONN_OFF,          /**< Stop sourcing VCONN on CC2. */
    PD_FRONTEND_WAIT_CC2_VCONN_OFF,     /**< Waiting for the command. */
    PD_FRONTEND_CC1_PATH_ON,            /**< Connect the CC1 signal path. */
    PD_FRONTEND_WAIT_CC1_PATH_ON,       /**< Waiting for the command. */
    PD_FRONTEND_CC2_PATH_ON,            /**< Connect the CC2 signal path. */
    PD_FRONTEND_WAIT_CC2_PATH_ON,       /**< Waiting for the command. */
    PD_FRONTEND_READY                   /**< Front-end configured, PD traffic may start. */
} pd_frontend_state_t;

/** Cable discovery sequence towards the cable plug on SOP'. */
typedef enum {
    PD_CABLE_IDLE = 0,                  /**< Discovery not started. */
    PD_CABLE_ENABLE_VCONN,              /**< Route VCONN to the cable plug. */
    PD_CABLE_WAIT_VCONN,                /**< Waiting for the VCONN switch and settle time. */
    PD_CABLE_SEND_DISCOVER_ID,          /**< Discover Identity queued for transmission. */
    PD_CABLE_WAIT_DISCOVER_ID,          /**< Waiting for the cable plug's reply. */
    PD_CABLE_DONE,                      /**< The plug answered; its VDOs are published. */
    PD_CABLE_FAILED                     /**< No usable reply; the port continues as SPR. */
} pd_cable_state_t;

/** Policy state machine context. */
typedef struct {
    pd_state_t state;               /**< Current policy state. */
    pd_state_t next_state;          /**< State to enter once the queued transfer ends. */
    uint8_t msg_id;                 /**< Transmit MessageID, three bits, wraps. */
    bool connected;                 /**< Partner attached. */
    uint8_t cc;                     /**< CC line in use: 1 or 2. */
    uint8_t pd_revision;            /**< Revision in force: 0 = PD 2.0, 1 = PD 3.x. */
    uint8_t source_cap_retry;       /**< Capability re-advertisements used. */
    uint8_t hard_reset_retry;       /**< Hard Reset repeats used. */
    uint16_t state_timer_ms;        /**< Time spent in the current state. */
    uint8_t selected_pdo;           /**< PDO index of the active contract. */
    uint16_t contract_mv;           /**< Contract voltage, 0 when there is none. */
    uint16_t contract_ma;           /**< Contract current. */
    uint8_t control_msg;            /**< Control message queued for SEND_CONTROL. */
} pd_context_t;

/** CC comparator measurement and debounce state. */
typedef struct {
    uint8_t phase;                  /**< Phase of the comparator sweep. */
    uint8_t stable_count;           /**< Consecutive detections that agreed. */
    uint8_t cc1_flags;              /**< bCC_* flags measured on CC1. */
    uint8_t cc2_flags;              /**< bCC_* flags measured on CC2. */
    bool ready;                     /**< A complete measurement is available. */
    uint8_t result;                 /**< Attached line: 0 none, 1 CC1, 2 CC2. */
    uint16_t check_ms;              /**< Time since the last periodic check. */
} pd_cc_detect_t;

/** The single message transfer context shared by the protocol layer. */
typedef struct {
    pd_tx_state_t state;            /**< Transmit progress. */
    pd_tx_result_t result;          /**< Outcome waiting to be consumed. */
    uint8_t len;                    /**< Frame length in bytes. */
    uint8_t try_count;              /**< Transmissions already made. */
    uint32_t wait_us;               /**< Microsecond stamp of the phase being timed. */
    bool expect_goodcrc;            /**< An acknowledgement is due for this frame. */
    uint8_t sop;                    /**< SOP of the frame on the wire. */
    uint8_t msg_id;                 /**< MessageID of the frame on the wire. */
} pd_tx_context_t;

/** Power sequencer context: one non-blocking step at a time. */
typedef struct {
    pd_power_state_t state;         /**< Current sequencing step. */
    pd_power_state_t last_state;    /**< Previous step; a change latches the timers. */
    bool target_enable;             /**< The output should be on. */
    uint16_t target_mv;             /**< Requested output voltage. */
    uint16_t target_ma;             /**< Requested current limit. */
    uint16_t output_mv;             /**< Voltage last applied, 0 while the output is off. */
    uint16_t settle_ms;             /**< Settle time computed for the current step. */
    uint16_t wait_ms;               /**< Time spent in the current step. */
    uint16_t state_enter_ms;        /**< wait_ms value when the step was entered. */
    /* Time spent in the current power state.  This is deliberately
     * independent from wait_ms: a new PPS/AVS request resets wait_ms while
     * the hardware transaction is still in the same state. */
    uint16_t state_elapsed_ms;      /**< Independent step timer, see the note above. */
    bool downshift_discharge;       /**< A downward transition discharges first. */
    bool downshift_failed;          /**< The discharge did not reach the target. */
    /* Rolling minimum of the measured output voltage. A 140 W load can be latched off by
     * its own under-voltage protection after a short sag, and the sag is invisible in the
     * instantaneous reading, so the minimum of the last PD_VBUS_MIN_WINDOW_MS is kept for
     * the diagnostic page. 0 means "no sample yet". */
    uint16_t vbus_min_mv;           /**< Deepest measured VBUS of the current window. */
    uint16_t vbus_min_report_mv;    /**< Value published for the last full window. */
    uint16_t vbus_min_window_ms;    /**< Age of the current window. */
} pd_power_context_t;

/** @name Shared protocol context and diagnostic state */
/** @{ */
extern __attribute__((aligned(4))) uint8_t s_pd_rx_buf[PD_RX_BUF_SIZE];
extern __attribute__((aligned(4))) uint8_t s_pd_rx_dma_buf[PD_RX_BUF_SIZE];
extern __attribute__((aligned(4))) uint8_t s_pd_rx_ring_buf[PD_RX_RING_LEN][PD_RX_BUF_SIZE];
extern uint8_t s_rx_ring_len[PD_RX_RING_LEN];
extern uint8_t s_rx_ring_sop[PD_RX_RING_LEN];
extern volatile uint8_t s_rx_ring_head;
extern volatile uint8_t s_rx_ring_tail;
extern __attribute__((aligned(4))) uint8_t s_pd_tx_buf[PD_TX_BUF_SIZE];
extern uint8_t s_pd_goodcrc_buf[PD_GOODCRC_LEN];
extern pd_fixed_pdo_t s_pdos[PD_SOURCE_MAX_FIXED_PDOS];
extern uint8_t s_pdo_count;
extern volatile bool s_rx_msg_pending;
extern volatile uint8_t s_rx_msg_len;
extern volatile uint8_t s_rx_sop;
extern volatile uint16_t s_tick_ms;
extern volatile bool s_goodcrc_tx_active;
extern volatile bool s_goodcrc_tx_pending;
extern volatile uint8_t s_goodcrc_watchdog_ms;
extern volatile uint8_t s_goodcrc_sop;
extern pd_context_t s_pd;
extern pd_cc_detect_t s_cc;
extern pd_tx_context_t s_tx;
extern pd_power_context_t s_power;
extern volatile uint8_t s_debug_power_request_reason;
extern volatile bool s_debug_power_request_enable;
extern volatile uint8_t s_debug_power_request_count;
extern volatile uint8_t s_debug_goodcrc_rx_msg_type;
extern volatile uint8_t s_debug_goodcrc_rx_msg_id;
extern volatile uint8_t s_debug_goodcrc_fail_msg_type;
extern volatile uint8_t s_debug_goodcrc_fail_msg_id;
extern volatile uint8_t s_debug_goodcrc_fail_stage;
extern volatile uint8_t s_debug_rx_hard_reset_count;
extern volatile uint8_t s_debug_rx_hard_reset_state;
extern volatile uint8_t s_debug_rx_hard_reset_power_state;
extern pd_frontend_state_t s_frontend_state;
extern pd_frontend_state_t s_frontend_last_state;
extern pd_cable_state_t s_cable_state;
extern uint16_t s_frontend_wait_ms;
extern uint16_t s_cable_wait_ms;
extern uint8_t s_cable_msg_id;
extern uint8_t s_cable2_msg_id;
extern uint8_t s_cable_pd_revision;
extern uint8_t s_last_msg_type;
extern uint8_t s_last_request_pdo;
extern uint8_t s_last_rdo_pos;
extern uint32_t s_last_rdo;
extern uint32_t s_last_spr_rdo;
extern uint16_t s_last_request_mv;
extern uint16_t s_last_request_ma;
extern bool s_last_request_is_pps;
extern bool s_last_request_is_epr;
extern bool s_pps_active;
extern bool s_epr_mode_active;
extern uint8_t s_active_supply_type;
extern bool s_epr_mode_tx_active;
extern bool s_epr_source_cap_pending;
extern bool s_epr_source_cap_tx_active;
extern uint8_t s_epr_source_cap_retry;
extern bool s_epr_advertise_ready;
extern uint8_t s_epr_last_rx_action;
extern uint8_t s_epr_last_tx_action;
extern uint8_t s_epr_last_tx_data;
extern uint8_t s_epr_fail_reason;
extern bool s_epr_enter_wait_active;
extern uint16_t s_epr_enter_wait_ms;
extern uint8_t s_debug_epr_chunk_phase;
extern uint8_t s_debug_goodcrc_recover_count;
/* Receive-path counters: frames seen by the PHY, and frames that had to be dropped because
 * the single queued slot was already occupied. Comparing them with the number of frames
 * the policy layer actually processed shows whether traffic is being lost between the
 * interrupt and pd_handle_message(). */
extern uint16_t s_debug_rx_frame_count;
extern uint16_t s_debug_rx_drop_count;
extern bool s_supports_5a;
extern bool s_source_cap_refresh_pending;
extern uint8_t s_source_cap_refresh_retries;
extern uint16_t s_source_cap_refresh_wait_ms;
extern uint16_t s_cable_current_ma;
extern uint16_t s_cable_max_vbus_mv;
extern bool s_cable_epr_capable;
extern uint32_t s_last_vdm_header;
extern uint32_t s_last_cable_vdo;
extern uint8_t s_last_vdm_sop;
extern uint8_t s_last_vdm_cmd;
extern uint8_t s_last_vdm_cmdt;
extern uint8_t s_cable_retry;
extern volatile bool s_pd_reset_pending;
extern uint16_t s_fault_bits;
extern bool s_fault_alert_pending;
extern bool s_fault_alert_sent;
extern bool s_fault_alert_tx_active;
extern uint16_t s_fault_alert_tx_bits;
extern bool s_ext_response_tx_active;
extern uint8_t s_partner_revision;
/** @} */

/** @name Internal encoding and state helpers */
/** @{ */
/** @brief Decode a little-endian 32-bit protocol value. */
uint32_t pd_get_u32_le(const uint8_t *buf);
/** @brief Encode a 32-bit protocol value in little-endian order. */
void pd_put_u32_le(uint8_t *buf, uint32_t value);
/** @brief Encode a 16-bit protocol value in little-endian order. */
void pd_put_u16_le(uint8_t *buf, uint16_t value);
/** @brief Return the negotiated VDM version bits for one SOP type. */
uint32_t pd_vdm_version_bits(uint8_t sop);
/** @brief Build a structured VDM header for a policy response. */
uint32_t pd_make_structured_vdm(uint16_t svid,
                                uint8_t cmd,
                                uint32_t cmdt,
                                uint8_t sop);

/** @brief Enter a policy state and reset its timing context. */
void pd_set_state(pd_state_t state);
/** @brief Clear the active contract after detach or reset. */
void pd_clear_contract_info(void);
/** @brief Reset PD negotiation context and optionally disable VBUS. */
void pd_reset_context(bool disable_power);
/** @brief Return non-zero while VBUS is transitioning or recovering. */
bool pd_is_power_transition_state(void);
/** @brief Leave EPR mode and recover through the SPR policy path. */
void pd_epr_exit_to_spr_recovery(void);
/** @} */

/** @name PHY and transmission services */
/** @{ */
/** @brief Program the USB-PD peripheral for the running clock. */
void pd_phy_init(void);
/** @brief Return both CC lines to the idle source configuration. */
void pd_phy_reset_cc(void);
/** @brief Write the two header bytes of one message into the transmit buffer. */
void pd_load_header_sop(uint8_t msg_type, uint8_t num_do, uint8_t ext, uint8_t sop);
/** @brief Return the next transmit message ID for one SOP type. */
uint8_t pd_tx_msg_id_for_sop(uint8_t sop);
/** @brief Select the active CC signal path. */
void pd_select_cc(uint8_t cc);
/** @brief Put the USB-PD PHY in receive mode and enable its IRQ. */
void pd_set_phy_rx_mode(void);
/** @brief Convert a received SOP identifier to its response SOP identifier. */
uint8_t pd_rx_sop_to_tx_sop(uint8_t rx_sop);
/** @brief Stop the GoodCRC response watchdog timer. */
void pd_goodcrc_timer_stop(void);
/** @brief Recover a GoodCRC watchdog that remained active too long. */
void pd_goodcrc_watchdog_task(void);
/** @brief Record the active EPR extended-message chunk phase. */
void pd_debug_note_epr_chunk_phase(uint8_t phase);
/** @brief Reset diagnostics for a new EPR entry attempt. */
void pd_debug_begin_epr_entry(void);
/** @brief Record the reason an EPR entry attempt failed. */
void pd_debug_note_epr_entry_failure(uint8_t reason);
/** @brief Start a non-blocking PD packet transmission for one SOP type. */
bool pd_tx_start_sop(uint8_t msg_type, const uint8_t *payload, uint8_t payload_len, uint8_t sop);
/** @brief Start a non-blocking SOP packet transmission. */
bool pd_tx_start(uint8_t msg_type, const uint8_t *payload, uint8_t payload_len);
/** @brief Start a non-blocking extended PD message transmission. */
bool pd_tx_start_extended(uint8_t msg_type, const uint8_t *payload, uint8_t data_len);
/** @brief Start transmission of one chunk from an extended PD message. */
bool pd_tx_start_extended_chunk(uint8_t msg_type,
                                   const uint8_t *payload,
                                   uint16_t data_len,
                                   uint8_t chunk_number);
/** @brief Start a zero-data control-message transmission. */
bool pd_tx_start_control(uint8_t msg_type);
/** @brief Start transmission of the current source capabilities. */
bool pd_tx_start_source_cap(void);
/** @brief Start a hardware hard-reset transmission. */
void pd_tx_start_hard_reset(void);
/** @brief Advance the transmit retry, timing, and GoodCRC state machine. */
void pd_tx_task(uint8_t elapsed_ms);
/** @brief Consume the result of the most recent transmission. */
bool pd_tx_take_result(pd_tx_result_t *result);
/** @brief Return non-zero when the transmitter accepts a new packet. */
bool pd_tx_is_idle(void);
/** @} */

/** @name Power, policy, cable, and connection services */
/** @{ */
/** @brief Return the current limit discovered from the active cable. */
uint16_t pd_current_limit_ma(void);
/** @brief Return the effective current limit advertised by the PPS APDO. */
uint16_t pd_effective_pps_ma(void);
/** @brief Return non-zero when EPR capability is currently advertised. */
bool pd_epr_mode_capable_is_advertised(void);
/** @brief Request a non-blocking controlled VBUS power transition. */
void pd_power_request(bool enable, uint16_t mv, uint16_t ma);
/** @brief Drop whatever transaction the output regulator still has queued. */
void pd_power_abort(void);
/** @brief Return non-zero when requested VBUS output is stable. */
bool pd_power_is_ready_on(void);
/** @brief Check whether the requested power state matches the active target. */
bool pd_power_request_is(bool enable, uint16_t mv);
/** @brief Return non-zero when the power state machine has failed. */
bool pd_power_has_error(void);
/** @brief Deepest output voltage seen in the last PD_VBUS_MIN_WINDOW_MS, in millivolts. */
uint16_t pd_power_vbus_min_mv(void);
/** @brief Record a PD diagnostic event. */
void pd_debug_note_event(uint8_t event);
/** @brief Record a failed PD transaction. */
void pd_debug_note_failure(uint8_t event);
/** @brief Reset source communication and EPR keep-alive timers. */
void pd_reset_source_comm_timers(void);
/** @brief Return non-zero when one or more PD faults are latched. */
bool pd_fault_active(void);
/** @brief Start an Alert message for the latched fault set. */
bool pd_tx_start_alert(void);
/** @brief Start a Get_Status response. */
bool pd_tx_start_status(void);
/** @brief Start one chunk of a Get_Status extended response. */
bool pd_tx_start_status_chunk(uint8_t chunk_number);
/** @brief Start a PPS status response for the active contract. */
bool pd_tx_start_pps_status(void);
/** @brief Start an EPR mode response. */
bool pd_tx_start_epr_mode(uint8_t action, uint8_t data);
/** @brief Start an extended-control message response. */
bool pd_tx_start_extended_control(uint8_t type, uint8_t data);
/** @brief Start the SPR extended source-capability response. */
bool pd_tx_start_source_cap_ext(void);
/** @brief Start one chunk of the SPR extended source-capability response. */
bool pd_tx_start_source_cap_ext_chunk(uint8_t chunk_number);
/** @brief Start the EPR extended source-capability response. */
bool pd_tx_start_epr_source_cap_ext(void);
/** @brief Start one chunk of the EPR extended source-capability response. */
bool pd_tx_start_epr_source_cap_ext_chunk(uint8_t chunk_number);
/** @brief Start a PD revision response. */
bool pd_tx_start_revision(void);
/** @brief Schedule and complete pending fault Alert transmissions. */
void pd_alert_task(void);
/** @brief Advance extended-message response scheduling. */
void pd_ext_response_task(uint8_t elapsed_ms);
/** @brief Advance EPR source-capability response scheduling. */
void pd_epr_source_cap_task(void);
/** @brief Advance the non-blocking VBUS power state machine. */
void pd_power_task(uint8_t elapsed_ms);
/** @brief Recover the controlled VBUS output to USB-PD safe 5 V. */
void pd_power_recover_to_vsafe5v(void);
/** @brief Read VBUS through the registered platform callback. */
uint16_t pd_sense_vbus_mv(void);
/** @brief Reset cable identity-discovery and VCONN state. */
void pd_cable_reset(void);
/** @brief Start the non-blocking cable Discover Identity exchange. */
void pd_cable_start_discovery(void);
/** @brief Return non-zero when cable discovery reached a terminal state. */
bool pd_cable_discovery_finished(void);
/** @brief Process a received cable SOP identity message. */
void pd_cable_handle_message(uint8_t num_do);
/** @brief Process a received structured VDM message. */
void pd_vdm_handle_message(uint8_t num_do);
/** @brief Start a Source Identity VDM response. */
bool pd_vdm_send_source_identity(uint8_t rx_sop, uint32_t request_header);
/** @brief Advance cable discovery and VCONN sequencing. */
void pd_cable_task(uint8_t elapsed_ms);
/** @brief Return non-zero when the current connection permits EPR. */
bool pd_epr_is_available(void);
/** @brief Return non-zero when local source capabilities support EPR. */
bool pd_epr_source_capable(void);
/** @brief Continue the cable check required before an EPR mode response. */
bool pd_epr_start_mode_response(uint8_t action, uint8_t data);
/** @brief Return non-zero when the current RDO requests EPR mode change. */
bool pd_epr_request_is_mode_request(void);
/** @brief Return non-zero when the current RDO selects an EPR PDO. */
bool pd_epr_request_is_pdo_request(void);
/** @brief Handle a valid received EPR mode request. */
bool pd_handle_epr_mode_request(void);
/** @brief Validate the current RDO against advertised EPR capabilities. */
bool pd_validate_epr_request(uint16_t *mv, uint16_t *ma, uint8_t *pdo_index);
/** @brief Validate the current RDO against advertised source capabilities. */
bool pd_validate_request(uint16_t *mv, uint16_t *ma, uint8_t *pdo_index);

/** @brief Return non-zero when the CH211 CC front-end is initialized. */
bool pd_frontend_is_ready(void);
/** @brief Restart the CH211 CC front-end setup sequence. */
void pd_frontend_restart(void);
/** @brief Advance the non-blocking CH211 CC front-end state machine. */
void pd_frontend_task(uint8_t elapsed_ms);
/** @brief Supply VCONN on the CC line the contract is not using. */
ch211_status_t pd_cc_vconn_enable(void);
/** @brief Non-zero while the VCONN switch is still being programmed. */
bool pd_cc_vconn_busy(void);
/** @brief Poll and debounce CC connection state. */
void pd_connection_task(uint8_t elapsed_ms);
/** @} */

#endif
