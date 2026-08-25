/**
 * @file pd_internal.h
 * @brief Internal state, constants, and cross-module declarations for PD.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
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

#define PD_RX_BUF_SIZE               34U
#define PD_TX_BUF_SIZE               34U
#define PD_GOODCRC_LEN               2U
#define PD_SOURCE_CAP_RETRY_MAX      3U
#define PD_TX_RETRY_MAX              3U
#define PD_HARD_RESET_RETRY_MAX      3U
#define PD_FIRST_CAP_DELAY_MS        120U
#define PD_REQUEST_TIMEOUT_MS        150U
#define PD_GOODCRC_TIMEOUT_MS        6U
#define PD_ACCEPT_DELAY_MS           2U
#define PD_SEND_STATE_TIMEOUT_MS     250U
#define PD_SOFT_RESET_SEND_TIMEOUT_MS 50U
#define PD_SOFT_RESET_ACCEPT_TIMEOUT_MS 50U
#define PD_POWER_SETTLE_MARGIN_MS    60U
#define PD_POWER_SETTLE_MIN_MS       80U
#define PD_APDO_STEP_SETTLE_MARGIN_MS 15U
#define PD_APDO_STEP_SETTLE_MIN_MS   25U
#define PD_PPS_LARGE_SETTLE_MARGIN_MS 45U
#define PD_PPS_SMALL_STEP_MAX_MV     500U
#define PD_AVS_SMALL_STEP_MAX_MV     1000U
#define PD_POWER_DOWNSHIFT_MIN_MV    1000U
#define PD_POWER_VBUS_READY_MARGIN_MV 500U
#define PD_POWER_VBUS_TARGET_TOLERANCE_MV 500U
#define PD_POWER_SLEW_MV_PER_MS      75U
#define PD_POWER_TIMEOUT_MS          500U
#define PD_POWER_EXTRA_TIMEOUT_MS    500U
#define PD_POWER_STATE_TIMEOUT_MS    2000U
#define PD_POWER_VBUS_DISCHARGE_MS   2000U
#define PD_FRONTEND_STATE_TIMEOUT_MS 500U
#define PD_TX_PHY_TIMEOUT_MS         5U
#define PD_GOODCRC_STUCK_TIMEOUT_MS  4U
#define PD_SOURCE_EPR_KEEPALIVE_TIMEOUT_MS 875U
#define PD_SOURCE_PPS_COMM_TIMEOUT_MS 13500U
#define PD_REVISION_QUERY_DELAY_MS    20U
#define PD_SENDER_RESPONSE_MAX_MS     50U
#define PD_DEBUG_POWER_REQ_NONE       0U
#define PD_DEBUG_POWER_REQ_ATTACHED   1U
#define PD_DEBUG_POWER_REQ_REQUEST    2U
#define PD_DEBUG_POWER_REQ_FAULT_OFF  3U
#define PD_DEBUG_POWER_REQ_HARD_RESET 4U
#define PD_DEBUG_POWER_REQ_FAULT_REC  5U
#define PD_VCONN_SETTLE_MS           50U
#define PD_CABLE_VCONN_TIMEOUT_MS     500U
#define PD_CABLE_DISCOVER_TIMEOUT_MS 50U
#define PD_CABLE_DISCOVER_RETRY_DELAY_MS 10U
#define PD_CABLE_DISCOVER_RETRY_MAX  10U
#define PD_CABLE_REDISCOVER_MS       150U
#define PD_EPR_ENTER_CABLE_WAIT_MS   550U
#define PD_EPR_SOURCE_CAP_RETRY_MAX  3U
#define PD_5A_REFRESH_RETRY_COUNT    1U
#define PD_5A_REFRESH_RETRY_DELAY_MS 80U
#define PD_VSAFE5V_MV                5000U
#define PD_DEFAULT_CURRENT_MA        3000U
#define PD_5A_CURRENT_MA             5000U
#define PD_VSAFE5V_MA                PD_DEFAULT_CURRENT_MA
#define PD_PPS_MIN_MV                5000U
#define PD_PPS_MAX_MV                21000U
#define PD_PPS_STATUS_DATA_LEN       4U
#define PD_SPR_AVS_MIN_MV            9000U
#define PD_SPR_AVS_SWITCH_MV         15000U
#define PD_SPR_AVS_MAX_MV            20000U
#define PD_EPR_FIXED_MV              28000U
#define PD_EPR_FIXED_MA              PD_5A_CURRENT_MA
#define PD_EPR_AVS_MIN_MV            15000U
#define PD_EPR_AVS_MAX_MV            PD_EPR_FIXED_MV
#define PD_EPR_AVS_PDP_W             PD_SOURCE_DEFAULT_POWER_W
#define PD_EPR_MIN_CABLE_VBUS_MV     50000U
#define PD_FIXED_PDO_DUAL_ROLE_POWER 0x20000000UL
#define PD_FIXED_PDO_USB_SUSPEND     0x10000000UL
#define PD_FIXED_PDO_UNCONSTRAINED   0x08000000UL
#define PD_FIXED_PDO_USB_COMM        0x04000000UL
#define PD_FIXED_PDO_DUAL_ROLE_DATA  0x02000000UL
#define PD_FIXED_PDO_UNCHUNKED_EXT   0x01000000UL
#define PD_FIXED_PDO_EPR_MODE        0x00800000UL
#define PD_SRC_CAP_FLAGS             0x00000000UL
#define PD_CC_DEBOUNCE_ATTACH        3U
#define PD_CC_DEBOUNCE_DETACH        5U
#define PD_CONNECTION_CHECK_MS       120U
#define PD_GOODCRC_RX_LEN            6U
#define PD_EXT_CHUNK_DATA_MAX        26U
#define PD_EXT_SOURCE_CAP_DATA_LEN   25U
#define PD_EXT_STATUS_DATA_LEN       6U
#define PD_EXT_PPS_STATUS            0x0CU
#define PD_EXT_EXTENDED_CONTROL      0x10U
#define PD_EXT_EPR_SOURCE_CAP        0x11U
#define PD_EXT_CONTROL_DATA_LEN      2U
#define PD_EXT_CTRL_EPR_GET_SRC_CAP  0x01U
#define PD_EXT_CTRL_EPR_GET_SNK_CAP  0x02U
#define PD_EXT_CTRL_EPR_KEEP_ALIVE   0x03U
#define PD_EXT_CTRL_EPR_KEEP_ALIVE_A 0x04U
#define PD_EXT_HEADER_DATA_SIZE_MASK 0x01FFU
#define PD_EXT_HEADER_REQUEST_CHUNK  0x0400U
#define PD_EXT_HEADER_CHUNK_NUM_SHIFT 11U
#define PD_EXT_HEADER_CHUNK_NUM_MASK 0x0FU
#define PD_EXT_HEADER_CHUNKED        0x8000U
#define PD_ALERT_ADO_FIXED_BATT      0x80000000UL
#define PD_ALERT_ADO_HOT_SWAP_BATT   0x40000000UL
#define PD_ALERT_ADO_POWER_STATE     0x20000000UL
#define PD_ALERT_ADO_OPERATING_COND  0x10000000UL
#define PD_ALERT_ADO_SOURCE_INPUT    0x08000000UL
#define PD_ALERT_ADO_OVP             0x04000000UL
#define PD_ALERT_ADO_OCP             0x02000000UL
#define PD_ALERT_ADO_OTP             0x01000000UL
#define PD_STATUS_EVENT_OCP          0x01U
#define PD_STATUS_EVENT_OTP          0x02U
#define PD_STATUS_EVENT_OVP          0x04U
#define PD_STATUS_EVENT_SOURCE_INPUT 0x08U
#define PD_STATUS_EVENT_OPERATING    0x10U
#define PD_STATUS_EVENT_POWER        0x20U
#define PD_STATUS_INPUT_EXTERNAL     0x01U
#define PD_STATUS_INPUT_DC           0x02U
#define PD_STATUS_TEMP_NORMAL        0x00U
#define PD_STATUS_TEMP_OVER          0x02U
#define PD_SOURCE_EXT_INPUT_EXTERNAL 0x01U
#define PD_SOURCE_EXT_INPUT_DC       0x02U
#define PD_PRODUCT_BCD_DEVICE        0x0100U
#define PD_REVISION_DATA_OBJECT      0x32120000UL
#define PD_RMDO_REV_MAJOR_MASK       0xF0000000UL
#define PD_RMDO_REV_MAJOR_SHIFT      28U
#define PD_RMDO_REV_MINOR_MASK       0x0F000000UL
#define PD_RMDO_REV_MINOR_SHIFT      24U

#ifndef PD_SOURCE_EXT_VID
#define PD_SOURCE_EXT_VID            0x0000U
#endif
#ifndef PD_SOURCE_EXT_PID
#define PD_SOURCE_EXT_PID            0x0000U
#endif
#ifndef PD_SOURCE_EXT_XID
#define PD_SOURCE_EXT_XID            0x00000000UL
#endif
#ifndef PD_SOURCE_EXT_FW_VERSION
#define PD_SOURCE_EXT_FW_VERSION     0x01U
#endif
#ifndef PD_SOURCE_EXT_HW_VERSION
#define PD_SOURCE_EXT_HW_VERSION     0x01U
#endif

#define PD_RDO_OBJECT_POS_MASK       0xF0000000UL
#define PD_RDO_OBJECT_POS_SHIFT      28U
#define PD_RDO_OPERATING_CUR_MASK    0x000003FFUL
#define PD_RDO_MAX_CUR_MASK          0x000FFC00UL
#define PD_RDO_MAX_CUR_SHIFT         10U
#define PD_RDO_EPR_CAPABLE           0x00400000UL

#define PD_EPR_MODE_FIELD_MASK       0xFFUL
#define PD_EPR_MODE_ACTION_SHIFT     24U
#define PD_EPR_MODE_DATA_SHIFT       16U
#define PD_EPR_MODE_ACTION_ENTER     0x01U
#define PD_EPR_MODE_ACTION_ENTER_ACK 0x02U
#define PD_EPR_MODE_ACTION_ENTER_OK  0x03U
#define PD_EPR_MODE_ACTION_FAILED    0x04U
#define PD_EPR_MODE_ACTION_EXIT      0x05U
#define PD_EPR_MODE_FAIL_UNKNOWN     0x00U
#define PD_EPR_MODE_FAIL_CABLE       0x01U
#define PD_EPR_MODE_FAIL_VCONN       0x02U
#define PD_EPR_MODE_FAIL_RDO         0x03U
#define PD_EPR_MODE_FAIL_UNABLE      0x04U
#define PD_EPR_MODE_FAIL_PDO         0x05U

#define PD_EPR_FAIL_NONE             0U
#define PD_EPR_FAIL_BUSY             1U
#define PD_EPR_FAIL_FAULT            2U
#define PD_EPR_FAIL_STATE            3U
#define PD_EPR_FAIL_NO_CONTRACT      4U
#define PD_EPR_FAIL_UNAVAILABLE      5U
#define PD_EPR_FAIL_UNKNOWN_ACTION   6U
#define PD_EPR_FAIL_TX_FAIL          7U
#define PD_EPR_FAIL_BAD_REQUEST      8U
#define PD_EPR_FAIL_REQ_FAULT        9U
#define PD_EPR_FAIL_REQ_LEN          10U
#define PD_EPR_FAIL_REQ_MODE         11U
#define PD_EPR_FAIL_REQ_NUM_DO       12U
#define PD_EPR_FAIL_REQ_OBJ          13U
#define PD_EPR_FAIL_REQ_PDO_COPY     14U
#define PD_EPR_FAIL_REQ_CURRENT      15U
#define PD_EPR_FAIL_REQ_VOLTAGE      16U
#define PD_EPR_FAIL_SPR_FALLBACK     17U

#define PD_PDO_TYPE_MASK             0xC0000000UL
#define PD_PDO_TYPE_FIXED            0x00000000UL
#define PD_PDO_TYPE_AUGMENTED        0xC0000000UL
#define PD_FIXED_PDO_VOLTAGE_MASK    0x000FFC00UL
#define PD_FIXED_PDO_CURRENT_MASK    0x000003FFUL
#define PD_APDO_TYPE_MASK            0x30000000UL
#define PD_APDO_TYPE_SPR_PPS         0x00000000UL
#define PD_APDO_TYPE_EPR_AVS         0x10000000UL
#define PD_APDO_TYPE_SPR_AVS         0x20000000UL
#define PD_AVS_APDO_PEAK_CUR_MASK    0x0C000000UL
#define PD_AVS_APDO_MAX_VOLT_MASK    0x03FE0000UL
#define PD_AVS_APDO_MIN_VOLT_MASK    0x0000FF00UL
#define PD_AVS_APDO_PDP_MASK         0x000000FFUL
#define PD_SPR_AVS_APDO_15V_CUR_MASK 0x000FFC00UL
#define PD_SPR_AVS_APDO_20V_CUR_MASK 0x000003FFUL
#define PD_SPR_AVS_APDO_15V_CUR_SHIFT 10U
#define PD_PPS_RDO_OP_CUR_MASK       0x0000007FUL
#define PD_PPS_RDO_VOLTAGE_MASK      0x000FFE00UL
#define PD_PPS_RDO_VOLTAGE_SHIFT     9U
#define PD_AVS_RDO_OP_CUR_MASK       0x0000007FUL
#define PD_AVS_RDO_VOLTAGE_MASK      0x001FFE00UL
#define PD_AVS_RDO_VOLTAGE_SHIFT     9U
#define PD_AVS_RDO_VOLTAGE_STEP_MV   25U
#define PD_AVS_RDO_100MV_ALIGN_MASK  0x03UL

#define PD_SID_PD                    0xFF00UL
#define PD_VDM_STRUCTURED            0x00008000UL
#define PD_VDM_VERSION_20            0x00002000UL
#define PD_VDM_CMDT_MASK             0x000000C0UL
#define PD_VDM_CMDT_INIT             0x00000000UL
#define PD_VDM_CMDT_ACK              0x00000040UL
#define PD_VDM_CMDT_NAK              0x00000080UL
#define PD_VDM_CMDT_BUSY             0x000000C0UL
#define PD_VDM_CMD_MASK              0x0000001FUL
#define PD_VDM_HEADER_KEEP_MASK      0xFFFF071FUL
#define PD_CABLE_VDO_CURRENT_MASK    0x00000060UL
#define PD_CABLE_VDO_CURRENT_3A      0x00000020UL
#define PD_CABLE_VDO_CURRENT_5A      0x00000040UL
#define PD_CABLE_VDO_MAX_VBUS_MASK   0x00000600UL
#define PD_CABLE_VDO_MAX_VBUS_SHIFT  9U
#define PD_CABLE_VDO_EPR_CAPABLE     0x00020000UL
#define PD_CABLE_MAX_VBUS_20V        0U
#define PD_CABLE_MAX_VBUS_30V        1U
#define PD_CABLE_MAX_VBUS_40V        2U
#define PD_CABLE_MAX_VBUS_50V        3U
#define PD_IDH_PRODUCT_TYPE_MASK     0x38000000UL
#define PD_IDH_PRODUCT_TYPE_SHIFT    27U
#define PD_PRODUCT_TYPE_PASSIVE_CABLE 3U
#define PD_PRODUCT_TYPE_ACTIVE_CABLE 4U
#define PD_VDO_INDEX_HEADER          0U
#define PD_VDO_INDEX_ID_HEADER       1U
#define PD_VDO_INDEX_CERT_STAT       2U
#define PD_VDO_INDEX_PRODUCT         3U
#define PD_VDO_INDEX_CABLE_FIRST     4U

typedef enum {
    PD_TX_IDLE = 0,
    PD_TX_WAIT_PHY_DONE,
    PD_TX_WAIT_GOODCRC
} pd_tx_state_t;

typedef enum {
    PD_TX_RESULT_NONE = 0,
    PD_TX_RESULT_OK,
    PD_TX_RESULT_FAIL
} pd_tx_result_t;

typedef enum {
    PD_POWER_OFF = 0,
    PD_POWER_CLOSE_HVCP,
    PD_POWER_WAIT_HVCP_CLOSED,
    PD_POWER_DISABLE_MP2980,
    PD_POWER_WAIT_MP2980_DISABLED,
    PD_POWER_ENABLE_MP2980_DISCHARGE,
    PD_POWER_WAIT_MP2980_DISCHARGE_ON,
    PD_POWER_ENABLE_VBUS_DISCHARGE,
    PD_POWER_WAIT_VBUS_DISCHARGE_ON,
    PD_POWER_VBUS_DISCHARGE,
    PD_POWER_DISABLE_VBUS_DISCHARGE,
    PD_POWER_WAIT_VBUS_DISCHARGE_OFF,
    PD_POWER_DISABLE_MP2980_DISCHARGE,
    PD_POWER_WAIT_MP2980_DISCHARGE_OFF,
    PD_POWER_READY_OFF,
    PD_POWER_SET_MP2980,
    PD_POWER_WAIT_MP2980,
    PD_POWER_OPEN_HVCP,
    PD_POWER_WAIT_HVCP_OPEN,
    PD_POWER_ENABLE_DOWNSHIFT_DISCHARGE,
    PD_POWER_WAIT_DOWNSHIFT_DISCHARGE_ON,
    PD_POWER_DOWNSHIFT_DISCHARGE,
    PD_POWER_DISABLE_DOWNSHIFT_DISCHARGE,
    PD_POWER_WAIT_DOWNSHIFT_DISCHARGE_OFF,
    PD_POWER_READY_ON,
    PD_POWER_ERROR
} pd_power_state_t;

typedef enum {
    PD_FRONTEND_CLOSE_HVCP = 0,
    PD_FRONTEND_WAIT_CLOSE_HVCP,
    PD_FRONTEND_CC1_RD_OFF,
    PD_FRONTEND_WAIT_CC1_RD_OFF,
    PD_FRONTEND_CC2_RD_OFF,
    PD_FRONTEND_WAIT_CC2_RD_OFF,
    PD_FRONTEND_CC1_VCONN_OFF,
    PD_FRONTEND_WAIT_CC1_VCONN_OFF,
    PD_FRONTEND_CC2_VCONN_OFF,
    PD_FRONTEND_WAIT_CC2_VCONN_OFF,
    PD_FRONTEND_CC1_PATH_ON,
    PD_FRONTEND_WAIT_CC1_PATH_ON,
    PD_FRONTEND_CC2_PATH_ON,
    PD_FRONTEND_WAIT_CC2_PATH_ON,
    PD_FRONTEND_READY
} pd_frontend_state_t;

typedef enum {
    PD_CABLE_IDLE = 0,
    PD_CABLE_ENABLE_VCONN,
    PD_CABLE_WAIT_VCONN,
    PD_CABLE_SEND_DISCOVER_ID,
    PD_CABLE_WAIT_DISCOVER_ID,
    PD_CABLE_DONE,
    PD_CABLE_FAILED
} pd_cable_state_t;

typedef struct {
    pd_state_t state;
    pd_state_t next_state;
    uint8_t msg_id;
    uint8_t connected;
    uint8_t cc;
    uint8_t pd_revision;
    uint8_t source_cap_retry;
    uint8_t hard_reset_retry;
    uint16_t state_timer_ms;
    uint8_t selected_pdo;
    uint16_t contract_mv;
    uint16_t contract_ma;
    uint8_t control_msg;
} pd_context_t;

typedef struct {
    uint8_t phase;
    uint8_t stable_count;
    uint8_t cc1_flags;
    uint8_t cc2_flags;
    uint8_t ready;
    uint8_t result;
    uint16_t check_ms;
} pd_cc_detect_t;

typedef struct {
    pd_tx_state_t state;
    pd_tx_result_t result;
    uint8_t len;
    uint8_t try_count;
    uint8_t wait_ms;
    uint8_t expect_goodcrc;
    uint8_t sop;
    uint8_t msg_id;
} pd_tx_context_t;

typedef struct {
    pd_power_state_t state;
    pd_power_state_t last_state;
    uint8_t target_enable;
    uint16_t target_mv;
    uint16_t target_ma;
    uint16_t output_mv;
    uint16_t settle_ms;
    uint16_t wait_ms;
    uint16_t state_enter_ms;
    /* Time spent in the current power state.  This is deliberately
     * independent from wait_ms: a new PPS/AVS request resets wait_ms while
     * the hardware transaction is still in the same state. */
    uint16_t state_elapsed_ms;
    uint8_t downshift_discharge;
    uint8_t downshift_failed;
} pd_power_context_t;

/** @name Shared protocol context and diagnostic state */
/** @{ */
extern __attribute__((aligned(4))) uint8_t s_pd_rx_buf[PD_RX_BUF_SIZE];
extern __attribute__((aligned(4))) uint8_t s_pd_rx_dma_buf[PD_RX_BUF_SIZE];
extern __attribute__((aligned(4))) uint8_t s_pd_rx_queued_buf[PD_RX_BUF_SIZE];
extern __attribute__((aligned(4))) uint8_t s_pd_tx_buf[PD_TX_BUF_SIZE];
extern uint8_t s_pd_goodcrc_buf[PD_GOODCRC_LEN];
extern pd_fixed_pdo_t s_pdos[PD_SOURCE_MAX_FIXED_PDOS];
extern uint8_t s_pdo_count;
extern volatile uint8_t s_rx_msg_pending;
extern volatile uint8_t s_rx_msg_queued;
extern volatile uint8_t s_rx_msg_len;
extern volatile uint8_t s_rx_msg_queued_len;
extern volatile uint8_t s_rx_sop;
extern volatile uint8_t s_rx_msg_queued_sop;
extern volatile uint16_t s_tick_ms;
extern volatile uint8_t s_goodcrc_tx_active;
extern volatile uint8_t s_goodcrc_tx_pending;
extern volatile uint8_t s_goodcrc_watchdog_ms;
extern volatile uint8_t s_goodcrc_sop;
extern pd_context_t s_pd;
extern pd_cc_detect_t s_cc;
extern pd_tx_context_t s_tx;
extern pd_power_context_t s_power;
extern volatile uint8_t s_debug_power_request_reason;
extern volatile uint8_t s_debug_power_request_enable;
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
extern pd_power_control_cb_t s_power_cb;
extern uint16_t s_frontend_wait_ms;
extern uint16_t s_cable_wait_ms;
extern uint16_t s_pd_tx_clk_cnt;
extern uint16_t s_pd_rx_clk_cnt;
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
extern uint8_t s_last_request_is_pps;
extern uint8_t s_last_request_is_epr;
extern uint8_t s_pps_active;
extern uint8_t s_epr_mode_active;
extern uint8_t s_active_supply_type;
extern uint8_t s_epr_mode_tx_active;
extern uint8_t s_epr_source_cap_pending;
extern uint8_t s_epr_source_cap_tx_active;
extern uint8_t s_epr_source_cap_retry;
extern uint8_t s_epr_advertise_ready;
extern uint8_t s_epr_last_rx_action;
extern uint8_t s_epr_last_tx_action;
extern uint8_t s_epr_last_tx_data;
extern uint8_t s_epr_fail_reason;
extern uint8_t s_epr_enter_wait_active;
extern uint16_t s_epr_enter_wait_ms;
extern uint8_t s_debug_epr_chunk_phase;
extern uint8_t s_debug_goodcrc_recover_count;
extern uint8_t s_supports_5a;
extern uint8_t s_source_cap_refresh_pending;
extern uint8_t s_source_cap_refresh_retries;
extern uint16_t s_source_cap_refresh_wait_ms;
extern uint16_t s_cable_current_ma;
extern uint16_t s_cable_max_vbus_mv;
extern uint8_t s_cable_epr_capable;
extern uint32_t s_last_vdm_header;
extern uint32_t s_last_cable_vdo;
extern uint8_t s_last_vdm_sop;
extern uint8_t s_last_vdm_cmd;
extern uint8_t s_last_vdm_cmdt;
extern uint8_t s_cable_retry;
extern volatile uint8_t s_pd_reset_pending;
extern uint16_t s_fault_bits;
extern uint8_t s_fault_alert_pending;
extern uint8_t s_fault_alert_sent;
extern uint8_t s_fault_alert_tx_active;
extern uint16_t s_fault_alert_tx_bits;
extern uint8_t s_ext_response_tx_active;
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
void pd_reset_context(uint8_t disable_power);
/** @brief Return non-zero while VBUS is transitioning or recovering. */
uint8_t pd_is_power_transition_state(void);
/** @brief Leave EPR mode and recover through the SPR policy path. */
void pd_epr_exit_to_spr_recovery(void);
/** @} */

/** @name PHY and transmission services */
/** @{ */
/** @brief Select the active CC signal path. */
void pd_select_cc(uint8_t cc);
/** @brief Apply the selected CC path to the USB-PD PHY. */
void pd_phy_select_cc(uint8_t cc);
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
uint8_t pd_tx_start_sop(uint8_t msg_type, const uint8_t *payload, uint8_t payload_len, uint8_t sop);
/** @brief Start a non-blocking SOP packet transmission. */
uint8_t pd_tx_start(uint8_t msg_type, const uint8_t *payload, uint8_t payload_len);
/** @brief Start a non-blocking extended PD message transmission. */
uint8_t pd_tx_start_extended(uint8_t msg_type, const uint8_t *payload, uint8_t data_len);
/** @brief Start transmission of one chunk from an extended PD message. */
uint8_t pd_tx_start_extended_chunk(uint8_t msg_type,
                                   const uint8_t *payload,
                                   uint16_t data_len,
                                   uint8_t chunk_number);
/** @brief Start a zero-data control-message transmission. */
uint8_t pd_tx_start_control(uint8_t msg_type);
/** @brief Start transmission of the current source capabilities. */
uint8_t pd_tx_start_source_cap(void);
/** @brief Start a hardware hard-reset transmission. */
void pd_tx_start_hard_reset(void);
/** @brief Advance the transmit retry, timing, and GoodCRC state machine. */
void pd_tx_task(uint8_t elapsed_ms);
/** @brief Consume the result of the most recent transmission. */
uint8_t pd_tx_take_result(pd_tx_result_t *result);
/** @brief Return non-zero when the transmitter accepts a new packet. */
uint8_t pd_tx_is_idle(void);
/** @} */

/** @name Power, policy, cable, and connection services */
/** @{ */
/** @brief Return the current limit discovered from the active cable. */
uint16_t pd_current_limit_ma(void);
/** @brief Return the effective current limit advertised by the PPS APDO. */
uint16_t pd_effective_pps_ma(void);
/** @brief Return non-zero when EPR capability is currently advertised. */
uint8_t pd_epr_mode_capable_is_advertised(void);
/** @brief Request a non-blocking controlled VBUS power transition. */
void pd_power_request(uint8_t enable, uint16_t mv, uint16_t ma);
/** @brief Return non-zero when requested VBUS output is stable. */
uint8_t pd_power_is_ready_on(void);
/** @brief Check whether the requested power state matches the active target. */
uint8_t pd_power_request_is(uint8_t enable, uint16_t mv);
/** @brief Return non-zero when the power state machine has failed. */
uint8_t pd_power_has_error(void);
/** @brief Record a PD diagnostic event. */
void pd_debug_note_event(uint8_t event);
/** @brief Record a failed PD transaction. */
void pd_debug_note_failure(uint8_t event);
/** @brief Reset source communication and EPR keep-alive timers. */
void pd_reset_source_comm_timers(void);
/** @brief Return non-zero when one or more PD faults are latched. */
uint8_t pd_fault_active(void);
/** @brief Start an Alert message for the latched fault set. */
uint8_t pd_tx_start_alert(void);
/** @brief Start a Get_Status response. */
uint8_t pd_tx_start_status(void);
/** @brief Start one chunk of a Get_Status extended response. */
uint8_t pd_tx_start_status_chunk(uint8_t chunk_number);
/** @brief Start a PPS status response for the active contract. */
uint8_t pd_tx_start_pps_status(void);
/** @brief Start an EPR mode response. */
uint8_t pd_tx_start_epr_mode(uint8_t action, uint8_t data);
/** @brief Start an extended-control message response. */
uint8_t pd_tx_start_extended_control(uint8_t type, uint8_t data);
/** @brief Start the SPR extended source-capability response. */
uint8_t pd_tx_start_source_cap_ext(void);
/** @brief Start one chunk of the SPR extended source-capability response. */
uint8_t pd_tx_start_source_cap_ext_chunk(uint8_t chunk_number);
/** @brief Start the EPR extended source-capability response. */
uint8_t pd_tx_start_epr_source_cap_ext(void);
/** @brief Start one chunk of the EPR extended source-capability response. */
uint8_t pd_tx_start_epr_source_cap_ext_chunk(uint8_t chunk_number);
/** @brief Start a PD revision response. */
uint8_t pd_tx_start_revision(void);
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
uint8_t pd_cable_discovery_finished(void);
/** @brief Process a received cable SOP identity message. */
void pd_cable_handle_message(uint8_t num_do);
/** @brief Process a received structured VDM message. */
void pd_vdm_handle_message(uint8_t num_do);
/** @brief Start a Source Identity VDM response. */
uint8_t pd_vdm_send_source_identity(uint8_t rx_sop, uint32_t request_header);
/** @brief Advance cable discovery and VCONN sequencing. */
void pd_cable_task(uint8_t elapsed_ms);
/** @brief Return non-zero when the current connection permits EPR. */
uint8_t pd_epr_is_available(void);
/** @brief Return non-zero when local source capabilities support EPR. */
uint8_t pd_epr_source_capable(void);
/** @brief Continue the cable check required before an EPR mode response. */
uint8_t pd_epr_start_mode_response(uint8_t action, uint8_t data);
/** @brief Return non-zero when the current RDO requests EPR mode change. */
uint8_t pd_epr_request_is_mode_request(void);
/** @brief Return non-zero when the current RDO selects an EPR PDO. */
uint8_t pd_epr_request_is_pdo_request(void);
/** @brief Handle a valid received EPR mode request. */
uint8_t pd_handle_epr_mode_request(void);
/** @brief Validate the current RDO against advertised EPR capabilities. */
uint8_t pd_validate_epr_request(uint16_t *mv, uint16_t *ma, uint8_t *pdo_index);
/** @brief Validate the current RDO against advertised source capabilities. */
uint8_t pd_validate_request(uint16_t *mv, uint16_t *ma, uint8_t *pdo_index);

/** @brief Return non-zero when the CH211 CC front-end is initialized. */
uint8_t pd_frontend_is_ready(void);
/** @brief Restart the CH211 CC front-end setup sequence. */
void pd_frontend_restart(void);
/** @brief Advance the non-blocking CH211 CC front-end state machine. */
void pd_frontend_task(uint8_t elapsed_ms);
/** @brief Poll and debounce CC connection state. */
void pd_connection_task(uint8_t elapsed_ms);
/** @} */

#endif
