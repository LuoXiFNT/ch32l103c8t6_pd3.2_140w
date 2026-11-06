/**
 * @file board_link.c
 * @brief UART link protocol and local/remote board data exchange.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-05
 */

#include "board_link.h"

#include "board_config.h"
#include "ch32l103.h"
#include "ina226_service.h"
#include "pd.h"

#include <string.h>

#define BOARD_LINK_MAGIC0          0xA5u
#define BOARD_LINK_MAGIC1          0x5Au
#define BOARD_LINK_VERSION         0x03u
#define BOARD_LINK_PAYLOAD_LEN     19u
#define BOARD_LINK_FRAME_LEN       (4u + BOARD_LINK_PAYLOAD_LEN + 1u)
#define BOARD_LINK_TX_PERIOD_MS    100u

#define BOARD_LINK_PAYLOAD_SEQ     4u
#define BOARD_LINK_PAYLOAD_FAULT0  5u
#define BOARD_LINK_PAYLOAD_C       7u
#define BOARD_LINK_PAYLOAD_A       15u

/* The sub board sends every 100 ms; missing three frames in a row means the link is
 * down and the remote readings must no longer be presented as live data. */
#define BOARD_LINK_REMOTE_TIMEOUT_MS  350u

/* Transmit state, shared between the task (main loop) and the USART interrupt.
 * s_tx_busy is the handshake: the frame buffer is only rewritten while it is 0, and
 * the interrupt only shifts bytes out while it is 1. */
static volatile bool s_tx_busy = false;        /**< Set while the interrupt is sending a frame. */
static volatile uint8_t s_tx_pos = 0u;         /**< Next byte index in s_tx_buf. */
static volatile uint8_t s_tx_len = 0u;         /**< Number of bytes in the current frame. */
static uint8_t s_tx_buf[BOARD_LINK_FRAME_LEN]; /**< Frame being transmitted. */
static volatile board_link_stats_t s_stats;    /**< Diagnostics, may be updated from the ISR. */

#if BOARD_IS_SUB
/* Sub board: the local snapshot is published here and sent on a fixed period. */
static board_link_remote_t s_local;            /**< Latest local channel snapshot. */
static volatile uint16_t s_tx_elapsed_ms = 0u; /**< Milliseconds since the last frame. */
static volatile bool s_tx_due = true;          /**< Set by the tick when a frame is due. */
static uint8_t s_tx_seq = 0u;                  /**< Rolling sequence number. */
#endif

#if BOARD_IS_MAIN
/* Main board: frames arrive in the USART interrupt and are validated there. */
static volatile board_link_remote_t s_remote;  /**< Latest valid remote snapshot. */
static uint8_t s_rx_buf[BOARD_LINK_FRAME_LEN]; /**< Frame being assembled. */
static uint8_t s_rx_index = 0u;                /**< Bytes collected so far. */
static volatile uint16_t s_rx_age_ms;          /**< Time since the last valid frame. */
#endif

/* Fault flags published by fault_service for the channel payloads. */
static volatile uint8_t s_fault_c;
static volatile uint8_t s_fault_a;

/**
 * @brief Calculate the two's-complement checksum for a link frame.
 * @param buf Frame bytes to include in the checksum.
 * @param len Number of bytes to process.
 * @return Checksum byte that makes the complete frame sum to zero.
 */
static uint8_t BoardLink_Checksum(const uint8_t *buf, uint8_t len)
{
    uint8_t sum = 0u;
    uint8_t i;

    for (i = 0u; i < len; i++) {
        sum = (uint8_t)(sum + buf[i]);
    }

    return (uint8_t)(0u - sum);
}

/**
 * @brief Map the local PD status to the board-link protocol identifier.
 * @param status Current PD status, or null for a disconnected channel.
 * @return Board-link protocol identifier.
 */
static uint8_t BoardLink_ProtocolFromStatus(const pd_status_t *status)
{
    if ((status == 0) || (status->connected == 0u)) {
        return BOARD_LINK_PROTOCOL_NONE;
    }

    /* The identifier describes the contract that is in force, not the exact policy state.
     * A PPS/AVS step cycles through Accept, apply-power and PS_RDY while the contract stays
     * valid, and reporting Type-C during those few milliseconds made a fast-stepping test
     * look like the contract had been lost. States that really have no contract (attach,
     * cap exchange, hard reset, detach) keep falling through to Type-C. */
    if ((status->contract_mv != 0u) &&
        (status->state != PD_STATE_DISCONNECTED) &&
        (status->state != PD_STATE_ATTACHED) &&
        (status->state != PD_STATE_SEND_SOURCE_CAP) &&
        (status->state != PD_STATE_HARD_RESET)) {
        switch (status->active_supply_type) {
        case PD_SUPPLY_TYPE_SPR_FIXED:
            return BOARD_LINK_PROTOCOL_SPR_FIXED;
        case PD_SUPPLY_TYPE_EPR_FIXED:
            return BOARD_LINK_PROTOCOL_EPR_FIXED;
        case PD_SUPPLY_TYPE_SPR_PPS:
            return BOARD_LINK_PROTOCOL_SPR_PPS;
        case PD_SUPPLY_TYPE_SPR_AVS:
            return BOARD_LINK_PROTOCOL_SPR_AVS;
        case PD_SUPPLY_TYPE_EPR_AVS:
            return BOARD_LINK_PROTOCOL_EPR_AVS;
        default:
            break;
        }
    }

    return BOARD_LINK_PROTOCOL_TYPEC;
}

void BoardLink_SetFaults(uint8_t fault_c, uint8_t fault_a)
{
    s_fault_c = fault_c;
    s_fault_a = fault_a;
}

void BoardLink_BuildChannels(board_link_channel_t *c, board_link_channel_t *a)
{
    pd_status_t pd_status;
    ina226_service_sample_t channel_c;
    ina226_service_sample_t channel_a;

    PD_GetStatus(&pd_status);
    INA226_Service_GetSamples(&channel_c, &channel_a);

    if (c != 0) {
        c->vbus_raw = channel_c.vbus_raw;
        c->curr_raw = channel_c.curr_raw;
        c->pwr_raw = channel_c.pwr_raw;
        c->fault = s_fault_c;
        c->protocol = BoardLink_ProtocolFromStatus(&pd_status);
    }

    if (a != 0) {
        a->vbus_raw = channel_a.vbus_raw;
        a->curr_raw = channel_a.curr_raw;
        a->pwr_raw = channel_a.pwr_raw;
        a->fault = s_fault_a;
        a->protocol = BOARD_LINK_PROTOCOL_BC12;
    }
}

#if BOARD_IS_SUB
/** @brief Encode a little-endian 16-bit value into a frame buffer. */
static void BoardLink_Put16(uint8_t *buf, uint8_t index, uint16_t value)
{
    buf[index] = (uint8_t)(value & 0xFFu);
    buf[(uint8_t)(index + 1u)] = (uint8_t)((value >> 8) & 0xFFu);
}

/** @brief Encode one channel snapshot into the local-board payload. */
static void BoardLink_EncodeChannel(uint8_t *buf, uint8_t index, const board_link_channel_t *ch)
{
    buf[index] = ch->fault;
    buf[(uint8_t)(index + 1u)] = ch->protocol;
    BoardLink_Put16(buf, (uint8_t)(index + 2u), ch->vbus_raw);
    BoardLink_Put16(buf, (uint8_t)(index + 4u), ch->curr_raw);
    BoardLink_Put16(buf, (uint8_t)(index + 6u), ch->pwr_raw);
}

/** @brief Assemble the next complete sub-board telemetry frame. */
static void BoardLink_BuildFrame(void)
{
    s_tx_buf[0] = BOARD_LINK_MAGIC0;
    s_tx_buf[1] = BOARD_LINK_MAGIC1;
    s_tx_buf[2] = BOARD_LINK_VERSION;
    s_tx_buf[3] = BOARD_LINK_PAYLOAD_LEN;
    s_tx_buf[BOARD_LINK_PAYLOAD_SEQ] = s_tx_seq++;
    BoardLink_Put16(s_tx_buf, BOARD_LINK_PAYLOAD_FAULT0, s_local.fault_bits);
    BoardLink_EncodeChannel(s_tx_buf, BOARD_LINK_PAYLOAD_C, &s_local.c);
    BoardLink_EncodeChannel(s_tx_buf, BOARD_LINK_PAYLOAD_A, &s_local.a);
    s_tx_buf[BOARD_LINK_FRAME_LEN - 1u] = BoardLink_Checksum(s_tx_buf, BOARD_LINK_FRAME_LEN - 1u);
}
/** @brief Start interrupt-driven transmission of the prepared frame. */
static void BoardLink_StartTx(uint8_t len)
{
    NVIC_DisableIRQ(USART1_IRQn);
    if (!s_tx_busy) {
        s_tx_pos = 0u;
        s_tx_len = len;
        s_tx_busy = true;
        s_stats.tx_frame_start++;
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    } else {
        s_stats.tx_busy_drop++;
    }
    NVIC_EnableIRQ(USART1_IRQn);
}
#endif

#if BOARD_IS_MAIN
/** @brief Decode a little-endian 16-bit value from a received frame. */
static uint16_t BoardLink_Get16(const uint8_t *buf, uint8_t index)
{
    return (uint16_t)((uint16_t)buf[index] | ((uint16_t)buf[(uint8_t)(index + 1u)] << 8));
}

/** @brief Decode one received payload channel into a snapshot structure. */
static void BoardLink_DecodeChannel(const uint8_t *buf, uint8_t index, volatile board_link_channel_t *ch)
{
    ch->fault = buf[index];
    ch->protocol = buf[(uint8_t)(index + 1u)];
    ch->vbus_raw = BoardLink_Get16(buf, (uint8_t)(index + 2u));
    ch->curr_raw = BoardLink_Get16(buf, (uint8_t)(index + 4u));
    ch->pwr_raw = BoardLink_Get16(buf, (uint8_t)(index + 6u));
}

/** @brief Validate and publish the currently buffered main-board frame. */
static bool BoardLink_ApplyFrame(void)
{
    if (BoardLink_Checksum(s_rx_buf, BOARD_LINK_FRAME_LEN) != 0u) {
        s_stats.rx_checksum_error++;
        return 0u;
    }
    if ((s_rx_buf[2] != BOARD_LINK_VERSION) || (s_rx_buf[3] != BOARD_LINK_PAYLOAD_LEN)) {
        s_stats.rx_header_error++;
        return 0u;
    }

    s_remote.seq = s_rx_buf[BOARD_LINK_PAYLOAD_SEQ];
    s_remote.fault_bits = BoardLink_Get16(s_rx_buf, BOARD_LINK_PAYLOAD_FAULT0);
    BoardLink_DecodeChannel(s_rx_buf, BOARD_LINK_PAYLOAD_C, &s_remote.c);
    BoardLink_DecodeChannel(s_rx_buf, BOARD_LINK_PAYLOAD_A, &s_remote.a);
    s_remote.valid = true;
    s_rx_age_ms = 0u;
    s_stats.rx_frame_ok++;
    return 1u;
}

/** @brief Search a rejected frame for the next synchronization marker. */
static void BoardLink_ResyncAfterBadFrame(void)
{
    uint8_t i;

    for (i = 1u; i < (BOARD_LINK_FRAME_LEN - 1u); i++) {
        if ((s_rx_buf[i] == BOARD_LINK_MAGIC0) &&
            (s_rx_buf[(uint8_t)(i + 1u)] == BOARD_LINK_MAGIC1)) {
            uint8_t j;
            uint8_t keep = (uint8_t)(BOARD_LINK_FRAME_LEN - i);

            for (j = 0u; j < keep; j++) {
                s_rx_buf[j] = s_rx_buf[(uint8_t)(i + j)];
            }
            s_rx_index = keep;
            s_stats.rx_resync++;
            return;
        }
    }

    s_rx_index = 0u;
}

/** @brief Feed one UART byte to the main-board frame parser. */
static void BoardLink_RxByte(uint8_t data)
{
    if (s_rx_index == 0u) {
        if (data == BOARD_LINK_MAGIC0) {
            s_rx_buf[s_rx_index++] = data;
        }
        return;
    }

    if (s_rx_index == 1u) {
        if (data == BOARD_LINK_MAGIC1) {
            s_rx_buf[s_rx_index++] = data;
        } else {
            s_rx_index = (data == BOARD_LINK_MAGIC0) ? 1u : 0u;
            s_rx_buf[0] = data;
        }
        return;
    }

    s_rx_buf[s_rx_index++] = data;
    if (s_rx_index >= BOARD_LINK_FRAME_LEN) {
        if (BoardLink_ApplyFrame()) {
            s_rx_index = 0u;
        } else {
            BoardLink_ResyncAfterBadFrame();
        }
    }
}
#endif

void BoardLink_TickMs(uint16_t elapsed_ms)
{
#if BOARD_IS_SUB
    s_tx_elapsed_ms = (uint16_t)(s_tx_elapsed_ms + elapsed_ms);
    if (s_tx_elapsed_ms >= BOARD_LINK_TX_PERIOD_MS) {
        s_tx_elapsed_ms = 0u;
        s_tx_due = true;
    }
#else
    /* Age the received frame; it saturates so the age can never wrap back into a
     * "fresh" value after a long silence. */
    if (s_rx_age_ms < BOARD_LINK_REMOTE_TIMEOUT_MS) {
        s_rx_age_ms = (uint16_t)(s_rx_age_ms + elapsed_ms);
    }
#endif
}

void BoardLink_Task(void)
{
#if BOARD_IS_SUB
    if ((!s_tx_due) || s_tx_busy) {
        return;
    }

    s_tx_due = false;
    BoardLink_BuildFrame();
    BoardLink_StartTx(BOARD_LINK_FRAME_LEN);
#endif
}

/**
 * @brief USART1 interrupt: assemble incoming frames or shift the next byte out.
 *
 * Receive side (main board only): a USART error aborts the frame in progress and is
 * counted, otherwise every byte is fed to the parser. Transmit side: while TXEIE is
 * enabled the interrupt drains s_tx_buf and clears s_tx_busy after the last byte,
 * which is the handshake that lets BoardLink_Task() build the next frame.
 */
void BoardLink_USART1_IRQHandler(void)
{
    uint16_t status = USART1->STATR;

#if BOARD_IS_MAIN
    if ((status & (USART_STATR_ORE | USART_STATR_NE | USART_STATR_FE)) != 0u) {
        (void)USART1->DATAR;
        s_stats.rx_usart_error++;
        if ((status & USART_STATR_ORE) != 0u) {
            s_stats.rx_ore++;
        }
        if ((status & USART_STATR_NE) != 0u) {
            s_stats.rx_ne++;
        }
        if ((status & USART_STATR_FE) != 0u) {
            s_stats.rx_fe++;
        }
        s_rx_index = 0u;
    } else if ((status & USART_STATR_RXNE) != 0u) {
        uint8_t data = (uint8_t)(USART1->DATAR & 0xFFu);
        BoardLink_RxByte(data);
    }
#endif

    if (((status & USART_STATR_TXE) != 0u) &&
        ((USART1->CTLR1 & USART_CTLR1_TXEIE) != 0u)) {
        if (s_tx_pos < s_tx_len) {
            USART1->DATAR = s_tx_buf[s_tx_pos++];
        } else {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            s_tx_busy = false;
            s_stats.tx_frame_done++;
        }
    }
}

void BoardLink_SetLocalData(const board_link_channel_t *c,
                            const board_link_channel_t *a,
                            uint16_t fault_bits)
{
#if BOARD_IS_SUB
    if ((c == 0) || (a == 0)) {
        return;
    }

    s_local.c = *c;
    s_local.a = *a;
    s_local.fault_bits = fault_bits;
#else
    (void)c;
    (void)a;
    (void)fault_bits;
#endif
}

void BoardLink_GetRemoteData(board_link_remote_t *remote)
{
    if (remote == 0) {
        return;
    }

#if BOARD_IS_MAIN
    NVIC_DisableIRQ(USART1_IRQn);
    remote->c.vbus_raw = s_remote.c.vbus_raw;
    remote->c.curr_raw = s_remote.c.curr_raw;
    remote->c.pwr_raw = s_remote.c.pwr_raw;
    remote->c.fault = s_remote.c.fault;
    remote->c.protocol = s_remote.c.protocol;
    remote->a.vbus_raw = s_remote.a.vbus_raw;
    remote->a.curr_raw = s_remote.a.curr_raw;
    remote->a.pwr_raw = s_remote.a.pwr_raw;
    remote->a.fault = s_remote.a.fault;
    remote->a.protocol = s_remote.a.protocol;
    remote->fault_bits = s_remote.fault_bits;
    remote->seq = s_remote.seq;
    remote->valid = s_remote.valid;
    NVIC_EnableIRQ(USART1_IRQn);
#else
    /* The sub board only transmits; there is nothing to report to its callers. */
    memset(remote, 0, sizeof(*remote));
    remote->c.protocol = BOARD_LINK_PROTOCOL_NONE;
    remote->a.protocol = BOARD_LINK_PROTOCOL_NONE;
#endif
}

bool BoardLink_RemoteOnline(void)
{
#if BOARD_IS_MAIN
    return (s_remote.valid &&
            (s_rx_age_ms < BOARD_LINK_REMOTE_TIMEOUT_MS)) ? 1u : 0u;
#else
    return false;
#endif
}

void BoardLink_GetStats(board_link_stats_t *stats)
{
    if (stats == 0) {
        return;
    }

    NVIC_DisableIRQ(USART1_IRQn);
    *stats = s_stats;
    NVIC_EnableIRQ(USART1_IRQn);
}
