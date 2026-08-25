/**
 * @file board_link.c
 * @brief UART link protocol and local/remote board data exchange.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "board_link.h"

#include "board_config.h"
#include "ina226_service.h"
#include "pd.h"

#define BOARD_LINK_MAGIC0          0xA5U
#define BOARD_LINK_MAGIC1          0x5AU
#define BOARD_LINK_VERSION         0x03U
#define BOARD_LINK_PAYLOAD_LEN     19U
#define BOARD_LINK_FRAME_LEN       (4U + BOARD_LINK_PAYLOAD_LEN + 1U)
#define BOARD_LINK_TX_PERIOD_MS    100U

#define BOARD_LINK_PAYLOAD_SEQ     4U
#define BOARD_LINK_PAYLOAD_FAULT0  5U
#define BOARD_LINK_PAYLOAD_C       7U
#define BOARD_LINK_PAYLOAD_A       15U

static volatile u8 s_tx_busy = 0U;
static volatile u8 s_tx_pos = 0U;
static volatile u8 s_tx_len = 0U;
static u8 s_tx_buf[BOARD_LINK_FRAME_LEN];
static volatile board_link_stats_t s_stats;

#if BOARD_IS_SUB
static board_link_remote_t s_local;
static volatile u16 s_tx_elapsed_ms = 0U;
static volatile u8 s_tx_due = 1U;
static u8 s_tx_seq = 0U;
#endif

#if BOARD_IS_MAIN
static volatile board_link_remote_t s_remote;
static u8 s_rx_buf[BOARD_LINK_FRAME_LEN];
static u8 s_rx_index = 0U;
#endif

static volatile u8 s_fault_c;
static volatile u8 s_fault_a;

/**
 * @brief Calculate the two's-complement checksum for a link frame.
 * @param buf Frame bytes to include in the checksum.
 * @param len Number of bytes to process.
 * @return Checksum byte that makes the complete frame sum to zero.
 */
static u8 BoardLink_Checksum(const u8 *buf, u8 len)
{
    u8 sum = 0U;
    u8 i;

    for (i = 0U; i < len; i++) {
        sum = (u8)(sum + buf[i]);
    }

    return (u8)(0U - sum);
}

/**
 * @brief Map the local PD status to the board-link protocol identifier.
 * @param status Current PD status, or null for a disconnected channel.
 * @return Board-link protocol identifier.
 */
static u8 BoardLink_ProtocolFromStatus(const pd_status_t *status)
{
    if ((status == 0) || (status->connected == 0U)) {
        return BOARD_LINK_PROTOCOL_NONE;
    }

    if ((status->state == PD_STATE_READY) && (status->contract_mv != 0U)) {
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

void BoardLink_SetFaults(u8 fault_c, u8 fault_a)
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
static void BoardLink_Put16(u8 *buf, u8 index, u16 value)
{
    buf[index] = (u8)(value & 0xFFU);
    buf[(u8)(index + 1U)] = (u8)((value >> 8) & 0xFFU);
}

/** @brief Encode one channel snapshot into the local-board payload. */
static void BoardLink_EncodeChannel(u8 *buf, u8 index, const board_link_channel_t *ch)
{
    buf[index] = ch->fault;
    buf[(u8)(index + 1U)] = ch->protocol;
    BoardLink_Put16(buf, (u8)(index + 2U), ch->vbus_raw);
    BoardLink_Put16(buf, (u8)(index + 4U), ch->curr_raw);
    BoardLink_Put16(buf, (u8)(index + 6U), ch->pwr_raw);
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
    s_tx_buf[BOARD_LINK_FRAME_LEN - 1U] = BoardLink_Checksum(s_tx_buf, BOARD_LINK_FRAME_LEN - 1U);
}
/** @brief Start interrupt-driven transmission of the prepared frame. */
static void BoardLink_StartTx(u8 len)
{
    if ((len == 0U) || (s_tx_busy != 0U)) {
        if (s_tx_busy != 0U) {
            s_stats.tx_busy_drop++;
        }
        return;
    }

    NVIC_DisableIRQ(USART1_IRQn);
    if (s_tx_busy == 0U) {
        s_tx_pos = 0U;
        s_tx_len = len;
        s_tx_busy = 1U;
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
static u16 BoardLink_Get16(const u8 *buf, u8 index)
{
    return (u16)((u16)buf[index] | ((u16)buf[(u8)(index + 1U)] << 8));
}

/** @brief Decode one received payload channel into a snapshot structure. */
static void BoardLink_DecodeChannel(const u8 *buf, u8 index, volatile board_link_channel_t *ch)
{
    ch->fault = buf[index];
    ch->protocol = buf[(u8)(index + 1U)];
    ch->vbus_raw = BoardLink_Get16(buf, (u8)(index + 2U));
    ch->curr_raw = BoardLink_Get16(buf, (u8)(index + 4U));
    ch->pwr_raw = BoardLink_Get16(buf, (u8)(index + 6U));
}

/** @brief Validate and publish the currently buffered main-board frame. */
static u8 BoardLink_ApplyFrame(void)
{
    if (BoardLink_Checksum(s_rx_buf, BOARD_LINK_FRAME_LEN) != 0U) {
        s_stats.rx_checksum_error++;
        return 0U;
    }
    if ((s_rx_buf[2] != BOARD_LINK_VERSION) || (s_rx_buf[3] != BOARD_LINK_PAYLOAD_LEN)) {
        s_stats.rx_header_error++;
        return 0U;
    }

    s_remote.seq = s_rx_buf[BOARD_LINK_PAYLOAD_SEQ];
    s_remote.fault_bits = BoardLink_Get16(s_rx_buf, BOARD_LINK_PAYLOAD_FAULT0);
    BoardLink_DecodeChannel(s_rx_buf, BOARD_LINK_PAYLOAD_C, &s_remote.c);
    BoardLink_DecodeChannel(s_rx_buf, BOARD_LINK_PAYLOAD_A, &s_remote.a);
    s_remote.valid = 1U;
    s_stats.rx_frame_ok++;
    return 1U;
}

/** @brief Search a rejected frame for the next synchronization marker. */
static void BoardLink_ResyncAfterBadFrame(void)
{
    u8 i;

    for (i = 1U; i < (BOARD_LINK_FRAME_LEN - 1U); i++) {
        if ((s_rx_buf[i] == BOARD_LINK_MAGIC0) &&
            (s_rx_buf[(u8)(i + 1U)] == BOARD_LINK_MAGIC1)) {
            u8 j;
            u8 keep = (u8)(BOARD_LINK_FRAME_LEN - i);

            for (j = 0U; j < keep; j++) {
                s_rx_buf[j] = s_rx_buf[(u8)(i + j)];
            }
            s_rx_index = keep;
            s_stats.rx_resync++;
            return;
        }
    }

    s_rx_index = 0U;
}

/** @brief Feed one UART byte to the main-board frame parser. */
static void BoardLink_RxByte(u8 data)
{
    if (s_rx_index == 0U) {
        if (data == BOARD_LINK_MAGIC0) {
            s_rx_buf[s_rx_index++] = data;
        }
        return;
    }

    if (s_rx_index == 1U) {
        if (data == BOARD_LINK_MAGIC1) {
            s_rx_buf[s_rx_index++] = data;
        } else {
            s_rx_index = (data == BOARD_LINK_MAGIC0) ? 1U : 0U;
            s_rx_buf[0] = data;
        }
        return;
    }

    s_rx_buf[s_rx_index++] = data;
    if (s_rx_index >= BOARD_LINK_FRAME_LEN) {
        if (BoardLink_ApplyFrame() != 0U) {
            s_rx_index = 0U;
        } else {
            BoardLink_ResyncAfterBadFrame();
        }
    }
}
#endif

void BoardLink_TickMs(u16 elapsed_ms)
{
#if BOARD_IS_SUB
    s_tx_elapsed_ms = (u16)(s_tx_elapsed_ms + elapsed_ms);
    if (s_tx_elapsed_ms >= BOARD_LINK_TX_PERIOD_MS) {
        s_tx_elapsed_ms = 0U;
        s_tx_due = 1U;
    }
#else
    (void)elapsed_ms;
#endif
}

void BoardLink_Task(void)
{
#if BOARD_IS_SUB
    if ((s_tx_due == 0U) || (s_tx_busy != 0U)) {
        return;
    }

    s_tx_due = 0U;
    BoardLink_BuildFrame();
    BoardLink_StartTx(BOARD_LINK_FRAME_LEN);
#endif
}

void BoardLink_USART1_IRQHandler(void)
{
    u16 status = USART1->STATR;

#if BOARD_IS_MAIN
    if ((status & (USART_STATR_ORE | USART_STATR_NE | USART_STATR_FE)) != 0U) {
        (void)USART1->DATAR;
        s_stats.rx_usart_error++;
        if ((status & USART_STATR_ORE) != 0U) {
            s_stats.rx_ore++;
        }
        if ((status & USART_STATR_NE) != 0U) {
            s_stats.rx_ne++;
        }
        if ((status & USART_STATR_FE) != 0U) {
            s_stats.rx_fe++;
        }
        s_rx_index = 0U;
    } else if ((status & USART_STATR_RXNE) != 0U) {
        u8 data = (u8)(USART1->DATAR & 0xFFU);
        BoardLink_RxByte(data);
    }
#endif

    if (((status & USART_STATR_TXE) != 0U) &&
        ((USART1->CTLR1 & USART_CTLR1_TXEIE) != 0U)) {
        if (s_tx_pos < s_tx_len) {
            USART1->DATAR = s_tx_buf[s_tx_pos++];
        } else {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            s_tx_busy = 0U;
            s_stats.tx_frame_done++;
        }
    }
}

void BoardLink_SetLocalData(const board_link_channel_t *c,
                            const board_link_channel_t *a,
                            u16 fault_bits)
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
    remote->c.vbus_raw = 0U;
    remote->c.curr_raw = 0U;
    remote->c.pwr_raw = 0U;
    remote->c.fault = 0U;
    remote->c.protocol = BOARD_LINK_PROTOCOL_NONE;
    remote->a.vbus_raw = 0U;
    remote->a.curr_raw = 0U;
    remote->a.pwr_raw = 0U;
    remote->a.fault = 0U;
    remote->a.protocol = BOARD_LINK_PROTOCOL_NONE;
    remote->fault_bits = 0U;
    remote->seq = 0U;
    remote->valid = 0U;
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
