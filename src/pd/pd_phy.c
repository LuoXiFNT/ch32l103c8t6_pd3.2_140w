/**
 * @file pd_phy.c
 * @brief USB-PD PHY interrupt handling, packet encoding, and TX/RX timing.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#include "pd_internal.h"
#include "ch32l103_init.h"

/* Packet framing, GoodCRC supervision, hard reset, and PHY interrupt paths. */

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* Bit timing of the peripheral, derived from the clock the core actually runs at. */
static uint16_t s_tx_clk_cnt;
static uint16_t s_rx_clk_cnt;

void pd_phy_init(void)
{
    if (SystemCoreClock >= 96000000ul) {
        s_tx_clk_cnt = UPD_TMR_TX_96M;
        s_rx_clk_cnt = UPD_TMR_RX_96M;
    } else if (SystemCoreClock >= 48000000ul) {
        s_tx_clk_cnt = UPD_TMR_TX_48M;
        s_rx_clk_cnt = UPD_TMR_RX_48M;
    } else if (SystemCoreClock >= 24000000ul) {
        s_tx_clk_cnt = UPD_TMR_TX_24M;
        s_rx_clk_cnt = UPD_TMR_RX_24M;
    } else {
        s_tx_clk_cnt = UPD_TMR_TX_12M;
        s_rx_clk_cnt = UPD_TMR_RX_12M;
    }

    USBPD->CONFIG = PD_DMA_EN | PD_FILT_ED;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;
}

void pd_phy_reset_cc(void)
{
    USBPD->PORT_CC1 = CC_CMP_66 | CC_PU_330;
    USBPD->PORT_CC2 = CC_CMP_66 | CC_PU_330;
}

/** @brief Check whether the received GoodCRC acknowledges the active transfer. */
static bool pd_goodcrc_matches_tx(uint8_t rx_len, uint8_t rx_sop)
{
    uint8_t expected_rx_sop;
    uint8_t rx_msg_id;

    if ((s_tx.state != PD_TX_WAIT_GOODCRC) ||
        (rx_len != PD_GOODCRC_RX_LEN) ||
        ((s_pd_rx_dma_buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC)) {
        return false;
    }

    expected_rx_sop = (s_tx.sop == UPD_SOP1) ? PD_RX_SOP1_HRST :
                      (s_tx.sop == UPD_SOP2) ? PD_RX_SOP2_CRST :
                                              PD_RX_SOP0;
    rx_msg_id = (uint8_t)((s_pd_rx_dma_buf[1] >> 1u) & 0x07u);
    return ((rx_sop == expected_rx_sop) && (rx_msg_id == s_tx.msg_id));
}

/** @brief Configure PHY receive mode with the requested interrupt state. */
static void pd_set_phy_rx_mode_irq(uint8_t enable_irq)
{
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= (uint16_t)(~PD_ALL_CLR);
    USBPD->CONFIG &= (uint16_t)(~IE_TX_END);
    USBPD->CONFIG |= (uint16_t)(IE_RX_ACT | IE_RX_RESET | PD_DMA_EN);
    USBPD->USBPD_DMA = (uint32_t)s_pd_rx_dma_buf;
    USBPD->CONTROL &= (uint8_t)(~PD_TX_EN);
    USBPD->BMC_CLK_CNT = s_rx_clk_cnt;
    USBPD->CONTROL |= BMC_START;
    if (enable_irq != 0u) {
        NVIC_EnableIRQ(USBPD_IRQn);
    }
}

void pd_set_phy_rx_mode(void)
{
    /* The CC check re-arms the receiver every PD_CONNECTION_CHECK_MS while the port is
     * attached. Programming the receiver clears PD_TX_EN and restarts the BMC engine, so
     * doing it while a GoodCRC is still on the wire cuts that frame short: the partner sees
     * a header without EOP, discards the packet and never acknowledges the frame that
     * triggered the GoodCRC. Skip the re-arm while a GoodCRC is pending or transmitting;
     * that transmission completes within the same millisecond and the receive path arms the
     * PHY again from pd_set_phy_rx_mode_irq() or from the dispatch in PD_Task(). */
    if ((s_goodcrc_tx_pending) || (s_goodcrc_tx_active)) {
        return;
    }
    pd_set_phy_rx_mode_irq(1u);
}

void pd_goodcrc_timer_stop(void)
{
    TIM_Cmd(TIM4, DISABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    s_goodcrc_tx_pending = false;
    s_goodcrc_watchdog_ms = 0u;
}

/** @brief Start the watchdog that waits for the GoodCRC response. */
static void pd_goodcrc_timer_start(void)
{
    TIM_Cmd(TIM4, DISABLE);
    TIM_SetCounter(TIM4, 0u);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    s_goodcrc_watchdog_ms = 0u;
    TIM_Cmd(TIM4, ENABLE);
}

/** @brief Recover transmit state after a GoodCRC timer remains asserted. */
static void pd_goodcrc_recover_from_stuck(void)
{
    uint8_t fail_stage = (s_goodcrc_tx_pending) ? 1u :
                         ((s_goodcrc_tx_active) ? 2u : 0u);

    if ((USBPD->STATUS & IF_TX_END) != 0u) {
        fail_stage |= 4u;
    }
    TIM_Cmd(TIM4, DISABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    USBPD->PORT_CC1 &= (uint16_t)(~CC_LVE);
    USBPD->PORT_CC2 &= (uint16_t)(~CC_LVE);
    USBPD->CONFIG &= (uint16_t)(~IE_TX_END);
    USBPD->CONTROL &= (uint8_t)(~PD_TX_EN);

    s_goodcrc_tx_pending = false;
    s_goodcrc_tx_active = false;
    s_goodcrc_watchdog_ms = 0u;
    if (s_debug_goodcrc_recover_count != 0xFFu) {
        s_debug_goodcrc_recover_count++;
    }
    s_debug_goodcrc_fail_msg_type = s_debug_goodcrc_rx_msg_type;
    s_debug_goodcrc_fail_msg_id = s_debug_goodcrc_rx_msg_id;
    s_debug_goodcrc_fail_stage = fail_stage;

    /* The frame whose GoodCRC did not complete was never accepted by the
     * Protocol Layer.  Do not pass it to policy: the partner will retry it
     * with the same MessageID.  Processing an unacknowledged Chunk Request
     * can transmit the next chunk before the requester is ready. */
    /* It did arrive, though, so the CC path is alive: restart the source communication
     * timers. Without this a stalled GoodCRC on an EPR_Keep_Alive left the traffic timer
     * running and the policy layer Hard Reset the port one second later, which the Sink
     * sees as a low-voltage disconnect. */
    pd_reset_source_comm_timers();
    if (!s_rx_msg_pending) {
        s_rx_msg_len = 0u;
    } else {
        s_rx_ring_head = s_rx_ring_tail;
    }
    pd_set_phy_rx_mode_irq(1u);
}

/** @brief Complete an acknowledged PHY transmission from the GoodCRC path. */
static void pd_goodcrc_complete_from_phy(void)
{
    USBPD->STATUS = IF_TX_END;
    s_goodcrc_tx_active = false;
    NVIC_DisableIRQ(USBPD_IRQn);
    s_rx_msg_pending = true;
}

void pd_goodcrc_watchdog_task(void)
{
    bool stuck = false;

    if ((!s_goodcrc_tx_pending) && (!s_goodcrc_tx_active)) {
        s_goodcrc_watchdog_ms = 0u;
        return;
    }

    /* The completion IRQ can be delayed while another fast interrupt is
     * active.  Consume the already-set PHY flag from the cooperative task so
     * a successfully transmitted GoodCRC is not misclassified as stuck. */
    if ((s_goodcrc_tx_active) &&
        ((USBPD->STATUS & IF_TX_END) != 0u)) {
        __disable_irq();
        if ((s_goodcrc_tx_active) &&
            ((USBPD->STATUS & IF_TX_END) != 0u)) {
            pd_goodcrc_complete_from_phy();
        }
        __enable_irq();
        return;
    }

    if (s_goodcrc_watchdog_ms >= PD_GOODCRC_STUCK_TIMEOUT_MS) {
        stuck = true;
    }

    if (stuck) {
        __disable_irq();
        pd_goodcrc_recover_from_stuck();
        __enable_irq();
    }
}

/** @brief Program and launch one encoded USB-PD packet in the PHY. */
static void pd_phy_start_packet(const uint8_t *buf, uint8_t len, uint8_t sop)
{
    if ((USBPD->CONFIG & CC_SEL) != 0u) {
        USBPD->PORT_CC2 |= CC_LVE;
    } else {
        USBPD->PORT_CC1 |= CC_LVE;
    }

    USBPD->BMC_CLK_CNT = s_tx_clk_cnt;
    USBPD->USBPD_DMA = (uint32_t)buf;
    USBPD->TX_SEL = sop;
    USBPD->BMC_TX_SZ = len;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->CONTROL |= BMC_START;
}

void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/** @brief Send the acknowledgement once the tTransmit delay has expired. */
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);

        if (s_goodcrc_tx_pending) {
            if (s_tx.state == PD_TX_WAIT_PHY_DONE) {
                /* The PHY may still be finishing our own message: the completion flag has not
                 * been reported yet. Starting the acknowledgement now would take the
                 * transmitter away from that packet and leave both frames without EOP.
                 * Dropping the acknowledgement is the cheaper failure: the received frame
                 * stays queued and the partner retransmits it with the same MessageID, which
                 * is the recovery the protocol defines. Re-arm the interrupt that the receive
                 * path masked when it scheduled this acknowledgement. */
                s_goodcrc_tx_pending = false;
                NVIC_EnableIRQ(USBPD_IRQn);
                return;
            }
            s_goodcrc_tx_pending = false;
            s_goodcrc_tx_active = true;
            USBPD->CONFIG |= IE_TX_END;
            USBPD->STATUS = IF_TX_END;
            pd_phy_start_packet(s_pd_goodcrc_buf, PD_GOODCRC_LEN, s_goodcrc_sop);
            /* Arm the packet before a pending USBPD IRQ can observe the
             * active flag and consume a stale completion. */
            NVIC_EnableIRQ(USBPD_IRQn);
        }
    }
}

/** @brief Start the current transmit attempt and arm its GoodCRC watchdog. */
static void pd_tx_start_attempt(void)
{
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;
    USBPD->CONFIG |= IE_TX_END;
    pd_phy_start_packet(s_pd_tx_buf, s_tx.len, s_tx.sop);
    s_tx.state = PD_TX_WAIT_PHY_DONE;
    s_tx.wait_us = CH32L103_GetUs();
    NVIC_EnableIRQ(USBPD_IRQn);
}

bool pd_tx_start_sop(uint8_t msg_type, const uint8_t *payload, uint8_t payload_len, uint8_t sop)
{
    uint8_t i;

    if ((s_tx.state != PD_TX_IDLE) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        (s_goodcrc_tx_pending) ||
        (s_goodcrc_tx_active) ||
        (payload_len > 28u) ||
        ((payload_len % 4u) != 0u)) {
        return false;
    }

    pd_load_header_sop(msg_type, (uint8_t)(payload_len / 4u), 0u, sop);
    for (i = 0u; i < payload_len; i++) {
        s_pd_tx_buf[2u + i] = payload[i];
    }

    s_tx.len = (uint8_t)(payload_len + 2u);
    s_tx.try_count = 0u;
    s_tx.expect_goodcrc = true;
    s_tx.sop = sop;
    s_tx.msg_id = pd_tx_msg_id_for_sop(sop);
    s_tx.result = PD_TX_RESULT_NONE;

    pd_tx_start_attempt();
    return true;
}

bool pd_tx_start(uint8_t msg_type, const uint8_t *payload, uint8_t payload_len)
{
    return pd_tx_start_sop(msg_type, payload, payload_len, UPD_SOP0);
}

bool pd_tx_start_extended_chunk(uint8_t msg_type,
                                   const uint8_t *payload,
                                   uint16_t data_len,
                                   uint8_t chunk_number)
{
    uint8_t i;
    uint16_t offset;
    uint8_t chunk_len;
    uint8_t payload_len;
    uint8_t num_do;
    uint16_t ext_header;

    if ((s_tx.state != PD_TX_IDLE) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        (s_goodcrc_tx_pending) ||
        (s_goodcrc_tx_active) ||
        (data_len > PD_EXT_HEADER_DATA_SIZE_MASK) ||
        (chunk_number > PD_EXT_HEADER_CHUNK_NUM_MASK)) {
        return false;
    }
    if ((data_len != 0u) && (payload == 0)) {
        return false;
    }

    offset = (uint16_t)chunk_number * PD_EXT_CHUNK_DATA_MAX;
    if (offset >= data_len) {
        return false;
    }
    chunk_len = (uint8_t)(data_len - offset);
    if (chunk_len > PD_EXT_CHUNK_DATA_MAX) {
        chunk_len = PD_EXT_CHUNK_DATA_MAX;
    }

    num_do = (uint8_t)(((uint8_t)(chunk_len + 2u) + 3u) / 4u);
    payload_len = (uint8_t)(num_do * 4u);

    pd_load_header_sop(msg_type, num_do, 1u, UPD_SOP0);

    /* The Chunked bit is only set when the message really is sent in several chunks (more
     * than PD_EXT_CHUNK_DATA_MAX bytes of payload). Short extended messages - Status,
     * PPS_Status, EPR_Keep_Alive_Ack - must have it clear; marking them as chunked makes a
     * compliant Sink expect further chunks and stall the AMS instead of accepting the
     * message. */
    ext_header = (uint16_t)((((uint16_t)chunk_number & PD_EXT_HEADER_CHUNK_NUM_MASK) <<
                             PD_EXT_HEADER_CHUNK_NUM_SHIFT) |
                            data_len);
    if (data_len > PD_EXT_CHUNK_DATA_MAX) {
        ext_header |= PD_EXT_HEADER_CHUNKED;
    }
    s_pd_tx_buf[2] = (uint8_t)(ext_header & 0xFFu);
    s_pd_tx_buf[3] = (uint8_t)((ext_header >> 8u) & 0xFFu);
    for (i = 2u; i < payload_len; i++) {
        s_pd_tx_buf[2u + i] = 0u;
    }
    for (i = 0u; i < chunk_len; i++) {
        s_pd_tx_buf[4u + i] = payload[offset + i];
    }

    s_tx.len = (uint8_t)(payload_len + 2u);
    s_tx.try_count = 0u;
    s_tx.expect_goodcrc = true;
    s_tx.sop = UPD_SOP0;
    s_tx.msg_id = s_pd.msg_id;
    s_tx.result = PD_TX_RESULT_NONE;

    pd_tx_start_attempt();
    return true;
}

bool pd_tx_start_extended(uint8_t msg_type, const uint8_t *payload, uint8_t data_len)
{
    return pd_tx_start_extended_chunk(msg_type, payload, data_len, 0u);
}

bool pd_tx_start_control(uint8_t msg_type)
{
    return pd_tx_start(msg_type, 0, 0u);
}

void pd_tx_start_hard_reset(void)
{
    if ((s_tx.state != PD_TX_IDLE) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        (s_goodcrc_tx_pending) ||
        (s_goodcrc_tx_active)) {
        return;
    }
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;
    USBPD->CONFIG |= IE_TX_END;
    s_tx.len = 0u;
    s_tx.try_count = 0u;
    s_tx.wait_us = CH32L103_GetUs();
    s_tx.expect_goodcrc = false;
    s_tx.sop = UPD_HARD_RESET;
    s_tx.result = PD_TX_RESULT_NONE;
    pd_phy_start_packet(0, 0u, UPD_HARD_RESET);
    s_tx.state = PD_TX_WAIT_PHY_DONE;
    NVIC_EnableIRQ(USBPD_IRQn);
}

/** @brief Handle PHY completion of the transmitted packet payload. */
static void pd_tx_phy_done_from_irq(void)
{
    USBPD->PORT_CC1 &= (uint16_t)(~CC_LVE);
    USBPD->PORT_CC2 &= (uint16_t)(~CC_LVE);

    if (!s_tx.expect_goodcrc) {
        s_tx.result = PD_TX_RESULT_OK;
        s_tx.state = PD_TX_IDLE;
        pd_set_phy_rx_mode_irq(1u);
    } else {
        pd_set_phy_rx_mode_irq(1u);
        s_tx.state = PD_TX_WAIT_GOODCRC;
        /* Section 7.31.1 starts the CRCReceiveTimer when the last bit of the Message EOP
         * has been transmitted, which is exactly what the completion flag reports. */
        s_tx.wait_us = CH32L103_GetUs();
    }
}

/** @brief Handle a GoodCRC receive interrupt during an active transmission. */
static void pd_tx_goodcrc_from_irq(void)
{
    if (s_tx.sop == UPD_SOP1) {
        s_cable_msg_id = (uint8_t)((s_cable_msg_id + 1u) & 0x07u);
    } else if (s_tx.sop == UPD_SOP2) {
        s_cable2_msg_id = (uint8_t)((s_cable2_msg_id + 1u) & 0x07u);
    } else {
        s_pd.msg_id = (uint8_t)((s_pd.msg_id + 1u) & 0x07u);
    }
    s_tx.result = PD_TX_RESULT_OK;
    s_tx.state = PD_TX_IDLE;
    pd_set_phy_rx_mode_irq(1u);
}

void pd_tx_task(uint8_t elapsed_ms)
{
    uint8_t rx_sop;

    /* Both transmit phases are timed against the microsecond counter, so the task cadence no
     * longer sets the resolution of the GoodCRC timeout. */
    (void)elapsed_ms;

    if ((s_goodcrc_tx_pending) ||
        (s_goodcrc_tx_active)) {
        return;
    }

    if (s_tx.state == PD_TX_WAIT_PHY_DONE) {
        if ((USBPD->STATUS & IF_TX_END) != 0u) {
            USBPD->STATUS = IF_TX_END;
            pd_tx_phy_done_from_irq();
            return;
        }
        if ((uint32_t)(CH32L103_GetUs() - s_tx.wait_us) >=
            ((uint32_t)PD_TX_PHY_TIMEOUT_MS * 1000ul)) {
            if (++s_tx.try_count < PD_TX_RETRY_MAX) {
                pd_tx_start_attempt();
            } else {
                s_tx.result = PD_TX_RESULT_FAIL;
                s_tx.state = PD_TX_IDLE;
                pd_set_phy_rx_mode();
            }
        }
        return;
    }

    if (s_tx.state == PD_TX_WAIT_GOODCRC) {
        if ((USBPD->STATUS & IF_RX_ACT) != 0u) {
            rx_sop = (uint8_t)(USBPD->STATUS & MASK_PD_STAT);
            USBPD->STATUS |= IF_RX_ACT;
            if (pd_goodcrc_matches_tx((uint8_t)USBPD->BMC_BYTE_CNT,
                                      rx_sop) != 0u) {
                pd_tx_goodcrc_from_irq();
                return;
            }
        }

        if ((uint32_t)(CH32L103_GetUs() - s_tx.wait_us) >= PD_GOODCRC_TIMEOUT_US) {
            if (++s_tx.try_count < PD_TX_RETRY_MAX) {
                pd_tx_start_attempt();
            } else {
                s_tx.result = PD_TX_RESULT_FAIL;
                s_tx.state = PD_TX_IDLE;
                pd_set_phy_rx_mode();
            }
        }
    }
}

bool pd_tx_take_result(pd_tx_result_t *result)
{
    if (s_tx.result == PD_TX_RESULT_NONE) {
        return false;
    }
    *result = s_tx.result;
    s_tx.result = PD_TX_RESULT_NONE;
    return true;
}

bool pd_tx_is_idle(void)
{
    return (s_tx.state == PD_TX_IDLE);
}


void pd_select_cc(uint8_t cc)
{
    if (cc == 2u) {
        USBPD->CONFIG |= CC_SEL;
    } else {
        USBPD->CONFIG &= (uint16_t)(~CC_SEL);
    }
    s_pd.cc = cc;
}


/** @brief USBPD interrupt: transmit end, frame received, or receive reset. */
void USBPD_IRQHandler(void)
{
    uint8_t i;
    uint8_t rx_len;
    uint8_t rx_sop;

    if (((USBPD->STATUS & IF_TX_END) != 0u) &&
        (!s_goodcrc_tx_active) &&
        (s_tx.state == PD_TX_WAIT_PHY_DONE)) {
        USBPD->PORT_CC1 &= (uint16_t)(~CC_LVE);
        USBPD->PORT_CC2 &= (uint16_t)(~CC_LVE);
        USBPD->STATUS = IF_TX_END;
        pd_tx_phy_done_from_irq();
    }

    if ((USBPD->STATUS & IF_RX_ACT) != 0u) {
        rx_sop = (uint8_t)(USBPD->STATUS & MASK_PD_STAT);
        USBPD->STATUS |= IF_RX_ACT;
        rx_len = (uint8_t)USBPD->BMC_BYTE_CNT;
        if (((rx_sop == PD_RX_SOP0) ||
             (rx_sop == PD_RX_SOP1_HRST) ||
             (rx_sop == PD_RX_SOP2_CRST)) &&
            (rx_len >= 6u) &&
            (rx_len <= PD_RX_BUF_SIZE)) {
            if (pd_goodcrc_matches_tx(rx_len, rx_sop) != 0u) {
                pd_tx_goodcrc_from_irq();
            } else if ((s_pd_rx_dma_buf[0] & 0x1Fu) != DEF_TYPE_GOODCRC) {
                if (s_debug_rx_frame_count < 0xFFFFu) {
                    s_debug_rx_frame_count++;
                }
                if (!s_rx_msg_pending) {
                    for (i = 0u; i < rx_len; i++) {
                        s_pd_rx_buf[i] = s_pd_rx_dma_buf[i];
                    }
                    s_rx_msg_len = rx_len;
                    s_rx_sop = rx_sop;
                } else {
                    /* Append to the received-frame ring. The policy layer drains it from the
                     * tail, so a burst of frames can no longer overwrite one another; the
                     * ring is deep enough for every message a partner can send back to back
                     * before the main loop runs. */
                    uint8_t next = (uint8_t)((s_rx_ring_head + 1u) % PD_RX_RING_LEN);

                    if (next != s_rx_ring_tail) {
                        for (i = 0u; i < rx_len; i++) {
                            s_pd_rx_ring_buf[s_rx_ring_head][i] = s_pd_rx_dma_buf[i];
                        }
                        s_rx_ring_len[s_rx_ring_head] = rx_len;
                        s_rx_ring_sop[s_rx_ring_head] = rx_sop;
                        s_rx_ring_head = next;
                    } else if (s_debug_rx_drop_count < 0xFFFFu) {
                        /* Ring full: this frame is lost, and the counter shows it. */
                        s_debug_rx_drop_count++;
                    }
                }
                s_pd_goodcrc_buf[0] = (uint8_t)((s_pd_rx_dma_buf[0] & 0xC0u) |
                                                ((rx_sop == PD_RX_SOP0) ? 0x20u : 0u) |
                                                DEF_TYPE_GOODCRC);
                s_pd_goodcrc_buf[1] = (uint8_t)((s_pd_rx_dma_buf[1] & 0x0Eu) |
                                                ((rx_sop == PD_RX_SOP0) ? 0x01u : 0u));
                s_debug_goodcrc_rx_msg_type =
                    (uint8_t)(s_pd_rx_dma_buf[0] & 0x1Fu);
                s_debug_goodcrc_rx_msg_id =
                    (uint8_t)((s_pd_rx_dma_buf[1] >> 1u) & 0x07u);
                s_goodcrc_sop = pd_rx_sop_to_tx_sop(rx_sop);
                s_goodcrc_tx_pending = true;
                NVIC_DisableIRQ(USBPD_IRQn);
                pd_goodcrc_timer_start();
            }
        } else {
            s_rx_msg_len = 0u;
        }
    }

    if ((USBPD->STATUS & IF_TX_END) != 0u) {
        USBPD->PORT_CC1 &= (uint16_t)(~CC_LVE);
        USBPD->PORT_CC2 &= (uint16_t)(~CC_LVE);
        USBPD->STATUS = IF_TX_END;
        if (s_goodcrc_tx_active) {
            pd_goodcrc_complete_from_phy();
        } else if (s_tx.state == PD_TX_WAIT_PHY_DONE) {
            pd_tx_phy_done_from_irq();
        }
    }

    if ((USBPD->STATUS & IF_RX_RESET) != 0u) {
        USBPD->STATUS = IF_RX_RESET;
        if (s_debug_rx_hard_reset_count != 0xFFu) {
            s_debug_rx_hard_reset_count++;
        }
        s_debug_rx_hard_reset_state = (uint8_t)s_pd.state;
        s_debug_rx_hard_reset_power_state = (uint8_t)s_power.state;
        s_rx_msg_pending = false;
        s_rx_msg_len = 0u;
        s_rx_ring_head = 0u;
        s_rx_ring_tail = 0u;
        s_goodcrc_tx_active = false;
        s_goodcrc_tx_pending = false;
        pd_goodcrc_timer_stop();
        s_pd_reset_pending = true;
    }
}
