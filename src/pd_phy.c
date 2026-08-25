/**
 * @file pd_phy.c
 * @brief USB-PD PHY interrupt handling, packet encoding, and TX/RX timing.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "pd_internal.h"

/* Packet framing, GoodCRC supervision, hard reset, and PHY interrupt paths. */

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/** @brief Construct byte zero of a USB-PD message header. */
static uint8_t pd_header_byte0(uint8_t msg_type, uint8_t sop)
{
    uint8_t revision = (sop == UPD_SOP0) ? s_pd.pd_revision :
                                          s_cable_pd_revision;
    uint8_t data_role = (sop == UPD_SOP0) ? 0x20U : 0U;

    return (uint8_t)((msg_type & 0x1FU) | data_role |
                     (revision ? 0x80U : 0x40U));
}

/** @brief Construct byte one of a USB-PD message header. */
static uint8_t pd_header_byte1_id(uint8_t num_do, uint8_t ext, uint8_t msg_id, uint8_t sop)
{
    uint8_t header = (uint8_t)((msg_id & 0x07U) << 1U);

    if (sop == UPD_SOP0) {
        header |= 0x01U;
    }
    header |= (uint8_t)((num_do & 0x07U) << 4U);
    if (ext != 0U) {
        header |= 0x80U;
    }
    return header;
}

/** @brief Return the next transmit message ID for the specified SOP. */
static uint8_t pd_tx_msg_id_for_sop(uint8_t sop)
{
    if (sop == UPD_SOP1) {
        return s_cable_msg_id;
    }
    if (sop == UPD_SOP2) {
        return s_cable2_msg_id;
    }
    return s_pd.msg_id;
}

/** @brief Check whether the received GoodCRC acknowledges the active transfer. */
static uint8_t pd_goodcrc_matches_tx(uint8_t rx_len, uint8_t rx_sop)
{
    uint8_t expected_rx_sop;
    uint8_t rx_msg_id;

    if ((s_tx.state != PD_TX_WAIT_GOODCRC) ||
        (rx_len != PD_GOODCRC_RX_LEN) ||
        ((s_pd_rx_dma_buf[0] & 0x1FU) != DEF_TYPE_GOODCRC)) {
        return 0U;
    }

    expected_rx_sop = (s_tx.sop == UPD_SOP1) ? PD_RX_SOP1_HRST :
                      (s_tx.sop == UPD_SOP2) ? PD_RX_SOP2_CRST :
                                              PD_RX_SOP0;
    rx_msg_id = (uint8_t)((s_pd_rx_dma_buf[1] >> 1U) & 0x07U);
    return ((rx_sop == expected_rx_sop) &&
            (rx_msg_id == s_tx.msg_id)) ? 1U : 0U;
}

/** @brief Load a SOP-specific USB-PD header into the PHY transmit buffer. */
static void pd_load_header_sop(uint8_t msg_type, uint8_t num_do, uint8_t ext, uint8_t sop)
{
    s_pd_tx_buf[0] = pd_header_byte0(msg_type, sop);
    s_pd_tx_buf[1] = pd_header_byte1_id(num_do, ext, pd_tx_msg_id_for_sop(sop), sop);
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
    USBPD->BMC_CLK_CNT = s_pd_rx_clk_cnt;
    USBPD->CONTROL |= BMC_START;
    if (enable_irq != 0U) {
        NVIC_EnableIRQ(USBPD_IRQn);
    }
}

void pd_set_phy_rx_mode(void)
{
    pd_set_phy_rx_mode_irq(1U);
}

void pd_goodcrc_timer_stop(void)
{
    TIM_Cmd(TIM4, DISABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    s_goodcrc_tx_pending = 0U;
    s_goodcrc_watchdog_ms = 0U;
}

/** @brief Start the watchdog that waits for the GoodCRC response. */
static void pd_goodcrc_timer_start(void)
{
    TIM_Cmd(TIM4, DISABLE);
    TIM_SetCounter(TIM4, 0U);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    s_goodcrc_watchdog_ms = 0U;
    TIM_Cmd(TIM4, ENABLE);
}

/** @brief Recover transmit state after a GoodCRC timer remains asserted. */
static void pd_goodcrc_recover_from_stuck(void)
{
    uint8_t fail_stage = (s_goodcrc_tx_pending != 0U) ? 1U :
                         ((s_goodcrc_tx_active != 0U) ? 2U : 0U);

    if ((USBPD->STATUS & IF_TX_END) != 0U) {
        fail_stage |= 4U;
    }
    TIM_Cmd(TIM4, DISABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    USBPD->PORT_CC1 &= (uint16_t)(~CC_LVE);
    USBPD->PORT_CC2 &= (uint16_t)(~CC_LVE);
    USBPD->CONFIG &= (uint16_t)(~IE_TX_END);
    USBPD->CONTROL &= (uint8_t)(~PD_TX_EN);

    s_goodcrc_tx_pending = 0U;
    s_goodcrc_tx_active = 0U;
    s_goodcrc_watchdog_ms = 0U;
    if (s_debug_goodcrc_recover_count != 0xFFU) {
        s_debug_goodcrc_recover_count++;
    }
    s_debug_goodcrc_fail_msg_type = s_debug_goodcrc_rx_msg_type;
    s_debug_goodcrc_fail_msg_id = s_debug_goodcrc_rx_msg_id;
    s_debug_goodcrc_fail_stage = fail_stage;

    /* The frame whose GoodCRC did not complete was never accepted by the
     * Protocol Layer.  Do not pass it to policy: the partner will retry it
     * with the same MessageID.  Processing an unacknowledged Chunk Request
     * can transmit the next chunk before the requester is ready. */
    if (s_rx_msg_queued != 0U) {
        s_rx_msg_queued = 0U;
        s_rx_msg_queued_len = 0U;
    } else if (s_rx_msg_pending == 0U) {
        s_rx_msg_len = 0U;
    }
    pd_set_phy_rx_mode_irq(1U);
}

/** @brief Complete an acknowledged PHY transmission from the GoodCRC path. */
static void pd_goodcrc_complete_from_phy(void)
{
    USBPD->STATUS = IF_TX_END;
    s_goodcrc_tx_active = 0U;
    NVIC_DisableIRQ(USBPD_IRQn);
    s_rx_msg_pending = 1U;
}

void pd_goodcrc_watchdog_task(void)
{
    uint8_t stuck = 0U;

    if ((s_goodcrc_tx_pending == 0U) && (s_goodcrc_tx_active == 0U)) {
        s_goodcrc_watchdog_ms = 0U;
        return;
    }

    /* The completion IRQ can be delayed while another fast interrupt is
     * active.  Consume the already-set PHY flag from the cooperative task so
     * a successfully transmitted GoodCRC is not misclassified as stuck. */
    if ((s_goodcrc_tx_active != 0U) &&
        ((USBPD->STATUS & IF_TX_END) != 0U)) {
        __disable_irq();
        if ((s_goodcrc_tx_active != 0U) &&
            ((USBPD->STATUS & IF_TX_END) != 0U)) {
            pd_goodcrc_complete_from_phy();
        }
        __enable_irq();
        return;
    }

    if (s_goodcrc_watchdog_ms >= PD_GOODCRC_STUCK_TIMEOUT_MS) {
        stuck = 1U;
    }

    if (stuck != 0U) {
        __disable_irq();
        pd_goodcrc_recover_from_stuck();
        __enable_irq();
    }
}

/** @brief Program and launch one encoded USB-PD packet in the PHY. */
static void pd_phy_start_packet(const uint8_t *buf, uint8_t len, uint8_t sop)
{
    if ((USBPD->CONFIG & CC_SEL) != 0U) {
        USBPD->PORT_CC2 |= CC_LVE;
    } else {
        USBPD->PORT_CC1 |= CC_LVE;
    }

    USBPD->BMC_CLK_CNT = s_pd_tx_clk_cnt;
    USBPD->USBPD_DMA = (uint32_t)buf;
    USBPD->TX_SEL = sop;
    USBPD->BMC_TX_SZ = len;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->CONTROL |= BMC_START;
}

void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);

        if (s_goodcrc_tx_pending != 0U) {
            s_goodcrc_tx_pending = 0U;
            s_goodcrc_tx_active = 1U;
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
    s_tx.wait_ms = 0U;
    NVIC_EnableIRQ(USBPD_IRQn);
}

uint8_t pd_tx_start_sop(uint8_t msg_type, const uint8_t *payload, uint8_t payload_len, uint8_t sop)
{
    uint8_t i;

    if ((s_tx.state != PD_TX_IDLE) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        (s_goodcrc_tx_pending != 0U) ||
        (s_goodcrc_tx_active != 0U) ||
        (payload_len > 28U) ||
        ((payload_len % 4U) != 0U)) {
        return 0U;
    }

    pd_load_header_sop(msg_type, (uint8_t)(payload_len / 4U), 0U, sop);
    for (i = 0U; i < payload_len; i++) {
        s_pd_tx_buf[2U + i] = payload[i];
    }

    s_tx.len = (uint8_t)(payload_len + 2U);
    s_tx.try_count = 0U;
    s_tx.expect_goodcrc = 1U;
    s_tx.sop = sop;
    s_tx.msg_id = pd_tx_msg_id_for_sop(sop);
    s_tx.result = PD_TX_RESULT_NONE;

    pd_tx_start_attempt();
    return 1U;
}

uint8_t pd_tx_start(uint8_t msg_type, const uint8_t *payload, uint8_t payload_len)
{
    return pd_tx_start_sop(msg_type, payload, payload_len, UPD_SOP0);
}

uint8_t pd_tx_start_extended_chunk(uint8_t msg_type,
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
        (s_goodcrc_tx_pending != 0U) ||
        (s_goodcrc_tx_active != 0U) ||
        (data_len > PD_EXT_HEADER_DATA_SIZE_MASK) ||
        (chunk_number > PD_EXT_HEADER_CHUNK_NUM_MASK)) {
        return 0U;
    }
    if ((data_len != 0U) && (payload == 0)) {
        return 0U;
    }

    offset = (uint16_t)chunk_number * PD_EXT_CHUNK_DATA_MAX;
    if (offset >= data_len) {
        return 0U;
    }
    chunk_len = (uint8_t)(data_len - offset);
    if (chunk_len > PD_EXT_CHUNK_DATA_MAX) {
        chunk_len = PD_EXT_CHUNK_DATA_MAX;
    }

    num_do = (uint8_t)(((uint8_t)(chunk_len + 2U) + 3U) / 4U);
    payload_len = (uint8_t)(num_do * 4U);

    pd_load_header_sop(msg_type, num_do, 1U, UPD_SOP0);

    ext_header = (uint16_t)(PD_EXT_HEADER_CHUNKED |
                            (((uint16_t)chunk_number & PD_EXT_HEADER_CHUNK_NUM_MASK) <<
                             PD_EXT_HEADER_CHUNK_NUM_SHIFT) |
                            data_len);
    s_pd_tx_buf[2] = (uint8_t)(ext_header & 0xFFU);
    s_pd_tx_buf[3] = (uint8_t)((ext_header >> 8U) & 0xFFU);
    for (i = 2U; i < payload_len; i++) {
        s_pd_tx_buf[2U + i] = 0U;
    }
    for (i = 0U; i < chunk_len; i++) {
        s_pd_tx_buf[4U + i] = payload[offset + i];
    }

    s_tx.len = (uint8_t)(payload_len + 2U);
    s_tx.try_count = 0U;
    s_tx.expect_goodcrc = 1U;
    s_tx.sop = UPD_SOP0;
    s_tx.msg_id = s_pd.msg_id;
    s_tx.result = PD_TX_RESULT_NONE;

    pd_tx_start_attempt();
    return 1U;
}

uint8_t pd_tx_start_extended(uint8_t msg_type, const uint8_t *payload, uint8_t data_len)
{
    return pd_tx_start_extended_chunk(msg_type, payload, data_len, 0U);
}

uint8_t pd_tx_start_control(uint8_t msg_type)
{
    return pd_tx_start(msg_type, 0, 0U);
}

void pd_tx_start_hard_reset(void)
{
    if ((s_tx.state != PD_TX_IDLE) ||
        (s_tx.result != PD_TX_RESULT_NONE) ||
        (s_goodcrc_tx_pending != 0U) ||
        (s_goodcrc_tx_active != 0U)) {
        return;
    }
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;
    USBPD->CONFIG |= IE_TX_END;
    s_tx.len = 0U;
    s_tx.try_count = 0U;
    s_tx.wait_ms = 0U;
    s_tx.expect_goodcrc = 0U;
    s_tx.sop = UPD_HARD_RESET;
    s_tx.result = PD_TX_RESULT_NONE;
    pd_phy_start_packet(0, 0U, UPD_HARD_RESET);
    s_tx.state = PD_TX_WAIT_PHY_DONE;
    NVIC_EnableIRQ(USBPD_IRQn);
}

/** @brief Handle PHY completion of the transmitted packet payload. */
static void pd_tx_phy_done_from_irq(void)
{
    USBPD->PORT_CC1 &= (uint16_t)(~CC_LVE);
    USBPD->PORT_CC2 &= (uint16_t)(~CC_LVE);

    if (s_tx.expect_goodcrc == 0U) {
        s_tx.result = PD_TX_RESULT_OK;
        s_tx.state = PD_TX_IDLE;
        pd_set_phy_rx_mode_irq(1U);
    } else {
        pd_set_phy_rx_mode_irq(1U);
        s_tx.state = PD_TX_WAIT_GOODCRC;
        s_tx.wait_ms = 0U;
    }
}

/** @brief Handle a GoodCRC receive interrupt during an active transmission. */
static void pd_tx_goodcrc_from_irq(void)
{
    if (s_tx.sop == UPD_SOP1) {
        s_cable_msg_id = (uint8_t)((s_cable_msg_id + 1U) & 0x07U);
    } else if (s_tx.sop == UPD_SOP2) {
        s_cable2_msg_id = (uint8_t)((s_cable2_msg_id + 1U) & 0x07U);
    } else {
        s_pd.msg_id = (uint8_t)((s_pd.msg_id + 1U) & 0x07U);
    }
    s_tx.result = PD_TX_RESULT_OK;
    s_tx.state = PD_TX_IDLE;
    pd_set_phy_rx_mode_irq(1U);
}

void pd_tx_task(uint8_t elapsed_ms)
{
    uint8_t rx_sop;

    if ((s_goodcrc_tx_pending != 0U) ||
        (s_goodcrc_tx_active != 0U)) {
        return;
    }

    s_tx.wait_ms = (uint8_t)(s_tx.wait_ms + elapsed_ms);

    if (s_tx.state == PD_TX_WAIT_PHY_DONE) {
        if ((USBPD->STATUS & IF_TX_END) != 0U) {
            USBPD->STATUS = IF_TX_END;
            pd_tx_phy_done_from_irq();
            return;
        }
        if (s_tx.wait_ms >= PD_TX_PHY_TIMEOUT_MS) {
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
        if ((USBPD->STATUS & IF_RX_ACT) != 0U) {
            rx_sop = (uint8_t)(USBPD->STATUS & MASK_PD_STAT);
            USBPD->STATUS |= IF_RX_ACT;
            if (pd_goodcrc_matches_tx((uint8_t)USBPD->BMC_BYTE_CNT,
                                      rx_sop) != 0U) {
                pd_tx_goodcrc_from_irq();
                return;
            }
        }

        if (s_tx.wait_ms >= PD_GOODCRC_TIMEOUT_MS) {
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

uint8_t pd_tx_take_result(pd_tx_result_t *result)
{
    if (s_tx.result == PD_TX_RESULT_NONE) {
        return 0U;
    }
    *result = s_tx.result;
    s_tx.result = PD_TX_RESULT_NONE;
    return 1U;
}

uint8_t pd_tx_is_idle(void)
{
    return (s_tx.state == PD_TX_IDLE) ? 1U : 0U;
}


void pd_select_cc(uint8_t cc)
{
    if (cc == 2U) {
        USBPD->CONFIG |= CC_SEL;
    } else {
        USBPD->CONFIG &= (uint16_t)(~CC_SEL);
    }
    s_pd.cc = cc;
}


void pd_phy_select_cc(uint8_t cc)
{
    if (cc == 2U) {
        USBPD->CONFIG |= CC_SEL;
    } else {
        USBPD->CONFIG &= (uint16_t)(~CC_SEL);
    }
}


uint8_t pd_rx_sop_to_tx_sop(uint8_t rx_sop)
{
    if (rx_sop == PD_RX_SOP1_HRST) {
        return UPD_SOP1;
    }
    if (rx_sop == PD_RX_SOP2_CRST) {
        return UPD_SOP2;
    }
    return UPD_SOP0;
}

void USBPD_IRQHandler(void)
{
    uint8_t i;
    uint8_t rx_len;
    uint8_t rx_sop;

    if (((USBPD->STATUS & IF_TX_END) != 0U) &&
        (s_goodcrc_tx_active == 0U) &&
        (s_tx.state == PD_TX_WAIT_PHY_DONE)) {
        USBPD->PORT_CC1 &= (uint16_t)(~CC_LVE);
        USBPD->PORT_CC2 &= (uint16_t)(~CC_LVE);
        USBPD->STATUS = IF_TX_END;
        pd_tx_phy_done_from_irq();
    }

    if ((USBPD->STATUS & IF_RX_ACT) != 0U) {
        rx_sop = (uint8_t)(USBPD->STATUS & MASK_PD_STAT);
        USBPD->STATUS |= IF_RX_ACT;
        rx_len = (uint8_t)USBPD->BMC_BYTE_CNT;
        if (((rx_sop == PD_RX_SOP0) ||
             (rx_sop == PD_RX_SOP1_HRST) ||
             (rx_sop == PD_RX_SOP2_CRST)) &&
            (rx_len >= 6U) &&
            (rx_len <= PD_RX_BUF_SIZE)) {
            if (pd_goodcrc_matches_tx(rx_len, rx_sop) != 0U) {
                pd_tx_goodcrc_from_irq();
            } else if ((s_pd_rx_dma_buf[0] & 0x1FU) != DEF_TYPE_GOODCRC) {
                if (s_rx_msg_pending == 0U) {
                    for (i = 0U; i < rx_len; i++) {
                        s_pd_rx_buf[i] = s_pd_rx_dma_buf[i];
                    }
                    s_rx_msg_len = rx_len;
                    s_rx_sop = rx_sop;
                } else {
                    /* One extra frame absorbs a tester changing PPS/AVS
                     * immediately after GoodCRC.  Keep the newest frame when
                     * several requests arrive before the policy task runs. */
                    for (i = 0U; i < rx_len; i++) {
                        s_pd_rx_queued_buf[i] = s_pd_rx_dma_buf[i];
                    }
                    s_rx_msg_queued_len = rx_len;
                    s_rx_msg_queued_sop = rx_sop;
                    s_rx_msg_queued = 1U;
                }
                s_pd_goodcrc_buf[0] = (uint8_t)((s_pd_rx_dma_buf[0] & 0xC0U) |
                                                ((rx_sop == PD_RX_SOP0) ? 0x20U : 0U) |
                                                DEF_TYPE_GOODCRC);
                s_pd_goodcrc_buf[1] = (uint8_t)((s_pd_rx_dma_buf[1] & 0x0EU) |
                                                ((rx_sop == PD_RX_SOP0) ? 0x01U : 0U));
                s_debug_goodcrc_rx_msg_type =
                    (uint8_t)(s_pd_rx_dma_buf[0] & 0x1FU);
                s_debug_goodcrc_rx_msg_id =
                    (uint8_t)((s_pd_rx_dma_buf[1] >> 1U) & 0x07U);
                s_goodcrc_sop = pd_rx_sop_to_tx_sop(rx_sop);
                s_goodcrc_tx_pending = 1U;
                NVIC_DisableIRQ(USBPD_IRQn);
                pd_goodcrc_timer_start();
            }
        } else {
            s_rx_msg_len = 0U;
        }
    }

    if ((USBPD->STATUS & IF_TX_END) != 0U) {
        USBPD->PORT_CC1 &= (uint16_t)(~CC_LVE);
        USBPD->PORT_CC2 &= (uint16_t)(~CC_LVE);
        USBPD->STATUS = IF_TX_END;
        if (s_goodcrc_tx_active != 0U) {
            pd_goodcrc_complete_from_phy();
        } else if (s_tx.state == PD_TX_WAIT_PHY_DONE) {
            pd_tx_phy_done_from_irq();
        }
    }

    if ((USBPD->STATUS & IF_RX_RESET) != 0U) {
        USBPD->STATUS = IF_RX_RESET;
        if (s_debug_rx_hard_reset_count != 0xFFU) {
            s_debug_rx_hard_reset_count++;
        }
        s_debug_rx_hard_reset_state = (uint8_t)s_pd.state;
        s_debug_rx_hard_reset_power_state = (uint8_t)s_power.state;
        s_rx_msg_pending = 0U;
        s_rx_msg_queued = 0U;
        s_rx_msg_len = 0U;
        s_rx_msg_queued_len = 0U;
        s_goodcrc_tx_active = 0U;
        s_goodcrc_tx_pending = 0U;
        pd_goodcrc_timer_stop();
        s_pd_reset_pending = 1U;
    }
}
