/**
 * @file pd_message.c
 * @brief USB-PD message header construction and SOP mapping.
 * @author LuoXiFNT
 * @date 2026-11-06
 * @lastModified 2026-11-06
 *
 * Protocol layer: turns protocol fields into the two header bytes that precede every
 * message, and maps between the receive SOP field and the SOP a response has to use.
 * Nothing here touches the peripheral; the transmit buffer it writes is handed to
 * pd_phy.c, which owns the transfer.
 */

#include "pd_internal.h"

/** @brief Construct byte zero of a USB-PD message header. */
static uint8_t pd_header_byte0(uint8_t msg_type, uint8_t sop)
{
    uint8_t revision = (sop == UPD_SOP0) ? s_pd.pd_revision :
                                          s_cable_pd_revision;
    uint8_t data_role = (sop == UPD_SOP0) ? 0x20u : 0u;

    return (uint8_t)((msg_type & 0x1Fu) | data_role |
                     (revision ? 0x80u : 0x40u));
}

/** @brief Construct byte one of a USB-PD message header. */
static uint8_t pd_header_byte1_id(uint8_t num_do, uint8_t ext, uint8_t msg_id, uint8_t sop)
{
    uint8_t header = (uint8_t)((msg_id & 0x07u) << 1u);

    if (sop == UPD_SOP0) {
        header |= 0x01u;
    }
    header |= (uint8_t)((num_do & 0x07u) << 4u);
    if (ext != 0u) {
        header |= 0x80u;
    }
    return header;
}

uint8_t pd_tx_msg_id_for_sop(uint8_t sop)
{
    if (sop == UPD_SOP1) {
        return s_cable_msg_id;
    }
    if (sop == UPD_SOP2) {
        return s_cable2_msg_id;
    }
    return s_pd.msg_id;
}

void pd_load_header_sop(uint8_t msg_type, uint8_t num_do, uint8_t ext, uint8_t sop)
{
    s_pd_tx_buf[0] = pd_header_byte0(msg_type, sop);
    s_pd_tx_buf[1] = pd_header_byte1_id(num_do, ext, pd_tx_msg_id_for_sop(sop), sop);
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
