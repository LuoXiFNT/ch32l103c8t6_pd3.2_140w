/**
 * @file pd_cc.c
 * @brief CC attach/detach detection and connection debounce state machine.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-06
 */

#include "pd_internal.h"
#include "ch32l103_init.h"

/** @brief Channel that carries VCONN: the CC line the contract is not using. */
static ch211_channel_t pd_cc_vconn_channel(void)
{
    return (s_pd.cc == 2u) ? CH211_CHANNEL_1 : CH211_CHANNEL_2;
}

ch211_status_t pd_cc_vconn_enable(void)
{
    return CH211_I2C_DMA_SetCcChannel(pd_cc_vconn_channel(), true, true, false, true);
}

bool pd_cc_vconn_busy(void)
{
    return (CH211_I2C_DMA_IsBusy() != 0u);
}

bool pd_frontend_is_ready(void)
{
    return (s_frontend_state == PD_FRONTEND_READY);
}


void pd_frontend_restart(void)
{
    CH211_I2C_DMA_Abort();
    s_frontend_state = PD_FRONTEND_CLOSE_HVCP;
    s_frontend_last_state = PD_FRONTEND_CLOSE_HVCP;
    s_frontend_wait_ms = 0u;
}


void pd_frontend_task(uint8_t elapsed_ms)
{
    if (s_frontend_state != s_frontend_last_state) {
        s_frontend_last_state = s_frontend_state;
        s_frontend_wait_ms = 0u;
    } else if (s_frontend_state != PD_FRONTEND_READY) {
        s_frontend_wait_ms = (uint16_t)(s_frontend_wait_ms + elapsed_ms);
        if (s_frontend_wait_ms >= PD_FRONTEND_STATE_TIMEOUT_MS) {
            pd_frontend_restart();
        }
    }

    switch (s_frontend_state) {
    case PD_FRONTEND_CLOSE_HVCP:
        if (CH211_I2C_DMA_SetHvcpLow(true) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CLOSE_HVCP;
        }
        break;

    case PD_FRONTEND_WAIT_CLOSE_HVCP:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_frontend_state = PD_FRONTEND_CC1_RD_OFF;
        }
        break;

    case PD_FRONTEND_CC1_RD_OFF:
        if (CH211_I2C_DMA_EnableCcRd(CH211_CHANNEL_1, false) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC1_RD_OFF;
        }
        break;

    case PD_FRONTEND_WAIT_CC1_RD_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_frontend_state = PD_FRONTEND_CC2_RD_OFF;
        }
        break;

    case PD_FRONTEND_CC2_RD_OFF:
        if (CH211_I2C_DMA_EnableCcRd(CH211_CHANNEL_2, false) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC2_RD_OFF;
        }
        break;

    case PD_FRONTEND_WAIT_CC2_RD_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_frontend_state = PD_FRONTEND_CC1_VCONN_OFF;
        }
        break;

    case PD_FRONTEND_CC1_VCONN_OFF:
        if (CH211_I2C_DMA_EnableVconn(CH211_CHANNEL_1, false) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC1_VCONN_OFF;
        }
        break;

    case PD_FRONTEND_WAIT_CC1_VCONN_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_frontend_state = PD_FRONTEND_CC2_VCONN_OFF;
        }
        break;

    case PD_FRONTEND_CC2_VCONN_OFF:
        if (CH211_I2C_DMA_EnableVconn(CH211_CHANNEL_2, false) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC2_VCONN_OFF;
        }
        break;

    case PD_FRONTEND_WAIT_CC2_VCONN_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_frontend_state = PD_FRONTEND_CC1_PATH_ON;
        }
        break;

    case PD_FRONTEND_CC1_PATH_ON:
        if (CH211_I2C_DMA_EnableCcPath(CH211_CHANNEL_1, true) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC1_PATH_ON;
        }
        break;

    case PD_FRONTEND_WAIT_CC1_PATH_ON:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_frontend_state = PD_FRONTEND_CC2_PATH_ON;
        }
        break;

    case PD_FRONTEND_CC2_PATH_ON:
        if (CH211_I2C_DMA_EnableCcPath(CH211_CHANNEL_2, true) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC2_PATH_ON;
        }
        break;

    case PD_FRONTEND_WAIT_CC2_PATH_ON:
        if (CH211_I2C_DMA_IsBusy() == 0u) {
            s_frontend_state = PD_FRONTEND_READY;
        }
        break;

    default:
        break;
    }
}

/** @brief Derive the active CC result from both front-end measurements. */
static uint8_t pd_calc_cc_result(uint8_t cc1, uint8_t cc2)
{
    uint8_t ret = 0u;

    if (((cc1 & bCC_CMP_66) != 0u) && ((cc1 & bCC_CMP_220) == 0u)) {
        if ((((cc2 & bCC_CMP_22) != 0u) && ((cc2 & bCC_CMP_66) == 0u)) ||
            ((cc2 & bCC_CMP_220) != 0u)) {
            ret = 1u;
        }
    }
    if (((cc2 & bCC_CMP_66) != 0u) && ((cc2 & bCC_CMP_220) == 0u)) {
        if (ret != 0u) {
            ret = 0u;
        } else if ((((cc1 & bCC_CMP_22) != 0u) && ((cc1 & bCC_CMP_66) == 0u)) ||
                   ((cc1 & bCC_CMP_220) != 0u)) {
            ret = 2u;
        }
    }
    return ret;
}

/** @brief Check whether the selected CC line remains attached. */
static bool pd_selected_cc_is_attached(void)
{
    uint8_t flags = (s_pd.cc == 2u) ? s_cc.cc2_flags : s_cc.cc1_flags;

    return (((flags & bCC_CMP_66) != 0u) && ((flags & bCC_CMP_220) == 0u));
}

/** @brief Reset PD state after a debounced cable detach. */
static void pd_detach_confirmed(void)
{
    s_cc.stable_count = 0u;
    s_cc.check_ms = 0u;
    s_pd.connected = false;
    pd_clear_contract_info();
    pd_reset_context(true);
}

/** @brief Poll and debounce CC measurements while attached or unattached. */
static void pd_cc_detect_task(void)
{
    switch (s_cc.phase) {
    case 0:
        s_cc.cc1_flags = 0u;
        s_cc.cc2_flags = 0u;
        USBPD->PORT_CC1 &= (uint16_t)(~(CC_CE | PA_CC_AI));
        USBPD->PORT_CC1 |= CC_CMP_22;
        s_cc.phase = 1u;
        break;

    case 1:
        if ((USBPD->PORT_CC1 & PA_CC_AI) != 0u) s_cc.cc1_flags |= bCC_CMP_22;
        USBPD->PORT_CC1 &= (uint16_t)(~(CC_CE | PA_CC_AI));
        USBPD->PORT_CC1 |= CC_CMP_66;
        s_cc.phase = 2u;
        break;

    case 2:
        if ((USBPD->PORT_CC1 & PA_CC_AI) != 0u) s_cc.cc1_flags |= bCC_CMP_66;
        if ((PD_CC_GPIO_PORT->INDR & PD_CC1_GPIO_PIN) != 0u) s_cc.cc1_flags |= bCC_CMP_220;
        USBPD->PORT_CC2 &= (uint16_t)(~(CC_CE | PA_CC_AI));
        USBPD->PORT_CC2 |= CC_CMP_22;
        s_cc.phase = 3u;
        break;

    case 3:
        if ((USBPD->PORT_CC2 & PA_CC_AI) != 0u) s_cc.cc2_flags |= bCC_CMP_22;
        USBPD->PORT_CC2 &= (uint16_t)(~(CC_CE | PA_CC_AI));
        USBPD->PORT_CC2 |= CC_CMP_66;
        s_cc.phase = 4u;
        break;

    default:
        if ((USBPD->PORT_CC2 & PA_CC_AI) != 0u) s_cc.cc2_flags |= bCC_CMP_66;
        if ((PD_CC_GPIO_PORT->INDR & PD_CC2_GPIO_PIN) != 0u) s_cc.cc2_flags |= bCC_CMP_220;
        s_cc.result = pd_calc_cc_result(s_cc.cc1_flags, s_cc.cc2_flags);
        s_cc.ready = true;
        s_cc.phase = 0u;
        break;
    }
}


void pd_connection_task(uint8_t elapsed_ms)
{
    uint8_t cc;

    if (s_pd.connected) {
        if (s_cc.check_ms < 0xFFFFu) {
            s_cc.check_ms = (uint16_t)(s_cc.check_ms + elapsed_ms);
        }

        /* The check rewrites the CC comparators and re-arms the receiver over several
         * millisecond slices, so keep it out of the window where an acknowledgement is
         * waiting to be sent or is on the wire. Checking again in a moment is harmless:
         * check_ms keeps counting and the detection simply starts later. */
        if (!pd_tx_is_idle() || s_goodcrc_tx_pending || s_goodcrc_tx_active) {
            return;
        }
        if ((s_pd.state != PD_STATE_READY) &&
            (s_rx_msg_pending) &&
            (s_cc.phase == 0u)) {
            return;
        }
        if ((s_cc.phase == 0u) &&
            (!s_cc.ready) &&
            (s_cc.check_ms < PD_CONNECTION_CHECK_MS)) {
            return;
        }
        pd_cc_detect_task();
        if (!s_cc.ready) {
            return;
        }
        s_cc.ready = false;
        s_cc.check_ms = 0u;
        if (pd_selected_cc_is_attached()) {
            s_cc.stable_count = 0u;
            pd_select_cc(s_pd.cc);
            pd_set_phy_rx_mode();
        } else if (++s_cc.stable_count >= PD_CC_DEBOUNCE_DETACH) {
            pd_detach_confirmed();
        } else {
            pd_select_cc(s_pd.cc);
            pd_set_phy_rx_mode();
        }
        return;
    }

    pd_cc_detect_task();
    if (!s_cc.ready) {
        return;
    }
    s_cc.ready = false;
    cc = s_cc.result;

    if (cc == 0u) {
        s_cc.stable_count = 0u;
    } else if (++s_cc.stable_count >= PD_CC_DEBOUNCE_ATTACH) {
        s_cc.stable_count = 0u;
        s_pd.connected = true;
        pd_select_cc(cc);
        pd_cable_start_discovery();
        if (!pd_fault_active()) {
            pd_power_request(true, PD_VSAFE5V_MV, PD_VSAFE5V_MA);
        } else {
            pd_power_request(false, 0u, 0u);
        }
        pd_set_state(PD_STATE_ATTACHED);
    }
}
