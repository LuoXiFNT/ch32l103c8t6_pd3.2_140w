/**
 * @file pd_cc.c
 * @brief CC attach/detach detection and connection debounce state machine.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "pd_internal.h"
#include "ch32l103_init.h"

uint8_t pd_frontend_is_ready(void)
{
    return (s_frontend_state == PD_FRONTEND_READY) ? 1U : 0U;
}


void pd_frontend_restart(void)
{
    CH211_I2C_DMA_Abort();
    s_frontend_state = PD_FRONTEND_CLOSE_HVCP;
    s_frontend_last_state = PD_FRONTEND_CLOSE_HVCP;
    s_frontend_wait_ms = 0U;
}


void pd_frontend_task(uint8_t elapsed_ms)
{
    if (s_frontend_state != s_frontend_last_state) {
        s_frontend_last_state = s_frontend_state;
        s_frontend_wait_ms = 0U;
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
        if (CH211_I2C_DMA_IsBusy() == 0U) {
            s_frontend_state = PD_FRONTEND_CC1_RD_OFF;
        }
        break;

    case PD_FRONTEND_CC1_RD_OFF:
        if (CH211_I2C_DMA_EnableCcRd(CH211_CHANNEL_1, false) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC1_RD_OFF;
        }
        break;

    case PD_FRONTEND_WAIT_CC1_RD_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0U) {
            s_frontend_state = PD_FRONTEND_CC2_RD_OFF;
        }
        break;

    case PD_FRONTEND_CC2_RD_OFF:
        if (CH211_I2C_DMA_EnableCcRd(CH211_CHANNEL_2, false) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC2_RD_OFF;
        }
        break;

    case PD_FRONTEND_WAIT_CC2_RD_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0U) {
            s_frontend_state = PD_FRONTEND_CC1_VCONN_OFF;
        }
        break;

    case PD_FRONTEND_CC1_VCONN_OFF:
        if (CH211_I2C_DMA_EnableVconn(CH211_CHANNEL_1, false) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC1_VCONN_OFF;
        }
        break;

    case PD_FRONTEND_WAIT_CC1_VCONN_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0U) {
            s_frontend_state = PD_FRONTEND_CC2_VCONN_OFF;
        }
        break;

    case PD_FRONTEND_CC2_VCONN_OFF:
        if (CH211_I2C_DMA_EnableVconn(CH211_CHANNEL_2, false) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC2_VCONN_OFF;
        }
        break;

    case PD_FRONTEND_WAIT_CC2_VCONN_OFF:
        if (CH211_I2C_DMA_IsBusy() == 0U) {
            s_frontend_state = PD_FRONTEND_CC1_PATH_ON;
        }
        break;

    case PD_FRONTEND_CC1_PATH_ON:
        if (CH211_I2C_DMA_EnableCcPath(CH211_CHANNEL_1, true) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC1_PATH_ON;
        }
        break;

    case PD_FRONTEND_WAIT_CC1_PATH_ON:
        if (CH211_I2C_DMA_IsBusy() == 0U) {
            s_frontend_state = PD_FRONTEND_CC2_PATH_ON;
        }
        break;

    case PD_FRONTEND_CC2_PATH_ON:
        if (CH211_I2C_DMA_EnableCcPath(CH211_CHANNEL_2, true) == CH211_OK) {
            s_frontend_state = PD_FRONTEND_WAIT_CC2_PATH_ON;
        }
        break;

    case PD_FRONTEND_WAIT_CC2_PATH_ON:
        if (CH211_I2C_DMA_IsBusy() == 0U) {
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
    uint8_t ret = 0U;

    if (((cc1 & bCC_CMP_66) != 0U) && ((cc1 & bCC_CMP_220) == 0U)) {
        if ((((cc2 & bCC_CMP_22) != 0U) && ((cc2 & bCC_CMP_66) == 0U)) ||
            ((cc2 & bCC_CMP_220) != 0U)) {
            ret = 1U;
        }
    }
    if (((cc2 & bCC_CMP_66) != 0U) && ((cc2 & bCC_CMP_220) == 0U)) {
        if (ret != 0U) {
            ret = 0U;
        } else if ((((cc1 & bCC_CMP_22) != 0U) && ((cc1 & bCC_CMP_66) == 0U)) ||
                   ((cc1 & bCC_CMP_220) != 0U)) {
            ret = 2U;
        }
    }
    return ret;
}

/** @brief Check whether the selected CC line remains attached. */
static uint8_t pd_selected_cc_is_attached(void)
{
    uint8_t flags = (s_pd.cc == 2U) ? s_cc.cc2_flags : s_cc.cc1_flags;

    return (((flags & bCC_CMP_66) != 0U) &&
            ((flags & bCC_CMP_220) == 0U)) ? 1U : 0U;
}

/** @brief Reset PD state after a debounced cable detach. */
static void pd_detach_confirmed(void)
{
    s_cc.stable_count = 0U;
    s_cc.check_ms = 0U;
    s_pd.connected = 0U;
    pd_clear_contract_info();
    pd_reset_context(1U);
}

/** @brief Poll and debounce CC measurements while attached or unattached. */
static void pd_cc_detect_task(void)
{
    switch (s_cc.phase) {
    case 0:
        s_cc.cc1_flags = 0U;
        s_cc.cc2_flags = 0U;
        USBPD->PORT_CC1 &= (uint16_t)(~(CC_CE | PA_CC_AI));
        USBPD->PORT_CC1 |= CC_CMP_22;
        s_cc.phase = 1U;
        break;

    case 1:
        if ((USBPD->PORT_CC1 & PA_CC_AI) != 0U) s_cc.cc1_flags |= bCC_CMP_22;
        USBPD->PORT_CC1 &= (uint16_t)(~(CC_CE | PA_CC_AI));
        USBPD->PORT_CC1 |= CC_CMP_66;
        s_cc.phase = 2U;
        break;

    case 2:
        if ((USBPD->PORT_CC1 & PA_CC_AI) != 0U) s_cc.cc1_flags |= bCC_CMP_66;
        if ((PD_CC_GPIO_PORT->INDR & PD_CC1_GPIO_PIN) != 0U) s_cc.cc1_flags |= bCC_CMP_220;
        USBPD->PORT_CC2 &= (uint16_t)(~(CC_CE | PA_CC_AI));
        USBPD->PORT_CC2 |= CC_CMP_22;
        s_cc.phase = 3U;
        break;

    case 3:
        if ((USBPD->PORT_CC2 & PA_CC_AI) != 0U) s_cc.cc2_flags |= bCC_CMP_22;
        USBPD->PORT_CC2 &= (uint16_t)(~(CC_CE | PA_CC_AI));
        USBPD->PORT_CC2 |= CC_CMP_66;
        s_cc.phase = 4U;
        break;

    default:
        if ((USBPD->PORT_CC2 & PA_CC_AI) != 0U) s_cc.cc2_flags |= bCC_CMP_66;
        if ((PD_CC_GPIO_PORT->INDR & PD_CC2_GPIO_PIN) != 0U) s_cc.cc2_flags |= bCC_CMP_220;
        s_cc.result = pd_calc_cc_result(s_cc.cc1_flags, s_cc.cc2_flags);
        s_cc.ready = 1U;
        s_cc.phase = 0U;
        break;
    }
}


void pd_connection_task(uint8_t elapsed_ms)
{
    uint8_t cc;

    if (s_pd.connected != 0U) {
        if (s_cc.check_ms < 0xFFFFU) {
            s_cc.check_ms = (uint16_t)(s_cc.check_ms + elapsed_ms);
        }

        if (pd_tx_is_idle() == 0U) {
            return;
        }
        if ((s_pd.state != PD_STATE_READY) &&
            (s_rx_msg_pending != 0U) &&
            (s_cc.phase == 0U)) {
            return;
        }
        if ((s_cc.phase == 0U) &&
            (s_cc.ready == 0U) &&
            (s_cc.check_ms < PD_CONNECTION_CHECK_MS)) {
            return;
        }
        pd_cc_detect_task();
        if (s_cc.ready == 0U) {
            return;
        }
        s_cc.ready = 0U;
        s_cc.check_ms = 0U;
        if (pd_selected_cc_is_attached() != 0U) {
            s_cc.stable_count = 0U;
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
    if (s_cc.ready == 0U) {
        return;
    }
    s_cc.ready = 0U;
    cc = s_cc.result;

    if (cc == 0U) {
        s_cc.stable_count = 0U;
    } else if (++s_cc.stable_count >= PD_CC_DEBOUNCE_ATTACH) {
        s_cc.stable_count = 0U;
        s_pd.connected = 1U;
        pd_select_cc(cc);
        pd_cable_start_discovery();
        if (pd_fault_active() == 0U) {
            pd_power_request(1U, PD_VSAFE5V_MV, PD_VSAFE5V_MA);
        } else {
            pd_power_request(0U, 0U, 0U);
        }
        pd_set_state(PD_STATE_ATTACHED);
    }
}
