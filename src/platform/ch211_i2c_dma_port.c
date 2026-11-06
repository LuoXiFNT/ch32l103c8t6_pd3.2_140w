/**
 * @file ch211_i2c_dma_port.c
 * @brief I2C DMA port implementation for CH211 Type-C/PD high-voltage interface IC driver.
 * @author LuoXiFNT
 * @date 2026-07-04
 * @lastModified 2026-11-06
 */

#include "ch211_i2c_dma_port.h"

/** Global CH211 device instance. */
/** Device instance; only this port touches it. */
static ch211_t ch211;
static ch32_i2c_dma_t *s_ch211_iic_dma = 0;
/** 1 while the KEY-pin interrupt output still has to be programmed. */
static uint8_t s_int_pin_pending = 0u;

/**
 * @brief Adapt a CH211 register write to the shared I2C DMA queue.
 * @param user Pointer to the CH32 I2C DMA context.
 * @param dev_addr_7bit CH211 7-bit I2C address.
 * @param reg Register address to write.
 * @param data Pointer to the register data.
 * @param len Number of data bytes, limited to eight.
 * @return int 0 when queued successfully, -1 on invalid input or queue failure.
 */
static int CH211_I2C_DMA_WriteReg(void *user,
                                  uint8_t dev_addr_7bit,
                                  uint8_t reg,
                                  const uint8_t *data,
                                  uint16_t len)
{
    ch32_i2c_dma_t *iic_dma = (ch32_i2c_dma_t *)user;
    uint8_t frame[9];
    uint16_t i;

    if ((iic_dma == 0) || (data == 0) || (len > 8u)) {
        return -1;
    }

    frame[0] = reg;
    for (i = 0u; i < len; i++) {
        frame[1u + i] = data[i];
    }

    return (CH32_I2C_DMA_Write(iic_dma,
                               (uint16_t)(dev_addr_7bit << 1u),
                               frame,
                               (uint16_t)(len + 1u)) != 0u) ? 0 : -1;
}

/**
 * @brief Adapt a CH211 register read to the shared I2C DMA queue.
 * @param user Pointer to the CH32 I2C DMA context.
 * @param dev_addr_7bit CH211 7-bit I2C address.
 * @param reg Register address to read.
 * @param data Pointer to receive the register data.
 * @param len Number of data bytes to read.
 * @return int 0 when queued successfully, -1 on invalid input or queue failure.
 */
static int CH211_I2C_DMA_ReadReg(void *user,
                                 uint8_t dev_addr_7bit,
                                 uint8_t reg,
                                 uint8_t *data,
                                 uint16_t len)
{
    ch32_i2c_dma_t *iic_dma = (ch32_i2c_dma_t *)user;

    /* Same bound as the write path: the shared queue holds one register address plus
     * at most eight data bytes per transaction. */
    if ((iic_dma == 0) || (data == 0) || (len == 0u) || (len > 8u)) {
        return -1;
    }

    return (CH32_I2C_DMA_Read(iic_dma,
                                 (uint16_t)(dev_addr_7bit << 1u),
                                 reg,
                                 data,
                                 len) != 0u) ? 0 : -1;
}

/**
 * @brief Report the shared I2C DMA queue idle state to the CH211 driver.
 * @param user Pointer to the CH32 I2C DMA context.
 * @return true when the queue has drained.
 */
static bool CH211_I2C_DMA_IsIdle(void *user)
{
    return CH32_I2C_DMA_IsIdle((ch32_i2c_dma_t *)user);
}

ch211_status_t CH211_I2C_DMA_InitDevice(ch211_t *dev,
                                        ch32_i2c_dma_t *iic_dma,
                                        uint8_t dev_addr_7bit)
{
    ch211_status_t st = ch211_init(dev,
                                   dev_addr_7bit,
                                   CH211_I2C_DMA_WriteReg,
                                   CH211_I2C_DMA_ReadReg,
                                   CH211_I2C_DMA_IsIdle,
                                   iic_dma);

    if (st != CH211_OK) {
        /* Do not publish a half-configured port. */
        s_ch211_iic_dma = 0;
        return st;
    }

    s_ch211_iic_dma = iic_dma;
    return CH211_OK;
}

ch211_status_t CH211_I2C_DMA_Init(ch32_i2c_dma_t *iic_dma,
                                  uint8_t dev_addr_7bit)
{
    ch211_status_t st = CH211_I2C_DMA_InitDevice(&ch211, iic_dma, dev_addr_7bit);

    if (st != CH211_OK) {
        return st;
    }

    /* CC_CTRL resets with both Sink Rd pull-downs enabled. This port is a
     * Source, so they must be cleared once before any CC signalling. */
    st = ch211_force_cc_rd_off(&ch211);
    if (st != CH211_OK) {
        return st;
    }

    /* The board routes the CH211 interrupt request to the KEY pin (INT_PIN=11),
     * while PIN_CFG resets to 00 (interrupt output off). Only one operation can
     * be in flight, so the pin is programmed by the task once this one drains.
     * KEY_OE and KEY_PD must stay 0: the KEY pin carries that interrupt output
     * through an external level shifter, so driving it low or adding a pull-down
     * would jam the interrupt line. */
    s_int_pin_pending = 1u;
    return CH211_OK;
}


ch211_status_t CH211_I2C_DMA_InitDefault(ch32_i2c_dma_t *iic_dma)
{
    return CH211_I2C_DMA_Init(iic_dma, CH211_DEFAULT_ADDRESS_7BIT);
}


void CH211_I2C_DMA_Task(void)
{
    if ((s_ch211_iic_dma != 0) && (CH32_I2C_DMA_HadError(s_ch211_iic_dma))) {
        ch211_abort(&ch211);
    }

    /* Finish the boot configuration: program the KEY pin as the interrupt
     * output once the Rd-clear read-modify-write has drained. */
    if ((s_int_pin_pending != 0u) && (!ch211_is_busy(&ch211))) {
        if (ch211_set_interrupt_pin(&ch211, CH211_INT_PIN_KEY) != CH211_BUSY) {
            s_int_pin_pending = 0u;
        }
    }

    (void)ch211_task(&ch211);
}


void CH211_I2C_DMA_Abort(void)
{
    ch211_abort(&ch211);
}


ch211_status_t CH211_I2C_DMA_GetSysStatus(ch211_sys_status_t *status)
{
    return ch211_get_sys_status(&ch211, status);
}



ch211_status_t CH211_I2C_DMA_EnableHvcpAuto(bool enable)
{
    return ch211_enable_hvcp_auto(&ch211, enable);
}

ch211_status_t CH211_I2C_DMA_SetHvcpLow(bool strong_pull_down)
{
    return ch211_set_hvcp_low(&ch211, strong_pull_down);
}


ch211_status_t CH211_I2C_DMA_SetCcChannel(ch211_channel_t channel,
                                          bool cch_connect_oe,
                                          bool ccl_connect_ge,
                                          bool rd_enable,
                                          bool vconn_enable)
{
    return ch211_set_cc_channel(&ch211,
                                channel,
                                cch_connect_oe,
                                ccl_connect_ge,
                                rd_enable,
                                vconn_enable);
}

ch211_status_t CH211_I2C_DMA_EnableCcPath(ch211_channel_t channel, bool enable)
{
    return ch211_enable_cc_path(&ch211, channel, enable);
}

ch211_status_t CH211_I2C_DMA_EnableCcRd(ch211_channel_t channel, bool enable)
{
    return ch211_enable_cc_rd(&ch211, channel, enable);
}

ch211_status_t CH211_I2C_DMA_EnableVconn(ch211_channel_t channel, bool enable)
{
    return ch211_enable_vconn(&ch211, channel, enable);
}

ch211_status_t CH211_I2C_DMA_SetVbusDischarge(bool enable)
{
    return ch211_set_vbus_discharge(&ch211, enable);
}

bool CH211_I2C_DMA_IsBusy(void)
{
    return ch211_is_busy(&ch211);
}
