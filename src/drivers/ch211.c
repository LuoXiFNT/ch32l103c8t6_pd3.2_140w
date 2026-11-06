/**
 * @file ch211.c
 * @brief Platform-independent non-blocking CH211 register driver implementation.
 * @author LuoXiFNT
 * @date 2026-07-04
 * @lastModified 2026-11-04
 */

#include "ch211.h"

#define CH211_OP_IDLE             (0u) /**< No asynchronous operation. */
#define CH211_OP_UPDATE_READ      (1u) /**< Read phase of read-modify-write. */
#define CH211_OP_UPDATE_WRITE     (2u) /**< Write phase of read-modify-write. */
#define CH211_OP_READ_REGISTER    (3u) /**< Direct register read. */
#define CH211_OP_GET_PIN_STAT     (4u) /**< PIN_STAT read. */
#define CH211_OP_GET_SYS_STAT     (5u) /**< SYS_STAT read. */
#define CH211_OP_GET_CC_CTRL      (6u) /**< CC_CTRL read. */
#define CH211_OP_GET_PIN_CFG      (7u) /**< PIN_CFG read. */
#define CH211_OP_GET_OD_CTRL      (8u) /**< OD_CTRL read. */
#define CH211_OP_GET_HVCP_CTRL    (9u) /**< HVCP_CTRL read. */
#define CH211_OP_GET_HVIO_KEY     (10u) /**< HVIO_KEY read. */
#define CH211_OP_GET_SYS_CFG      (11u) /**< SYS_CFG read. */
#define CH211_OP_READ_REGISTERS   (12u) /**< Burst register read. */

/**
 * @brief Validate that the device and write callback are non-null.
 * @param dev Pointer to the CH211 device instance.
 * @return ch211_status_t CH211_OK if valid, CH211_ERROR_NULL otherwise.
 */
static ch211_status_t ch211_check_write_dev(ch211_t *dev)
{
    if ((dev == 0) || (dev->write_reg == 0)) {
        return CH211_ERROR_NULL;
    }
    return CH211_OK;
}

/**
 * @brief Validate that the device, read callback, and idle callback are non-null.
 * @param dev Pointer to the CH211 device instance.
 * @return ch211_status_t CH211_OK if valid, CH211_ERROR_NULL otherwise.
 */
static ch211_status_t ch211_check_read_dev(ch211_t *dev)
{
    if ((dev == 0) || (dev->read_reg == 0) || (dev->is_bus_idle == 0)) {
        return CH211_ERROR_NULL;
    }
    return CH211_OK;
}

/**
 * @brief Check whether the I2C bus is idle via the registered callback.
 * @param dev Pointer to the CH211 device instance.
 * @return true when idle or no callback is installed.
 */
static uint8_t ch211_bus_is_idle(ch211_t *dev)
{
    if ((dev == 0) || (dev->is_bus_idle == 0)) {
        return 1u;
    }
    return dev->is_bus_idle(dev->user);
}

/**
 * @brief Validate a CH211 register address.
 * @param reg Register address to validate.
 * @return ch211_status_t CH211_OK if supported, CH211_ERROR_RANGE otherwise.
 */
static ch211_status_t ch211_validate_reg(uint8_t reg)
{
    return (reg <= CH211_REG_SYS_STAT) ? CH211_OK : CH211_ERROR_RANGE;
}

/**
 * @brief Polling-style single-register read: report the latched result or start
 *        a new asynchronous read. Call it repeatedly until CH211_OK is returned.
 * @param dev Pointer to the CH211 device instance.
 * @param op Async operation code used to identify completion.
 * @param reg Register address to read.
 * @param data Pointer to receive the register value.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY while pending,
 *         or an error code.
 */
static ch211_status_t ch211_poll_read(ch211_t *dev,
                                      uint8_t op,
                                      uint8_t reg,
                                      uint8_t *data)
{
    ch211_status_t st = ch211_check_read_dev(dev);

    if (st != CH211_OK) {
        return st;
    }

    if (data == 0) {
        return CH211_ERROR_NULL;
    }

    if (ch211_validate_reg(reg) != CH211_OK) {
        return CH211_ERROR_RANGE;
    }

    if (dev->async_op == op) {
        if (dev->poll_ready == 0u) {
            return CH211_BUSY;
        }
        *data = dev->io_data[0];
        dev->async_op = CH211_OP_IDLE;
        dev->poll_ready = 0u;
        return CH211_OK;
    }

    if (dev->async_op != CH211_OP_IDLE) {
        return CH211_BUSY;
    }

    if (ch211_bus_is_idle(dev) == 0u) {
        return CH211_BUSY;
    }

    if (dev->read_reg(dev->user, dev->dev_addr_7bit, reg, &dev->io_data[0], 1u) != 0) {
        return CH211_ERROR;
    }

    dev->direct_read_reg = reg;
    dev->direct_read_len = 1u;
    dev->async_op = op;
    dev->poll_ready = 0u;
    return CH211_BUSY;
}

/**
 * @brief Read a block of consecutive registers with one transfer.
 *
 * The CH211 increments the register address after every byte, so one
 * transaction can return several registers. Poll it until CH211_OK; the
 * destination buffer must stay valid until then.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param reg First register address, 0x00 to 0x07.
 * @param data Destination buffer for the block.
 * @param len Number of registers to read, at most 8.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY while pending,
 *         error code otherwise.
 */
ch211_status_t ch211_read_registers(ch211_t *dev, uint8_t reg,
                                    uint8_t *data, uint8_t len)
{
    ch211_status_t st = ch211_check_read_dev(dev);
    uint8_t i;

    if (st != CH211_OK) {
        return st;
    }

    if (data == 0) {
        return CH211_ERROR_NULL;
    }

    if ((len == 0u) || (len > sizeof(dev->io_data))) {
        return CH211_ERROR_RANGE;
    }

    if (((uint16_t)reg + (uint16_t)len) > (uint16_t)(CH211_REG_SYS_STAT + 1u)) {
        return CH211_ERROR_RANGE;
    }

    if (dev->async_op == CH211_OP_READ_REGISTERS) {
        if ((dev->direct_read_reg != reg) || (dev->direct_read_len != len)) {
            return CH211_BUSY;
        }
        if (dev->poll_ready == 0u) {
            return CH211_BUSY;
        }
        for (i = 0u; i < len; i++) {
            data[i] = dev->io_data[i];
        }
        dev->async_op = CH211_OP_IDLE;
        dev->poll_ready = 0u;
        return CH211_OK;
    }

    if (dev->async_op != CH211_OP_IDLE) {
        return CH211_BUSY;
    }

    if (ch211_bus_is_idle(dev) == 0u) {
        return CH211_BUSY;
    }

    if (dev->read_reg(dev->user, dev->dev_addr_7bit, reg, dev->io_data, len) != 0) {
        return CH211_ERROR;
    }

    dev->direct_read_reg = reg;
    dev->direct_read_len = len;
    dev->async_op = CH211_OP_READ_REGISTERS;
    dev->poll_ready = 0u;
    return CH211_BUSY;
}

ch211_status_t ch211_init(ch211_t *dev,
                          uint8_t dev_addr_7bit,
                          ch211_write_reg_cb_t write_reg,
                          ch211_read_reg_cb_t read_reg,
                          ch211_bus_idle_cb_t is_bus_idle,
                          void *user)
{
    if ((dev == 0) || (write_reg == 0)) {
        return CH211_ERROR_NULL;
    }

    if (dev_addr_7bit != CH211_ADDRESS_NORMAL_7BIT) {
        return CH211_ERROR_RANGE;
    }

    dev->dev_addr_7bit = dev_addr_7bit;
    dev->write_reg = write_reg;
    dev->read_reg = read_reg;
    dev->is_bus_idle = is_bus_idle;
    dev->user = user;
    dev->async_op = CH211_OP_IDLE;
    dev->update_reg = 0u;
    dev->update_mask = 0u;
    dev->update_value = 0u;
    dev->direct_read_reg = 0u;
    dev->direct_read_len = 0u;
    dev->poll_ready = 0u;
    for (uint8_t i = 0u; i < sizeof(dev->io_data); i++) {
        dev->io_data[i] = 0u;
    }

    return CH211_OK;
}

ch211_status_t ch211_task(ch211_t *dev)
{
    uint8_t data;
    ch211_status_t st;

    if (dev == 0) {
        return CH211_ERROR_NULL;
    }

    if (dev->async_op == CH211_OP_IDLE) {
        return CH211_OK;
    }

    /* Every pending operation completes when the queued transfer drains.
     * Latch that edge so unrelated bus activity cannot hide it. */
    if (dev->poll_ready == 0u) {
        if (ch211_bus_is_idle(dev) == 0u) {
            return CH211_BUSY;
        }
        dev->poll_ready = 1u;
    }

    if (dev->async_op == CH211_OP_UPDATE_READ) {
        data = (uint8_t)((dev->io_data[0] & (uint8_t)(~dev->update_mask)) |
                         (dev->update_value & dev->update_mask));

        st = ch211_write_register(dev, dev->update_reg, data);
        if (st != CH211_OK) {
            dev->async_op = CH211_OP_IDLE;
            dev->poll_ready = 0u;
            return st;
        }

        dev->async_op = CH211_OP_UPDATE_WRITE;
        dev->poll_ready = 0u;
        return CH211_BUSY;
    }

    if (dev->async_op == CH211_OP_UPDATE_WRITE) {
        dev->async_op = CH211_OP_IDLE;
        dev->poll_ready = 0u;
        return CH211_OK;
    }

    /* Read operation finished: keep poll_ready set for the polling caller. */
    return CH211_BUSY;
}

bool ch211_is_busy(const ch211_t *dev)
{
    if (dev == 0) {
        return false;
    }
    return dev->async_op != CH211_OP_IDLE;
}

ch211_status_t ch211_write_register(ch211_t *dev, uint8_t reg, uint8_t data)
{
    ch211_status_t st = ch211_check_write_dev(dev);
    if (st != CH211_OK) {
        return st;
    }

    if (ch211_validate_reg(reg) != CH211_OK) {
        return CH211_ERROR_RANGE;
    }

    /* Fire-and-forget like the MP2980 driver: the transfer is queued and the
     * shared queue keeps the ordering, so a busy bus is not an error here. */
    return (dev->write_reg(dev->user, dev->dev_addr_7bit, reg, &data, 1u) == 0) ? CH211_OK : CH211_ERROR;
}

ch211_status_t ch211_read_register(ch211_t *dev, uint8_t reg, uint8_t *data)
{
    if ((dev != 0) && (dev->async_op == CH211_OP_READ_REGISTER) &&
        (dev->direct_read_reg != reg)) {
        return CH211_BUSY;
    }
    return ch211_poll_read(dev, CH211_OP_READ_REGISTER, reg, data);
}

ch211_status_t ch211_update_bits(ch211_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    ch211_status_t st = ch211_check_read_dev(dev);
    if (st != CH211_OK) {
        return st;
    }

    if (ch211_validate_reg(reg) != CH211_OK) {
        return CH211_ERROR_RANGE;
    }

    if (dev->async_op != CH211_OP_IDLE) {
        return CH211_BUSY;
    }

    if (ch211_bus_is_idle(dev) == 0u) {
        return CH211_BUSY;
    }

    if (dev->read_reg(dev->user, dev->dev_addr_7bit, reg, &dev->io_data[0], 1u) != 0) {
        return CH211_ERROR;
    }

    dev->update_reg = reg;
    dev->update_mask = mask;
    dev->update_value = value;
    dev->async_op = CH211_OP_UPDATE_READ;
    return CH211_OK;
}

ch211_status_t ch211_write_registers(ch211_t *dev, uint8_t reg,
                                     const uint8_t *data, uint8_t len)
{
    ch211_status_t st = ch211_check_write_dev(dev);

    if (st != CH211_OK) {
        return st;
    }

    if (data == 0) {
        return CH211_ERROR_NULL;
    }

    if ((len == 0u) || (len > 8u)) {
        return CH211_ERROR_RANGE;
    }

    /* The device increments the address after every byte, so the block must
     * stay inside the register file. */
    if (((uint16_t)reg + (uint16_t)len) > (uint16_t)(CH211_REG_SYS_STAT + 1u)) {
        return CH211_ERROR_RANGE;
    }

    if (dev->write_reg(dev->user, dev->dev_addr_7bit, reg, data, len) != 0) {
        return CH211_ERROR;
    }

    return CH211_OK;
}

ch211_status_t ch211_get_pin_status(ch211_t *dev, ch211_pin_status_t *status)
{
    uint8_t raw;
    ch211_status_t st;

    if (status == 0) {
        return CH211_ERROR_NULL;
    }

    st = ch211_poll_read(dev, CH211_OP_GET_PIN_STAT, CH211_REG_PIN_STAT, &raw);
    if (st != CH211_OK) {
        return st;
    }

    status->raw = raw;
    status->cci2 = ((raw & CH211_PIN_STAT_CCI2) != 0u);
    status->cci1 = ((raw & CH211_PIN_STAT_CCI1) != 0u);
    status->vbus_over_voltage = ((raw & CH211_PIN_STAT_VBUS_OV) != 0u);
    status->vbus_ready = ((raw & CH211_PIN_STAT_VBUS_RDY) != 0u);
    status->hvio_high_threshold = ((raw & CH211_PIN_STAT_HVHI) != 0u);
    status->hvio_low_threshold = ((raw & CH211_PIN_STAT_HVLI) != 0u);
    status->key_high_threshold = ((raw & CH211_PIN_STAT_KEYHI) != 0u);
    status->key_low_threshold = ((raw & CH211_PIN_STAT_KEYLI) != 0u);
    return CH211_OK;
}

ch211_status_t ch211_get_sys_status(ch211_t *dev, ch211_sys_status_t *status)
{
    uint8_t raw;
    ch211_status_t st;

    if (status == 0) {
        return CH211_ERROR_NULL;
    }

    st = ch211_poll_read(dev, CH211_OP_GET_SYS_STAT, CH211_REG_SYS_STAT, &raw);
    if (st != CH211_OK) {
        return st;
    }

    status->raw = raw;
    status->ldo33_off = ((raw & CH211_SYS_STAT_LDO33_OFF) != 0u);
    status->over_temperature_reset = ((raw & CH211_SYS_STAT_OT_RST) != 0u);
    status->vbus_over_voltage = ((raw & CH211_SYS_STAT_VBUS_OV) != 0u);
    status->vbus_ready = ((raw & CH211_SYS_STAT_VBUS_RDY) != 0u);
    status->vbus_last = ((raw & CH211_SYS_STAT_VBUS_LAST) != 0u);
    status->vbus_exist = ((raw & CH211_SYS_STAT_VBUS_EXIST) != 0u);
    status->vsys_exist = ((raw & CH211_SYS_STAT_VSYS_EXIST) != 0u);
    return CH211_OK;
}

ch211_status_t ch211_set_sda_pullup(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_PIN_CFG,
                             CH211_PIN_CFG_SDA_PU,
                             enable ? CH211_PIN_CFG_SDA_PU : 0u);
}

ch211_status_t ch211_set_interrupt_pin(ch211_t *dev, ch211_int_pin_t pin)
{
    if ((uint8_t)pin > 3u) {
        return CH211_ERROR_RANGE;
    }
    return ch211_update_bits(dev,
                             CH211_REG_PIN_CFG,
                             CH211_PIN_CFG_INT_PIN_MASK,
                             (uint8_t)pin);
}

ch211_status_t ch211_set_interrupt_enable_mask(ch211_t *dev, uint8_t interrupt_mask)
{
    const uint8_t mask = (uint8_t)(CH211_PIN_CFG_CC2_IE |
                                   CH211_PIN_CFG_CC1_IE |
                                   CH211_PIN_CFG_HVIO_IE |
                                   CH211_PIN_CFG_KEY_IE |
                                   CH211_PIN_CFG_VBUS_DOWN_IE);
    return ch211_update_bits(dev, CH211_REG_PIN_CFG, mask, (uint8_t)(interrupt_mask & mask));
}

ch211_status_t ch211_force_cc_rd_off(ch211_t *dev)
{
    /* The reset value of CC_CTRL leaves both Rd pull-downs enabled. A Source
     * port must never present them, so clear both in one read-modify-write
     * without touching OE/GE/VCE. */
    return ch211_update_bits(dev,
                             CH211_REG_CC_CTRL,
                             (uint8_t)(CH211_CC_CTRL_CC2_PD | CH211_CC_CTRL_CC1_PD),
                             0u);
}

ch211_status_t ch211_set_cc_control(ch211_t *dev, uint8_t cc_ctrl)
{
    /* Source port: refuse any value that would enable the Sink Rd pull-down. */
    if ((cc_ctrl & (uint8_t)(CH211_CC_CTRL_CC2_PD | CH211_CC_CTRL_CC1_PD)) != 0u) {
        return CH211_ERROR_RANGE;
    }

    return ch211_write_register(dev, CH211_REG_CC_CTRL, cc_ctrl);
}

ch211_status_t ch211_get_cc_control(ch211_t *dev, ch211_cc_control_t *cfg)
{
    uint8_t raw = 0u;
    ch211_status_t st;

    if (cfg == 0) {
        return CH211_ERROR_NULL;
    }

    st = ch211_poll_read(dev, CH211_OP_GET_CC_CTRL, CH211_REG_CC_CTRL, &raw);
    if (st != CH211_OK) {
        return st;
    }

    cfg->raw = raw;
    cfg->cc2_vconn = ((raw & CH211_CC_CTRL_CC2_VCE) != 0u);
    cfg->cc2_rd = ((raw & CH211_CC_CTRL_CC2_PD) != 0u);
    cfg->cc2_connect = ((raw & CH211_CC_CTRL_CC2_OE) != 0u);
    cfg->cc2_ground = ((raw & CH211_CC_CTRL_CC2_GE) != 0u);
    cfg->cc1_vconn = ((raw & CH211_CC_CTRL_CC1_VCE) != 0u);
    cfg->cc1_rd = ((raw & CH211_CC_CTRL_CC1_PD) != 0u);
    cfg->cc1_connect = ((raw & CH211_CC_CTRL_CC1_OE) != 0u);
    cfg->cc1_ground = ((raw & CH211_CC_CTRL_CC1_GE) != 0u);
    return CH211_OK;
}

ch211_status_t ch211_get_pin_cfg(ch211_t *dev, ch211_pin_cfg_t *cfg)
{
    uint8_t raw = 0u;
    ch211_status_t st;

    if (cfg == 0) {
        return CH211_ERROR_NULL;
    }

    st = ch211_poll_read(dev, CH211_OP_GET_PIN_CFG, CH211_REG_PIN_CFG, &raw);
    if (st != CH211_OK) {
        return st;
    }

    cfg->raw = raw;
    cfg->cc2_interrupt = ((raw & CH211_PIN_CFG_CC2_IE) != 0u);
    cfg->cc1_interrupt = ((raw & CH211_PIN_CFG_CC1_IE) != 0u);
    cfg->hvio_interrupt = ((raw & CH211_PIN_CFG_HVIO_IE) != 0u);
    cfg->key_interrupt = ((raw & CH211_PIN_CFG_KEY_IE) != 0u);
    cfg->vbus_down_interrupt = ((raw & CH211_PIN_CFG_VBUS_DOWN_IE) != 0u);
    cfg->sda_pullup = ((raw & CH211_PIN_CFG_SDA_PU) != 0u);
    cfg->int_pin = (uint8_t)(raw & CH211_PIN_CFG_INT_PIN_MASK);
    return CH211_OK;
}

ch211_status_t ch211_get_od_control(ch211_t *dev, ch211_od_control_t *cfg)
{
    uint8_t raw = 0u;
    ch211_status_t st;

    if (cfg == 0) {
        return CH211_ERROR_NULL;
    }

    st = ch211_poll_read(dev, CH211_OP_GET_OD_CTRL, CH211_REG_OD_CTRL, &raw);
    if (st != CH211_OK) {
        return st;
    }

    cfg->raw = raw;
    cfg->od2_connect = ((raw & CH211_OD_CTRL_OD2_OE) != 0u);
    cfg->od2_ccl = ((raw & CH211_OD_CTRL_OD2_GE) != 0u);
    cfg->od2_float_gnd = ((raw & CH211_OD_CTRL_OD2_PE) != 0u);
    cfg->od1_connect = ((raw & CH211_OD_CTRL_OD1_OE) != 0u);
    cfg->od1_ccl = ((raw & CH211_OD_CTRL_OD1_GE) != 0u);
    cfg->od1_float_gnd = ((raw & CH211_OD_CTRL_OD1_PE) != 0u);
    return CH211_OK;
}

ch211_status_t ch211_get_hvcp_control(ch211_t *dev, ch211_hvcp_control_t *cfg)
{
    uint8_t raw = 0u;
    ch211_status_t st;

    if (cfg == 0) {
        return CH211_ERROR_NULL;
    }

    st = ch211_poll_read(dev, CH211_OP_GET_HVCP_CTRL, CH211_REG_HVCP_CTRL, &raw);
    if (st != CH211_OK) {
        return st;
    }

    cfg->vbus_discharge = ((raw & CH211_HVCP_CTRL_VBUS_DISC) != 0u);
    cfg->auto_boost = ((raw & CH211_HVCP_CTRL_CP_AUTO) != 0u);
    cfg->manual_boost = ((raw & CH211_HVCP_CTRL_CP_AE) != 0u);
    cfg->manual_pullup = ((raw & CH211_HVCP_CTRL_CP_PU) != 0u);
    cfg->low_pull_down = ((raw & CH211_HVCP_CTRL_CP_LE) != 0u);
    cfg->low_drive_strong = ((raw & CH211_HVCP_CTRL_CP_LX) != 0u);
    return CH211_OK;
}

ch211_status_t ch211_get_hvio_key(ch211_t *dev, ch211_hvio_key_t *cfg)
{
    uint8_t raw = 0u;
    ch211_status_t st;

    if (cfg == 0) {
        return CH211_ERROR_NULL;
    }

    st = ch211_poll_read(dev, CH211_OP_GET_HVIO_KEY, CH211_REG_HVIO_KEY, &raw);
    if (st != CH211_OK) {
        return st;
    }

    cfg->raw = raw;
    cfg->key_pull_down = ((raw & CH211_HVIO_KEY_KEY_PD) != 0u);
    cfg->key_output_low = ((raw & CH211_HVIO_KEY_KEY_OE) != 0u);
    cfg->hvio_pullup = ((raw & CH211_HVIO_KEY_HV_PU) != 0u);
    cfg->hvio_pull_down = ((raw & CH211_HVIO_KEY_HV_PD) != 0u);
    cfg->hvio_output_low = ((raw & CH211_HVIO_KEY_HV_OE) != 0u);
    return CH211_OK;
}

ch211_status_t ch211_get_sys_cfg(ch211_t *dev, ch211_sys_cfg_t *cfg)
{
    uint8_t raw = 0u;
    ch211_status_t st;

    if (cfg == 0) {
        return CH211_ERROR_NULL;
    }

    st = ch211_poll_read(dev, CH211_OP_GET_SYS_CFG, CH211_REG_SYS_CFG, &raw);
    if (st != CH211_OK) {
        return st;
    }

    cfg->raw = raw;
    cfg->ldo33_off = ((raw & CH211_SYS_CFG_LDO33_OFF) != 0u);
    cfg->cc_high_threshold_3v3 = ((raw & CH211_SYS_CFG_CC_HVT3V) != 0u);
    cfg->auto_hvcp_on_fault = ((raw & CH211_SYS_CFG_CPLE_OVOT) != 0u);
    cfg->auto_reset_on_vbus_ov = ((raw & CH211_SYS_CFG_RST_OV) != 0u);
    cfg->ldo33_wake = ((raw & CH211_SYS_CFG_LDO33_WAKE) != 0u);
    cfg->ldo33_voltage = (uint8_t)(raw & CH211_SYS_CFG_LDO_VSEL_MASK);
    return CH211_OK;
}

ch211_status_t ch211_set_cc_channel(ch211_t *dev,
                                    ch211_channel_t channel,
                                    bool cch_connect_oe,
                                    bool ccl_connect_ge,
                                    bool rd_enable,
                                    bool vconn_enable)
{
    uint8_t mask;
    uint8_t value;

    /* Source port: the Sink Rd pull-down must never be enabled. The bit stays
     * in the mask so that a stray 1 left by the reset value is cleared. */
    if (rd_enable) {
        return CH211_ERROR_RANGE;
    }

    if (channel == CH211_CHANNEL_1) {
        mask = (uint8_t)(CH211_CC_CTRL_CC1_VCE | CH211_CC_CTRL_CC1_PD |
                         CH211_CC_CTRL_CC1_OE | CH211_CC_CTRL_CC1_GE);
        value = (uint8_t)((vconn_enable ? CH211_CC_CTRL_CC1_VCE : 0u) |
                          (cch_connect_oe ? CH211_CC_CTRL_CC1_OE : 0u) |
                          (ccl_connect_ge ? CH211_CC_CTRL_CC1_GE : 0u));
    } else if (channel == CH211_CHANNEL_2) {
        mask = (uint8_t)(CH211_CC_CTRL_CC2_VCE | CH211_CC_CTRL_CC2_PD |
                         CH211_CC_CTRL_CC2_OE | CH211_CC_CTRL_CC2_GE);
        value = (uint8_t)((vconn_enable ? CH211_CC_CTRL_CC2_VCE : 0u) |
                          (cch_connect_oe ? CH211_CC_CTRL_CC2_OE : 0u) |
                          (ccl_connect_ge ? CH211_CC_CTRL_CC2_GE : 0u));
    } else {
        return CH211_ERROR_RANGE;
    }

    return ch211_update_bits(dev, CH211_REG_CC_CTRL, mask, value);
}

ch211_status_t ch211_enable_cc_path(ch211_t *dev, ch211_channel_t channel, bool enable)
{
    uint8_t mask;
    uint8_t value;

    if (channel == CH211_CHANNEL_1) {
        mask = (uint8_t)(CH211_CC_CTRL_CC1_OE | CH211_CC_CTRL_CC1_GE);
        value = enable ? mask : 0u;
    } else if (channel == CH211_CHANNEL_2) {
        mask = (uint8_t)(CH211_CC_CTRL_CC2_OE | CH211_CC_CTRL_CC2_GE);
        value = enable ? mask : 0u;
    } else {
        return CH211_ERROR_RANGE;
    }

    return ch211_update_bits(dev, CH211_REG_CC_CTRL, mask, value);
}

ch211_status_t ch211_enable_cc_rd(ch211_t *dev, ch211_channel_t channel, bool enable)
{
    uint8_t mask;

    /* Source port: presenting the Sink Rd pull-down would load the CC line and
     * break Rp/Ra detection, so only the "off" direction is allowed. */
    if (enable) {
        return CH211_ERROR_RANGE;
    }

    if (channel == CH211_CHANNEL_1) {
        mask = CH211_CC_CTRL_CC1_PD;
    } else if (channel == CH211_CHANNEL_2) {
        mask = CH211_CC_CTRL_CC2_PD;
    } else {
        return CH211_ERROR_RANGE;
    }

    return ch211_update_bits(dev, CH211_REG_CC_CTRL, mask, 0u);
}

ch211_status_t ch211_enable_vconn(ch211_t *dev, ch211_channel_t channel, bool enable)
{
    uint8_t mask;

    if (channel == CH211_CHANNEL_1) {
        mask = CH211_CC_CTRL_CC1_VCE;
    } else if (channel == CH211_CHANNEL_2) {
        mask = CH211_CC_CTRL_CC2_VCE;
    } else {
        return CH211_ERROR_RANGE;
    }

    return ch211_update_bits(dev, CH211_REG_CC_CTRL, mask, enable ? mask : 0u);
}

ch211_status_t ch211_set_od_channel(ch211_t *dev,
                                    ch211_channel_t channel,
                                    bool odh_connect_oe,
                                    bool odl_connect_ccl_ge,
                                    bool odl_float_pe)
{
    uint8_t mask;
    uint8_t value;

    if (channel == CH211_CHANNEL_1) {
        mask = (uint8_t)(CH211_OD_CTRL_OD1_OE | CH211_OD_CTRL_OD1_GE | CH211_OD_CTRL_OD1_PE);
        value = (uint8_t)((odh_connect_oe ? CH211_OD_CTRL_OD1_OE : 0u) |
                          (odl_connect_ccl_ge ? CH211_OD_CTRL_OD1_GE : 0u) |
                          (odl_float_pe ? CH211_OD_CTRL_OD1_PE : 0u));
    } else if (channel == CH211_CHANNEL_2) {
        mask = (uint8_t)(CH211_OD_CTRL_OD2_OE | CH211_OD_CTRL_OD2_GE | CH211_OD_CTRL_OD2_PE);
        value = (uint8_t)((odh_connect_oe ? CH211_OD_CTRL_OD2_OE : 0u) |
                          (odl_connect_ccl_ge ? CH211_OD_CTRL_OD2_GE : 0u) |
                          (odl_float_pe ? CH211_OD_CTRL_OD2_PE : 0u));
    } else {
        return CH211_ERROR_RANGE;
    }

    return ch211_update_bits(dev, CH211_REG_OD_CTRL, mask, value);
}

ch211_status_t ch211_set_vbus_discharge(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_HVCP_CTRL,
                             CH211_HVCP_CTRL_VBUS_DISC,
                             enable ? CH211_HVCP_CTRL_VBUS_DISC : 0u);
}

ch211_status_t ch211_enable_hvcp_auto(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_HVCP_CTRL,
                             (uint8_t)(CH211_HVCP_CTRL_CP_AUTO | CH211_HVCP_CTRL_CP_AE |
                                       CH211_HVCP_CTRL_CP_PU | CH211_HVCP_CTRL_CP_LE |
                                       CH211_HVCP_CTRL_CP_LX),
                             enable ? CH211_HVCP_CTRL_CP_AUTO : 0u);
}

ch211_status_t ch211_set_hvcp_low(ch211_t *dev, bool strong_pull_down)
{
    uint8_t value = (uint8_t)(CH211_HVCP_CTRL_CP_LE |
                              (strong_pull_down ? CH211_HVCP_CTRL_CP_LX : 0u));
    return ch211_update_bits(dev,
                             CH211_REG_HVCP_CTRL,
                             (uint8_t)(CH211_HVCP_CTRL_CP_AUTO | CH211_HVCP_CTRL_CP_AE |
                                       CH211_HVCP_CTRL_CP_PU | CH211_HVCP_CTRL_CP_LE |
                                       CH211_HVCP_CTRL_CP_LX),
                             value);
}

ch211_status_t ch211_set_hvcp_vbus_pullup(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_HVCP_CTRL,
                             (uint8_t)(CH211_HVCP_CTRL_CP_AUTO | CH211_HVCP_CTRL_CP_LE |
                                       CH211_HVCP_CTRL_CP_AE | CH211_HVCP_CTRL_CP_PU |
                                       CH211_HVCP_CTRL_CP_LX),
                             enable ? CH211_HVCP_CTRL_CP_PU : 0u);
}

ch211_status_t ch211_set_hvcp_control(ch211_t *dev, const ch211_hvcp_control_t *control)
{
    const uint8_t mask = (uint8_t)(CH211_HVCP_CTRL_VBUS_DISC |
                                   CH211_HVCP_CTRL_CP_AUTO |
                                   CH211_HVCP_CTRL_CP_AE |
                                   CH211_HVCP_CTRL_CP_PU |
                                   CH211_HVCP_CTRL_CP_LE |
                                   CH211_HVCP_CTRL_CP_LX);
    uint8_t value;

    if (control == 0) {
        return CH211_ERROR_NULL;
    }

    if (control->auto_boost) {
        /* Datasheet: CP_LE must be 0 while CP_AUTO is set, and the manual
         * controls belong to the other mode, so auto mode wins. */
        value = CH211_HVCP_CTRL_CP_AUTO;
    } else {
        value = (uint8_t)((control->manual_boost ? CH211_HVCP_CTRL_CP_AE : 0u) |
                          (control->manual_pullup ? CH211_HVCP_CTRL_CP_PU : 0u) |
                          (control->low_pull_down ? CH211_HVCP_CTRL_CP_LE : 0u) |
                          ((control->low_pull_down && control->low_drive_strong) ?
                           CH211_HVCP_CTRL_CP_LX : 0u));
    }

    if (control->vbus_discharge) {
        value |= CH211_HVCP_CTRL_VBUS_DISC;
    }

    return ch211_update_bits(dev, CH211_REG_HVCP_CTRL, mask, value);
}

ch211_status_t ch211_set_hvio_output_low(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_HVIO_KEY,
                             CH211_HVIO_KEY_HV_OE,
                             enable ? CH211_HVIO_KEY_HV_OE : 0u);
}

ch211_status_t ch211_set_key_output_low(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_HVIO_KEY,
                             CH211_HVIO_KEY_KEY_OE,
                             enable ? CH211_HVIO_KEY_KEY_OE : 0u);
}

ch211_status_t ch211_set_hvio_pullup(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_HVIO_KEY,
                             CH211_HVIO_KEY_HV_PU,
                             enable ? CH211_HVIO_KEY_HV_PU : 0u);
}

ch211_status_t ch211_set_hvio_pull_down(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_HVIO_KEY,
                             CH211_HVIO_KEY_HV_PD,
                             enable ? CH211_HVIO_KEY_HV_PD : 0u);
}

ch211_status_t ch211_set_key_pull_down(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_HVIO_KEY,
                             CH211_HVIO_KEY_KEY_PD,
                             enable ? CH211_HVIO_KEY_KEY_PD : 0u);
}

ch211_status_t ch211_set_ldo33_off(ch211_t *dev, bool off)
{
    return ch211_update_bits(dev,
                             CH211_REG_SYS_CFG,
                             CH211_SYS_CFG_LDO33_OFF,
                             off ? CH211_SYS_CFG_LDO33_OFF : 0u);
}

ch211_status_t ch211_set_ldo33_voltage(ch211_t *dev, ch211_ldo33_voltage_t voltage)
{
    if ((uint8_t)voltage > 7u) {
        return CH211_ERROR_RANGE;
    }
    return ch211_update_bits(dev,
                             CH211_REG_SYS_CFG,
                             CH211_SYS_CFG_LDO_VSEL_MASK,
                             (uint8_t)voltage);
}

ch211_status_t ch211_set_ldo33_interrupt_wake(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_SYS_CFG,
                             CH211_SYS_CFG_LDO33_WAKE,
                             enable ? CH211_SYS_CFG_LDO33_WAKE : 0u);
}

ch211_status_t ch211_set_cc_high_threshold_3v3(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_SYS_CFG,
                             CH211_SYS_CFG_CC_HVT3V,
                             enable ? CH211_SYS_CFG_CC_HVT3V : 0u);
}

ch211_status_t ch211_set_auto_hvcp_pull_down_on_fault(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_SYS_CFG,
                             CH211_SYS_CFG_CPLE_OVOT,
                             enable ? CH211_SYS_CFG_CPLE_OVOT : 0u);
}

ch211_status_t ch211_set_auto_reset_on_vbus_ov(ch211_t *dev, bool enable)
{
    return ch211_update_bits(dev,
                             CH211_REG_SYS_CFG,
                             CH211_SYS_CFG_RST_OV,
                             enable ? CH211_SYS_CFG_RST_OV : 0u);
}

void ch211_abort(ch211_t *dev)
{
    if (dev != 0) {
        dev->async_op = CH211_OP_IDLE;
        dev->poll_ready = 0u;
    }
}
