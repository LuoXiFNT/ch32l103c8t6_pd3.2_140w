/**
 * @file mp2980.c
 * @brief Platform-independent non-blocking MP2980 register driver implementation.
 * @author LuoXiFNT
 * @date 2026-06-19
 * @lastModified 2026-06-19
 */

#include "mp2980.h"

/** Minimum reference voltage in mV. */
#define MP2980_VREF_MIN_MV  (300u)
/** Maximum reference voltage in mV. */
#define MP2980_VREF_MAX_MV  (2047u)

/**
 * @name Async operation codes for internal state machine.
 * @{ */
#define MP2980_OP_IDLE              (0u)   /**< No operation in progress. */
#define MP2980_OP_UPDATE_READ       (1u)   /**< Read-modify-write: read phase. */
#define MP2980_OP_UPDATE_WRITE      (2u)   /**< Read-modify-write: write phase. */
#define MP2980_OP_GET_REF           (3u)   /**< Get reference voltage. */
#define MP2980_OP_GET_VOUT          (4u)   /**< Get output voltage. */
#define MP2980_OP_GET_GO            (5u)   /**< Check GO bit (voltage change busy). */
#define MP2980_OP_GET_SLEW          (6u)   /**< Get slew rate. */
#define MP2980_OP_GET_FSW           (7u)   /**< Get switching frequency. */
#define MP2980_OP_GET_OCP           (8u)   /**< Get OCP mode. */
#define MP2980_OP_GET_ILIM          (9u)   /**< Get current limit threshold. */
#define MP2980_OP_GET_ILIM_CURRENT  (10u)  /**< Get current limit in mA. */
#define MP2980_OP_GET_INT_STATUS    (11u)  /**< Get interrupt status. */
#define MP2980_OP_GET_INT_MASK      (12u)  /**< Get interrupt mask. */
/** @} */

/** ILIM threshold voltage lookup table in 0.1 mV units. */
static const uint16_t mp2980_ilim_threshold_0p1mv[8] = {
    279u, 333u, 393u, 451u, 512u, 568u, 628u, 687u
};

/**
 * @brief Validate that the device and write callback are non-null.
 * @param dev Pointer to the MP2980 device instance.
 * @return mp2980_status_t MP2980_OK if valid, MP2980_ERROR_NULL otherwise.
 */
static mp2980_status_t mp2980_check_write_dev(mp2980_t *dev)
{
    if ((dev == 0) || (dev->write_reg == 0)) {
        return MP2980_ERROR_NULL;
    }
    return MP2980_OK;
}

/**
 * @brief Validate that the device, read callback, and idle callback are non-null.
 * @param dev Pointer to the MP2980 device instance.
 * @return mp2980_status_t MP2980_OK if valid, MP2980_ERROR_NULL otherwise.
 */
static mp2980_status_t mp2980_check_read_dev(mp2980_t *dev)
{
    if ((dev == 0) || (dev->read_reg == 0) || (dev->is_bus_idle == 0)) {
        return MP2980_ERROR_NULL;
    }
    return MP2980_OK;
}

/**
 * @brief Check whether the I2C bus is idle via the registered callback.
 * @param dev Pointer to the MP2980 device instance.
 * @return uint8_t 1 if idle or no callback, 0 if busy.
 */
static uint8_t mp2980_bus_is_idle(mp2980_t *dev)
{
    if ((dev == 0) || (dev->is_bus_idle == 0)) {
        return 1u;
    }
    return dev->is_bus_idle(dev->user);
}

/**
 * @brief Start an async single-register read operation.
 * @param dev Pointer to the MP2980 device instance.
 * @param op Async operation code for identification on completion.
 * @param reg Register address to read.
 * @return mp2980_status_t MP2980_BUSY if started, MP2980_OK if already idle,
 *         error code on failure.
 */
static mp2980_status_t mp2980_start_read1(mp2980_t *dev, uint8_t op, uint8_t reg)
{
    mp2980_status_t st = mp2980_check_read_dev(dev);
    if (st != MP2980_OK) {
        return st;
    }

    if (dev->async_op != MP2980_OP_IDLE) {
        return MP2980_BUSY;
    }

    if (mp2980_bus_is_idle(dev) == 0u) {
        return MP2980_BUSY;
    }

    st = mp2980_read_register(dev, reg, &dev->io_data[0]);
    if (st != MP2980_OK) {
        return st;
    }

    dev->async_op = op;
    dev->poll_ready = 0u;
    dev->bus_was_idle = 0u;
    return MP2980_BUSY;
}

/**
 * @brief Start an async two-register read operation.
 * @param dev Pointer to the MP2980 device instance.
 * @param op Async operation code for identification on completion.
 * @param reg0 First register address.
 * @param reg1 Second register address.
 * @return mp2980_status_t MP2980_BUSY if started, error code on failure.
 */
static mp2980_status_t mp2980_start_read2(mp2980_t *dev,
                                          uint8_t op,
                                          uint8_t reg0,
                                          uint8_t reg1)
{
    mp2980_status_t st = mp2980_check_read_dev(dev);
    if (st != MP2980_OK) {
        return st;
    }

    if (dev->async_op != MP2980_OP_IDLE) {
        return MP2980_BUSY;
    }

    if (mp2980_bus_is_idle(dev) == 0u) {
        return MP2980_BUSY;
    }

    st = mp2980_read_register(dev, reg0, &dev->io_data[0]);
    if (st != MP2980_OK) {
        return st;
    }

    st = mp2980_read_register(dev, reg1, &dev->io_data[1]);
    if (st != MP2980_OK) {
        return st;
    }

    dev->async_op = op;
    dev->poll_ready = 0u;
    dev->bus_was_idle = 0u;
    return MP2980_BUSY;
}

/**
 * @brief Polling-style single-register read: check if previous result is ready,
 *        or start a new async read. Must be called repeatedly until MP2980_OK.
 * @param dev Pointer to the MP2980 device instance.
 * @param op Async operation code for identification.
 * @param reg Register address to read.
 * @param data Pointer to store the result.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if still pending.
 */
static mp2980_status_t mp2980_poll_read1(mp2980_t *dev,
                                         uint8_t op,
                                         uint8_t reg,
                                         uint8_t *data)
{
    if (data == 0) {
        return MP2980_ERROR_NULL;
    }

    if ((dev != 0) && (dev->async_op == op)) {
        if (dev->poll_ready == 0u) {
            return MP2980_BUSY;
        }
        *data = dev->io_data[0];
        dev->async_op = MP2980_OP_IDLE;
        dev->poll_ready = 0u;
        return MP2980_OK;
    }

    return mp2980_start_read1(dev, op, reg);
}

/**
 * @brief Polling-style two-register read. Must be called repeatedly until MP2980_OK.
 * @param dev Pointer to the MP2980 device instance.
 * @param op Async operation code for identification.
 * @param reg0 First register address.
 * @param reg1 Second register address.
 * @param data0 Pointer to store first register result.
 * @param data1 Pointer to store second register result.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if still pending.
 */
static mp2980_status_t mp2980_poll_read2(mp2980_t *dev,
                                         uint8_t op,
                                         uint8_t reg0,
                                         uint8_t reg1,
                                         uint8_t *data0,
                                         uint8_t *data1)
{
    if ((data0 == 0) || (data1 == 0)) {
        return MP2980_ERROR_NULL;
    }

    if ((dev != 0) && (dev->async_op == op)) {
        if (dev->poll_ready == 0u) {
            return MP2980_BUSY;
        }
        *data0 = dev->io_data[0];
        *data1 = dev->io_data[1];
        dev->async_op = MP2980_OP_IDLE;
        dev->poll_ready = 0u;
        return MP2980_OK;
    }

    return mp2980_start_read2(dev, op, reg0, reg1);
}

/* Initialize the MP2980 device instance and register callbacks. */
mp2980_status_t mp2980_init(mp2980_t *dev,
                            uint8_t dev_addr_7bit,
                            float vout_feedback_ratio,
                            mp2980_write_reg_cb_t write_reg,
                            mp2980_read_reg_cb_t read_reg,
                            mp2980_bus_idle_cb_t is_bus_idle,
                            void *user)
{
    if ((dev == 0) || (write_reg == 0)) {
        return MP2980_ERROR_NULL;
    }

    if ((dev_addr_7bit != MP2980_ADDRESS_60H) &&
        (dev_addr_7bit != MP2980_ADDRESS_62H) &&
        (dev_addr_7bit != MP2980_ADDRESS_64H) &&
        (dev_addr_7bit != MP2980_ADDRESS_66H)) {
        return MP2980_ERROR_RANGE;
    }

    if (vout_feedback_ratio <= 0.0f) {
        return MP2980_ERROR_RANGE;
    }

    dev->dev_addr_7bit = dev_addr_7bit;
    dev->vout_feedback_ratio = vout_feedback_ratio;
    dev->avg_current_sense_mohm = 0.0f;
    dev->write_reg = write_reg;
    dev->read_reg = read_reg;
    dev->is_bus_idle = is_bus_idle;
    dev->user = user;
    dev->async_op = MP2980_OP_IDLE;
    dev->update_reg = 0u;
    dev->update_mask = 0u;
    dev->update_value = 0u;
    dev->poll_ready = 0u;
    dev->io_data[0] = 0u;
    dev->io_data[1] = 0u;

    return MP2980_OK;
}

/* Drive the asynchronous state machine. Call periodically from main loop. */
mp2980_status_t mp2980_task(mp2980_t *dev)
{
    uint8_t data;
    mp2980_status_t st;

    if (dev == 0) {
        return MP2980_ERROR_NULL;
    }

    if (dev->async_op == MP2980_OP_UPDATE_READ) {
        if (mp2980_bus_is_idle(dev) == 0u) {
            return MP2980_BUSY;
        }

        data = (uint8_t)((dev->io_data[0] & (uint8_t)(~dev->update_mask)) |
                         (dev->update_value & dev->update_mask));
        if (dev->update_reg == MP2980_REG_CONTROL1) {
            data |= MP2980_CONTROL1_RESERVED_MASK;
        }

        st = mp2980_write_register(dev, dev->update_reg, data);
        if (st != MP2980_OK) {
            dev->async_op = MP2980_OP_IDLE;
            return st;
        }

        dev->async_op = MP2980_OP_UPDATE_WRITE;
        dev->bus_was_idle = 0u;
        return MP2980_BUSY;
    }

    if (dev->async_op == MP2980_OP_UPDATE_WRITE) {
        if (mp2980_bus_is_idle(dev) != 0u) {
            dev->bus_was_idle = 1u;
        }
        if (dev->bus_was_idle != 0u) {
            dev->async_op = MP2980_OP_IDLE;
            return MP2980_OK;
        }
        return MP2980_BUSY;
    }

    /* Latch bus-idle event for poll read operations */
    if ((dev->async_op != MP2980_OP_IDLE) &&
        (dev->async_op != MP2980_OP_UPDATE_READ) &&
        (dev->async_op != MP2980_OP_UPDATE_WRITE)) {
        if (mp2980_bus_is_idle(dev) != 0u) {
            dev->bus_was_idle = 1u;
        }
        if (dev->bus_was_idle != 0u) {
            dev->poll_ready = 1u;
        }
        return MP2980_BUSY;
    }

    return (dev->async_op == MP2980_OP_IDLE) ? MP2980_OK : MP2980_BUSY;
}

/* Check whether the device has a pending async operation. */
uint8_t mp2980_is_busy(const mp2980_t *dev)
{
    if (dev == 0) {
        return 0u;
    }
    return (dev->async_op != MP2980_OP_IDLE) ? 1u : 0u;
}

/* Write a register directly via the registered write callback. */
mp2980_status_t mp2980_write_register(mp2980_t *dev, uint8_t reg, uint8_t data)
{
    mp2980_status_t st = mp2980_check_write_dev(dev);
    if (st != MP2980_OK) {
        return st;
    }

    return (dev->write_reg(dev->user, dev->dev_addr_7bit, reg, data) == 0) ? MP2980_OK : MP2980_ERROR;
}

/* Read a register directly via the registered read callback. */
mp2980_status_t mp2980_read_register(mp2980_t *dev, uint8_t reg, uint8_t *data)
{
    mp2980_status_t st = mp2980_check_read_dev(dev);
    if (st != MP2980_OK) {
        return st;
    }
    if (data == 0) {
        return MP2980_ERROR_NULL;
    }

    return (dev->read_reg(dev->user, dev->dev_addr_7bit, reg, data) == 0) ? MP2980_OK : MP2980_ERROR;
}

/* Start an async read-modify-write operation on a register. */
mp2980_status_t mp2980_update_bits(mp2980_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    mp2980_status_t st = mp2980_check_read_dev(dev);
    if (st != MP2980_OK) {
        return st;
    }

    if (dev->async_op != MP2980_OP_IDLE) {
        return MP2980_BUSY;
    }

    if (mp2980_bus_is_idle(dev) == 0u) {
        return MP2980_BUSY;
    }

    st = mp2980_read_register(dev, reg, &dev->io_data[0]);
    if (st != MP2980_OK) {
        return st;
    }

    dev->update_reg = reg;
    dev->update_mask = mask;
    dev->update_value = value;
    dev->async_op = MP2980_OP_UPDATE_READ;
    return MP2980_OK;
}

/* Enable power output (set ENPWR bit). */
mp2980_status_t mp2980_enable_power(mp2980_t *dev)
{
    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL1,
                              MP2980_CONTROL1_ENPWR_MASK,
                              MP2980_CONTROL1_ENPWR_MASK);
}

/* Disable power output (clear ENPWR bit). */
mp2980_status_t mp2980_disable_power(mp2980_t *dev)
{
    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL1,
                              MP2980_CONTROL1_ENPWR_MASK,
                              0u);
}

/* Set the reference voltage by writing REF_LSB and REF_MSB registers. */
mp2980_status_t mp2980_set_reference_mv(mp2980_t *dev, uint16_t vref_mv)
{
    mp2980_status_t st;

    if ((vref_mv < MP2980_VREF_MIN_MV) || (vref_mv > MP2980_VREF_MAX_MV)) {
        return MP2980_ERROR_RANGE;
    }

    st = mp2980_write_register(dev, MP2980_REG_REF_LSB, (uint8_t)(vref_mv & 0x07u));
    if (st != MP2980_OK) {
        return st;
    }

    return mp2980_write_register(dev, MP2980_REG_REF_MSB, (uint8_t)((vref_mv >> 3) & 0xFFu));
}

/* Get the reference voltage (async, poll via task). */
mp2980_status_t mp2980_get_reference_mv(mp2980_t *dev, uint16_t *vref_mv)
{
    uint8_t lsb = 0u;
    uint8_t msb = 0u;
    mp2980_status_t st;

    if (vref_mv == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read2(dev,
                           MP2980_OP_GET_REF,
                           MP2980_REG_REF_LSB,
                           MP2980_REG_REF_MSB,
                           &lsb,
                           &msb);
    if (st != MP2980_OK) {
        return st;
    }

    *vref_mv = (uint16_t)(((uint16_t)msb << 3) | (uint16_t)(lsb & 0x07u));
    return MP2980_OK;
}

/* Start a voltage change by writing CONTROL1 with the GO bit set. */
mp2980_status_t mp2980_start_voltage_change(mp2980_t *dev)
{
    return mp2980_write_register(dev,
                                 MP2980_REG_CONTROL1,
                                 (uint8_t)(MP2980_CONTROL1_PD_CONFIG |
                                           MP2980_CONTROL1_ENPWR_MASK |
                                           MP2980_CONTROL1_GO_BIT_MASK));
}

/* Check if a voltage change is in progress (reads GO bit). */
mp2980_status_t mp2980_is_voltage_change_busy(mp2980_t *dev, bool *busy)
{
    uint8_t data;
    mp2980_status_t st;

    if (busy == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read1(dev, MP2980_OP_GET_GO, MP2980_REG_CONTROL1, &data);
    if (st != MP2980_OK) {
        return st;
    }

    *busy = ((data & MP2980_CONTROL1_GO_BIT_MASK) != 0u);
    return MP2980_OK;
}

/* Non-blocking check: returns OK if voltage change is done. */
mp2980_status_t mp2980_wait_voltage_change_done(mp2980_t *dev,
                                                uint16_t timeout_ms,
                                                void (*delay_ms)(uint32_t ms))
{
    bool busy = false;
    mp2980_status_t st;

    (void)timeout_ms;
    (void)delay_ms;

    st = mp2980_is_voltage_change_busy(dev, &busy);
    if (st != MP2980_OK) {
        return st;
    }

    return busy ? MP2980_BUSY : MP2980_OK;
}

/* Set output voltage in mV (calculates VREF + starts change). */
mp2980_status_t mp2980_set_output_voltage_mv(mp2980_t *dev, uint16_t vout_mv)
{
    float vref_mv_f;
    uint16_t vref_mv;
    mp2980_status_t st = mp2980_check_write_dev(dev);
    if (st != MP2980_OK) {
        return st;
    }

    if (dev->vout_feedback_ratio <= 0.0f) {
        return MP2980_ERROR_RANGE;
    }

    vref_mv_f = ((float)vout_mv / dev->vout_feedback_ratio) + 0.5f;
    if ((vref_mv_f < (float)MP2980_VREF_MIN_MV) ||
        (vref_mv_f > (float)MP2980_VREF_MAX_MV)) {
        return MP2980_ERROR_RANGE;
    }

    vref_mv = (uint16_t)vref_mv_f;

    st = mp2980_set_reference_mv(dev, vref_mv);
    if (st != MP2980_OK) {
        return st;
    }

    return mp2980_start_voltage_change(dev);
}

/* Get output voltage in mV (async, poll via task). */
mp2980_status_t mp2980_get_output_voltage_mv(mp2980_t *dev, uint16_t *vout_mv)
{
    uint8_t lsb = 0u;
    uint8_t msb = 0u;
    uint16_t vref_mv;
    float vout_mv_f;
    mp2980_status_t st;

    if ((dev == 0) || (vout_mv == 0)) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read2(dev,
                           MP2980_OP_GET_VOUT,
                           MP2980_REG_REF_LSB,
                           MP2980_REG_REF_MSB,
                           &lsb,
                           &msb);
    if (st != MP2980_OK) {
        return st;
    }

    vref_mv = (uint16_t)(((uint16_t)msb << 3) | (uint16_t)(lsb & 0x07u));
    vout_mv_f = ((float)vref_mv * dev->vout_feedback_ratio) + 0.5f;
    if (vout_mv_f > 65535.0f) {
        return MP2980_ERROR_RANGE;
    }

    *vout_mv = (uint16_t)vout_mv_f;
    return MP2980_OK;
}

/* Set the output voltage slew rate. */
mp2980_status_t mp2980_set_slew_rate(mp2980_t *dev, mp2980_slew_rate_t slew_rate)
{
    if ((uint8_t)slew_rate > 3u) {
        return MP2980_ERROR_RANGE;
    }

    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL1,
                              MP2980_CONTROL1_SR_MASK,
                              (uint8_t)(((uint8_t)slew_rate << 6) & MP2980_CONTROL1_SR_MASK));
}

/* Get the output voltage slew rate (async, poll via task). */
mp2980_status_t mp2980_get_slew_rate(mp2980_t *dev, mp2980_slew_rate_t *slew_rate)
{
    uint8_t data;
    mp2980_status_t st;

    if (slew_rate == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read1(dev, MP2980_OP_GET_SLEW, MP2980_REG_CONTROL1, &data);
    if (st != MP2980_OK) {
        return st;
    }

    *slew_rate = (mp2980_slew_rate_t)((data & MP2980_CONTROL1_SR_MASK) >> 6);
    return MP2980_OK;
}

/* Get the configured feedback divider ratio. */
mp2980_status_t mp2980_get_vout_feedback_ratio(mp2980_t *dev, float *ratio)
{
    if ((dev == 0) || (ratio == 0)) {
        return MP2980_ERROR_NULL;
    }

    *ratio = dev->vout_feedback_ratio;
    return MP2980_OK;
}

/* Enable or disable output discharge on disable. */
mp2980_status_t mp2980_set_discharge(mp2980_t *dev, bool enable)
{
    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL1,
                              MP2980_CONTROL1_DISCHG_MASK,
                              enable ? MP2980_CONTROL1_DISCHG_MASK : 0u);
}

/* Enable or disable dithering. */
mp2980_status_t mp2980_set_dither(mp2980_t *dev, bool enable)
{
    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL1,
                              MP2980_CONTROL1_DITHER_MASK,
                              enable ? MP2980_CONTROL1_DITHER_MASK : 0u);
}

/* Enable or disable power good latch. */
mp2980_status_t mp2980_set_png_latch(mp2980_t *dev, bool enable)
{
    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL1,
                              MP2980_CONTROL1_PNG_LATCH_MASK,
                              enable ? MP2980_CONTROL1_PNG_LATCH_MASK : 0u);
}

/* Set the switching frequency. */
mp2980_status_t mp2980_set_switching_frequency(mp2980_t *dev, mp2980_frequency_t frequency)
{
    if ((uint8_t)frequency > 3u) {
        return MP2980_ERROR_RANGE;
    }

    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL2,
                              MP2980_CONTROL2_FSW_MASK,
                              (uint8_t)(((uint8_t)frequency << 6) & MP2980_CONTROL2_FSW_MASK));
}

/* Get the switching frequency (async, poll via task). */
mp2980_status_t mp2980_get_switching_frequency(mp2980_t *dev, mp2980_frequency_t *frequency)
{
    uint8_t data;
    mp2980_status_t st;

    if (frequency == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read1(dev, MP2980_OP_GET_FSW, MP2980_REG_CONTROL2, &data);
    if (st != MP2980_OK) {
        return st;
    }

    *frequency = (mp2980_frequency_t)((data & MP2980_CONTROL2_FSW_MASK) >> 6);
    return MP2980_OK;
}

/* Enable or disable buck-boost frequency mode. */
mp2980_status_t mp2980_set_buck_boost_frequency_mode(mp2980_t *dev, bool enable)
{
    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL2,
                              MP2980_CONTROL2_BB_FSW_MASK,
                              enable ? MP2980_CONTROL2_BB_FSW_MASK : 0u);
}

/* Set the overcurrent protection mode. */
mp2980_status_t mp2980_set_ocp_mode(mp2980_t *dev, mp2980_ocp_mode_t mode)
{
    if ((uint8_t)mode > 2u) {
        return MP2980_ERROR_RANGE;
    }

    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL2,
                              MP2980_CONTROL2_OCP_MODE_MASK,
                              (uint8_t)(((uint8_t)mode << 2) & MP2980_CONTROL2_OCP_MODE_MASK));
}

/* Get the overcurrent protection mode (async, poll via task). */
mp2980_status_t mp2980_get_ocp_mode(mp2980_t *dev, mp2980_ocp_mode_t *mode)
{
    uint8_t data;
    mp2980_status_t st;

    if (mode == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read1(dev, MP2980_OP_GET_OCP, MP2980_REG_CONTROL2, &data);
    if (st != MP2980_OK) {
        return st;
    }

    *mode = (mp2980_ocp_mode_t)((data & MP2980_CONTROL2_OCP_MODE_MASK) >> 2);
    return MP2980_OK;
}

/* Set the overvoltage protection mode. */
mp2980_status_t mp2980_set_ovp_mode(mp2980_t *dev, mp2980_ovp_mode_t mode)
{
    if ((uint8_t)mode > 2u) {
        return MP2980_ERROR_RANGE;
    }

    return mp2980_update_bits(dev,
                              MP2980_REG_CONTROL2,
                              MP2980_CONTROL2_OVP_MODE_MASK,
                              (uint8_t)mode);
}

/* Store the average current sense resistor value for current calculations. */
void mp2980_set_average_current_sense_resistor(mp2980_t *dev, float rsense_mohm)
{
    if (dev == 0) {
        return;
    }

    dev->avg_current_sense_mohm = rsense_mohm;
}

/* Set the current limit threshold (ILIM register). */
mp2980_status_t mp2980_set_current_limit_threshold(mp2980_t *dev, mp2980_ilim_t ilim)
{
    if ((uint8_t)ilim > 7u) {
        return MP2980_ERROR_RANGE;
    }

    return mp2980_update_bits(dev, MP2980_REG_ILIM, MP2980_ILIM_MASK, (uint8_t)ilim);
}

/* Get the current limit threshold (async, poll via task). */
mp2980_status_t mp2980_get_current_limit(mp2980_t *dev, mp2980_ilim_t *ilim)
{
    uint8_t data;
    mp2980_status_t st;

    if (ilim == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read1(dev, MP2980_OP_GET_ILIM, MP2980_REG_ILIM, &data);
    if (st != MP2980_OK) {
        return st;
    }

    *ilim = (mp2980_ilim_t)(data & MP2980_ILIM_MASK);
    return MP2980_OK;
}

/* Get the current limit in mA (async, uses rsense + ILIM register). */
mp2980_status_t mp2980_get_current_limit_current_ma(mp2980_t *dev, uint16_t *current_ma)
{
    uint8_t data;
    uint8_t ilim;
    float threshold_mv;
    float current_ma_f;
    mp2980_status_t st;

    if ((dev == 0) || (current_ma == 0)) {
        return MP2980_ERROR_NULL;
    }

    if (dev->avg_current_sense_mohm <= 0.0f) {
        return MP2980_ERROR_RANGE;
    }

    st = mp2980_poll_read1(dev, MP2980_OP_GET_ILIM_CURRENT, MP2980_REG_ILIM, &data);
    if (st != MP2980_OK) {
        return st;
    }

    ilim = (uint8_t)(data & MP2980_ILIM_MASK);
    threshold_mv = (float)mp2980_ilim_threshold_0p1mv[ilim] / 10.0f;
    current_ma_f = (threshold_mv / dev->avg_current_sense_mohm) * 1000.0f;
    if (current_ma_f > 65535.0f) {
        return MP2980_ERROR_RANGE;
    }

    *current_ma = (uint16_t)(current_ma_f + 0.5f);
    return MP2980_OK;
}

/* Get the threshold voltage in mV for a given ILIM level. */
uint8_t mp2980_get_current_limit_threshold_voltage_mv(mp2980_ilim_t ilim)
{
    if ((uint8_t)ilim > 7u) {
        return 0u;
    }

    return (uint8_t)((mp2980_ilim_threshold_0p1mv[(uint8_t)ilim] + 5u) / 10u);
}

/* Set current limit by desired current in mA (selects nearest ILIM level). */
mp2980_status_t mp2980_set_current_limit_by_current_ma(mp2980_t *dev, uint16_t current_ma)
{
    uint16_t target_0p1mv;
    float target_0p1mv_f;
    uint8_t i;

    if (dev == 0) {
        return MP2980_ERROR_NULL;
    }
    if (dev->avg_current_sense_mohm <= 0.0f) {
        return MP2980_ERROR_RANGE;
    }

    target_0p1mv_f = ((float)current_ma / 1000.0f) * dev->avg_current_sense_mohm * 10.0f;
    if (target_0p1mv_f > 65535.0f) {
        return MP2980_ERROR_RANGE;
    }

    target_0p1mv = (uint16_t)(target_0p1mv_f + 0.5f);

    for (i = 0u; i < 8u; i++) {
        if (mp2980_ilim_threshold_0p1mv[i] >= target_0p1mv) {
            return mp2980_set_current_limit_threshold(dev, (mp2980_ilim_t)i);
        }
    }

    return MP2980_ERROR_RANGE;
}

/* Convert IMON pin ADC reading (mV) to output current (mA). */
mp2980_status_t mp2980_convert_imon_mv_to_ma(mp2980_t *dev, uint16_t adc_mv, uint16_t *current_ma)
{
    float current_ma_f;

    if ((dev == 0) || (current_ma == 0)) {
        return MP2980_ERROR_NULL;
    }

    if (dev->avg_current_sense_mohm <= 0.0f) {
        return MP2980_ERROR_RANGE;
    }

    current_ma_f = ((float)adc_mv * 1000.0f) /
                   (MP2980_IMON_GAIN_V_PER_V * dev->avg_current_sense_mohm);
    if (current_ma_f > 65535.0f) {
        return MP2980_ERROR_RANGE;
    }

    *current_ma = (uint16_t)(current_ma_f + 0.5f);
    return MP2980_OK;
}

/* Get interrupt status (async, poll via task). */
mp2980_status_t mp2980_get_interrupt_status(mp2980_t *dev, mp2980_interrupt_status_t *status)
{
    uint8_t data;
    mp2980_status_t st;

    if (status == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read1(dev, MP2980_OP_GET_INT_STATUS, MP2980_REG_INTERRUPT_STATUS, &data);
    if (st != MP2980_OK) {
        return st;
    }

    status->raw = data;
    status->otp = ((data & MP2980_INT_OTP_MASK) != 0u);
    status->ovp = ((data & MP2980_INT_OVP_MASK) != 0u);
    status->ocp = ((data & MP2980_INT_OCP_MASK) != 0u);
    status->png = ((data & MP2980_INT_PNG_MASK) != 0u);
    return MP2980_OK;
}

/* Clear all interrupt status flags. */
mp2980_status_t mp2980_clear_interrupt_status(mp2980_t *dev)
{
    return mp2980_write_register(dev, MP2980_REG_INTERRUPT_STATUS, 0xFFu);
}

/* Set the interrupt mask. */
mp2980_status_t mp2980_set_interrupt_mask(mp2980_t *dev, uint8_t mask)
{
    return mp2980_write_register(dev, MP2980_REG_INTERRUPT_MASK, (uint8_t)(mask & MP2980_INT_ALL_MASK));
}

/* Get the interrupt mask (async, poll via task). */
mp2980_status_t mp2980_get_interrupt_mask(mp2980_t *dev, uint8_t *mask)
{
    uint8_t data;
    mp2980_status_t st;

    if (mask == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read1(dev, MP2980_OP_GET_INT_MASK, MP2980_REG_INTERRUPT_MASK, &data);
    if (st != MP2980_OK) {
        return st;
    }

    *mask = (uint8_t)(data & MP2980_INT_ALL_MASK);
    return MP2980_OK;
}

/* Abort any pending async operation and reset to IDLE. */
void mp2980_abort(mp2980_t *dev)
{
    if (dev != 0) {
        dev->async_op = MP2980_OP_IDLE;
        dev->poll_ready = 0u;
        dev->bus_was_idle = 0u;
    }
}
