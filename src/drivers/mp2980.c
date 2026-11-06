/**
 * @file mp2980.c
 * @brief Platform-independent non-blocking MP2980 register driver implementation.
 * @author LuoXiFNT
 * @date 2026-06-19
 * @lastModified 2026-11-04
 */

#include "mp2980.h"

/** Minimum reference voltage in mV (datasheet: 300mV). */
#define MP2980_VREF_MIN_MV  (300u)
/** Maximum reference voltage in mV (11-bit VREF field, datasheet: 2.047V). */
#define MP2980_VREF_MAX_MV  (2047u)

/** Sentinel address meaning "no second register" for the shared read helper. */
#define MP2980_REG_NONE     (0xFFu)

/**
 * @brief Async operation codes for the internal state machine.
 *
 * Each polling API owns one code so that a completion latched by
 * mp2980_task() can only be consumed by the caller that started it.
 */
typedef enum {
    MP2980_OP_IDLE = 0u,            /**< No operation in progress. */
    MP2980_OP_UPDATE_READ,          /**< Read-modify-write: read phase. */
    MP2980_OP_UPDATE_WRITE,         /**< Read-modify-write: write phase. */
    MP2980_OP_GET_REF,              /**< Get reference voltage. */
    MP2980_OP_GET_VOUT,             /**< Get output voltage. */
    MP2980_OP_GET_CONTROL1,         /**< Read CONTROL1 (slew, discharge, dither, latch, GO, ENPWR). */
    MP2980_OP_GET_CONTROL2,         /**< Read CONTROL2 (switching frequency, OCP and OVP mode). */
    MP2980_OP_GET_ILIM,             /**< Get current limit threshold. */
    MP2980_OP_GET_ILIM_CURRENT,     /**< Get current limit in mA. */
    MP2980_OP_GET_INT_STATUS,       /**< Get interrupt status. */
    MP2980_OP_GET_INT_MASK          /**< Get interrupt mask. */
} mp2980_op_t;

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
 * @brief Check whether the transfer started by this device has finished.
 * @param dev Pointer to the MP2980 device instance.
 * @return true when idle or no callback is installed.
 */
static uint8_t mp2980_bus_is_idle(mp2980_t *dev)
{
    if ((dev == 0) || (dev->is_bus_idle == 0)) {
        return 1u;
    }
    return dev->is_bus_idle(dev->user);
}

/**
 * @brief Queue a one- or two-register read and arm the async operation.
 * @param dev Pointer to the MP2980 device instance.
 * @param op Async operation code identifying the caller on completion.
 * @param reg0 First register address to read.
 * @param reg1 Second register address, or MP2980_REG_NONE for a single read.
 * @return mp2980_status_t MP2980_OK if the read was queued, MP2980_BUSY if the
 *         driver or the bus is already in use, error code otherwise.
 */
static mp2980_status_t mp2980_start_read(mp2980_t *dev, uint8_t op,
                                         uint8_t reg0, uint8_t reg1)
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
    if ((st == MP2980_OK) && (reg1 != MP2980_REG_NONE)) {
        st = mp2980_read_register(dev, reg1, &dev->io_data[1]);
    }
    if (st != MP2980_OK) {
        return st;
    }

    dev->async_op = op;
    dev->poll_ready = 0u;
    return MP2980_OK;
}

/**
 * @brief Polling-style read: either report the latched result or start a read.
 *        Must be called repeatedly until MP2980_OK is returned.
 * @param dev Pointer to the MP2980 device instance.
 * @param op Async operation code for identification.
 * @param reg0 First register address to read.
 * @param reg1 Second register address, or MP2980_REG_NONE for a single read.
 * @param out Result buffer; out[1] is written only when reg1 is used.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if still pending.
 */
static mp2980_status_t mp2980_poll_read(mp2980_t *dev, uint8_t op,
                                        uint8_t reg0, uint8_t reg1,
                                        uint8_t *out)
{
    mp2980_status_t st;

    if (out == 0) {
        return MP2980_ERROR_NULL;
    }

    if ((dev != 0) && (dev->async_op == op)) {
        if (dev->poll_ready == 0u) {
            return MP2980_BUSY;
        }
        out[0] = dev->io_data[0];
        if (reg1 != MP2980_REG_NONE) {
            out[1] = dev->io_data[1];
        }
        dev->async_op = MP2980_OP_IDLE;
        dev->poll_ready = 0u;
        return MP2980_OK;
    }

    st = mp2980_start_read(dev, op, reg0, reg1);
    return (st == MP2980_OK) ? MP2980_BUSY : st;
}

/**
 * @brief Polling-style read of one register field.
 * @param dev Pointer to the MP2980 device instance.
 * @param op Async operation code for identification.
 * @param reg Register address to read.
 * @param mask Bit mask applied to the register value.
 * @param field Pointer to store the masked value, unshifted.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if still pending.
 */
static mp2980_status_t mp2980_poll_read_field(mp2980_t *dev, uint8_t op,
                                              uint8_t reg, uint8_t mask,
                                              uint8_t *field)
{
    uint8_t raw = 0u;
    mp2980_status_t st = mp2980_poll_read(dev, op, reg, MP2980_REG_NONE, &raw);

    if (st != MP2980_OK) {
        return st;
    }

    *field = (uint8_t)(raw & mask);
    return MP2980_OK;
}

/**
 * @brief Combine the REF_LSB/REF_MSB register pair into an 11-bit mV value.
 * @param lsb REF_LSB register value, bits[2:0] are significant.
 * @param msb REF_MSB register value, bits[7:0] hold VREF bits[10:3].
 * @return uint16_t Reference voltage in mV.
 */
static uint16_t mp2980_decode_vref_mv(uint8_t lsb, uint8_t msb)
{
    return (uint16_t)(((uint16_t)msb << 3) | (uint16_t)(lsb & 0x07u));
}

/**
 * @brief Polling-style read of the reference voltage register pair.
 * @param dev Pointer to the MP2980 device instance.
 * @param op Async operation code for identification.
 * @param vref_mv Pointer to store the reference voltage in mV.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if still pending.
 */
static mp2980_status_t mp2980_poll_read_reference_mv(mp2980_t *dev, uint8_t op,
                                                     uint16_t *vref_mv)
{
    uint8_t raw[2] = {0u, 0u};
    mp2980_status_t st = mp2980_poll_read(dev, op,
                                          MP2980_REG_REF_LSB, MP2980_REG_REF_MSB,
                                          raw);

    if (st != MP2980_OK) {
        return st;
    }

    *vref_mv = mp2980_decode_vref_mv(raw[0], raw[1]);
    return MP2980_OK;
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

    /* Valid addresses are 0x60 | (ADR << 1): 0x60, 0x62, 0x64 or 0x66. */
    if ((dev_addr_7bit & 0xF9u) != MP2980_ADDRESS_60H) {
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

    if (dev->async_op == MP2980_OP_IDLE) {
        return MP2980_OK;
    }

    /* Every pending operation completes when the queued transfer drains.
     * Latch that edge so unrelated bus activity cannot hide it. */
    if (dev->poll_ready == 0u) {
        if (mp2980_bus_is_idle(dev) == 0u) {
            return MP2980_BUSY;
        }
        dev->poll_ready = 1u;
    }

    if (dev->async_op == MP2980_OP_UPDATE_READ) {
        data = (uint8_t)((dev->io_data[0] & (uint8_t)(~dev->update_mask)) |
                         (dev->update_value & dev->update_mask));
        if (dev->update_reg == MP2980_REG_CONTROL1) {
            /* bit[2] is reserved: preserve its reset value instead of writing 0. */
            data |= MP2980_CONTROL1_RESERVED_MASK;
        }

        st = mp2980_write_register(dev, dev->update_reg, data);
        if (st != MP2980_OK) {
            dev->async_op = MP2980_OP_IDLE;
            dev->poll_ready = 0u;
            return st;
        }

        dev->async_op = MP2980_OP_UPDATE_WRITE;
        dev->poll_ready = 0u;
        return MP2980_BUSY;
    }

    if (dev->async_op == MP2980_OP_UPDATE_WRITE) {
        dev->async_op = MP2980_OP_IDLE;
        dev->poll_ready = 0u;
        return MP2980_OK;
    }

    /* Read operation finished: keep poll_ready set for the polling caller. */
    return MP2980_BUSY;
}

/* Check whether the device has a pending async operation. */
bool mp2980_is_busy(const mp2980_t *dev)
{
    if (dev == 0) {
        return false;
    }
    return dev->async_op != MP2980_OP_IDLE;
}

/* Write a register directly via the registered write callback. */
mp2980_status_t mp2980_write_register(mp2980_t *dev, uint8_t reg, uint8_t data)
{
    mp2980_status_t st = mp2980_check_write_dev(dev);

    if (st != MP2980_OK) {
        return st;
    }

    return (dev->write_reg(dev->user, dev->dev_addr_7bit, reg, data) == 0) ?
           MP2980_OK : MP2980_ERROR;
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

    return (dev->read_reg(dev->user, dev->dev_addr_7bit, reg, data) == 0) ?
           MP2980_OK : MP2980_ERROR;
}

/* Start an async read-modify-write operation on a register. */
mp2980_status_t mp2980_update_bits(mp2980_t *dev, uint8_t reg,
                                   uint8_t mask, uint8_t value)
{
    mp2980_status_t st = mp2980_check_read_dev(dev);

    if (st != MP2980_OK) {
        return st;
    }

    dev->update_reg = reg;
    dev->update_mask = mask;
    dev->update_value = value;

    /* The write phase runs from mp2980_task() once the read completes. */
    return mp2980_start_read(dev, MP2980_OP_UPDATE_READ, reg, MP2980_REG_NONE);
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

    /* Both registers belong to one 11-bit field; the new value only takes
     * effect after mp2980_start_voltage_change(), so write order is free. */
    st = mp2980_write_register(dev, MP2980_REG_REF_LSB, (uint8_t)(vref_mv & 0x07u));
    if (st != MP2980_OK) {
        return st;
    }

    return mp2980_write_register(dev, MP2980_REG_REF_MSB, (uint8_t)((vref_mv >> 3) & 0xFFu));
}

/* Get the reference voltage (async, poll via task). */
mp2980_status_t mp2980_get_reference_mv(mp2980_t *dev, uint16_t *vref_mv)
{
    if (vref_mv == 0) {
        return MP2980_ERROR_NULL;
    }

    return mp2980_poll_read_reference_mv(dev, MP2980_OP_GET_REF, vref_mv);
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
    mp2980_control1_t control1;
    mp2980_status_t st;

    if (busy == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_get_control1(dev, &control1);
    if (st != MP2980_OK) {
        return st;
    }

    *busy = control1.go;
    return MP2980_OK;
}

/**
 * @brief Report whether the voltage change started by the GO bit has finished.
 * @param dev Pointer to the MP2980 device instance.
 * @return mp2980_status_t MP2980_OK when the change is done, MP2980_BUSY while the
 *         GO bit is still set, error code otherwise.
 */
mp2980_status_t mp2980_check_voltage_change_done(mp2980_t *dev)
{
    bool busy = false;
    mp2980_status_t st;

    st = mp2980_is_voltage_change_busy(dev, &busy);
    if (st != MP2980_OK) {
        return st;
    }

    return busy ? MP2980_BUSY : MP2980_OK;
}

/* Convert an output voltage target into the 11-bit VREF register value. */
mp2980_status_t mp2980_convert_vout_mv_to_vref_mv(mp2980_t *dev, uint16_t vout_mv,
                                                  uint16_t *vref_mv)
{
    float vref_mv_f;

    if ((dev == 0) || (vref_mv == 0)) {
        return MP2980_ERROR_NULL;
    }

    if (dev->vout_feedback_ratio <= 0.0f) {
        return MP2980_ERROR_RANGE;
    }

    vref_mv_f = ((float)vout_mv / dev->vout_feedback_ratio) + 0.5f;
    if ((vref_mv_f < (float)MP2980_VREF_MIN_MV) ||
        (vref_mv_f > (float)MP2980_VREF_MAX_MV)) {
        return MP2980_ERROR_RANGE;
    }

    *vref_mv = (uint16_t)vref_mv_f;
    return MP2980_OK;
}

/* Set output voltage in mV (calculates VREF + starts change). */
mp2980_status_t mp2980_set_output_voltage_mv(mp2980_t *dev, uint16_t vout_mv)
{
    uint16_t vref_mv = 0u;
    mp2980_status_t st = mp2980_check_write_dev(dev);

    if (st != MP2980_OK) {
        return st;
    }

    st = mp2980_convert_vout_mv_to_vref_mv(dev, vout_mv, &vref_mv);
    if (st != MP2980_OK) {
        return st;
    }

    st = mp2980_set_reference_mv(dev, vref_mv);
    if (st != MP2980_OK) {
        return st;
    }

    return mp2980_start_voltage_change(dev);
}

/* Get output voltage in mV (async, poll via task). */
mp2980_status_t mp2980_get_output_voltage_mv(mp2980_t *dev, uint16_t *vout_mv)
{
    uint16_t vref_mv = 0u;
    float vout_mv_f;
    mp2980_status_t st;

    if ((dev == 0) || (vout_mv == 0)) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read_reference_mv(dev, MP2980_OP_GET_VOUT, &vref_mv);
    if (st != MP2980_OK) {
        return st;
    }

    vout_mv_f = ((float)vref_mv * dev->vout_feedback_ratio) + 0.5f;
    if (vout_mv_f > 65535.0f) {
        return MP2980_ERROR_RANGE;
    }

    *vout_mv = (uint16_t)vout_mv_f;
    return MP2980_OK;
}

/* Read CONTROL1 and decode every bit (async, poll via task). */
mp2980_status_t mp2980_get_control1(mp2980_t *dev, mp2980_control1_t *out)
{
    uint8_t raw = 0u;
    mp2980_status_t st;

    if (out == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read(dev, MP2980_OP_GET_CONTROL1,
                          MP2980_REG_CONTROL1, MP2980_REG_NONE, &raw);
    if (st != MP2980_OK) {
        return st;
    }

    out->raw = raw;
    out->sr = (uint8_t)((raw & MP2980_CONTROL1_SR_MASK) >> 6);
    out->discharge = ((raw & MP2980_CONTROL1_DISCHG_MASK) != 0u);
    out->dither = ((raw & MP2980_CONTROL1_DITHER_MASK) != 0u);
    out->png_latch = ((raw & MP2980_CONTROL1_PNG_LATCH_MASK) != 0u);
    out->go = ((raw & MP2980_CONTROL1_GO_BIT_MASK) != 0u);
    out->enpwr = ((raw & MP2980_CONTROL1_ENPWR_MASK) != 0u);
    return MP2980_OK;
}

/* Read CONTROL2 and decode every bit (async, poll via task). */
mp2980_status_t mp2980_get_control2(mp2980_t *dev, mp2980_control2_t *out)
{
    uint8_t raw = 0u;
    mp2980_status_t st;

    if (out == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read(dev, MP2980_OP_GET_CONTROL2,
                          MP2980_REG_CONTROL2, MP2980_REG_NONE, &raw);
    if (st != MP2980_OK) {
        return st;
    }

    out->raw = raw;
    out->fsw = (uint8_t)((raw & MP2980_CONTROL2_FSW_MASK) >> 6);
    out->bb_fsw = ((raw & MP2980_CONTROL2_BB_FSW_MASK) != 0u);
    out->ocp_mode = (uint8_t)((raw & MP2980_CONTROL2_OCP_MODE_MASK) >> 2);
    out->ovp_mode = (uint8_t)(raw & MP2980_CONTROL2_OVP_MODE_MASK);
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
                              (uint8_t)((uint8_t)slew_rate << 6));
}

/* Get the output voltage slew rate (async, poll via task). */
mp2980_status_t mp2980_get_slew_rate(mp2980_t *dev, mp2980_slew_rate_t *slew_rate)
{
    mp2980_control1_t control1;
    mp2980_status_t st;

    if (slew_rate == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_get_control1(dev, &control1);
    if (st != MP2980_OK) {
        return st;
    }

    *slew_rate = (mp2980_slew_rate_t)control1.sr;
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
                              (uint8_t)((uint8_t)frequency << 6));
}

/* Get the switching frequency (async, poll via task). */
mp2980_status_t mp2980_get_switching_frequency(mp2980_t *dev, mp2980_frequency_t *frequency)
{
    mp2980_control2_t control2;
    mp2980_status_t st;

    if (frequency == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_get_control2(dev, &control2);
    if (st != MP2980_OK) {
        return st;
    }

    *frequency = (mp2980_frequency_t)control2.fsw;
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
                              (uint8_t)((uint8_t)mode << 2));
}

/* Get the overcurrent protection mode (async, poll via task). */
mp2980_status_t mp2980_get_ocp_mode(mp2980_t *dev, mp2980_ocp_mode_t *mode)
{
    mp2980_control2_t control2;
    mp2980_status_t st;

    if (mode == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_get_control2(dev, &control2);
    if (st != MP2980_OK) {
        return st;
    }

    *mode = (mp2980_ocp_mode_t)control2.ocp_mode;
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
    uint8_t field = 0u;
    mp2980_status_t st;

    if (ilim == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read_field(dev, MP2980_OP_GET_ILIM,
                                MP2980_REG_ILIM, MP2980_ILIM_MASK, &field);
    if (st != MP2980_OK) {
        return st;
    }

    *ilim = (mp2980_ilim_t)field;
    return MP2980_OK;
}

/* Get the current limit in mA (async, uses rsense + ILIM register). */
mp2980_status_t mp2980_get_current_limit_current_ma(mp2980_t *dev, uint16_t *current_ma)
{
    uint8_t data = 0u;
    float threshold_mv;
    float current_ma_f;
    mp2980_status_t st;

    if ((dev == 0) || (current_ma == 0)) {
        return MP2980_ERROR_NULL;
    }

    if (dev->avg_current_sense_mohm <= 0.0f) {
        return MP2980_ERROR_RANGE;
    }

    st = mp2980_poll_read(dev, MP2980_OP_GET_ILIM_CURRENT,
                          MP2980_REG_ILIM, MP2980_REG_NONE, &data);
    if (st != MP2980_OK) {
        return st;
    }

    threshold_mv = (float)mp2980_ilim_threshold_0p1mv[data & MP2980_ILIM_MASK] / 10.0f;
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

/* Set current limit by desired current in mA (picks the first level that covers it). */
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
mp2980_status_t mp2980_convert_imon_mv_to_ma(mp2980_t *dev, uint16_t adc_mv,
                                             uint16_t *current_ma)
{
    float current_ma_f;

    if ((dev == 0) || (current_ma == 0)) {
        return MP2980_ERROR_NULL;
    }

    if (dev->avg_current_sense_mohm <= 0.0f) {
        return MP2980_ERROR_RANGE;
    }

    /* Datasheet Eq.(5): V_IMON(mV) = 18.8 x I_OUT(A) x R_SENS(mOhm). */
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
    uint8_t data = 0u;
    mp2980_status_t st;

    if (status == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read(dev, MP2980_OP_GET_INT_STATUS,
                          MP2980_REG_INTERRUPT_STATUS, MP2980_REG_NONE, &data);
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
    return mp2980_write_register(dev, MP2980_REG_INTERRUPT_MASK,
                                 (uint8_t)(mask & MP2980_INT_ALL_MASK));
}

/* Get the interrupt mask (async, poll via task). */
mp2980_status_t mp2980_get_interrupt_mask(mp2980_t *dev, uint8_t *mask)
{
    uint8_t field = 0u;
    mp2980_status_t st;

    if (mask == 0) {
        return MP2980_ERROR_NULL;
    }

    st = mp2980_poll_read_field(dev, MP2980_OP_GET_INT_MASK,
                                MP2980_REG_INTERRUPT_MASK,
                                MP2980_INT_ALL_MASK, &field);
    if (st != MP2980_OK) {
        return st;
    }

    *mask = field;
    return MP2980_OK;
}

/* Abort any pending async operation and reset to IDLE. */
void mp2980_abort(mp2980_t *dev)
{
    if (dev != 0) {
        dev->async_op = MP2980_OP_IDLE;
        dev->poll_ready = 0u;
    }
}
