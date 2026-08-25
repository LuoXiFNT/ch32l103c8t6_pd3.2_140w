/**
 * @file ina226.c
 * @brief Platform-independent non-blocking INA226 register driver implementation.
 * @author LuoXiFNT
 * @date 2026-07-04
 * @lastModified 2026-08-23
 */
#include "ina226.h"

#define INA226_OP_IDLE             (0u) /**< No asynchronous operation. */
#define INA226_OP_UPDATE_READ      (1u) /**< Read phase of read-modify-write. */
#define INA226_OP_UPDATE_WRITE     (2u) /**< Write phase of read-modify-write. */
#define INA226_OP_READ_REGISTER    (3u) /**< Direct register read. */
#define INA226_OP_GET_CONFIG       (4u) /**< Configuration read. */
#define INA226_OP_GET_SHUNT        (5u) /**< Shunt-voltage read. */
#define INA226_OP_GET_BUS          (6u) /**< Bus-voltage read. */
#define INA226_OP_GET_POWER        (7u) /**< Power read. */
#define INA226_OP_GET_CURRENT      (8u) /**< Current read. */
#define INA226_OP_GET_CAL          (9u) /**< Calibration read. */
#define INA226_OP_GET_MASK         (10u) /**< Mask/enable read. */
#define INA226_OP_GET_ALERT_LIMIT  (11u) /**< Alert-limit read. */
#define INA226_OP_GET_MFR_ID       (12u) /**< Manufacturer ID read. */
#define INA226_OP_GET_DIE_ID       (13u) /**< Die ID read. */

/**
 * @brief Decode a big-endian 16-bit register value.
 * @param data Pointer to the two-byte register data.
 * @return uint16_t Decoded register value.
 */
static uint16_t ina226_make_u16_be(const uint8_t data[2])
{
    return (uint16_t)(((uint16_t)data[0] << 8u) | (uint16_t)data[1]);
}

/**
 * @brief Encode a 16-bit register value in big-endian order.
 * @param value Register value to encode.
 * @param data Destination two-byte buffer.
 */
static void ina226_put_u16_be(uint16_t value, uint8_t data[2])
{
    data[0] = (uint8_t)((value >> 8u) & 0xFFu);
    data[1] = (uint8_t)(value & 0xFFu);
}

/**
 * @brief Validate that the device and write callback are non-null.
 * @param dev Pointer to the INA226 device instance.
 * @return ina226_status_t INA226_OK if valid, INA226_ERROR_NULL otherwise.
 */
static ina226_status_t ina226_check_write_dev(ina226_t *dev)
{
    if ((dev == 0) || (dev->write_reg == 0)) return INA226_ERROR_NULL;
    return INA226_OK;
}

/**
 * @brief Validate that the device, read callback, and idle callback are non-null.
 * @param dev Pointer to the INA226 device instance.
 * @return ina226_status_t INA226_OK if valid, INA226_ERROR_NULL otherwise.
 */
static ina226_status_t ina226_check_read_dev(ina226_t *dev)
{
    if ((dev == 0) || (dev->read_reg == 0) || (dev->is_bus_idle == 0)) return INA226_ERROR_NULL;
    return INA226_OK;
}

/**
 * @brief Check whether the I2C bus is idle via the registered callback.
 * @param dev Pointer to the INA226 device instance.
 * @return uint8_t 1 if idle or no callback, 0 if busy.
 */
static uint8_t ina226_bus_is_idle(ina226_t *dev)
{
    if ((dev == 0) || (dev->is_bus_idle == 0)) return 1u;
    return dev->is_bus_idle(dev->user);
}

/**
 * @brief Start an asynchronous two-byte register read.
 * @param dev Pointer to the INA226 device instance.
 * @param op Asynchronous operation code for completion identification.
 * @param reg Register address to read.
 * @return ina226_status_t INA226_BUSY if started, otherwise an error code.
 */
static ina226_status_t ina226_start_read16(ina226_t *dev, uint8_t op, uint8_t reg)
{
    ina226_status_t st = ina226_check_read_dev(dev);
    if (st != INA226_OK) return st;
    if (dev->async_op != INA226_OP_IDLE) return INA226_BUSY;
    if (ina226_bus_is_idle(dev) == 0u) return INA226_BUSY;

    if (dev->read_reg(dev->user, dev->dev_addr_7bit, reg, dev->io_data, 2u) != 0)
        return INA226_ERROR;

    dev->direct_read_reg = reg;
    dev->async_op = op;
    dev->poll_ready = 0u;
    dev->bus_was_idle = 0u;
    return INA226_BUSY;
}

/**
 * @brief Complete or start a polling-style two-byte register read.
 * @param dev Pointer to the INA226 device instance.
 * @param op Asynchronous operation code for completion identification.
 * @param reg Register address to read.
 * @param data Pointer to store the completed register value.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY while pending,
 *         or an error code.
 */
static ina226_status_t ina226_poll_read16(ina226_t *dev, uint8_t op, uint8_t reg, uint16_t *data)
{
    if (data == 0) return INA226_ERROR_NULL;

    if ((dev != 0) && (dev->async_op == op)) {
        if (dev->poll_ready == 0u) return INA226_BUSY;
        *data = ina226_make_u16_be(dev->io_data);
        dev->async_op = INA226_OP_IDLE;
        dev->poll_ready = 0u;
        return INA226_OK;
    }
    return ina226_start_read16(dev, op, reg);
}

/**
 * @brief Validate an INA226 address in the supported 0x40-0x4F range.
 * @param dev_addr_7bit Seven-bit INA226 device address.
 * @return ina226_status_t INA226_OK if valid, INA226_ERROR_RANGE otherwise.
 */
static ina226_status_t ina226_validate_address(uint8_t dev_addr_7bit)
{
    if ((dev_addr_7bit < INA226_ADDRESS_A1_GND_A0_GND) ||
        (dev_addr_7bit > INA226_ADDRESS_A1_SCL_A0_SCL))
        return INA226_ERROR_RANGE;
    return INA226_OK;
}

ina226_status_t ina226_init(ina226_t *dev, uint8_t dev_addr_7bit,
                            uint16_t shunt_resistor_mohm, uint16_t current_lsb_ua,
                            ina226_write_reg_cb_t write_reg, ina226_read_reg_cb_t read_reg,
                            ina226_bus_idle_cb_t is_bus_idle, void *user)
{
    if ((dev == 0) || (write_reg == 0)) return INA226_ERROR_NULL;
    if (ina226_validate_address(dev_addr_7bit) != INA226_OK) return INA226_ERROR_RANGE;
    if ((shunt_resistor_mohm == 0u) || (current_lsb_ua == 0u)) return INA226_ERROR_RANGE;

    dev->dev_addr_7bit = dev_addr_7bit;
    dev->shunt_resistor_mohm = shunt_resistor_mohm;
    dev->current_lsb_ua = current_lsb_ua;
    dev->write_reg = write_reg;
    dev->read_reg = read_reg;
    dev->is_bus_idle = is_bus_idle;
    dev->user = user;
    dev->async_op = INA226_OP_IDLE;
    dev->update_reg = 0u;
    dev->update_mask = 0u;
    dev->update_value = 0u;
    dev->direct_read_reg = 0u;
    dev->poll_ready = 0u;
    dev->io_data[0] = 0u;
    dev->io_data[1] = 0u;
    return INA226_OK;
}

ina226_status_t ina226_task(ina226_t *dev)
{
    uint16_t data;
    ina226_status_t st;

    if (dev == 0) return INA226_ERROR_NULL;

    if (dev->async_op == INA226_OP_UPDATE_READ) {
        if (ina226_bus_is_idle(dev) == 0u) return INA226_BUSY;
        data = ina226_make_u16_be(dev->io_data);
        data = (uint16_t)((data & (uint16_t)(~dev->update_mask)) |
                          (dev->update_value & dev->update_mask));
        st = ina226_write_register(dev, dev->update_reg, data);
        if (st != INA226_OK) { dev->async_op = INA226_OP_IDLE; return st; }
        dev->async_op = INA226_OP_UPDATE_WRITE;
        dev->bus_was_idle = 0u;
        return INA226_BUSY;
    }

    if (dev->async_op == INA226_OP_UPDATE_WRITE) {
        if (ina226_bus_is_idle(dev) != 0u) {
            dev->bus_was_idle = 1u;
        }
        if (dev->bus_was_idle != 0u) {
            dev->async_op = INA226_OP_IDLE;
            return INA226_OK;
        }
        return INA226_BUSY;
    }

    if ((dev->async_op != INA226_OP_IDLE) &&
        (dev->async_op != INA226_OP_UPDATE_READ) &&
        (dev->async_op != INA226_OP_UPDATE_WRITE)) {
        if (ina226_bus_is_idle(dev) != 0u) {
            dev->bus_was_idle = 1u;
        }
        if (dev->bus_was_idle != 0u) {
            dev->poll_ready = 1u;
        }
        return INA226_BUSY;
    }

    return (dev->async_op == INA226_OP_IDLE) ? INA226_OK : INA226_BUSY;
}

uint8_t ina226_is_busy(const ina226_t *dev)
{
    if (dev == 0) return 0u;
    return (dev->async_op != INA226_OP_IDLE) ? 1u : 0u;
}

ina226_status_t ina226_write_register(ina226_t *dev, uint8_t reg, uint16_t data)
{
    uint8_t bytes[2];
    ina226_status_t st = ina226_check_write_dev(dev);
    if (st != INA226_OK) return st;
    if (ina226_bus_is_idle(dev) == 0u) return INA226_BUSY;
    ina226_put_u16_be(data, bytes);
    return (dev->write_reg(dev->user, dev->dev_addr_7bit, reg, bytes, 2u) == 0) ? INA226_OK : INA226_ERROR;
}

ina226_status_t ina226_read_register(ina226_t *dev, uint8_t reg, uint16_t *data)
{
    if ((dev != 0) && (dev->async_op == INA226_OP_READ_REGISTER) &&
        (dev->direct_read_reg != reg)) return INA226_BUSY;
    return ina226_poll_read16(dev, INA226_OP_READ_REGISTER, reg, data);
}

ina226_status_t ina226_update_bits(ina226_t *dev, uint8_t reg, uint16_t mask, uint16_t value)
{
    ina226_status_t st = ina226_check_read_dev(dev);
    if (st != INA226_OK) return st;
    if (dev->async_op != INA226_OP_IDLE) return INA226_BUSY;
    if (ina226_bus_is_idle(dev) == 0u) return INA226_BUSY;
    if (dev->read_reg(dev->user, dev->dev_addr_7bit, reg, dev->io_data, 2u) != 0)
        return INA226_ERROR;
    dev->update_reg = reg;
    dev->update_mask = mask;
    dev->update_value = value;
    dev->async_op = INA226_OP_UPDATE_READ;
    return INA226_OK;
}

void ina226_set_current_lsb_ua(ina226_t *dev, uint16_t current_lsb_ua)
{
    if ((dev != 0) && (current_lsb_ua != 0u)) dev->current_lsb_ua = current_lsb_ua;
}

void ina226_set_shunt_resistor_mohm(ina226_t *dev, uint16_t shunt_resistor_mohm)
{
    if ((dev != 0) && (shunt_resistor_mohm != 0u)) dev->shunt_resistor_mohm = shunt_resistor_mohm;
}

uint16_t ina226_calculate_calibration(uint16_t shunt_resistor_mohm, uint16_t current_lsb_ua)
{
    uint32_t denom, cal;
    if ((shunt_resistor_mohm == 0u) || (current_lsb_ua == 0u)) return 0u;
    denom = (uint32_t)shunt_resistor_mohm * (uint32_t)current_lsb_ua;
    cal = 5120000UL / denom;
    if (cal > 0x7FFFu) cal = 0x7FFFu;
    return (uint16_t)cal;
}

int32_t ina226_convert_shunt_voltage_uv(uint16_t raw)
{
    return ((int32_t)((int16_t)raw) * (int32_t)INA226_SHUNT_LSB_0P1UV) / 10;
}

uint16_t ina226_convert_bus_voltage_mv(uint16_t raw)
{
    return (uint16_t)(((uint32_t)(raw & 0x7FFFu) *
                       (uint32_t)INA226_BUS_LSB_UV) / 1000UL);
}

int32_t ina226_convert_current_ma(uint16_t raw, uint16_t current_lsb_ua)
{
    if (current_lsb_ua == 0u) {
        return 0;
    }

    return ((int32_t)((int16_t)raw) * (int32_t)current_lsb_ua) / 1000;
}

uint32_t ina226_convert_power_mw(uint16_t raw, uint16_t current_lsb_ua)
{
    return ((uint32_t)raw * 25UL * (uint32_t)current_lsb_ua) / 1000UL;
}

ina226_status_t ina226_set_calibration(ina226_t *dev, uint16_t calibration)
{
    if (calibration > 0x7FFFu) return INA226_ERROR_RANGE;
    return ina226_write_register(dev, INA226_REG_CALIBRATION, calibration);
}

ina226_status_t ina226_calibrate(ina226_t *dev)
{
    uint16_t cal;
    if (dev == 0) return INA226_ERROR_NULL;
    cal = ina226_calculate_calibration(dev->shunt_resistor_mohm, dev->current_lsb_ua);
    if (cal == 0u) return INA226_ERROR_RANGE;
    return ina226_set_calibration(dev, cal);
}

uint16_t ina226_build_config(ina226_averages_t averages, ina226_conversion_time_t bus_ct,
                             ina226_conversion_time_t shunt_ct, ina226_mode_t mode)
{
    return (uint16_t)(INA226_CONFIG_RESERVED_MASK |
        (((uint16_t)averages << INA226_CONFIG_AVG_SHIFT) & INA226_CONFIG_AVG_MASK) |
        (((uint16_t)bus_ct << INA226_CONFIG_VBUSCT_SHIFT) & INA226_CONFIG_VBUSCT_MASK) |
        (((uint16_t)shunt_ct << INA226_CONFIG_VSHCT_SHIFT) & INA226_CONFIG_VSHCT_MASK) |
        ((uint16_t)mode & INA226_CONFIG_MODE_MASK));
}

ina226_status_t ina226_reset(ina226_t *dev)
{
    return ina226_write_register(dev, INA226_REG_CONFIGURATION, INA226_CONFIG_RESET_MASK);
}

ina226_status_t ina226_configure(ina226_t *dev, ina226_averages_t averages,
                                 ina226_conversion_time_t bus_ct,
                                 ina226_conversion_time_t shunt_ct, ina226_mode_t mode)
{
    if (((uint8_t)averages > 7u) || ((uint8_t)bus_ct > 7u) ||
        ((uint8_t)shunt_ct > 7u) || ((uint8_t)mode > 7u)) return INA226_ERROR_RANGE;
    return ina226_write_register(dev, INA226_REG_CONFIGURATION,
                                 ina226_build_config(averages, bus_ct, shunt_ct, mode));
}

ina226_status_t ina226_get_configuration(ina226_t *dev, uint16_t *config)
{
    return ina226_poll_read16(dev, INA226_OP_GET_CONFIG, INA226_REG_CONFIGURATION, config);
}

ina226_status_t ina226_get_shunt_voltage_uv(ina226_t *dev, int32_t *shunt_uv)
{
    uint16_t raw_u16;
    ina226_status_t st;
    if (shunt_uv == 0) return INA226_ERROR_NULL;
    st = ina226_poll_read16(dev, INA226_OP_GET_SHUNT, INA226_REG_SHUNT_VOLTAGE, &raw_u16);
    if (st != INA226_OK) return st;
    *shunt_uv = ina226_convert_shunt_voltage_uv(raw_u16);
    return INA226_OK;
}

ina226_status_t ina226_get_bus_voltage_mv(ina226_t *dev, uint16_t *bus_mv)
{
    uint16_t raw;
    ina226_status_t st;
    if (bus_mv == 0) return INA226_ERROR_NULL;
    st = ina226_poll_read16(dev, INA226_OP_GET_BUS, INA226_REG_BUS_VOLTAGE, &raw);
    if (st != INA226_OK) return st;
    *bus_mv = ina226_convert_bus_voltage_mv(raw);
    return INA226_OK;
}

ina226_status_t ina226_get_current_ma(ina226_t *dev, int32_t *current_ma)
{
    uint16_t raw_u16;
    ina226_status_t st;
    if ((dev == 0) || (current_ma == 0)) return INA226_ERROR_NULL;
    st = ina226_poll_read16(dev, INA226_OP_GET_CURRENT, INA226_REG_CURRENT, &raw_u16);
    if (st != INA226_OK) return st;
    *current_ma = ina226_convert_current_ma(raw_u16, dev->current_lsb_ua);
    return INA226_OK;
}

ina226_status_t ina226_get_power_mw(ina226_t *dev, uint32_t *power_mw)
{
    uint16_t raw;
    ina226_status_t st;
    if ((dev == 0) || (power_mw == 0)) return INA226_ERROR_NULL;
    st = ina226_poll_read16(dev, INA226_OP_GET_POWER, INA226_REG_POWER, &raw);
    if (st != INA226_OK) return st;
    *power_mw = ina226_convert_power_mw(raw, dev->current_lsb_ua);
    return INA226_OK;
}

ina226_status_t ina226_get_calibration(ina226_t *dev, uint16_t *calibration)
{
    return ina226_poll_read16(dev, INA226_OP_GET_CAL, INA226_REG_CALIBRATION, calibration);
}

ina226_status_t ina226_set_mask_enable(ina226_t *dev, uint16_t mask_enable)
{
    return ina226_write_register(dev, INA226_REG_MASK_ENABLE, mask_enable);
}

ina226_status_t ina226_get_mask_enable(ina226_t *dev, ina226_mask_enable_status_t *status)
{
    uint16_t raw;
    ina226_status_t st;
    if (status == 0) return INA226_ERROR_NULL;
    st = ina226_poll_read16(dev, INA226_OP_GET_MASK, INA226_REG_MASK_ENABLE, &raw);
    if (st != INA226_OK) return st;
    status->raw = raw;
    status->shunt_over_voltage_alert_enabled = ((raw & INA226_MASK_SOL) != 0u);
    status->shunt_under_voltage_alert_enabled = ((raw & INA226_MASK_SUL) != 0u);
    status->bus_over_voltage_alert_enabled = ((raw & INA226_MASK_BOL) != 0u);
    status->bus_under_voltage_alert_enabled = ((raw & INA226_MASK_BUL) != 0u);
    status->power_over_limit_alert_enabled = ((raw & INA226_MASK_POL) != 0u);
    status->conversion_ready_alert_enabled = ((raw & INA226_MASK_CNVR) != 0u);
    status->alert_function_flag = ((raw & INA226_MASK_AFF) != 0u);
    status->conversion_ready_flag = ((raw & INA226_MASK_CVRF) != 0u);
    status->math_overflow_flag = ((raw & INA226_MASK_OVF) != 0u);
    status->alert_polarity_high = ((raw & INA226_MASK_APOL) != 0u);
    status->latch_enabled = ((raw & INA226_MASK_LEN) != 0u);
    return INA226_OK;
}

ina226_status_t ina226_set_alert_limit(ina226_t *dev, uint16_t alert_limit)
{
    return ina226_write_register(dev, INA226_REG_ALERT_LIMIT, alert_limit);
}

ina226_status_t ina226_get_alert_limit(ina226_t *dev, uint16_t *alert_limit)
{
    return ina226_poll_read16(dev, INA226_OP_GET_ALERT_LIMIT, INA226_REG_ALERT_LIMIT, alert_limit);
}

ina226_status_t ina226_enable_conversion_ready_alert(ina226_t *dev, bool enable)
{
    return ina226_update_bits(dev, INA226_REG_MASK_ENABLE, INA226_MASK_CNVR,
                              enable ? INA226_MASK_CNVR : 0u);
}

ina226_status_t ina226_set_alert_latch(ina226_t *dev, bool enable)
{
    return ina226_update_bits(dev, INA226_REG_MASK_ENABLE, INA226_MASK_LEN,
                              enable ? INA226_MASK_LEN : 0u);
}

ina226_status_t ina226_set_alert_polarity_high(ina226_t *dev, bool active_high)
{
    return ina226_update_bits(dev, INA226_REG_MASK_ENABLE, INA226_MASK_APOL,
                              active_high ? INA226_MASK_APOL : 0u);
}

ina226_status_t ina226_get_manufacturer_id(ina226_t *dev, uint16_t *manufacturer_id)
{
    return ina226_poll_read16(dev, INA226_OP_GET_MFR_ID, INA226_REG_MANUFACTURER_ID, manufacturer_id);
}

ina226_status_t ina226_get_die_id(ina226_t *dev, uint16_t *die_id)
{
    return ina226_poll_read16(dev, INA226_OP_GET_DIE_ID, INA226_REG_DIE_ID, die_id);
}

/* Abort any pending async operation and reset to IDLE. */
void ina226_abort(ina226_t *dev)
{
    if (dev != 0) {
        dev->async_op = INA226_OP_IDLE;
        dev->poll_ready = 0u;
        dev->bus_was_idle = 0u;
    }
}
