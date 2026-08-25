/**
 * @file ina226.h
 * @brief Platform-independent non-blocking driver for INA226 current/power monitor.
 * @author LuoXiFNT
 * @date 2026-07-04
 * @lastModified 2026-08-23
 */

#ifndef INA226_H
#define INA226_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @name I2C 7-bit Device Addresses */
/** @{ */
#define INA226_ADDRESS_A1_GND_A0_GND  (0x40u) /**< A1=GND, A0=GND. */
#define INA226_ADDRESS_A1_GND_A0_VS   (0x41u)
#define INA226_ADDRESS_A1_GND_A0_SDA  (0x42u)
#define INA226_ADDRESS_A1_GND_A0_SCL  (0x43u)
#define INA226_ADDRESS_A1_VS_A0_GND   (0x44u)
#define INA226_ADDRESS_A1_VS_A0_VS    (0x45u)
#define INA226_ADDRESS_A1_VS_A0_SDA   (0x46u)
#define INA226_ADDRESS_A1_VS_A0_SCL   (0x47u)
#define INA226_ADDRESS_A1_SDA_A0_GND  (0x48u)
#define INA226_ADDRESS_A1_SDA_A0_VS   (0x49u)
#define INA226_ADDRESS_A1_SDA_A0_SDA  (0x4Au)
#define INA226_ADDRESS_A1_SDA_A0_SCL  (0x4Bu)
#define INA226_ADDRESS_A1_SCL_A0_GND  (0x4Cu)
#define INA226_ADDRESS_A1_SCL_A0_VS   (0x4Du)
#define INA226_ADDRESS_A1_SCL_A0_SDA  (0x4Eu)
#define INA226_ADDRESS_A1_SCL_A0_SCL  (0x4Fu) /**< A1=SCL, A0=SCL. */
/** @} */

/** @name Register Addresses */
/** @{ */
#define INA226_REG_CONFIGURATION      (0x00u) /**< Configuration register. */
#define INA226_REG_SHUNT_VOLTAGE      (0x01u)
#define INA226_REG_BUS_VOLTAGE        (0x02u)
#define INA226_REG_POWER              (0x03u)
#define INA226_REG_CURRENT            (0x04u)
#define INA226_REG_CALIBRATION        (0x05u)
#define INA226_REG_MASK_ENABLE        (0x06u)
#define INA226_REG_ALERT_LIMIT        (0x07u)
#define INA226_REG_MANUFACTURER_ID    (0xFEu)
#define INA226_REG_DIE_ID             (0xFFu) /**< Die identification register. */
/** @} */

/** @name Fixed register constants */
/** @{ */
#define INA226_CONFIGURATION_DEFAULT  (0x4127u)
#define INA226_MANUFACTURER_ID_VALUE  (0x5449u)
#define INA226_DIE_ID_VALUE           (0x2260u)
#define INA226_BUS_LSB_UV             (1250u)
#define INA226_SHUNT_LSB_0P1UV        (25u)
/** @} */

/** @name Configuration Register Bit Masks */
/** @{ */
#define INA226_CONFIG_RESET_MASK      (0x8000u)
#define INA226_CONFIG_RESERVED_MASK   (0x4000u)
#define INA226_CONFIG_AVG_MASK        (0x0E00u)
#define INA226_CONFIG_VBUSCT_MASK     (0x01C0u)
#define INA226_CONFIG_VSHCT_MASK      (0x0038u)
#define INA226_CONFIG_MODE_MASK       (0x0007u)
#define INA226_CONFIG_AVG_SHIFT       (9u)
#define INA226_CONFIG_VBUSCT_SHIFT    (6u)
#define INA226_CONFIG_VSHCT_SHIFT     (3u)
/** @} */

/** @name Mask/Enable Register Bit Masks */
/** @{ */
#define INA226_MASK_SOL               (0x8000u)
#define INA226_MASK_SUL               (0x4000u)
#define INA226_MASK_BOL               (0x2000u)
#define INA226_MASK_BUL               (0x1000u)
#define INA226_MASK_POL               (0x0800u)
#define INA226_MASK_CNVR              (0x0400u)
#define INA226_MASK_AFF               (0x0010u)
#define INA226_MASK_CVRF              (0x0008u)
#define INA226_MASK_OVF               (0x0004u)
#define INA226_MASK_APOL              (0x0002u)
#define INA226_MASK_LEN               (0x0001u)
/** @} */

/** @brief INA226 driver return status codes. */
typedef enum {
    INA226_OK = 0,              /**< Operation completed successfully. */
    INA226_BUSY = 1,            /**< Operation is still in progress. */
    INA226_ERROR = -1,          /**< Generic bus or device error. */
    INA226_ERROR_NULL = -2,     /**< Null pointer or missing callback. */
    INA226_ERROR_RANGE = -3,    /**< Parameter is outside the valid range. */
    INA226_ERROR_TIMEOUT = -4   /**< Operation timed out. */
} ina226_status_t;

typedef enum {
    INA226_AVERAGES_1    = 0u,
    INA226_AVERAGES_4    = 1u,
    INA226_AVERAGES_16   = 2u,
    INA226_AVERAGES_64   = 3u,
    INA226_AVERAGES_128  = 4u,
    INA226_AVERAGES_256  = 5u,
    INA226_AVERAGES_512  = 6u,
    INA226_AVERAGES_1024 = 7u
} ina226_averages_t;

typedef enum {
    INA226_CONV_TIME_140US  = 0u,
    INA226_CONV_TIME_204US  = 1u,
    INA226_CONV_TIME_332US  = 2u,
    INA226_CONV_TIME_588US  = 3u,
    INA226_CONV_TIME_1100US = 4u,
    INA226_CONV_TIME_2116US = 5u,
    INA226_CONV_TIME_4156US = 6u,
    INA226_CONV_TIME_8244US = 7u
} ina226_conversion_time_t;

typedef enum {
    INA226_MODE_POWER_DOWN_0             = 0u,
    INA226_MODE_SHUNT_TRIGGERED         = 1u,
    INA226_MODE_BUS_TRIGGERED           = 2u,
    INA226_MODE_SHUNT_BUS_TRIGGERED     = 3u,
    INA226_MODE_POWER_DOWN_1             = 4u,
    INA226_MODE_SHUNT_CONTINUOUS        = 5u,
    INA226_MODE_BUS_CONTINUOUS          = 6u,
    INA226_MODE_SHUNT_BUS_CONTINUOUS    = 7u
} ina226_mode_t;

/** @brief Decoded MASK_ENABLE register status. */
typedef struct {
    bool shunt_over_voltage_alert_enabled; /**< Shunt over-voltage alert enable. */
    bool shunt_under_voltage_alert_enabled;/**< Shunt under-voltage alert enable. */
    bool bus_over_voltage_alert_enabled;   /**< Bus over-voltage alert enable. */
    bool bus_under_voltage_alert_enabled;  /**< Bus under-voltage alert enable. */
    bool power_over_limit_alert_enabled;   /**< Power-over-limit alert enable. */
    bool conversion_ready_alert_enabled;  /**< Conversion-ready alert enable. */
    bool alert_function_flag;              /**< Alert function flag. */
    bool conversion_ready_flag;            /**< Conversion-ready flag. */
    bool math_overflow_flag;               /**< Math overflow flag. */
    bool alert_polarity_high;              /**< Active-high alert polarity. */
    bool latch_enabled;                    /**< Latched alert mode. */
    uint16_t raw;                          /**< Raw MASK_ENABLE value. */
} ina226_mask_enable_status_t;

typedef int (*ina226_write_reg_cb_t)(void *user, uint8_t dev_addr_7bit,
                                     uint8_t reg, const uint8_t *data, uint16_t len);
typedef int (*ina226_read_reg_cb_t)(void *user, uint8_t dev_addr_7bit,
                                    uint8_t reg, uint8_t *data, uint16_t len);
typedef uint8_t (*ina226_bus_idle_cb_t)(void *user);

/** @brief INA226 device instance and asynchronous operation state. */
typedef struct {
    uint8_t dev_addr_7bit;           /**< 7-bit I2C address. */
    uint16_t shunt_resistor_mohm;    /**< Shunt resistance in mOhm. */
    uint16_t current_lsb_ua;         /**< Current LSB in microamps. */
    ina226_write_reg_cb_t write_reg; /**< Register write callback. */
    ina226_read_reg_cb_t read_reg;   /**< Register read callback. */
    ina226_bus_idle_cb_t is_bus_idle;/**< Bus idle callback. */
    void *user;                      /**< Callback context pointer. */

    uint8_t async_op;                /**< Current asynchronous operation. */
    uint8_t update_reg;              /**< Register used by update_bits. */
    uint16_t update_mask;             /**< Bit mask used by update_bits. */
    uint16_t update_value;            /**< Bit value used by update_bits. */
    uint8_t direct_read_reg;         /**< Register used by direct read. */
    volatile uint8_t poll_ready;     /**< Set when an asynchronous read completes. */
    uint8_t bus_was_idle;            /**< Idle-edge latch for polling. */
    uint8_t io_data[2];              /**< Temporary transfer buffer. */
} ina226_t;

/** @brief Initialize an INA226 instance and register bus callbacks. */
ina226_status_t ina226_init(ina226_t *dev, uint8_t dev_addr_7bit,
                            uint16_t shunt_resistor_mohm, uint16_t current_lsb_ua,
                            ina226_write_reg_cb_t write_reg, ina226_read_reg_cb_t read_reg,
                            ina226_bus_idle_cb_t is_bus_idle, void *user);
/** @brief Advance the non-blocking INA226 state machine. */
ina226_status_t ina226_task(ina226_t *dev);
/** @brief Return non-zero when an asynchronous operation is pending. */
uint8_t ina226_is_busy(const ina226_t *dev);
/** @brief Write one 16-bit register value. */
ina226_status_t ina226_write_register(ina226_t *dev, uint8_t reg, uint16_t data);
/** @brief Read one 16-bit register value asynchronously. */
ina226_status_t ina226_read_register(ina226_t *dev, uint8_t reg, uint16_t *data);
/** @brief Start a read-modify-write operation for selected bits. */
ina226_status_t ina226_update_bits(ina226_t *dev, uint8_t reg, uint16_t mask, uint16_t value);
/** @brief Set the current conversion LSB in microamps. */
void ina226_set_current_lsb_ua(ina226_t *dev, uint16_t current_lsb_ua);
/** @brief Set the shunt resistance used for calibration. */
void ina226_set_shunt_resistor_mohm(ina226_t *dev, uint16_t shunt_resistor_mohm);
/** @brief Calculate the INA226 calibration register value. */
uint16_t ina226_calculate_calibration(uint16_t shunt_resistor_mohm, uint16_t current_lsb_ua);
/** @brief Convert a raw shunt-voltage register value to microvolts. */
int32_t ina226_convert_shunt_voltage_uv(uint16_t raw);
/** @brief Convert a raw bus-voltage register value to millivolts. */
uint16_t ina226_convert_bus_voltage_mv(uint16_t raw);
/** @brief Convert a raw current register value to milliamps. */
int32_t ina226_convert_current_ma(uint16_t raw, uint16_t current_lsb_ua);
/** @brief Convert a raw power register value to milliwatts. */
uint32_t ina226_convert_power_mw(uint16_t raw, uint16_t current_lsb_ua);
/** @brief Write the calibration register. */
ina226_status_t ina226_set_calibration(ina226_t *dev, uint16_t calibration);
/** @brief Calculate and write the calibration register. */
ina226_status_t ina226_calibrate(ina226_t *dev);
/** @brief Build a configuration register value from conversion settings. */
uint16_t ina226_build_config(ina226_averages_t averages, ina226_conversion_time_t bus_ct,
                             ina226_conversion_time_t shunt_ct, ina226_mode_t mode);
/** @brief Reset the INA226 configuration register. */
ina226_status_t ina226_reset(ina226_t *dev);
/** @brief Configure averaging, conversion times, and operating mode. */
ina226_status_t ina226_configure(ina226_t *dev, ina226_averages_t averages,
                                 ina226_conversion_time_t bus_ct,
                                 ina226_conversion_time_t shunt_ct, ina226_mode_t mode);
/** @brief Read the configuration register. */
ina226_status_t ina226_get_configuration(ina226_t *dev, uint16_t *config);
/** @brief Read and convert the shunt voltage. */
ina226_status_t ina226_get_shunt_voltage_uv(ina226_t *dev, int32_t *shunt_uv);
/** @brief Read and convert the bus voltage. */
ina226_status_t ina226_get_bus_voltage_mv(ina226_t *dev, uint16_t *bus_mv);
/** @brief Read and convert the current. */
ina226_status_t ina226_get_current_ma(ina226_t *dev, int32_t *current_ma);
/** @brief Read and convert the power. */
ina226_status_t ina226_get_power_mw(ina226_t *dev, uint32_t *power_mw);
/** @brief Read the calibration register. */
ina226_status_t ina226_get_calibration(ina226_t *dev, uint16_t *calibration);
/** @brief Write the MASK_ENABLE register. */
ina226_status_t ina226_set_mask_enable(ina226_t *dev, uint16_t mask_enable);
/** @brief Read and decode the MASK_ENABLE register. */
ina226_status_t ina226_get_mask_enable(ina226_t *dev, ina226_mask_enable_status_t *status);
/** @brief Write the alert-limit register. */
ina226_status_t ina226_set_alert_limit(ina226_t *dev, uint16_t alert_limit);
/** @brief Read the alert-limit register. */
ina226_status_t ina226_get_alert_limit(ina226_t *dev, uint16_t *alert_limit);
/** @brief Enable or disable the conversion-ready alert. */
ina226_status_t ina226_enable_conversion_ready_alert(ina226_t *dev, bool enable);
/** @brief Enable or disable alert latching. */
ina226_status_t ina226_set_alert_latch(ina226_t *dev, bool enable);
/** @brief Select active-high or active-low alert polarity. */
ina226_status_t ina226_set_alert_polarity_high(ina226_t *dev, bool active_high);
/** @brief Read the manufacturer identification register. */
ina226_status_t ina226_get_manufacturer_id(ina226_t *dev, uint16_t *manufacturer_id);
/** @brief Read the die identification register. */
ina226_status_t ina226_get_die_id(ina226_t *dev, uint16_t *die_id);

/** @brief Abort the pending operation and return the instance to idle. */
void ina226_abort(ina226_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* INA226_H */
