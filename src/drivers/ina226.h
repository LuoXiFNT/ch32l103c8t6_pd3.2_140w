/**
 * @file ina226.h
 * @brief Platform-independent non-blocking driver for INA226 current/power monitor.
 * @author LuoXiFNT
 * @date 2026-07-04
 * @lastModified 2026-11-05
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
#define INA226_ADDRESS_A1_GND_A0_VS   (0x41u) /**< A1=GND, A0=VS. */
#define INA226_ADDRESS_A1_GND_A0_SDA  (0x42u) /**< A1=GND, A0=SDA. */
#define INA226_ADDRESS_A1_GND_A0_SCL  (0x43u) /**< A1=GND, A0=SCL. */
#define INA226_ADDRESS_A1_VS_A0_GND   (0x44u) /**< A1=VS, A0=GND. */
#define INA226_ADDRESS_A1_VS_A0_VS    (0x45u) /**< A1=VS, A0=VS. */
#define INA226_ADDRESS_A1_VS_A0_SDA   (0x46u) /**< A1=VS, A0=SDA. */
#define INA226_ADDRESS_A1_VS_A0_SCL   (0x47u) /**< A1=VS, A0=SCL. */
#define INA226_ADDRESS_A1_SDA_A0_GND  (0x48u) /**< A1=SDA, A0=GND. */
#define INA226_ADDRESS_A1_SDA_A0_VS   (0x49u) /**< A1=SDA, A0=VS. */
#define INA226_ADDRESS_A1_SDA_A0_SDA  (0x4Au) /**< A1=SDA, A0=SDA. */
#define INA226_ADDRESS_A1_SDA_A0_SCL  (0x4Bu) /**< A1=SDA, A0=SCL. */
#define INA226_ADDRESS_A1_SCL_A0_GND  (0x4Cu) /**< A1=SCL, A0=GND. */
#define INA226_ADDRESS_A1_SCL_A0_VS   (0x4Du) /**< A1=SCL, A0=VS. */
#define INA226_ADDRESS_A1_SCL_A0_SDA  (0x4Eu) /**< A1=SCL, A0=SDA. */
#define INA226_ADDRESS_A1_SCL_A0_SCL  (0x4Fu) /**< A1=SCL, A0=SCL. */
/** @} */

/** @name Register Addresses */
/** @{ */
#define INA226_REG_CONFIGURATION      (0x00u) /**< Configuration register. */
#define INA226_REG_SHUNT_VOLTAGE      (0x01u) /**< Shunt voltage register (2.5 uV/LSB, signed). */
#define INA226_REG_BUS_VOLTAGE        (0x02u) /**< Bus voltage register (1.25 mV/LSB). */
#define INA226_REG_POWER              (0x03u) /**< Power register (25 x current LSB per bit). */
#define INA226_REG_CURRENT            (0x04u) /**< Current register (current LSB per bit). */
#define INA226_REG_CALIBRATION        (0x05u) /**< Calibration register. */
#define INA226_REG_MASK_ENABLE        (0x06u) /**< Mask/enable and status flags. */
#define INA226_REG_ALERT_LIMIT        (0x07u) /**< Alert limit register. */
#define INA226_REG_MANUFACTURER_ID    (0xFEu) /**< Manufacturer identification (0x5449). */
#define INA226_REG_DIE_ID             (0xFFu) /**< Die identification (0x2260). */
/** @} */

/** @name Fixed register constants */
/** @{ */
#define INA226_CONFIGURATION_DEFAULT  (0x4127u) /**< Power-on reset value of the configuration register. */
#define INA226_MANUFACTURER_ID_VALUE  (0x5449u) /**< Expected manufacturer ID. */
#define INA226_DIE_ID_VALUE           (0x2260u) /**< Expected die ID. */
#define INA226_BUS_LSB_UV             (1250u) /**< Bus voltage LSB in microvolts. */
#define INA226_SHUNT_LSB_0P1UV        (25u) /**< Shunt voltage LSB in 0.1 uV units (2.5 uV). */
/** @} */

/** @name Configuration Register Bit Masks */
/** @{ */
#define INA226_CONFIG_RESET_MASK      (0x8000u) /**< RST: self-clearing system reset. */
#define INA226_CONFIG_RESERVED_MASK   (0x4000u) /**< Reserved bit, must be written as 1. */
#define INA226_CONFIG_AVG_MASK        (0x0E00u) /**< AVG[2:0]: averaging mode. */
#define INA226_CONFIG_VBUSCT_MASK     (0x01C0u) /**< VBUSCT[2:0]: bus conversion time. */
#define INA226_CONFIG_VSHCT_MASK      (0x0038u) /**< VSHCT[2:0]: shunt conversion time. */
#define INA226_CONFIG_MODE_MASK       (0x0007u) /**< MODE[2:0]: operating mode. */
#define INA226_CONFIG_AVG_SHIFT       (9u) /**< Bit position of AVG. */
#define INA226_CONFIG_VBUSCT_SHIFT    (6u) /**< Bit position of VBUSCT. */
#define INA226_CONFIG_VSHCT_SHIFT     (3u) /**< Bit position of VSHCT. */
/** @} */

/** @name Mask/Enable Register Bit Masks */
/** @{ */
#define INA226_MASK_SOL               (0x8000u) /**< SOL: shunt over-voltage alert enable. */
#define INA226_MASK_SUL               (0x4000u) /**< SUL: shunt under-voltage alert enable. */
#define INA226_MASK_BOL               (0x2000u) /**< BOL: bus over-voltage alert enable. */
#define INA226_MASK_BUL               (0x1000u) /**< BUL: bus under-voltage alert enable. */
#define INA226_MASK_POL               (0x0800u) /**< POL: power over-limit alert enable. */
#define INA226_MASK_CNVR              (0x0400u) /**< CNVR: conversion-ready alert enable. */
#define INA226_MASK_AFF               (0x0010u) /**< AFF: alert function flag (read-only). */
#define INA226_MASK_CVRF              (0x0008u) /**< CVRF: conversion-ready flag (read-only). */
#define INA226_MASK_OVF               (0x0004u) /**< OVF: math overflow flag (read-only). */
#define INA226_MASK_APOL              (0x0002u) /**< APOL: alert polarity selection. */
#define INA226_MASK_LEN               (0x0001u) /**< LEN: alert latch enable. */
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

/** @brief Averaging mode selection (AVG[2:0]). */
typedef enum {
    INA226_AVERAGES_1    = 0u, /**< 1 sample. */
    INA226_AVERAGES_4    = 1u, /**< 4 samples. */
    INA226_AVERAGES_16   = 2u, /**< 16 samples. */
    INA226_AVERAGES_64   = 3u, /**< 64 samples. */
    INA226_AVERAGES_128  = 4u, /**< 128 samples. */
    INA226_AVERAGES_256  = 5u, /**< 256 samples. */
    INA226_AVERAGES_512  = 6u, /**< 512 samples. */
    INA226_AVERAGES_1024 = 7u  /**< 1024 samples. */
} ina226_averages_t;

/** @brief Conversion time selection (VBUSCT[2:0] / VSHCT[2:0]). */
typedef enum {
    INA226_CONV_TIME_140US  = 0u, /**< 140 us. */
    INA226_CONV_TIME_204US  = 1u, /**< 204 us. */
    INA226_CONV_TIME_332US  = 2u, /**< 332 us. */
    INA226_CONV_TIME_588US  = 3u, /**< 588 us. */
    INA226_CONV_TIME_1100US = 4u, /**< 1.1 ms. */
    INA226_CONV_TIME_2116US = 5u, /**< 2.116 ms. */
    INA226_CONV_TIME_4156US = 6u, /**< 4.156 ms. */
    INA226_CONV_TIME_8244US = 7u  /**< 8.244 ms. */
} ina226_conversion_time_t;

/** @brief Operating mode selection (MODE[2:0]). */
typedef enum {
    INA226_MODE_POWER_DOWN_0          = 0u, /**< Power-down. */
    INA226_MODE_SHUNT_TRIGGERED       = 1u, /**< Shunt voltage, single trigger. */
    INA226_MODE_BUS_TRIGGERED         = 2u, /**< Bus voltage, single trigger. */
    INA226_MODE_SHUNT_BUS_TRIGGERED   = 3u, /**< Shunt and bus, single trigger. */
    INA226_MODE_POWER_DOWN_1          = 4u, /**< Power-down. */
    INA226_MODE_SHUNT_CONTINUOUS      = 5u, /**< Shunt voltage, continuous. */
    INA226_MODE_BUS_CONTINUOUS        = 6u, /**< Bus voltage, continuous. */
    INA226_MODE_SHUNT_BUS_CONTINUOUS  = 7u  /**< Shunt and bus, continuous. */
} ina226_mode_t;

/** @brief Decoded MASK_ENABLE register status. */
typedef struct {
    bool shunt_over_voltage_alert_enabled; /**< SOL: shunt over-voltage alert enable. */
    bool shunt_under_voltage_alert_enabled;/**< SUL: shunt under-voltage alert enable. */
    bool bus_over_voltage_alert_enabled;   /**< BOL: bus over-voltage alert enable. */
    bool bus_under_voltage_alert_enabled;  /**< BUL: bus under-voltage alert enable. */
    bool power_over_limit_alert_enabled;   /**< POL: power over-limit alert enable. */
    bool conversion_ready_alert_enabled;  /**< CNVR: conversion-ready alert enable. */
    bool alert_function_flag;              /**< AFF: alert function flag. */
    bool conversion_ready_flag;            /**< CVRF: conversion-ready flag. */
    bool math_overflow_flag;               /**< OVF: math overflow flag. */
    bool alert_polarity_high;              /**< APOL: active-high alert polarity. */
    bool latch_enabled;                    /**< LEN: latched alert mode. */
    uint16_t raw;                          /**< Raw MASK_ENABLE value. */
} ina226_mask_enable_status_t;

/**
 * @brief Callback type for writing register bytes via I2C.
 * @param user User-defined context pointer.
 * @param dev_addr_7bit 7-bit I2C device address.
 * @param reg Register address.
 * @param data Bytes to write; the callback must copy them before returning.
 * @param len Number of data bytes.
 * @return int 0 on success, -1 on error.
 */
typedef int (*ina226_write_reg_cb_t)(void *user, uint8_t dev_addr_7bit,
                                     uint8_t reg, const uint8_t *data, uint16_t len);

/**
 * @brief Callback type for reading register bytes via I2C.
 * @param user User-defined context pointer.
 * @param dev_addr_7bit 7-bit I2C device address.
 * @param reg Register address.
 * @param data Destination buffer, filled when the transfer completes.
 * @param len Number of data bytes.
 * @return int 0 on success, -1 on error.
 */
typedef int (*ina226_read_reg_cb_t)(void *user, uint8_t dev_addr_7bit,
                                    uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief Callback type for checking I2C bus idle status.
 * @param user User-defined context pointer.
 * @return true when idle.
 */
typedef bool (*ina226_bus_idle_cb_t)(void *user);

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
    uint16_t update_mask;            /**< Bit mask used by update_bits. */
    uint16_t update_value;           /**< Bit value used by update_bits. */
    uint8_t direct_read_reg;         /**< Register used by direct read. */
    volatile uint8_t poll_ready;     /**< Set by ina226_task() when the queued transfer finished. */
    uint8_t io_data[2];              /**< Temporary transfer buffer. */
} ina226_t;

/**
 * @brief Initialize an INA226 instance and register bus callbacks.
 *
 * write_reg is mandatory. read_reg and is_bus_idle may be NULL for a write-only
 * port; every read API then returns INA226_ERROR_NULL. The address must be in
 * the 0x40-0x4F range and both the shunt resistance and the current LSB must be
 * non-zero so that calibration and the current/power scaling stay defined.
 *
 * @param dev Pointer to the INA226 device instance.
 * @param dev_addr_7bit 7-bit I2C address, INA226_ADDRESS_* selection.
 * @param shunt_resistor_mohm Shunt resistance in mOhm, must be non-zero.
 * @param current_lsb_ua Current LSB in microamps, must be non-zero.
 * @param write_reg Register write callback.
 * @param read_reg Register read callback.
 * @param is_bus_idle Bus idle check callback.
 * @param user User-defined context pointer for the callbacks.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_init(ina226_t *dev, uint8_t dev_addr_7bit,
                            uint16_t shunt_resistor_mohm, uint16_t current_lsb_ua,
                            ina226_write_reg_cb_t write_reg, ina226_read_reg_cb_t read_reg,
                            ina226_bus_idle_cb_t is_bus_idle, void *user);

/**
 * @brief Advance the non-blocking INA226 state machine.
 *
 * Call it periodically from the main loop. Each call performs at most one bus
 * action and only when the bus has drained.
 *
 * @param dev Pointer to the INA226 device instance.
 * @return ina226_status_t INA226_OK when idle or done, INA226_BUSY while pending.
 */
ina226_status_t ina226_task(ina226_t *dev);

/**
 * @brief Return non-zero when an asynchronous operation is pending.
 * @param dev Pointer to the INA226 device instance.
 * @return true when an asynchronous operation is pending.
 */
bool ina226_is_busy(const ina226_t *dev);

/**
 * @brief Write one 16-bit register value.
 *
 * The transfer is queued and the call returns immediately: ordering is kept by
 * the shared queue, so a busy bus is not reported as an error.
 *
 * @param dev Pointer to the INA226 device instance.
 * @param reg Register address, 0x00-0x07; the two ID registers are read-only.
 * @param data Register value to write.
 * @return ina226_status_t INA226_OK when the transfer is queued, error code otherwise.
 */
ina226_status_t ina226_write_register(ina226_t *dev, uint8_t reg, uint16_t data);

/**
 * @brief Read one 16-bit register value asynchronously.
 *
 * Poll it until INA226_OK. A pending read of another register keeps the call at
 * INA226_BUSY, while a pending read of the same register is continued.
 *
 * @param dev Pointer to the INA226 device instance.
 * @param reg Register address, 0x00-0x07 or 0xFE/0xFF.
 * @param data Pointer to store the register value.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY while pending,
 *         error code otherwise.
 */
ina226_status_t ina226_read_register(ina226_t *dev, uint8_t reg, uint16_t *data);

/**
 * @brief Start a read-modify-write operation for selected bits.
 *
 * Needs both callbacks: the read half fetches the current value, the write half
 * stores the merged one. Only one operation is kept in flight at a time.
 *
 * @param dev Pointer to the INA226 device instance.
 * @param reg Register address, 0x00-0x07 or 0xFE/0xFF (the two ID registers are read-only).
 * @param mask Bits to modify.
 * @param value Bit values to apply; only bits in mask are used.
 * @return ina226_status_t INA226_OK if started, INA226_BUSY if busy,
 *         INA226_ERROR_NULL when no read callback is bound, other error codes
 *         otherwise.
 */
ina226_status_t ina226_update_bits(ina226_t *dev, uint8_t reg, uint16_t mask, uint16_t value);

/**
 * @brief Set the current conversion LSB in microamps.
 * @param dev Pointer to the INA226 device instance.
 * @param current_lsb_ua Current LSB in microamps; zero is ignored.
 */
void ina226_set_current_lsb_ua(ina226_t *dev, uint16_t current_lsb_ua);

/**
 * @brief Set the shunt resistance used for calibration.
 * @param dev Pointer to the INA226 device instance.
 * @param shunt_resistor_mohm Shunt resistance in mOhm; zero is ignored.
 */
void ina226_set_shunt_resistor_mohm(ina226_t *dev, uint16_t shunt_resistor_mohm);

/**
 * @brief Calculate the INA226 calibration register value.
 *
 * Implements CAL = 0.00512 / (R_shunt * current_LSB) in integer form and clamps
 * the result to the 15-bit register range.
 *
 * @param shunt_resistor_mohm Shunt resistance in mOhm.
 * @param current_lsb_ua Current LSB in microamps.
 * @return uint16_t Calibration value, 0 for invalid inputs.
 */
uint16_t ina226_calculate_calibration(uint16_t shunt_resistor_mohm, uint16_t current_lsb_ua);

/**
 * @brief Convert a raw shunt-voltage register value to microvolts.
 * @param raw Raw 16-bit shunt voltage register value (signed two's complement).
 * @return int32_t Shunt voltage in microvolts.
 */
int32_t ina226_convert_shunt_voltage_uv(uint16_t raw);

/**
 * @brief Convert a raw bus-voltage register value to millivolts.
 * @param raw Raw 16-bit bus voltage register value; bit 15 is not part of the
 *        measurement and is masked off.
 * @return uint16_t Bus voltage in millivolts.
 */
uint16_t ina226_convert_bus_voltage_mv(uint16_t raw);

/**
 * @brief Convert a raw current register value to milliamps.
 * @param raw Raw 16-bit current register value (signed two's complement).
 * @param current_lsb_ua Current LSB in microamps.
 * @return int32_t Current in milliamps.
 */
int32_t ina226_convert_current_ma(uint16_t raw, uint16_t current_lsb_ua);

/**
 * @brief Convert a raw power register value to milliwatts.
 * @param raw Raw 16-bit power register value.
 * @param current_lsb_ua Current LSB in microamps.
 * @return uint32_t Power in milliwatts.
 */
uint32_t ina226_convert_power_mw(uint16_t raw, uint16_t current_lsb_ua);

/**
 * @brief Write the calibration register.
 * @param dev Pointer to the INA226 device instance.
 * @param calibration Calibration value, at most 0x7FFF.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_set_calibration(ina226_t *dev, uint16_t calibration);

/**
 * @brief Calculate and write the calibration register from the instance setup.
 * @param dev Pointer to the INA226 device instance.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_calibrate(ina226_t *dev);

/**
 * @brief Build a configuration register value from conversion settings.
 * @param averages Averaging mode.
 * @param bus_ct Bus conversion time.
 * @param shunt_ct Shunt conversion time.
 * @param mode Operating mode.
 * @return uint16_t Configuration register value, reserved bit included.
 */
uint16_t ina226_build_config(ina226_averages_t averages, ina226_conversion_time_t bus_ct,
                             ina226_conversion_time_t shunt_ct, ina226_mode_t mode);

/**
 * @brief Generate a system reset and return every register to its power-on value.
 *
 * Writes the RST bit; the device resets all registers and clears RST itself, so
 * no follow-up write is needed.
 *
 * @param dev Pointer to the INA226 device instance.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_reset(ina226_t *dev);

/**
 * @brief Configure averaging, conversion times, and operating mode.
 * @param dev Pointer to the INA226 device instance.
 * @param averages Averaging mode.
 * @param bus_ct Bus conversion time.
 * @param shunt_ct Shunt conversion time.
 * @param mode Operating mode.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_configure(ina226_t *dev, ina226_averages_t averages,
                                 ina226_conversion_time_t bus_ct,
                                 ina226_conversion_time_t shunt_ct, ina226_mode_t mode);

/**
 * @brief Read the configuration register.
 *
 * Returns the raw value: decode the fields with INA226_CONFIG_AVG_MASK/SHIFT,
 * INA226_CONFIG_VBUSCT_MASK/SHIFT, INA226_CONFIG_VSHCT_MASK/SHIFT and
 * INA226_CONFIG_MODE_MASK.
 *
 * @param dev Pointer to the INA226 device instance.
 * @param config Pointer to store the raw configuration value.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_configuration(ina226_t *dev, uint16_t *config);

/**
 * @brief Read and convert the shunt voltage.
 * @param dev Pointer to the INA226 device instance.
 * @param shunt_uv Pointer to store the shunt voltage in microvolts.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_shunt_voltage_uv(ina226_t *dev, int32_t *shunt_uv);

/**
 * @brief Read and convert the bus voltage.
 * @param dev Pointer to the INA226 device instance.
 * @param bus_mv Pointer to store the bus voltage in millivolts.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_bus_voltage_mv(ina226_t *dev, uint16_t *bus_mv);

/**
 * @brief Read and convert the current.
 * @param dev Pointer to the INA226 device instance.
 * @param current_ma Pointer to store the current in milliamps.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_current_ma(ina226_t *dev, int32_t *current_ma);

/**
 * @brief Read and convert the power.
 * @param dev Pointer to the INA226 device instance.
 * @param power_mw Pointer to store the power in milliwatts.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_power_mw(ina226_t *dev, uint32_t *power_mw);

/**
 * @brief Read the calibration register.
 * @param dev Pointer to the INA226 device instance.
 * @param calibration Pointer to store the calibration value.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_calibration(ina226_t *dev, uint16_t *calibration);

/**
 * @brief Write the MASK_ENABLE register.
 * @param dev Pointer to the INA226 device instance.
 * @param mask_enable Register value to write.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_set_mask_enable(ina226_t *dev, uint16_t mask_enable);

/**
 * @brief Read and decode the MASK_ENABLE register.
 * @param dev Pointer to the INA226 device instance.
 * @param status Pointer to store the decoded status.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_mask_enable(ina226_t *dev, ina226_mask_enable_status_t *status);

/**
 * @brief Write the alert-limit register.
 * @param dev Pointer to the INA226 device instance.
 * @param alert_limit Register value to write.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_set_alert_limit(ina226_t *dev, uint16_t alert_limit);

/**
 * @brief Read the alert-limit register.
 * @param dev Pointer to the INA226 device instance.
 * @param alert_limit Pointer to store the register value.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_alert_limit(ina226_t *dev, uint16_t *alert_limit);

/**
 * @brief Enable or disable the conversion-ready alert.
 * @param dev Pointer to the INA226 device instance.
 * @param enable true to set CNVR.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_enable_conversion_ready_alert(ina226_t *dev, bool enable);

/**
 * @brief Enable or disable the shunt over-voltage alert.
 * @param dev Pointer to the INA226 device instance.
 * @param enable true to set SOL.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_enable_shunt_over_voltage_alert(ina226_t *dev, bool enable);

/**
 * @brief Enable or disable the shunt under-voltage alert.
 * @param dev Pointer to the INA226 device instance.
 * @param enable true to set SUL.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_enable_shunt_under_voltage_alert(ina226_t *dev, bool enable);

/**
 * @brief Enable or disable the bus over-voltage alert.
 * @param dev Pointer to the INA226 device instance.
 * @param enable true to set BOL.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_enable_bus_over_voltage_alert(ina226_t *dev, bool enable);

/**
 * @brief Enable or disable the bus under-voltage alert.
 * @param dev Pointer to the INA226 device instance.
 * @param enable true to set BUL.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_enable_bus_under_voltage_alert(ina226_t *dev, bool enable);

/**
 * @brief Enable or disable the power over-limit alert.
 * @param dev Pointer to the INA226 device instance.
 * @param enable true to set POL.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_enable_power_over_limit_alert(ina226_t *dev, bool enable);

/**
 * @brief Enable or disable alert latching.
 * @param dev Pointer to the INA226 device instance.
 * @param enable true to latch the alert until it is cleared.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_set_alert_latch(ina226_t *dev, bool enable);

/**
 * @brief Select active-high or active-low alert polarity.
 * @param dev Pointer to the INA226 device instance.
 * @param active_high true for active-high, false for active-low (open-drain default).
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t ina226_set_alert_polarity_high(ina226_t *dev, bool active_high);

/**
 * @brief Read the manufacturer identification register.
 * @param dev Pointer to the INA226 device instance.
 * @param manufacturer_id Pointer to store the ID (expected INA226_MANUFACTURER_ID_VALUE).
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_manufacturer_id(ina226_t *dev, uint16_t *manufacturer_id);

/**
 * @brief Read the die identification register.
 * @param dev Pointer to the INA226 device instance.
 * @param die_id Pointer to store the ID (expected INA226_DIE_ID_VALUE).
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY if pending.
 */
ina226_status_t ina226_get_die_id(ina226_t *dev, uint16_t *die_id);

/**
 * @brief Abort the pending operation and return the instance to idle.
 * @param dev Pointer to the INA226 device instance.
 */
void ina226_abort(ina226_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* INA226_H */
