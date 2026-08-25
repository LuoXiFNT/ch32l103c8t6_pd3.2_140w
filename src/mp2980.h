/**
 * @file mp2980.h
 * @brief Platform-independent non-blocking driver for MP2980 power management IC.
 * @author LuoXiFNT
 * @date 2026-06-19
 * @lastModified 2026-06-19
 */

#ifndef MP2980_H
#define MP2980_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @name I2C Device Addresses */
/** @{ */
#define MP2980_ADDRESS_60H    (0x60u)     /**< I2C address when ADR=0 (pin to GND). */
#define MP2980_ADDRESS_62H    (0x62u)     /**< I2C address when ADR=1. */
#define MP2980_ADDRESS_64H    (0x64u)     /**< I2C address when ADR=2. */
#define MP2980_ADDRESS_66H    (0x66u)     /**< I2C address when ADR=3 (pin to 3.3V). */
/** @} */

/** @name Register Addresses */
/** @{ */
#define MP2980_REG_REF_LSB           (0x00u) /**< Reference voltage LSB (bits[2:0]). */
#define MP2980_REG_REF_MSB           (0x01u) /**< Reference voltage MSB (bits[10:3]). */
#define MP2980_REG_CONTROL1          (0x02u) /**< Control register 1 (slew, discharge, power, GO). */
#define MP2980_REG_CONTROL2          (0x03u) /**< Control register 2 (freq, OCP, OVP mode). */
#define MP2980_REG_ILIM              (0x04u) /**< Current limit threshold register. */
#define MP2980_REG_INTERRUPT_STATUS  (0x05u) /**< Interrupt status (read-only, clears on read). */
#define MP2980_REG_INTERRUPT_MASK    (0x06u) /**< Interrupt mask register. */
/** @} */

/** @name Control Register 1 Bit Masks */
/** @{ */
#define MP2980_CONTROL1_SR_MASK          (0xC0u) /**< Slew rate select bits[7:6]. */
#define MP2980_CONTROL1_DISCHG_MASK      (0x20u) /**< Output discharge enable bit[5]. */
#define MP2980_CONTROL1_DITHER_MASK      (0x10u) /**< Dithering enable bit[4]. */
#define MP2980_CONTROL1_PNG_LATCH_MASK   (0x08u) /**< Power good latch enable bit[3]. */
#define MP2980_CONTROL1_RESERVED_MASK    (0x04u) /**< Reserved bit[2] (must always be set). */
#define MP2980_CONTROL1_GO_BIT_MASK      (0x02u) /**< GO bit[1] to start voltage change. */
#define MP2980_CONTROL1_ENPWR_MASK       (0x01u) /**< Power enable bit[0]. */
#define MP2980_CONTROL1_DEFAULT          (0x44u) /**< Reset value (SR=01, reserved=1, EN=0). */
#define MP2980_CONTROL1_PD_CONFIG        (0x84u) /**< PD configuration (SR=10, reserved=1, EN=0). */
/** @} */

/** @name Control Register 2 Bit Masks */
/** @{ */
#define MP2980_CONTROL2_FSW_MASK         (0xC0u) /**< Switching frequency select bits[7:6]. */
#define MP2980_CONTROL2_BB_FSW_MASK      (0x10u) /**< Buck-boost frequency mode bit[4]. */
#define MP2980_CONTROL2_OCP_MODE_MASK    (0x0Cu) /**< Overcurrent protection mode bits[3:2]. */
#define MP2980_CONTROL2_OVP_MODE_MASK    (0x03u) /**< Overvoltage protection mode bits[1:0]. */
/** @} */

/** @name Current Limit Register Masks */
/** @{ */
#define MP2980_ILIM_MASK                 (0x07u) /**< Current limit threshold select bits[2:0]. */
/** @} */

/** @name Interrupt Register Bit Masks */
/** @{ */
#define MP2980_INT_OTP_MASK              (0x10u) /**< Overtemperature protection flag bit[4]. */
#define MP2980_INT_OVP_MASK              (0x04u) /**< Overvoltage protection flag bit[2]. */
#define MP2980_INT_OCP_MASK              (0x02u) /**< Overcurrent protection flag bit[1]. */
#define MP2980_INT_PNG_MASK              (0x01u) /**< Power not good flag bit[0]. */
#define MP2980_INT_ALL_MASK              (MP2980_INT_OTP_MASK | MP2980_INT_OVP_MASK | \
                                          MP2980_INT_OCP_MASK | MP2980_INT_PNG_MASK) /**< All interrupt flags mask. */
/** @} */

/** @name IMON Gain */
/** @{ */
#define MP2980_IMON_GAIN_V_PER_V         (18.8f) /**< IMON pin voltage gain (V/V). */
/** @} */

/**
 * @brief MP2980 driver return status codes.
 */
typedef enum {
    MP2980_OK = 0,              /**< Operation completed successfully. */
    MP2980_BUSY = 1,            /**< Operation in progress, call again later. */
    MP2980_ERROR = -1,          /**< Generic error. */
    MP2980_ERROR_NULL = -2,     /**< Null pointer error. */
    MP2980_ERROR_RANGE = -3,    /**< Parameter out of range. */
    MP2980_ERROR_TIMEOUT = -4   /**< Operation timed out. */
} mp2980_status_t;

/**
 * @brief Output voltage slew rate settings.
 */
typedef enum {
    MP2980_SLEW_RATE_38MV_PER_MS  = 0u, /**< 38 mV/ms. */
    MP2980_SLEW_RATE_50MV_PER_MS  = 1u, /**< 50 mV/ms. */
    MP2980_SLEW_RATE_75MV_PER_MS  = 2u, /**< 75 mV/ms. */
    MP2980_SLEW_RATE_150MV_PER_MS = 3u  /**< 150 mV/ms. */
} mp2980_slew_rate_t;

/**
 * @brief Switching frequency settings.
 */
typedef enum {
    MP2980_FREQUENCY_200KHZ = 0u, /**< 200 kHz. */
    MP2980_FREQUENCY_300KHZ = 1u, /**< 300 kHz. */
    MP2980_FREQUENCY_400KHZ = 2u, /**< 400 kHz. */
    MP2980_FREQUENCY_600KHZ = 3u  /**< 600 kHz. */
} mp2980_frequency_t;

/**
 * @brief Current limit threshold levels (ILIM threshold voltage).
 */
typedef enum {
    MP2980_ILIM_27P9MV = 0u, /**< 27.9 mV. */
    MP2980_ILIM_33P3MV = 1u, /**< 33.3 mV. */
    MP2980_ILIM_39P3MV = 2u, /**< 39.3 mV. */
    MP2980_ILIM_45P1MV = 3u, /**< 45.1 mV. */
    MP2980_ILIM_51P2MV = 4u, /**< 51.2 mV. */
    MP2980_ILIM_56P8MV = 5u, /**< 56.8 mV. */
    MP2980_ILIM_62P8MV = 6u, /**< 62.8 mV. */
    MP2980_ILIM_68P7MV = 7u  /**< 68.7 mV. */
} mp2980_ilim_t;

/**
 * @brief Overcurrent protection (OCP) modes.
 */
typedef enum {
    MP2980_OCP_MODE_CYCLE_BY_CYCLE = 0u, /**< Cycle-by-cycle current limit. */
    MP2980_OCP_MODE_HICCUP         = 1u, /**< Hiccup mode. */
    MP2980_OCP_MODE_LATCH_OFF      = 2u  /**< Latch-off mode. */
} mp2980_ocp_mode_t;

/**
 * @brief Overvoltage protection (OVP) modes.
 */
typedef enum {
    MP2980_OVP_MODE_NONE      = 0u, /**< No OVP action. */
    MP2980_OVP_MODE_DISCHARGE = 1u, /**< Discharge output on OVP. */
    MP2980_OVP_MODE_LATCH_OFF = 2u  /**< Latch-off on OVP. */
} mp2980_ovp_mode_t;

/**
 * @brief MP2980 interrupt status structure.
 */
typedef struct {
    bool otp;           /**< Overtemperature protection flag. */
    bool ovp;           /**< Overvoltage protection flag. */
    bool ocp;           /**< Overcurrent protection flag. */
    bool png;           /**< Power not good flag. */
    uint8_t raw;        /**< Raw register value. */
} mp2980_interrupt_status_t;

/**
 * @brief Callback type for writing a register via I2C.
 * @param user User-defined context pointer.
 * @param dev_addr_7bit 7-bit I2C device address.
 * @param reg Register address.
 * @param data Data byte to write.
 * @return int 0 on success, -1 on error.
 */
typedef int (*mp2980_write_reg_cb_t)(void *user, uint8_t dev_addr_7bit, uint8_t reg, uint8_t data);

/**
 * @brief Callback type for reading a register via I2C.
 * @param user User-defined context pointer.
 * @param dev_addr_7bit 7-bit I2C device address.
 * @param reg Register address.
 * @param data Pointer to store the read data.
 * @return int 0 on success, -1 on error.
 */
typedef int (*mp2980_read_reg_cb_t)(void *user, uint8_t dev_addr_7bit, uint8_t reg, uint8_t *data);

/**
 * @brief Callback type for checking I2C bus idle status.
 * @param user User-defined context pointer.
 * @return uint8_t 1 if idle, 0 if busy.
 */
typedef uint8_t (*mp2980_bus_idle_cb_t)(void *user);

/**
 * @brief MP2980 device instance structure.
 */
typedef struct {
    uint8_t dev_addr_7bit;              /**< 7-bit I2C device address. */
    float vout_feedback_ratio;          /**< Output voltage feedback divider ratio. */
    float avg_current_sense_mohm;       /**< Average current sense resistor in mOhm. */
    mp2980_write_reg_cb_t write_reg;    /**< Register write callback. */
    mp2980_read_reg_cb_t read_reg;      /**< Register read callback. */
    mp2980_bus_idle_cb_t is_bus_idle;   /**< Bus idle check callback. */
    void *user;                         /**< User-defined context for callbacks. */

    uint8_t async_op;                   /**< Current async operation (0 = idle). */
    uint8_t update_reg;                 /**< Register being modified by update_bits. */
    uint8_t update_mask;                /**< Bit mask for update_bits operation. */
    uint8_t update_value;               /**< Bit value for update_bits operation. */
    volatile uint8_t poll_ready;        /**< Set by mp2980_task() when poll read completes. */
    uint8_t bus_was_idle;               /**< Latch: bus was observed idle during poll op. */
    uint8_t io_data[2];                 /**< Temporary I/O data buffer. */
} mp2980_t;

/**
 * @brief Initialize the MP2980 device instance and register callbacks.
 * @param dev Pointer to the MP2980 device instance.
 * @param dev_addr_7bit 7-bit I2C device address.
 * @param vout_feedback_ratio Output voltage feedback divider ratio.
 * @param write_reg Register write callback function.
 * @param read_reg Register read callback function.
 * @param is_bus_idle Bus idle check callback function.
 * @param user User-defined context pointer for callbacks.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_init(mp2980_t *dev,
                            uint8_t dev_addr_7bit,
                            float vout_feedback_ratio,
                            mp2980_write_reg_cb_t write_reg,
                            mp2980_read_reg_cb_t read_reg,
                            mp2980_bus_idle_cb_t is_bus_idle,
                            void *user);

/**
 * @brief Drive the asynchronous state machine. Call periodically from main loop.
 * @param dev Pointer to the MP2980 device instance.
 * @return mp2980_status_t MP2980_OK when idle/done, MP2980_BUSY while pending.
 */
mp2980_status_t mp2980_task(mp2980_t *dev);
/**
 * @brief Check whether the device has a pending async operation.
 * @param dev Pointer to the MP2980 device instance.
 * @return uint8_t 1 if busy, 0 if idle.
 */
uint8_t mp2980_is_busy(const mp2980_t *dev);

/**
 * @brief Write a register directly via the registered write callback.
 * @param dev Pointer to the MP2980 device instance.
 * @param reg Register address.
 * @param data Data byte to write.
 * @return mp2980_status_t MP2980_OK on success, MP2980_ERROR on failure.
 */
mp2980_status_t mp2980_write_register(mp2980_t *dev, uint8_t reg, uint8_t data);
/**
 * @brief Read a register directly via the registered read callback.
 * @param dev Pointer to the MP2980 device instance.
 * @param reg Register address.
 * @param data Pointer to store the read data.
 * @return mp2980_status_t MP2980_OK on success, MP2980_ERROR on failure.
 */
mp2980_status_t mp2980_read_register(mp2980_t *dev, uint8_t reg, uint8_t *data);
/**
 * @brief Start an async read-modify-write operation on a register.
 * @param dev Pointer to the MP2980 device instance.
 * @param reg Register address.
 * @param mask Bit mask for bits to modify.
 * @param value Bit values to apply (only bits in mask are used).
 * @return mp2980_status_t MP2980_OK if started, MP2980_BUSY if busy, error otherwise.
 */
mp2980_status_t mp2980_update_bits(mp2980_t *dev, uint8_t reg, uint8_t mask, uint8_t value);

/**
 * @brief Enable power output (set ENPWR bit).
 * @param dev Pointer to the MP2980 device instance.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_enable_power(mp2980_t *dev);
/**
 * @brief Disable power output (clear ENPWR bit).
 * @param dev Pointer to the MP2980 device instance.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_disable_power(mp2980_t *dev);

/**
 * @brief Set the reference voltage by writing REF_LSB and REF_MSB registers.
 * @param dev Pointer to the MP2980 device instance.
 * @param vref_mv Reference voltage in mV (300-2047).
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_reference_mv(mp2980_t *dev, uint16_t vref_mv);
/**
 * @brief Get the reference voltage in mV (async, poll via task).
 * @param dev Pointer to the MP2980 device instance.
 * @param vref_mv Pointer to store the reference voltage in mV.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_get_reference_mv(mp2980_t *dev, uint16_t *vref_mv);
/**
 * @brief Start a voltage change by writing CONTROL1 with the GO bit set.
 * @param dev Pointer to the MP2980 device instance.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_start_voltage_change(mp2980_t *dev);
/**
 * @brief Check if a voltage change is in progress by reading the GO bit.
 * @param dev Pointer to the MP2980 device instance.
 * @param busy Pointer to store busy status (true = GO bit set).
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_is_voltage_change_busy(mp2980_t *dev, bool *busy);
/**
 * @brief Non-blocking check for voltage change completion.
 * @param dev Pointer to the MP2980 device instance.
 * @param timeout_ms Reserved for future blocking implementation.
 * @param delay_ms Reserved for future blocking implementation.
 * @return mp2980_status_t MP2980_OK if done, MP2980_BUSY if still changing.
 */
mp2980_status_t mp2980_wait_voltage_change_done(mp2980_t *dev,
                                                uint16_t timeout_ms,
                                                void (*delay_ms)(uint32_t ms));

/**
 * @brief Abort any pending async operation and reset to IDLE.
 * @param dev Pointer to the MP2980 device instance.
 */
void mp2980_abort(mp2980_t *dev);

/**
 * @brief Set output voltage in mV (calculates VREF from feedback ratio + starts change).
 * @param dev Pointer to the MP2980 device instance.
 * @param vout_mv Desired output voltage in mV.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_output_voltage_mv(mp2980_t *dev, uint16_t vout_mv);
/**
 * @brief Get output voltage in mV (async, reads REF_LSB + REF_MSB via polling).
 * @param dev Pointer to the MP2980 device instance.
 * @param vout_mv Pointer to store the output voltage in mV.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_get_output_voltage_mv(mp2980_t *dev, uint16_t *vout_mv);

/**
 * @brief Set the output voltage slew rate.
 * @param dev Pointer to the MP2980 device instance.
 * @param slew_rate Slew rate selection.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_slew_rate(mp2980_t *dev, mp2980_slew_rate_t slew_rate);
/**
 * @brief Get the output voltage slew rate (async, poll via task).
 * @param dev Pointer to the MP2980 device instance.
 * @param slew_rate Pointer to store the slew rate.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_get_slew_rate(mp2980_t *dev, mp2980_slew_rate_t *slew_rate);
/**
 * @brief Enable or disable output discharge on disable.
 * @param dev Pointer to the MP2980 device instance.
 * @param enable true to enable discharge, false to disable.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_discharge(mp2980_t *dev, bool enable);
/**
 * @brief Enable or disable dithering.
 * @param dev Pointer to the MP2980 device instance.
 * @param enable true to enable dithering, false to disable.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_dither(mp2980_t *dev, bool enable);
/**
 * @brief Enable or disable power good latch.
 * @param dev Pointer to the MP2980 device instance.
 * @param enable true to enable latch, false to disable.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_png_latch(mp2980_t *dev, bool enable);

/**
 * @brief Set the switching frequency.
 * @param dev Pointer to the MP2980 device instance.
 * @param frequency Switching frequency selection.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_switching_frequency(mp2980_t *dev, mp2980_frequency_t frequency);
/**
 * @brief Get the switching frequency (async, poll via task).
 * @param dev Pointer to the MP2980 device instance.
 * @param frequency Pointer to store the switching frequency.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_get_switching_frequency(mp2980_t *dev, mp2980_frequency_t *frequency);
/**
 * @brief Get the configured feedback divider ratio.
 * @param dev Pointer to the MP2980 device instance.
 * @param ratio Pointer to store the feedback ratio.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_get_vout_feedback_ratio(mp2980_t *dev, float *ratio);

/**
 * @brief Enable or disable buck-boost frequency mode.
 * @param dev Pointer to the MP2980 device instance.
 * @param enable true to enable, false to disable.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_buck_boost_frequency_mode(mp2980_t *dev, bool enable);
/**
 * @brief Set the overcurrent protection mode.
 * @param dev Pointer to the MP2980 device instance.
 * @param mode OCP mode selection.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_ocp_mode(mp2980_t *dev, mp2980_ocp_mode_t mode);
/**
 * @brief Get the overcurrent protection mode (async, poll via task).
 * @param dev Pointer to the MP2980 device instance.
 * @param mode Pointer to store the OCP mode.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_get_ocp_mode(mp2980_t *dev, mp2980_ocp_mode_t *mode);
/**
 * @brief Set the overvoltage protection mode.
 * @param dev Pointer to the MP2980 device instance.
 * @param mode OVP mode selection.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_ovp_mode(mp2980_t *dev, mp2980_ovp_mode_t mode);

/**
 * @brief Set the average current sense resistor value for current calculations.
 * @param dev Pointer to the MP2980 device instance.
 * @param rsense_mohm Current sense resistor in mOhm.
 */
void mp2980_set_average_current_sense_resistor(mp2980_t *dev, float rsense_mohm);
/**
 * @brief Set the current limit threshold by ILIM level.
 * @param dev Pointer to the MP2980 device instance.
 * @param ilim Current limit threshold selection.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_current_limit_threshold(mp2980_t *dev, mp2980_ilim_t ilim);
/**
 * @brief Get the current limit threshold (async, poll via task).
 * @param dev Pointer to the MP2980 device instance.
 * @param ilim Pointer to store the ILIM level.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_get_current_limit(mp2980_t *dev, mp2980_ilim_t *ilim);
/**
 * @brief Get the current limit in mA (async, reads ILIM register, converts using rsense).
 * @param dev Pointer to the MP2980 device instance.
 * @param current_ma Pointer to store the current limit in mA.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_get_current_limit_current_ma(mp2980_t *dev, uint16_t *current_ma);
/**
 * @brief Set current limit by desired current in mA (selects nearest ILIM level).
 * @param dev Pointer to the MP2980 device instance.
 * @param current_ma Desired current limit in mA.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_current_limit_by_current_ma(mp2980_t *dev, uint16_t current_ma);
/**
 * @brief Get the threshold voltage in mV for a given ILIM level (synchronous, no I2C).
 * @param ilim ILIM level.
 * @return uint8_t Threshold voltage in mV.
 */
uint8_t mp2980_get_current_limit_threshold_voltage_mv(mp2980_ilim_t ilim);

/**
 * @brief Convert IMON pin ADC reading (mV) to output current (mA).
 * @param dev Pointer to the MP2980 device instance.
 * @param adc_mv IMON pin voltage measured by ADC in mV.
 * @param current_ma Pointer to store the calculated current in mA.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_convert_imon_mv_to_ma(mp2980_t *dev, uint16_t adc_mv, uint16_t *current_ma);

/**
 * @brief Get interrupt status register (async, poll via task).
 * @param dev Pointer to the MP2980 device instance.
 * @param status Pointer to store the parsed interrupt status.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_get_interrupt_status(mp2980_t *dev, mp2980_interrupt_status_t *status);
/**
 * @brief Clear all interrupt status flags by writing 0xFF to the status register.
 * @param dev Pointer to the MP2980 device instance.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_clear_interrupt_status(mp2980_t *dev);
/**
 * @brief Set the interrupt mask register.
 * @param dev Pointer to the MP2980 device instance.
 * @param mask Interrupt mask bits to set (only MP2980_INT_*_MASK bits are valid).
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t mp2980_set_interrupt_mask(mp2980_t *dev, uint8_t mask);
/**
 * @brief Get the interrupt mask register (async, poll via task).
 * @param dev Pointer to the MP2980 device instance.
 * @param mask Pointer to store the interrupt mask.
 * @return mp2980_status_t MP2980_OK on completion, MP2980_BUSY if pending.
 */
mp2980_status_t mp2980_get_interrupt_mask(mp2980_t *dev, uint8_t *mask);

#ifdef __cplusplus
}
#endif

#endif /* MP2980_H */
