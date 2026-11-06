/**
 * @file ch211.h
 * @brief Platform-independent non-blocking driver for CH211 Type-C/PD high-voltage interface IC.
 * @author LuoXiFNT
 * @date 2026-07-04
 * @lastModified 2026-11-04
 */

#ifndef CH211_H
#define CH211_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @name I2C 7-bit Device Addresses */
/** @{ */
#define CH211_ADDRESS_NORMAL_7BIT      (0x35u) /**< Register read/write address. */
#define CH211_ADDRESS_FAST_READ_7BIT   (0x34u) /**< Fast-read address; reads start at register 0 with auto-increment. */
/** @} */

/** @name Register Addresses */
/** @{ */
#define CH211_REG_PIN_STAT             (0x00u) /**< Pin status register (read-only). */
#define CH211_REG_PIN_CFG              (0x01u) /**< Pin interrupt configuration. */
#define CH211_REG_CC_CTRL              (0x02u) /**< CC path control register. */
#define CH211_REG_OD_CTRL              (0x03u) /**< Open-drain path control register. */
#define CH211_REG_HVCP_CTRL            (0x04u) /**< High-voltage charge-pump control. */
#define CH211_REG_HVIO_KEY             (0x05u) /**< HVIO and KEY output control. */
#define CH211_REG_SYS_CFG              (0x06u) /**< System configuration register. */
#define CH211_REG_SYS_STAT             (0x07u) /**< System status register (read-only). */
/** @} */

/** @name Power-on reset values */
/** @{ */
#define CH211_PIN_STAT_DEFAULT         (0x0Fu) /**< Reset value of PIN_STAT. */
#define CH211_PIN_CFG_DEFAULT          (0x00u) /**< Reset value of PIN_CFG. */
#define CH211_CC_CTRL_DEFAULT          (0x44u) /**< Reset value of CC_CTRL; both Rd pull-downs are enabled. */
#define CH211_OD_CTRL_DEFAULT          (0xAAu) /**< Reset value of OD_CTRL. */
#define CH211_HVCP_CTRL_DEFAULT        (0x02u) /**< Reset value of HVCP_CTRL (CP_LE set). */
#define CH211_HVIO_KEY_DEFAULT         (0x00u) /**< Reset value of HVIO_KEY. */
#define CH211_SYS_CFG_DEFAULT          (0x08u) /**< Reset value of SYS_CFG (LDO33_WAKE set). */
#define CH211_SYS_STAT_DEFAULT         (0x01u) /**< Reset value of SYS_STAT (VSYS present). */
/** @} */

/** @name PIN_STAT Register Bit Masks */
/** @{ */
#define CH211_PIN_STAT_CCI2            (0x80u) /**< CC2 pin input state. */
#define CH211_PIN_STAT_CCI1            (0x40u) /**< CC1 pin input state. */
#define CH211_PIN_STAT_VBUS_OV         (0x20u) /**< VBUS over-voltage. */
#define CH211_PIN_STAT_VBUS_RDY        (0x10u) /**< VBUS ready. */
#define CH211_PIN_STAT_HVHI            (0x08u) /**< HVIO above VREFH. */
#define CH211_PIN_STAT_HVLI            (0x04u) /**< HVIO below VREFL. */
#define CH211_PIN_STAT_KEYHI           (0x02u) /**< KEY above VREFH. */
#define CH211_PIN_STAT_KEYLI           (0x01u) /**< KEY below VREFL. */
/** @} */

/** @name PIN_CFG Register Bit Masks */
/** @{ */
#define CH211_PIN_CFG_CC2_IE           (0x80u) /**< CC2 low-level interrupt enable. */
#define CH211_PIN_CFG_CC1_IE           (0x40u) /**< CC1 low-level interrupt enable. */
#define CH211_PIN_CFG_HVIO_IE          (0x20u) /**< HVIO low-level interrupt enable. */
#define CH211_PIN_CFG_KEY_IE           (0x10u) /**< KEY low-level interrupt enable. */
#define CH211_PIN_CFG_VBUS_DOWN_IE     (0x08u) /**< VBUS removal interrupt enable. */
#define CH211_PIN_CFG_SDA_PU           (0x04u) /**< Internal SDA pull-up enable. */
#define CH211_PIN_CFG_INT_PIN_MASK     (0x03u) /**< Interrupt output pin selection. */
/** @} */

/** @name CC_CTRL Register Bit Masks */
/** @{ */
#define CH211_CC_CTRL_CC2_VCE          (0x80u) /**< CC2 VCONN output enable. */
#define CH211_CC_CTRL_CC2_PD           (0x40u) /**< CC2 Sink Rd pull-down enable (forbidden on a Source). */
#define CH211_CC_CTRL_CC2_OE           (0x20u) /**< CC2 pin to middle node connect enable. */
#define CH211_CC_CTRL_CC2_GE           (0x10u) /**< CC2 middle node to CCL2 connect enable. */
#define CH211_CC_CTRL_CC1_VCE          (0x08u) /**< CC1 VCONN output enable. */
#define CH211_CC_CTRL_CC1_PD           (0x04u) /**< CC1 Sink Rd pull-down enable (forbidden on a Source). */
#define CH211_CC_CTRL_CC1_OE           (0x02u) /**< CC1 pin to middle node connect enable. */
#define CH211_CC_CTRL_CC1_GE           (0x01u) /**< CC1 middle node to CCL1 connect enable. */
/** @} */

/** @name OD_CTRL Register Bit Masks */
/** @{ */
#define CH211_OD_CTRL_OD2_OE           (0x80u) /**< OD2 pin connect enable. */
#define CH211_OD_CTRL_OD2_GE           (0x40u) /**< OD2 pin to CCL2 connect enable. */
#define CH211_OD_CTRL_OD2_PE           (0x20u) /**< OD2 disconnect from GND. */
#define CH211_OD_CTRL_OD1_OE           (0x08u) /**< OD1 pin connect enable. */
#define CH211_OD_CTRL_OD1_GE           (0x04u) /**< OD1 pin to CCL1 connect enable. */
#define CH211_OD_CTRL_OD1_PE           (0x02u) /**< OD1 disconnect from GND. */
/** @} */

/** @name HVCP_CTRL Register Bit Masks */
/** @{ */
#define CH211_HVCP_CTRL_VBUS_DISC      (0x80u) /**< VBUS discharge enable. */
#define CH211_HVCP_CTRL_CP_AUTO        (0x10u) /**< Automatic charge-pump boost enable. */
#define CH211_HVCP_CTRL_CP_AE          (0x08u) /**< Manual charge-pump boost prepare. */
#define CH211_HVCP_CTRL_CP_PU          (0x04u) /**< Manual HVCP pull-up from VBUS. */
#define CH211_HVCP_CTRL_CP_LE          (0x02u) /**< Manual HVCP pull-down enable. */
#define CH211_HVCP_CTRL_CP_LX          (0x01u) /**< Strong pull-down drive instead of weak. */
/** @} */

/** @name HVIO_KEY Register Bit Masks */
/** @{ */
#define CH211_HVIO_KEY_KEY_PD          (0x20u) /**< KEY pull-down to VREFM enable. */
#define CH211_HVIO_KEY_KEY_OE          (0x10u) /**< KEY open-drain output enable. */
#define CH211_HVIO_KEY_HV_PU           (0x04u) /**< HVIO pull-up to VDD49 enable. */
#define CH211_HVIO_KEY_HV_PD           (0x02u) /**< HVIO pull-down to VREFM enable. */
#define CH211_HVIO_KEY_HV_OE           (0x01u) /**< HVIO open-drain output enable. */
/** @} */

/** @name SYS_CFG Register Bit Masks */
/** @{ */
#define CH211_SYS_CFG_LDO33_OFF        (0x80u) /**< VDD33 output disable. */
#define CH211_SYS_CFG_CC_HVT3V         (0x40u) /**< CC high threshold for VDD49 at 3.3V. */
#define CH211_SYS_CFG_CPLE_OVOT        (0x20u) /**< Automatic HVCP pull-down on OV/OT. */
#define CH211_SYS_CFG_RST_OV           (0x10u) /**< Automatic CC/OD/HVCP reset on VBUS OV. */
#define CH211_SYS_CFG_LDO33_WAKE       (0x08u) /**< VDD33 interrupt wake-up enable. */
#define CH211_SYS_CFG_LDO_VSEL_MASK    (0x07u) /**< VDD33 voltage selection. */
/** @} */

/** @name SYS_STAT Register Bit Masks */
/** @{ */
#define CH211_SYS_STAT_LDO33_OFF       (0x80u) /**< VDD33 disabled. */
#define CH211_SYS_STAT_OT_RST          (0x40u) /**< Over-temperature reset occurred. */
#define CH211_SYS_STAT_VBUS_OV         (0x20u) /**< VBUS over-voltage. */
#define CH211_SYS_STAT_VBUS_RDY        (0x10u) /**< VBUS ready. */
#define CH211_SYS_STAT_VBUS_LAST       (0x08u) /**< VBUS ready state of the previous SYS_STAT read. */
#define CH211_SYS_STAT_VBUS_EXIST      (0x02u) /**< VBUS present. */
#define CH211_SYS_STAT_VSYS_EXIST      (0x01u) /**< VSYS present. */
/** @} */

/**
 * @brief CH211 driver return status codes.
 */
typedef enum {
    CH211_OK = 0,              /**< Operation completed successfully. */
    CH211_BUSY = 1,            /**< Operation is still in progress. */
    CH211_ERROR = -1,          /**< Generic bus or device error. */
    CH211_ERROR_NULL = -2,     /**< Null pointer or missing callback. */
    CH211_ERROR_RANGE = -3,    /**< Parameter is outside the valid range. */
    CH211_ERROR_TIMEOUT = -4   /**< Operation timed out. */
} ch211_status_t;

/**
 * @brief CH211 CC/OD channel index.
 */
typedef enum {
    CH211_CHANNEL_1 = 1u, /**< Channel 1. */
    CH211_CHANNEL_2 = 2u  /**< Channel 2. */
} ch211_channel_t;

/**
 * @brief CH211 interrupt output pin selection.
 */
typedef enum {
    CH211_INT_PIN_DISABLED = 0u, /**< Interrupt output disabled. */
    CH211_INT_PIN_SCL      = 1u, /**< Interrupt on SCL pin. */
    CH211_INT_PIN_HVIO     = 2u, /**< Interrupt on HVIO pin. */
    CH211_INT_PIN_KEY      = 3u  /**< Interrupt on KEY pin. */
} ch211_int_pin_t;

/**
 * @brief CH211 VDD33 output voltage selection.
 */
typedef enum {
    CH211_LDO33_3V3 = 0u, /**< 3.3V. */
    CH211_LDO33_3V0 = 1u, /**< 3.0V. */
    CH211_LDO33_2V7 = 2u, /**< 2.7V. */
    CH211_LDO33_2V4 = 3u, /**< 2.4V. */
    CH211_LDO33_3V6 = 4u, /**< 3.6V. */
    CH211_LDO33_3V9 = 5u, /**< 3.9V. */
    CH211_LDO33_4V2 = 6u, /**< 4.2V. */
    CH211_LDO33_4V5 = 7u  /**< 4.5V. */
} ch211_ldo33_voltage_t;

/**
 * @brief Parsed PIN_STAT register status.
 */
typedef struct {
    bool cci2;                 /**< CC2 input state. */
    bool cci1;                 /**< CC1 input state. */
    bool vbus_over_voltage;    /**< VBUS over-voltage flag. */
    bool vbus_ready;           /**< VBUS ready flag. */
    bool hvio_high_threshold;  /**< HVIO high-threshold flag. */
    bool hvio_low_threshold;   /**< HVIO low-threshold flag. */
    bool key_high_threshold;   /**< KEY high-threshold flag. */
    bool key_low_threshold;    /**< KEY low-threshold flag. */
    uint8_t raw;               /**< Raw PIN_STAT value. */
} ch211_pin_status_t;

/**
 * @brief Parsed SYS_STAT register status.
 */
typedef struct {
    bool ldo33_off;             /**< LDO33 disabled flag. */
    bool over_temperature_reset;/**< Over-temperature reset flag. */
    bool vbus_over_voltage;     /**< VBUS over-voltage flag. */
    bool vbus_ready;            /**< VBUS ready flag. */
    bool vbus_last;             /**< VBUS was previously present. */
    bool vbus_exist;            /**< VBUS present flag. */
    bool vsys_exist;            /**< VSYS present flag. */
    uint8_t raw;                /**< Raw SYS_STAT value. */
} ch211_sys_status_t;

/**
 * @brief HVCP_CTRL manual/auto mode combination.
 *
 * CP_AUTO (automatic boost) excludes the manual controls; the datasheet
 * requires CP_LE to be 0 while CP_AUTO is set, so auto_boost wins over the
 * manual fields. Because the setter writes the whole group, this type is the
 * only way to request a manual combination such as boost prepare plus VBUS
 * pull-up plus low-side pull-down.
 */
typedef struct {
    bool vbus_discharge;    /**< VBUS_DISC: enable the VBUS discharge path. */
    bool auto_boost;        /**< CP_AUTO: run the charge pump automatically. */
    bool manual_boost;      /**< CP_AE: prepare the charge pump manually. */
    bool manual_pullup;     /**< CP_PU: pull the HVCP pin up from VBUS. */
    bool low_pull_down;     /**< CP_LE: enable the HVCP pin pull-down. */
    bool low_drive_strong;  /**< CP_LX: strong pull-down drive instead of weak. */
} ch211_hvcp_control_t;

/**
 * @brief Parsed PIN_CFG register contents.
 */
typedef struct {
    uint8_t raw;                /**< Raw PIN_CFG value. */
    bool cc2_interrupt;         /**< CC2 low-level interrupt enable. */
    bool cc1_interrupt;         /**< CC1 low-level interrupt enable. */
    bool hvio_interrupt;        /**< HVIO low-level interrupt enable. */
    bool key_interrupt;         /**< KEY low-level interrupt enable. */
    bool vbus_down_interrupt;   /**< VBUS removal interrupt enable. */
    bool sda_pullup;            /**< Internal SDA pull-up enable. */
    uint8_t int_pin;            /**< Interrupt output pin selection, 0-3. */
} ch211_pin_cfg_t;

/**
 * @brief Parsed CC_CTRL register contents.
 */
typedef struct {
    uint8_t raw;                /**< Raw CC_CTRL value. */
    bool cc2_vconn;             /**< CC2 VCONN output enable. */
    bool cc2_rd;                /**< CC2 Sink Rd pull-down enable. */
    bool cc2_connect;           /**< CC2 pin to middle node connect enable. */
    bool cc2_ground;            /**< CC2 middle node to CCL2 connect enable. */
    bool cc1_vconn;             /**< CC1 VCONN output enable. */
    bool cc1_rd;                /**< CC1 Sink Rd pull-down enable. */
    bool cc1_connect;           /**< CC1 pin to middle node connect enable. */
    bool cc1_ground;            /**< CC1 middle node to CCL1 connect enable. */
} ch211_cc_control_t;

/**
 * @brief Parsed OD_CTRL register contents.
 */
typedef struct {
    uint8_t raw;                /**< Raw OD_CTRL value. */
    bool od2_connect;           /**< OD2 pin connect enable. */
    bool od2_ccl;               /**< OD2 pin to CCL2 connect enable. */
    bool od2_float_gnd;         /**< OD2 disconnected from GND. */
    bool od1_connect;           /**< OD1 pin connect enable. */
    bool od1_ccl;               /**< OD1 pin to CCL1 connect enable. */
    bool od1_float_gnd;         /**< OD1 disconnected from GND. */
} ch211_od_control_t;

/**
 * @brief Parsed HVIO_KEY register contents.
 */
typedef struct {
    uint8_t raw;                /**< Raw HVIO_KEY value. */
    bool key_pull_down;         /**< KEY pull-down to VREFM enable. */
    bool key_output_low;        /**< KEY open-drain output enable. */
    bool hvio_pullup;           /**< HVIO pull-up to VDD49 enable. */
    bool hvio_pull_down;        /**< HVIO pull-down to VREFM enable. */
    bool hvio_output_low;       /**< HVIO open-drain output enable. */
} ch211_hvio_key_t;

/**
 * @brief Parsed SYS_CFG register contents.
 */
typedef struct {
    uint8_t raw;                /**< Raw SYS_CFG value. */
    bool ldo33_off;             /**< VDD33 output disabled. */
    bool cc_high_threshold_3v3; /**< CC high threshold for VDD49 at 3.3V. */
    bool auto_hvcp_on_fault;    /**< Automatic HVCP pull-down on OV/OT. */
    bool auto_reset_on_vbus_ov; /**< Automatic CC/OD/HVCP reset on VBUS OV. */
    bool ldo33_wake;            /**< VDD33 interrupt wake-up enable. */
    uint8_t ldo33_voltage;      /**< VDD33 voltage selection, 0-7. */
} ch211_sys_cfg_t;

/**
 * @brief Callback type for writing register bytes via I2C.
 * @param user User-defined context pointer.
 * @param dev_addr_7bit 7-bit I2C device address.
 * @param reg Register address.
 * @param data Bytes to write; the callback must copy them before returning.
 * @param len Number of data bytes.
 * @return int 0 on success, -1 on error.
 */
typedef int (*ch211_write_reg_cb_t)(void *user,
                                    uint8_t dev_addr_7bit,
                                    uint8_t reg,
                                    const uint8_t *data,
                                    uint16_t len);

/**
 * @brief Callback type for reading register bytes via I2C.
 * @param user User-defined context pointer.
 * @param dev_addr_7bit 7-bit I2C device address.
 * @param reg Register address.
 * @param data Destination buffer, filled when the transfer completes.
 * @param len Number of data bytes.
 * @return int 0 on success, -1 on error.
 */
typedef int (*ch211_read_reg_cb_t)(void *user,
                                   uint8_t dev_addr_7bit,
                                   uint8_t reg,
                                   uint8_t *data,
                                   uint16_t len);

/**
 * @brief Callback type for checking I2C bus idle status.
 * @param user User-defined context pointer.
 * @return true when idle.
 */
typedef bool (*ch211_bus_idle_cb_t)(void *user);

/**
 * @brief CH211 device instance and asynchronous operation state.
 */
typedef struct {
    uint8_t dev_addr_7bit;          /**< 7-bit I2C address. */
    ch211_write_reg_cb_t write_reg; /**< Register write callback. */
    ch211_read_reg_cb_t read_reg;   /**< Register read callback. */
    ch211_bus_idle_cb_t is_bus_idle;/**< Bus idle callback. */
    void *user;                     /**< Callback context pointer. */

    uint8_t async_op;               /**< Current asynchronous operation. */
    uint8_t update_reg;             /**< Register used by update_bits. */
    uint8_t update_mask;            /**< Bit mask used by update_bits. */
    uint8_t update_value;           /**< Bit value used by update_bits. */
    uint8_t direct_read_reg;        /**< First register used by a direct or burst read. */
    uint8_t direct_read_len;        /**< Number of registers used by a burst read. */
    volatile uint8_t poll_ready;    /**< Set by ch211_task() when the queued transfer finished. */
    uint8_t io_data[8];             /**< Temporary transfer buffer. */
} ch211_t;

/**
 * @brief Initialize a CH211 instance and register bus callbacks.
 *
 * write_reg is mandatory. read_reg and is_bus_idle may be NULL for a
 * write-only port; every read API then returns CH211_ERROR_NULL. Only the
 * normal register address (0x35) is accepted.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param dev_addr_7bit 7-bit I2C device address, must be CH211_ADDRESS_NORMAL_7BIT.
 * @param write_reg Register write callback.
 * @param read_reg Register read callback.
 * @param is_bus_idle Bus idle check callback.
 * @param user User-defined context pointer for the callbacks.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_init(ch211_t *dev,
                          uint8_t dev_addr_7bit,
                          ch211_write_reg_cb_t write_reg,
                          ch211_read_reg_cb_t read_reg,
                          ch211_bus_idle_cb_t is_bus_idle,
                          void *user);

/**
 * @brief Advance the non-blocking CH211 state machine.
 *
 * Call it periodically from the main loop. Each call performs at most one bus
 * action and only when the bus has drained.
 *
 * @param dev Pointer to the CH211 device instance.
 * @return ch211_status_t CH211_OK when idle or done, CH211_BUSY while pending.
 */
ch211_status_t ch211_task(ch211_t *dev);

/**
 * @brief Return non-zero when an asynchronous operation is pending.
 * @param dev Pointer to the CH211 device instance.
 * @return true when an asynchronous operation is pending.
 */
bool ch211_is_busy(const ch211_t *dev);

/**
 * @brief Write one register byte through the bus callback.
 *
 * The transfer is queued and the call returns immediately, matching the
 * MP2980 driver: ordering is kept by the shared queue, so a busy bus is not
 * reported as an error.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param reg Register address, 0x00 to 0x07.
 * @param data Byte to write.
 * @return ch211_status_t CH211_OK when the transfer is queued, error code otherwise.
 */
ch211_status_t ch211_write_register(ch211_t *dev, uint8_t reg, uint8_t data);

/**
 * @brief Write a block of consecutive registers with one transfer.
 *
 * The CH211 increments the register address after every byte, so one
 * transaction can update several registers. The bus callback copies the bytes
 * during the call, and the write completes as soon as it is queued.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param reg First register address, 0x00 to 0x07.
 * @param data Bytes to write.
 * @param len Number of registers to write, 1 to 8.
 * @return ch211_status_t CH211_OK when the transfer is queued, error code otherwise.
 */
ch211_status_t ch211_write_registers(ch211_t *dev, uint8_t reg,
                                     const uint8_t *data, uint8_t len);

/**
 * @brief Read one register byte through the asynchronous bus callback.
 *
 * Poll it until CH211_OK. A pending read of another register keeps the call at
 * CH211_BUSY, while a pending read of the same register is continued.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param reg Register address, 0x00 to 0x07.
 * @param data Pointer to store the read byte.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY while pending,
 *         error code otherwise.
 */
ch211_status_t ch211_read_register(ch211_t *dev, uint8_t reg, uint8_t *data);

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
 * @param len Number of registers to read, 1 to 8.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY while pending,
 *         error code otherwise.
 */
ch211_status_t ch211_read_registers(ch211_t *dev, uint8_t reg,
                                    uint8_t *data, uint8_t len);

/**
 * @brief Start a read-modify-write operation for selected register bits.
 * @param dev Pointer to the CH211 device instance.
 * @param reg Register address, 0x00 to 0x07.
 * @param mask Bits to modify.
 * @param value Bit values to apply; only bits in mask are used.
 * @return ch211_status_t CH211_OK if started, CH211_BUSY if busy, error otherwise.
 */
ch211_status_t ch211_update_bits(ch211_t *dev, uint8_t reg, uint8_t mask, uint8_t value);

/**
 * @brief Read and decode the PIN_STAT register.
 * @param dev Pointer to the CH211 device instance.
 * @param status Pointer to store the decoded status.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY if pending.
 */
ch211_status_t ch211_get_pin_status(ch211_t *dev, ch211_pin_status_t *status);

/**
 * @brief Read and decode the SYS_STAT register.
 * @param dev Pointer to the CH211 device instance.
 * @param status Pointer to store the decoded status.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY if pending.
 */
ch211_status_t ch211_get_sys_status(ch211_t *dev, ch211_sys_status_t *status);

/**
 * @brief Read and decode the PIN_CFG register.
 * @param dev Pointer to the CH211 device instance.
 * @param cfg Pointer to store the decoded configuration.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY if pending.
 */
ch211_status_t ch211_get_pin_cfg(ch211_t *dev, ch211_pin_cfg_t *cfg);

/**
 * @brief Read and decode the CC_CTRL register.
 * @param dev Pointer to the CH211 device instance.
 * @param cfg Pointer to store the decoded configuration.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY if pending.
 */
ch211_status_t ch211_get_cc_control(ch211_t *dev, ch211_cc_control_t *cfg);

/**
 * @brief Read and decode the OD_CTRL register.
 * @param dev Pointer to the CH211 device instance.
 * @param cfg Pointer to store the decoded configuration.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY if pending.
 */
ch211_status_t ch211_get_od_control(ch211_t *dev, ch211_od_control_t *cfg);

/**
 * @brief Read and decode the HVCP_CTRL register.
 * @param dev Pointer to the CH211 device instance.
 * @param cfg Pointer to store the decoded configuration.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY if pending.
 */
ch211_status_t ch211_get_hvcp_control(ch211_t *dev, ch211_hvcp_control_t *cfg);

/**
 * @brief Read and decode the HVIO_KEY register.
 * @param dev Pointer to the CH211 device instance.
 * @param cfg Pointer to store the decoded configuration.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY if pending.
 */
ch211_status_t ch211_get_hvio_key(ch211_t *dev, ch211_hvio_key_t *cfg);

/**
 * @brief Read and decode the SYS_CFG register.
 * @param dev Pointer to the CH211 device instance.
 * @param cfg Pointer to store the decoded configuration.
 * @return ch211_status_t CH211_OK on completion, CH211_BUSY if pending.
 */
ch211_status_t ch211_get_sys_cfg(ch211_t *dev, ch211_sys_cfg_t *cfg);

/**
 * @brief Enable or disable the SDA pull-up.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to enable the internal pull-up.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_sda_pullup(ch211_t *dev, bool enable);

/**
 * @brief Select the CH211 interrupt output pin.
 * @param dev Pointer to the CH211 device instance.
 * @param pin Interrupt output selection.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_interrupt_pin(ch211_t *dev, ch211_int_pin_t pin);

/**
 * @brief Update the PIN_CFG interrupt-enable mask.
 * @param dev Pointer to the CH211 device instance.
 * @param interrupt_mask CH211_PIN_CFG_*_IE bits to modify.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_interrupt_enable_mask(ch211_t *dev, uint8_t interrupt_mask);

/**
 * @brief Write the complete CC_CTRL register value.
 *
 * This port is a Source, so a value that enables the Sink Rd pull-down
 * (CC1_PD or CC2_PD) is rejected with CH211_ERROR_RANGE.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param cc_ctrl Register value to write.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_cc_control(ch211_t *dev, uint8_t cc_ctrl);

/**
 * @brief Configure one CC channel.
 *
 * rd_enable must stay false: the Sink Rd pull-down is never valid on a Source
 * port, so passing true is rejected with CH211_ERROR_RANGE. The pull-down bit
 * is still cleared by the read-modify-write, which also removes the reset
 * value's enable state.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param channel Channel to configure.
 * @param cch_connect_oe Connect the CCH pin to the middle node.
 * @param ccl_connect_ge Disconnect the middle node from GND and connect CCL.
 * @param rd_enable Must be false.
 * @param vconn_enable Output VCONN on the channel.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_cc_channel(ch211_t *dev,
                                    ch211_channel_t channel,
                                    bool cch_connect_oe,
                                    bool ccl_connect_ge,
                                    bool rd_enable,
                                    bool vconn_enable);

/**
 * @brief Enable or disable the CC connection path.
 * @param dev Pointer to the CH211 device instance.
 * @param channel Channel to configure.
 * @param enable true to connect CCH and CCL, false to leave them open.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_enable_cc_path(ch211_t *dev, ch211_channel_t channel, bool enable);

/**
 * @brief Clear the Sink Rd pull-down of one channel.
 *
 * Only the "off" direction is accepted; enable=true returns CH211_ERROR_RANGE.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param channel Channel to clear.
 * @param enable Must be false.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_enable_cc_rd(ch211_t *dev, ch211_channel_t channel, bool enable);

/**
 * @brief Clear the Sink Rd pull-down on both CC channels.
 *
 * CC_CTRL resets with both Rd bits set, which is invalid for a Source port.
 * This clears them in one read-modify-write without touching OE/GE/VCE.
 *
 * @param dev Pointer to the CH211 device instance.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_force_cc_rd_off(ch211_t *dev);

/**
 * @brief Enable or disable VCONN on a CC channel.
 * @param dev Pointer to the CH211 device instance.
 * @param channel Channel to configure.
 * @param enable true to output VDD49 on the channel.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_enable_vconn(ch211_t *dev, ch211_channel_t channel, bool enable);

/**
 * @brief Configure the open-drain path for one channel.
 * @param dev Pointer to the CH211 device instance.
 * @param channel Channel to configure.
 * @param odh_connect_oe Connect the ODH pin to ODL.
 * @param odl_connect_ccl_ge Connect ODL to CCL.
 * @param odl_float_pe Disconnect ODL from GND.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_od_channel(ch211_t *dev,
                                    ch211_channel_t channel,
                                    bool odh_connect_oe,
                                    bool odl_connect_ccl_ge,
                                    bool odl_float_pe);

/**
 * @brief Enable or disable VBUS discharge.
 *
 * The discharge switch is a protection path: the datasheet recommends not
 * leaving it on for a long time.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to discharge VBUS.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_vbus_discharge(ch211_t *dev, bool enable);

/**
 * @brief Select the automatic HVCP boost mode.
 *
 * Automatic boost (CP_AUTO) and the manual controls (CP_AE/CP_PU/CP_LE/CP_LX)
 * are mutually exclusive: this call rewrites the whole manual group, which is
 * what the datasheet requires because CP_LE must be 0 while CP_AUTO is set.
 * Use ch211_set_hvcp_control() when a manual combination is needed.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to run the charge pump automatically, false to stop it.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_enable_hvcp_auto(ch211_t *dev, bool enable);

/**
 * @brief Configure the HVCP low-side pull-down strength (manual mode).
 *
 * @param dev Pointer to the CH211 device instance.
 * @param strong_pull_down true for a strong pull-down drive, false for weak.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_hvcp_low(ch211_t *dev, bool strong_pull_down);

/**
 * @brief Enable or disable the HVCP VBUS pull-up (manual mode).
 *
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to pull the HVCP pin up from VBUS, false to release it.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_hvcp_vbus_pullup(ch211_t *dev, bool enable);

/**
 * @brief Write the complete HVCP control group in one read-modify-write.
 *
 * This is the only entry point that can build a manual-mode combination, for
 * example manual boost (CP_AE) together with the VBUS pull-up (CP_PU) and the
 * low-side pull-down (CP_LE/CP_LX). When auto_boost is set, the manual fields
 * are ignored and cleared, matching the datasheet requirement that CP_LE is 0
 * while CP_AUTO is set. The VBUS discharge bit is applied as given.
 *
 * @param dev Pointer to the CH211 device instance.
 * @param control Requested combination; must not be NULL.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_hvcp_control(ch211_t *dev, const ch211_hvcp_control_t *control);

/**
 * @brief Drive the HVIO output low when enabled.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to pull the HVIO open-drain output low.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_hvio_output_low(ch211_t *dev, bool enable);

/**
 * @brief Drive the KEY output low when enabled.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to pull the KEY open-drain output low.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_key_output_low(ch211_t *dev, bool enable);

/**
 * @brief Enable or disable the HVIO pull-up.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to pull HVIO up to VDD49.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_hvio_pullup(ch211_t *dev, bool enable);

/**
 * @brief Pull the HVIO pin down to VREFM when enabled.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to enable the HVIO pull-down.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_hvio_pull_down(ch211_t *dev, bool enable);

/**
 * @brief Pull the KEY pin down to VREFM when enabled.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to enable the KEY pull-down.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_key_pull_down(ch211_t *dev, bool enable);

/**
 * @brief Enable or disable the internal 3.3 V LDO.
 * @param dev Pointer to the CH211 device instance.
 * @param off true to turn the VDD33 output off.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_ldo33_off(ch211_t *dev, bool off);

/**
 * @brief Select the internal 3.3 V LDO output voltage.
 * @param dev Pointer to the CH211 device instance.
 * @param voltage Output voltage selection.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_ldo33_voltage(ch211_t *dev, ch211_ldo33_voltage_t voltage);

/**
 * @brief Enable or disable LDO33 interrupt wake-up.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to let an interrupt clear LDO33_OFF.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_ldo33_interrupt_wake(ch211_t *dev, bool enable);

/**
 * @brief Enable or disable the 3.3 V CC threshold.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true when VDD49 is 3.3 V, false when it is 4.9 V.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_cc_high_threshold_3v3(ch211_t *dev, bool enable);

/**
 * @brief Enable or disable automatic HVCP pull-down on fault.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to force the HVCP pull-down on VBUS over-voltage or over-temperature.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_auto_hvcp_pull_down_on_fault(ch211_t *dev, bool enable);

/**
 * @brief Enable or disable automatic reset on VBUS over-voltage.
 * @param dev Pointer to the CH211 device instance.
 * @param enable true to reset CC_CTRL, OD_CTRL and HVCP_CTRL on VBUS over-voltage.
 * @return ch211_status_t CH211_OK on success, error code otherwise.
 */
ch211_status_t ch211_set_auto_reset_on_vbus_ov(ch211_t *dev, bool enable);

/**
 * @brief Abort the pending operation and return the instance to idle.
 * @param dev Pointer to the CH211 device instance.
 */
void ch211_abort(ch211_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* CH211_H */
