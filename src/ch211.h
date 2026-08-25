/**
 * @file ch211.h
 * @brief Platform-independent non-blocking driver for CH211 Type-C/PD high-voltage interface IC.
 * @author LuoXiFNT
 * @date 2026-07-04
 * @lastModified 2026-08-23
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
#define CH211_ADDRESS_FAST_READ_7BIT   (0x34u) /**< Fast read address from register 0. */
/** @} */

/** @name Register Addresses */
/** @{ */
#define CH211_REG_PIN_STAT             (0x00u) /**< Pin status register. */
#define CH211_REG_PIN_CFG              (0x01u) /**< Pin interrupt configuration. */
#define CH211_REG_CC_CTRL              (0x02u) /**< CC path control register. */
#define CH211_REG_OD_CTRL              (0x03u) /**< Open-drain path control register. */
#define CH211_REG_HVCP_CTRL            (0x04u) /**< High-voltage charge-pump control. */
#define CH211_REG_HVIO_KEY             (0x05u) /**< HVIO and KEY output control. */
#define CH211_REG_SYS_CFG              (0x06u) /**< System configuration register. */
#define CH211_REG_SYS_STAT             (0x07u) /**< System status register. */
/** @} */

/** @name Power-on reset values */
/** @{ */
#define CH211_PIN_STAT_DEFAULT         (0x0Fu) /**< Reset value of PIN_STAT. */
#define CH211_PIN_CFG_DEFAULT          (0x00u) /**< Reset value of PIN_CFG. */
#define CH211_CC_CTRL_DEFAULT          (0x44u) /**< Reset value of CC_CTRL. */
#define CH211_OD_CTRL_DEFAULT          (0xAAu) /**< Reset value of OD_CTRL. */
#define CH211_HVCP_CTRL_DEFAULT        (0x02u) /**< Reset value of HVCP_CTRL. */
#define CH211_HVIO_KEY_DEFAULT         (0x00u) /**< Reset value of HVIO_KEY. */
#define CH211_SYS_CFG_DEFAULT          (0x08u) /**< Reset value of SYS_CFG. */
#define CH211_SYS_STAT_DEFAULT         (0x01u) /**< Reset value of SYS_STAT. */
/** @} */

/** @name PIN_STAT Register Bit Masks */
/** @{ */
#define CH211_PIN_STAT_CCI2            (0x80u)
#define CH211_PIN_STAT_CCI1            (0x40u)
#define CH211_PIN_STAT_VBUS_OV         (0x20u)
#define CH211_PIN_STAT_VBUS_RDY        (0x10u)
#define CH211_PIN_STAT_HVHI            (0x08u)
#define CH211_PIN_STAT_HVLI            (0x04u)
#define CH211_PIN_STAT_KEYHI           (0x02u)
#define CH211_PIN_STAT_KEYLI           (0x01u)
/** @} */

/** @name PIN_CFG Register Bit Masks */
/** @{ */
#define CH211_PIN_CFG_CC2_IE           (0x80u)
#define CH211_PIN_CFG_CC1_IE           (0x40u)
#define CH211_PIN_CFG_HVIO_IE          (0x20u)
#define CH211_PIN_CFG_KEY_IE           (0x10u)
#define CH211_PIN_CFG_VBUS_DOWN_IE     (0x08u)
#define CH211_PIN_CFG_SDA_PU           (0x04u)
#define CH211_PIN_CFG_INT_PIN_MASK     (0x03u)
/** @} */

/** @name CC_CTRL Register Bit Masks */
/** @{ */
#define CH211_CC_CTRL_CC2_VCE          (0x80u)
#define CH211_CC_CTRL_CC2_PD           (0x40u)
#define CH211_CC_CTRL_CC2_OE           (0x20u)
#define CH211_CC_CTRL_CC2_GE           (0x10u)
#define CH211_CC_CTRL_CC1_VCE          (0x08u)
#define CH211_CC_CTRL_CC1_PD           (0x04u)
#define CH211_CC_CTRL_CC1_OE           (0x02u)
#define CH211_CC_CTRL_CC1_GE           (0x01u)
/** @} */

/** @name OD_CTRL Register Bit Masks */
/** @{ */
#define CH211_OD_CTRL_OD2_OE           (0x80u)
#define CH211_OD_CTRL_OD2_GE           (0x40u)
#define CH211_OD_CTRL_OD2_PE           (0x20u)
#define CH211_OD_CTRL_OD1_OE           (0x08u)
#define CH211_OD_CTRL_OD1_GE           (0x04u)
#define CH211_OD_CTRL_OD1_PE           (0x02u)
/** @} */

/** @name HVCP_CTRL Register Bit Masks */
/** @{ */
#define CH211_HVCP_CTRL_VBUS_DISC      (0x80u)
#define CH211_HVCP_CTRL_CP_AUTO        (0x10u)
#define CH211_HVCP_CTRL_CP_AE          (0x08u)
#define CH211_HVCP_CTRL_CP_PU          (0x04u)
#define CH211_HVCP_CTRL_CP_LE          (0x02u)
#define CH211_HVCP_CTRL_CP_LX          (0x01u)
/** @} */

/** @name HVIO_KEY Register Bit Masks */
/** @{ */
#define CH211_HVIO_KEY_KEY_PD          (0x20u)
#define CH211_HVIO_KEY_KEY_OE          (0x10u)
#define CH211_HVIO_KEY_HV_PU           (0x04u)
#define CH211_HVIO_KEY_HV_PD           (0x02u)
#define CH211_HVIO_KEY_HV_OE           (0x01u)
/** @} */

/** @name SYS_CFG Register Bit Masks */
/** @{ */
#define CH211_SYS_CFG_LDO33_OFF        (0x80u)
#define CH211_SYS_CFG_CC_HVT3V         (0x40u)
#define CH211_SYS_CFG_CPLE_OVOT        (0x20u)
#define CH211_SYS_CFG_RST_OV           (0x10u)
#define CH211_SYS_CFG_LDO33_WAKE       (0x08u)
#define CH211_SYS_CFG_LDO_VSEL_MASK    (0x07u)
/** @} */

/** @name SYS_STAT Register Bit Masks */
/** @{ */
#define CH211_SYS_STAT_LDO33_OFF       (0x80u)
#define CH211_SYS_STAT_OT_RST          (0x40u)
#define CH211_SYS_STAT_VBUS_OV         (0x20u)
#define CH211_SYS_STAT_VBUS_RDY        (0x10u)
#define CH211_SYS_STAT_VBUS_LAST       (0x08u)
#define CH211_SYS_STAT_VBUS_EXIST      (0x02u)
#define CH211_SYS_STAT_VSYS_EXIST      (0x01u)
/** @} */

/**
 * @brief CH211 driver return status codes.
 */
/** @brief CH211 driver return status codes. */
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
/** @brief CH211 CC/OD channel index. */
typedef enum {
    CH211_CHANNEL_1 = 1u, /**< Channel 1. */
    CH211_CHANNEL_2 = 2u  /**< Channel 2. */
} ch211_channel_t;

/**
 * @brief CH211 interrupt output pin selection.
 */
/** @brief Interrupt output pin selection. */
typedef enum {
    CH211_INT_PIN_DISABLED = 0u, /**< Interrupt output disabled. */
    CH211_INT_PIN_SCL      = 1u, /**< Interrupt on SCL pin. */
    CH211_INT_PIN_HVIO     = 2u, /**< Interrupt on HVIO pin. */
    CH211_INT_PIN_KEY      = 3u  /**< Interrupt on KEY pin. */
} ch211_int_pin_t;

/**
 * @brief CH211 VDD33 output voltage selection.
 */
/** @brief VDD33 output voltage selection. */
typedef enum {
    CH211_LDO33_3V3 = 0u,
    CH211_LDO33_3V0 = 1u,
    CH211_LDO33_2V7 = 2u,
    CH211_LDO33_2V4 = 3u,
    CH211_LDO33_3V6 = 4u,
    CH211_LDO33_3V9 = 5u,
    CH211_LDO33_4V2 = 6u,
    CH211_LDO33_4V5 = 7u
} ch211_ldo33_voltage_t;

/**
 * @brief Parsed PIN_STAT register status.
 */
/** @brief Parsed PIN_STAT register status. */
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
/** @brief Parsed SYS_STAT register status. */
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
 * @brief Callback type for writing register bytes via I2C.
 */
typedef int (*ch211_write_reg_cb_t)(void *user,
                                    uint8_t dev_addr_7bit,
                                    uint8_t reg,
                                    const uint8_t *data,
                                    uint16_t len);

/**
 * @brief Callback type for reading register bytes via I2C.
 */
typedef int (*ch211_read_reg_cb_t)(void *user,
                                   uint8_t dev_addr_7bit,
                                   uint8_t reg,
                                   uint8_t *data,
                                   uint16_t len);

/**
 * @brief Callback type for checking I2C bus idle status.
 */
typedef uint8_t (*ch211_bus_idle_cb_t)(void *user);

/**
 * @brief CH211 device instance structure.
 */
/** @brief CH211 device instance and asynchronous operation state. */
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
    uint8_t direct_read_reg;        /**< Register used by direct read. */
    volatile uint8_t poll_ready;    /**< Set when an asynchronous read completes. */
    uint8_t bus_was_idle;           /**< Idle-edge latch for polling. */
    uint8_t io_data[8];             /**< Temporary transfer buffer. */
} ch211_t;

/** @brief Initialize a CH211 instance and register bus callbacks. */
ch211_status_t ch211_init(ch211_t *dev,
                          uint8_t dev_addr_7bit,
                          ch211_write_reg_cb_t write_reg,
                          ch211_read_reg_cb_t read_reg,
                          ch211_bus_idle_cb_t is_bus_idle,
                          void *user);

/** @brief Advance the non-blocking CH211 state machine. */
ch211_status_t ch211_task(ch211_t *dev);
/** @brief Return non-zero when an asynchronous operation is pending. */
uint8_t ch211_is_busy(const ch211_t *dev);

/** @brief Write one register byte through the bus callback. */
ch211_status_t ch211_write_register(ch211_t *dev, uint8_t reg, uint8_t data);
/** @brief Read one register byte through the asynchronous bus callback. */
ch211_status_t ch211_read_register(ch211_t *dev, uint8_t reg, uint8_t *data);
/** @brief Start a read-modify-write operation for selected register bits. */
ch211_status_t ch211_update_bits(ch211_t *dev, uint8_t reg, uint8_t mask, uint8_t value);

/** @brief Read and decode the PIN_STAT register. */
ch211_status_t ch211_read_pin_status(ch211_t *dev, ch211_pin_status_t *status);
/** @brief Read and decode the SYS_STAT register. */
ch211_status_t ch211_read_sys_status(ch211_t *dev, ch211_sys_status_t *status);

/** @brief Enable or disable the SDA pull-up. */
ch211_status_t ch211_set_sda_pullup(ch211_t *dev, bool enable);
/** @brief Select the CH211 interrupt output pin. */
ch211_status_t ch211_set_interrupt_pin(ch211_t *dev, ch211_int_pin_t pin);
/** @brief Update the PIN_CFG interrupt-enable mask. */
ch211_status_t ch211_set_interrupt_enable_mask(ch211_t *dev, uint8_t interrupt_mask);

/** @brief Write the complete CC_CTRL register value. */
ch211_status_t ch211_set_cc_control(ch211_t *dev, uint8_t cc_ctrl);
/** @brief Read the complete CC_CTRL register value. */
ch211_status_t ch211_get_cc_control(ch211_t *dev, uint8_t *cc_ctrl);
/** @brief Configure all path-control bits for one CC channel. */
ch211_status_t ch211_set_cc_channel(ch211_t *dev,
                                    ch211_channel_t channel,
                                    bool cch_connect_oe,
                                    bool ccl_connect_ge,
                                    bool rd_enable,
                                    bool vconn_enable);
/** @brief Enable or disable the CC connection path. */
ch211_status_t ch211_enable_cc_path(ch211_t *dev, ch211_channel_t channel, bool enable);
/** @brief Enable or disable the CC Rd pull-down. */
ch211_status_t ch211_enable_cc_rd(ch211_t *dev, ch211_channel_t channel, bool enable);
/** @brief Enable or disable VCONN on a CC channel. */
ch211_status_t ch211_enable_vconn(ch211_t *dev, ch211_channel_t channel, bool enable);

/** @brief Configure the open-drain path for one channel. */
ch211_status_t ch211_set_od_channel(ch211_t *dev,
                                    ch211_channel_t channel,
                                    bool odh_connect_oe,
                                    bool odl_connect_ccl_ge,
                                    bool odl_float_pe);

/** @brief Enable or disable VBUS discharge. */
ch211_status_t ch211_set_vbus_discharge(ch211_t *dev, bool enable);
/** @brief Enable or disable automatic HVCP pull-down. */
ch211_status_t ch211_enable_hvcp_auto(ch211_t *dev, bool enable);
/** @brief Configure the HVCP low-side pull-down strength. */
ch211_status_t ch211_set_hvcp_low(ch211_t *dev, bool strong_pull_down);
/** @brief Enable or disable the HVCP VBUS pull-up. */
ch211_status_t ch211_set_hvcp_vbus_pullup(ch211_t *dev, bool enable);

/** @brief Drive the HVIO output low when enabled. */
ch211_status_t ch211_set_hvio_output_low(ch211_t *dev, bool enable);
/** @brief Drive the KEY output low when enabled. */
ch211_status_t ch211_set_key_output_low(ch211_t *dev, bool enable);
/** @brief Enable or disable the HVIO pull-up. */
ch211_status_t ch211_set_hvio_pullup(ch211_t *dev, bool enable);

/** @brief Enable or disable the internal 3.3 V LDO. */
ch211_status_t ch211_set_ldo33_off(ch211_t *dev, bool off);
/** @brief Select the internal 3.3 V LDO output voltage. */
ch211_status_t ch211_set_ldo33_voltage(ch211_t *dev, ch211_ldo33_voltage_t voltage);
/** @brief Enable or disable LDO33 interrupt wake-up. */
ch211_status_t ch211_set_ldo33_interrupt_wake(ch211_t *dev, bool enable);
/** @brief Enable or disable the 3.3 V CC threshold. */
ch211_status_t ch211_set_cc_high_threshold_3v3(ch211_t *dev, bool enable);
/** @brief Enable or disable automatic HVCP pull-down on fault. */
ch211_status_t ch211_set_auto_hvcp_pull_down_on_fault(ch211_t *dev, bool enable);
/** @brief Enable or disable automatic reset on VBUS over-voltage. */
ch211_status_t ch211_set_auto_reset_on_vbus_ov(ch211_t *dev, bool enable);

/** @brief Abort the pending operation and return the instance to idle. */
void ch211_abort(ch211_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* CH211_H */
