/**
 * @file ch211_i2c_dma_port.h
 * @brief I2C DMA port layer for CH211 Type-C/PD high-voltage interface IC driver.
 * @author LuoXiFNT
 * @date 2026-07-04
 * @lastModified 2026-08-23
 */

#ifndef CH211_I2C_DMA_PORT_H
#define CH211_I2C_DMA_PORT_H

#include <stdint.h>
#include "ch211.h"
#include "ch32l103_i2c_dma.h"

#define CH211_INT_GPIO_PORT           GPIOB
#define CH211_INT_GPIO_PIN            GPIO_Pin_14

/** Default 7-bit I2C address of the CH211 normal register interface. */
#define CH211_DEFAULT_ADDRESS_7BIT     CH211_ADDRESS_NORMAL_7BIT

/** Global CH211 device instance. */
extern ch211_t ch211;

/** @brief Bind a CH211 instance to the shared I2C DMA queue. */
ch211_status_t CH211_I2C_DMA_InitDevice(ch211_t *dev,
                                        ch32_i2c_dma_t *iic_dma,
                                        uint8_t dev_addr_7bit);

/** @brief Initialize the global CH211 instance. */
ch211_status_t CH211_I2C_DMA_Init(ch32_i2c_dma_t *iic_dma,
                                  uint8_t dev_addr_7bit);

/** @brief Initialize the global CH211 instance with its default address. */
ch211_status_t CH211_I2C_DMA_InitDefault(ch32_i2c_dma_t *iic_dma);

/** @brief Advance the global CH211 state machine. */
void CH211_I2C_DMA_Task(void);

/** @brief Read the global CH211 PIN_STAT status. */
ch211_status_t CH211_I2C_DMA_ReadPinStatus(ch211_pin_status_t *status);
/** @brief Read the global CH211 SYS_STAT status. */
ch211_status_t CH211_I2C_DMA_ReadSysStatus(ch211_sys_status_t *status);
/** @brief Configure the interrupt output pin. */
ch211_status_t CH211_I2C_DMA_SetInterruptPin(ch211_int_pin_t pin);
/** @brief Enable or disable the SDA pull-up. */
ch211_status_t CH211_I2C_DMA_SetSdaPullup(bool enable);
/** @brief Enable or disable automatic HVCP pull-down. */
ch211_status_t CH211_I2C_DMA_EnableHvcpAuto(bool enable);
/** @brief Configure HVCP low-side pull-down strength. */
ch211_status_t CH211_I2C_DMA_SetHvcpLow(bool strong_pull_down);
/** @brief Enable or disable the HVCP VBUS pull-up. */
ch211_status_t CH211_I2C_DMA_SetHvcpVbusPullup(bool enable);
/** @brief Configure all CC path bits for one channel. */
ch211_status_t CH211_I2C_DMA_SetCcChannel(ch211_channel_t channel,
                                          bool cch_connect_oe,
                                          bool ccl_connect_ge,
                                          bool rd_enable,
                                          bool vconn_enable);
/** @brief Enable or disable the CC connection path. */
ch211_status_t CH211_I2C_DMA_EnableCcPath(ch211_channel_t channel, bool enable);
/** @brief Enable or disable the CC Rd pull-down. */
ch211_status_t CH211_I2C_DMA_EnableCcRd(ch211_channel_t channel, bool enable);
/** @brief Enable or disable VCONN. */
ch211_status_t CH211_I2C_DMA_EnableVconn(ch211_channel_t channel, bool enable);
/** @brief Enable or disable VBUS discharge. */
ch211_status_t CH211_I2C_DMA_SetVbusDischarge(bool enable);
/** @brief Return non-zero when the global CH211 instance is busy. */
uint8_t CH211_I2C_DMA_IsBusy(void);

/**
 * @brief Abort any pending CH211 async operation.
 *        Call on disconnect to ensure clean state for next connection.
 */
void CH211_I2C_DMA_Abort(void);

#endif /* CH211_I2C_DMA_PORT_H */
