/**
 * @file mp2980_i2c_dma_port.h
 * @brief I2C DMA port layer for MP2980 power management IC driver.
 * @author LuoXiFNT
 * @date 2026-06-19
 * @lastModified 2026-08-23
 */

#ifndef MP2980_I2C_DMA_PORT_H
#define MP2980_I2C_DMA_PORT_H

#include <stdint.h>
#include "mp2980.h"
#include "ch32l103_i2c_dma.h"

#define MP2980_INT_GPIO_PORT                GPIOA
#define MP2980_INT_GPIO_PIN                 GPIO_Pin_0

/** Default 7-bit I2C address of the MP2980. */
#define MP2980_DEFAULT_ADDRESS_7BIT          MP2980_ADDRESS_66H
/** Default output voltage feedback divider ratio. */
#define MP2980_DEFAULT_VOUT_FEEDBACK_RATIO   (15.0f)
/** Default average current sense resistor value in mOhm. */
#define MP2980_DEFAULT_AVG_RSENSE_MOHM       (10.0f)
/** Default current sense resistor value in mOhm. */
#define MP2980_DEFAULT_RCS_MOHM              (5.0f)
/** Default output voltage in mV. */
#define MP2980_DEFAULT_OUTPUT_MV             (5000u)
/** Default current limit target in mA. 6A with 10mOhm Riavg selects the 62.8mV ILIM level. */
#define MP2980_DEFAULT_CURRENT_LIMIT_MA      (6000u)
/** Hardware OCP margin above the negotiated current, used to tolerate load transients. */
#define MP2980_CURRENT_LIMIT_MARGIN_MA       (1000u)

/** Global MP2980 device instance. */
extern mp2980_t mp2980;

/**
 * @brief Initialize the MP2980 with full parameters and register I2C DMA callbacks.
 * @param iic_dma Pointer to the I2C DMA handle.
 * @param dev_addr_7bit 7-bit I2C address of the MP2980.
 * @param vout_feedback_ratio Output voltage feedback divider ratio.
 * @param avg_current_sense_mohm Average current sense resistor in mOhm.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t MP2980_I2C_DMA_Init(ch32_i2c_dma_t *iic_dma,
                                    uint8_t dev_addr_7bit,
                                    float vout_feedback_ratio,
                                    float avg_current_sense_mohm);

/**
 * @brief Initialize the MP2980 with default parameters and set default output voltage.
 * @param iic_dma Pointer to the I2C DMA handle.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t MP2980_I2C_DMA_InitDefault(ch32_i2c_dma_t *iic_dma);

/**
 * @brief Set the MP2980 output voltage.
 * @param vout_mv Desired output voltage in mV.
 * @return mp2980_status_t MP2980_OK on success, error code otherwise.
 */
mp2980_status_t MP2980_I2C_DMA_SetOutputVoltageMv(uint16_t vout_mv);
/** @brief Set output voltage and negotiated-current hardware limit. */
mp2980_status_t MP2980_I2C_DMA_SetOutputVoltageCurrentMv(uint16_t vout_mv, uint16_t current_limit_ma);
/** @brief Disable MP2980 power output. */
mp2980_status_t MP2980_I2C_DMA_DisablePower(void);
/** @brief Enable or disable output discharge. */
mp2980_status_t MP2980_I2C_DMA_SetDischarge(bool enable);
/** @brief Poll for completion of the latest voltage change. */
mp2980_status_t MP2980_I2C_DMA_WaitVoltageChangeDone(void);
/** @brief Return non-zero when a port-level or driver operation is pending. */
uint8_t MP2980_I2C_DMA_IsBusy(void);
/** @brief Read and decode the MP2980 interrupt status. */
mp2980_status_t MP2980_I2C_DMA_ReadInterruptStatus(mp2980_interrupt_status_t *status);
/** @brief Clear all MP2980 interrupt status flags. */
mp2980_status_t MP2980_I2C_DMA_ClearInterruptStatus(void);

/**
 * @brief Periodic task to drive the MP2980 asynchronous state machine.
 *        Must be called periodically from the main loop.
 */
void MP2980_I2C_DMA_Task(void);

/**
 * @brief Abort any pending MP2980 async operation and reset port-level state.
 *        Call on disconnect to ensure clean state for next connection.
 */
void MP2980_I2C_DMA_Abort(void);

#endif /* MP2980_I2C_DMA_PORT_H */
