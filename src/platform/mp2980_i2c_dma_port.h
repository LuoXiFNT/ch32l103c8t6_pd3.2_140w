/**
 * @file mp2980_i2c_dma_port.h
 * @brief I2C DMA port layer for MP2980 power management IC driver.
 * @author LuoXiFNT
 * @date 2026-06-19
 * @lastModified 2026-11-05
 *
 * Platform layer: this file is the only place that knows how MP2980 transfers run
 * on this MCU, so it is what gets rewritten when the platform changes. The driver
 * in src/drivers/mp2980.c stays platform independent.
 *
 * Porting note: the driver only needs three callbacks. On a non-DMA platform
 * implement them as blocking transfers and return 1 from the idle callback;
 * the driver's state machine then completes in a single tick. Never block
 * inside a callback on a DMA platform: queue the transfer and report its
 * completion through the idle callback.
 */

#ifndef MP2980_I2C_DMA_PORT_H
#define MP2980_I2C_DMA_PORT_H

#include <stdint.h>
#include <stdbool.h>
#include "mp2980.h"
#include "ch32_i2c_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MP2980_INT_GPIO_PORT                GPIOA
#define MP2980_INT_GPIO_PIN                 GPIO_Pin_0

/** Default 7-bit I2C address of the MP2980. */
#define MP2980_DEFAULT_ADDRESS_7BIT          MP2980_ADDRESS_66H
/** Default output voltage feedback divider ratio (R1+R2)/R2. */
#define MP2980_DEFAULT_VOUT_FEEDBACK_RATIO   (15.0f)
/** Default average current-sense resistor (IAVGP/IAVGN) in mOhm; used by ILIM and IMON math. */
#define MP2980_DEFAULT_AVG_RSENSE_MOHM       (10.0f)
/** Default switching current-sense resistor (CSP/CSN) in mOhm; board value, kept as the documented fallback. */
#define MP2980_DEFAULT_RCS_MOHM              (5.0f)
/** Default output voltage in mV; board default that callers may fall back on, init keeps the output off. */
#define MP2980_DEFAULT_OUTPUT_MV             (5000u)
/** Default current limit target in mA. 6A with 10mOhm Riavg selects the 62.8mV ILIM level. */
#define MP2980_DEFAULT_CURRENT_LIMIT_MA      (6000u)
/** Hardware OCP margin above the negotiated current, used to tolerate load transients. */
#define MP2980_CURRENT_LIMIT_MARGIN_MA       (1000u)

/** Global MP2980 device instance. */

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
 * @brief Set output voltage and negotiated-current hardware limit.
 *
 *  Call it once per main-loop pass with the newest request. The sequence
 *  programs the current limit first, then the reference voltage, and returns
 *  MP2980_OK once the queued writes have drained. A request arriving while a
 *  step is in flight replaces the target and restarts the sequence. A vREF
 *  value that is already applied is not written again.
 *
 * @param vout_mv Desired output voltage in mV.
 * @param current_limit_ma Limit requested by the PD contract, in mA.
 * @return mp2980_status_t MP2980_OK when the target is applied, MP2980_BUSY
 *         while a step is pending, error code otherwise.
 */
mp2980_status_t MP2980_I2C_DMA_SetOutputVoltageCurrentMv(uint16_t vout_mv, uint16_t current_limit_ma);
/** @brief Disable MP2980 power output. */
mp2980_status_t MP2980_I2C_DMA_DisablePower(void);
/** @brief Enable or disable output discharge. */
mp2980_status_t MP2980_I2C_DMA_SetDischarge(bool enable);
/** @brief Return non-zero when a port-level or driver operation is pending. */
bool MP2980_I2C_DMA_IsBusy(void);
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


#ifdef __cplusplus
}
#endif
#endif /* MP2980_I2C_DMA_PORT_H */
