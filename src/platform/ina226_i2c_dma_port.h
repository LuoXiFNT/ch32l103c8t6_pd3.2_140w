/**
 * @file ina226_i2c_dma_port.h
 * @brief INA226 I2C DMA port and single-device access interface.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-05
 *
 * Platform layer: the only file that knows how INA226 transfers run on this MCU, so
 * it is what gets rewritten when the platform changes. The driver in
 * src/drivers/ina226.c stays platform independent.
 */

#ifndef INA226_I2C_DMA_PORT_H
#define INA226_I2C_DMA_PORT_H

#include "ina226.h"
#include "ch32_i2c_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bind an INA226 instance to the shared I2C DMA queue.
 * @param dev Pointer to the INA226 instance to initialize.
 * @param iic_dma Pointer to the shared I2C DMA handle.
 * @param dev_addr_7bit 7-bit INA226 address.
 * @param shunt_resistor_mohm Shunt resistance in mOhm.
 * @param current_lsb_ua Current LSB in microamps.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t INA226_I2C_DMA_InitDevice(ina226_t *dev,
                                          ch32_i2c_dma_t *iic_dma,
                                          uint8_t dev_addr_7bit,
                                          uint16_t shunt_resistor_mohm,
                                          uint16_t current_lsb_ua);

/**
 * @brief Advance one INA226 instance state machine.
 * @param dev Pointer to the INA226 instance.
 * @return ina226_status_t INA226_OK when idle or done, INA226_BUSY while pending.
 */
ina226_status_t INA226_I2C_DMA_Task(ina226_t *dev);

/**
 * @brief Write one INA226 register through I2C DMA.
 * @param dev Pointer to the INA226 instance.
 * @param reg Register address.
 * @param data 16-bit register value.
 * @return ina226_status_t INA226_OK when the transfer is queued, error code otherwise.
 */
ina226_status_t INA226_I2C_DMA_WriteRegister(ina226_t *dev, uint8_t reg, uint16_t data);

/**
 * @brief Read one INA226 register through I2C DMA.
 * @param dev Pointer to the INA226 instance.
 * @param reg Register address.
 * @param data Pointer to store the register value.
 * @return ina226_status_t INA226_OK on completion, INA226_BUSY while pending.
 */
ina226_status_t INA226_I2C_DMA_ReadRegister(ina226_t *dev, uint8_t reg, uint16_t *data);

/**
 * @brief Calculate and write the INA226 calibration register.
 * @param dev Pointer to the INA226 instance.
 * @return ina226_status_t INA226_OK on success, error code otherwise.
 */
ina226_status_t INA226_I2C_DMA_Calibrate(ina226_t *dev);

/**
 * @brief Abort a pending INA226 operation.
 * @param dev Pointer to the INA226 instance.
 */
void INA226_I2C_DMA_Abort(ina226_t *dev);


#ifdef __cplusplus
}
#endif
#endif /* INA226_I2C_DMA_PORT_H */
