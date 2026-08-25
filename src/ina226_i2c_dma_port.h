/**
 * @file ina226_i2c_dma_port.h
 * @brief INA226 I2C DMA port and single-device access interface.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef INA226_I2C_DMA_PORT_H
#define INA226_I2C_DMA_PORT_H

#include "ina226.h"
#include "ch32l103_i2c_dma.h"

/** @brief Bind an INA226 instance to the shared I2C DMA queue. */
ina226_status_t INA226_I2C_DMA_InitDevice(ina226_t *dev,
                                          ch32_i2c_dma_t *iic_dma,
                                          uint8_t dev_addr_7bit,
                                          uint16_t shunt_resistor_mohm,
                                          uint16_t current_lsb_ua);

/** @brief Advance one INA226 instance state machine. */
ina226_status_t INA226_I2C_DMA_Task(ina226_t *dev);
/** @brief Return non-zero when an INA226 operation is pending. */
uint8_t INA226_I2C_DMA_IsBusy(const ina226_t *dev);
/** @brief Write one INA226 register through I2C DMA. */
ina226_status_t INA226_I2C_DMA_WriteRegister(ina226_t *dev, uint8_t reg, uint16_t data);
/** @brief Read one INA226 register through I2C DMA. */
ina226_status_t INA226_I2C_DMA_ReadRegister(ina226_t *dev, uint8_t reg, uint16_t *data);
/** @brief Apply the service's default conversion configuration. */
ina226_status_t INA226_I2C_DMA_ConfigureDefault(ina226_t *dev);
/** @brief Calculate and write the INA226 calibration register. */
ina226_status_t INA226_I2C_DMA_Calibrate(ina226_t *dev);
/** @brief Read and convert bus voltage in millivolts. */
ina226_status_t INA226_I2C_DMA_GetBusVoltageMv(ina226_t *dev, uint16_t *bus_mv);
/** @brief Read and convert current in milliamps. */
ina226_status_t INA226_I2C_DMA_GetCurrentMa(ina226_t *dev, int32_t *current_ma);
/** @brief Read and convert power in milliwatts. */
ina226_status_t INA226_I2C_DMA_GetPowerMw(ina226_t *dev, uint32_t *power_mw);
/** @brief Abort a pending INA226 operation. */
void INA226_I2C_DMA_Abort(ina226_t *dev);

#endif /* INA226_I2C_DMA_PORT_H */
