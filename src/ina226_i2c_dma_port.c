/**
 * @file ina226_i2c_dma_port.c
 * @brief I2C DMA callback adapter for the platform-independent INA226 driver.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "ina226_i2c_dma_port.h"

/**
 * @brief Adapt the INA226 write callback to the shared I2C DMA queue.
 * @param user Pointer to the shared I2C DMA context.
 * @param dev_addr_7bit Seven-bit INA226 device address.
 * @param reg Register address to write.
 * @param data Pointer to register data.
 * @param len Number of data bytes.
 * @return int 0 if queued successfully, otherwise -1.
 */
static int INA226_I2C_DMA_WriteReg(void *user,
                                   uint8_t dev_addr_7bit,
                                   uint8_t reg,
                                   const uint8_t *data,
                                   uint16_t len)
{
    ch32_i2c_dma_t *dma = (ch32_i2c_dma_t *)user;
    uint8_t frame[3];
    uint16_t i;

    if ((dma == 0) || (data == 0) || (len == 0u) || (len > 2u)) {
        return -1;
    }

    frame[0] = reg;
    for (i = 0u; i < len; i++) {
        frame[1u + i] = data[i];
    }

    return (CH32_I2C_DMA_Write(dma,
                               (uint16_t)(dev_addr_7bit << 1u),
                               frame,
                               (uint16_t)(len + 1u)) != 0u) ? 0 : -1;
}

/**
 * @brief Adapt the INA226 read callback to the shared I2C DMA queue.
 * @param user Pointer to the shared I2C DMA context.
 * @param dev_addr_7bit Seven-bit INA226 device address.
 * @param reg Register address to read.
 * @param data Destination buffer for register data.
 * @param len Number of data bytes.
 * @return int 0 if queued successfully, otherwise -1.
 */
static int INA226_I2C_DMA_ReadReg(void *user,
                                  uint8_t dev_addr_7bit,
                                  uint8_t reg,
                                  uint8_t *data,
                                  uint16_t len)
{
    ch32_i2c_dma_t *dma = (ch32_i2c_dma_t *)user;

    if ((dma == 0) || (data == 0) || (len == 0u) || (len > 2u)) {
        return -1;
    }

    return (CH32_I2C_DMA_Read(dma,
                                 (uint16_t)(dev_addr_7bit << 1u),
                                 reg,
                                 data,
                                 len) != 0u) ? 0 : -1;
}

/**
 * @brief Return the shared I2C DMA idle state to the platform-independent driver.
 * @param user Pointer to the shared I2C DMA context.
 * @return uint8_t 1 if idle or no context is supplied, 0 if busy.
 */
static uint8_t INA226_I2C_DMA_IsIdle(void *user)
{
    ch32_i2c_dma_t *dma = (ch32_i2c_dma_t *)user;

    return (dma == 0) ? 1u : CH32_I2C_DMA_IsIdle(dma);
}

ina226_status_t INA226_I2C_DMA_InitDevice(ina226_t *dev,
                                          ch32_i2c_dma_t *iic_dma,
                                          uint8_t dev_addr_7bit,
                                          uint16_t shunt_resistor_mohm,
                                          uint16_t current_lsb_ua)
{
    return ina226_init(dev,
                       dev_addr_7bit,
                       shunt_resistor_mohm,
                       current_lsb_ua,
                       INA226_I2C_DMA_WriteReg,
                       INA226_I2C_DMA_ReadReg,
                       INA226_I2C_DMA_IsIdle,
                       iic_dma);
}

ina226_status_t INA226_I2C_DMA_Task(ina226_t *dev)
{
    return ina226_task(dev);
}

uint8_t INA226_I2C_DMA_IsBusy(const ina226_t *dev)
{
    return ina226_is_busy(dev);
}

ina226_status_t INA226_I2C_DMA_WriteRegister(ina226_t *dev, uint8_t reg, uint16_t data)
{
    return ina226_write_register(dev, reg, data);
}

ina226_status_t INA226_I2C_DMA_ReadRegister(ina226_t *dev, uint8_t reg, uint16_t *data)
{
    return ina226_read_register(dev, reg, data);
}

ina226_status_t INA226_I2C_DMA_ConfigureDefault(ina226_t *dev)
{
    return ina226_configure(dev,
                            INA226_AVERAGES_16,
                            INA226_CONV_TIME_1100US,
                            INA226_CONV_TIME_1100US,
                            INA226_MODE_SHUNT_BUS_CONTINUOUS);
}

ina226_status_t INA226_I2C_DMA_Calibrate(ina226_t *dev)
{
    return ina226_calibrate(dev);
}

ina226_status_t INA226_I2C_DMA_GetBusVoltageMv(ina226_t *dev, uint16_t *bus_mv)
{
    return ina226_get_bus_voltage_mv(dev, bus_mv);
}

ina226_status_t INA226_I2C_DMA_GetCurrentMa(ina226_t *dev, int32_t *current_ma)
{
    return ina226_get_current_ma(dev, current_ma);
}

ina226_status_t INA226_I2C_DMA_GetPowerMw(ina226_t *dev, uint32_t *power_mw)
{
    return ina226_get_power_mw(dev, power_mw);
}

void INA226_I2C_DMA_Abort(ina226_t *dev)
{
    ina226_abort(dev);
}
