/**
 * @file ina226_service.c
 * @brief Dual-channel INA226 sampling service and converted board measurements.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "ina226_service.h"

#include "ina226_i2c_dma_port.h"

#define INA226_SERVICE_A_ADDR_7BIT       INA226_ADDRESS_A1_GND_A0_GND
#define INA226_SERVICE_C_ADDR_7BIT       INA226_ADDRESS_A1_GND_A0_VS
#define INA226_SERVICE_SHUNT_MOHM        10U

typedef enum {
    INA226_SERVICE_INIT_CFG_C = 0U,
    INA226_SERVICE_INIT_CAL_C,
    INA226_SERVICE_INIT_CFG_A,
    INA226_SERVICE_INIT_CAL_A,
    INA226_SERVICE_READ_VBUS_C,
    INA226_SERVICE_READ_CURR_C,
    INA226_SERVICE_READ_PWR_C,
    INA226_SERVICE_READ_VBUS_A,
    INA226_SERVICE_READ_CURR_A,
    INA226_SERVICE_READ_PWR_A
} ina226_service_state_t;

static ch32_i2c_dma_t *s_iic_dma;
static ina226_t s_ina226_c;
static ina226_t s_ina226_a;
static ina226_service_sample_t s_sample_c;
static ina226_service_sample_t s_sample_a;
static ina226_service_state_t s_state = INA226_SERVICE_INIT_CFG_C;

void INA226_Service_Init(ch32_i2c_dma_t *iic_dma)
{
    s_iic_dma = iic_dma;
    s_sample_c.vbus_raw = 0U;
    s_sample_c.curr_raw = 0U;
    s_sample_c.pwr_raw = 0U;
    s_sample_a.vbus_raw = 0U;
    s_sample_a.curr_raw = 0U;
    s_sample_a.pwr_raw = 0U;
    s_state = INA226_SERVICE_INIT_CFG_C;

    (void)INA226_I2C_DMA_InitDevice(&s_ina226_c,
                                    iic_dma,
                                    INA226_SERVICE_C_ADDR_7BIT,
                                    INA226_SERVICE_SHUNT_MOHM,
                                    INA226_SERVICE_CURRENT_LSB_UA);
    (void)INA226_I2C_DMA_InitDevice(&s_ina226_a,
                                    iic_dma,
                                    INA226_SERVICE_A_ADDR_7BIT,
                                    INA226_SERVICE_SHUNT_MOHM,
                                    INA226_SERVICE_CURRENT_LSB_UA);
}

void INA226_Service_I2C_DMA_Task(void)
{
    (void)INA226_I2C_DMA_Task(&s_ina226_c);
    (void)INA226_I2C_DMA_Task(&s_ina226_a);
}

void INA226_Service_Abort(void)
{
    INA226_I2C_DMA_Abort(&s_ina226_c);
    INA226_I2C_DMA_Abort(&s_ina226_a);
    s_state = INA226_SERVICE_INIT_CFG_C;
}

/**
 * @brief Start one configuration-register write in the sampling sequence.
 * @param dev Pointer to the INA226 device instance.
 * @param value Configuration register value to write.
 * @param next_state State to enter after the write is accepted.
 * @return uint8_t 1 if the state advanced, otherwise 0.
 */
static uint8_t INA226_WriteStep(ina226_t *dev,
                                uint16_t value,
                                ina226_service_state_t next_state)
{
    ina226_status_t status;

    if ((s_iic_dma == 0) || (CH32_I2C_DMA_IsIdle(s_iic_dma) == 0U)) {
        return 0U;
    }

    status = INA226_I2C_DMA_WriteRegister(dev, INA226_REG_CONFIGURATION, value);
    if (status == INA226_OK) {
        s_state = next_state;
        return 1U;
    }

    return 0U;
}

/**
 * @brief Start one calibration operation in the sampling sequence.
 * @param dev Pointer to the INA226 device instance.
 * @param next_state State to enter after calibration is accepted.
 * @return uint8_t 1 if the state advanced, otherwise 0.
 */
static uint8_t INA226_CalibrateStep(ina226_t *dev,
                                    ina226_service_state_t next_state)
{
    ina226_status_t status;

    if ((s_iic_dma == 0) || (CH32_I2C_DMA_IsIdle(s_iic_dma) == 0U)) {
        return 0U;
    }

    status = INA226_I2C_DMA_Calibrate(dev);
    if (status == INA226_OK) {
        s_state = next_state;
        return 1U;
    }

    return 0U;
}

/**
 * @brief Start one register read and advance to its next sampling state.
 * @param dev Pointer to the INA226 device instance.
 * @param reg Register address to read.
 * @param value Pointer to store the raw register value.
 * @param next_state State to enter after the read completes.
 * @return uint8_t 1 if the state advanced, otherwise 0.
 */
static uint8_t INA226_ReadStep(ina226_t *dev,
                               uint8_t reg,
                               uint16_t *value,
                               ina226_service_state_t next_state)
{
    ina226_status_t status;

    if ((s_iic_dma == 0) || (CH32_I2C_DMA_IsIdle(s_iic_dma) == 0U)) {
        return 0U;
    }

    status = INA226_I2C_DMA_ReadRegister(dev, reg, value);
    if (status == INA226_OK) {
        s_state = next_state;
        return 1U;
    }

    return 0U;
}

void INA226_Service_UpdateSamples(void)
{
    switch (s_state) {
    case INA226_SERVICE_INIT_CFG_C:
        (void)INA226_WriteStep(&s_ina226_c,
                               INA226_CONFIGURATION_DEFAULT,
                               INA226_SERVICE_INIT_CAL_C);
        break;

    case INA226_SERVICE_INIT_CAL_C:
        (void)INA226_CalibrateStep(&s_ina226_c,
                                   INA226_SERVICE_INIT_CFG_A);
        break;

    case INA226_SERVICE_INIT_CFG_A:
        (void)INA226_WriteStep(&s_ina226_a,
                               INA226_CONFIGURATION_DEFAULT,
                               INA226_SERVICE_INIT_CAL_A);
        break;

    case INA226_SERVICE_INIT_CAL_A:
        (void)INA226_CalibrateStep(&s_ina226_a,
                                   INA226_SERVICE_READ_VBUS_C);
        break;

    case INA226_SERVICE_READ_VBUS_C:
        (void)INA226_ReadStep(&s_ina226_c,
                              INA226_REG_BUS_VOLTAGE,
                              &s_sample_c.vbus_raw,
                              INA226_SERVICE_READ_CURR_C);
        break;

    case INA226_SERVICE_READ_CURR_C:
        (void)INA226_ReadStep(&s_ina226_c,
                              INA226_REG_CURRENT,
                              &s_sample_c.curr_raw,
                              INA226_SERVICE_READ_PWR_C);
        break;

    case INA226_SERVICE_READ_PWR_C:
        (void)INA226_ReadStep(&s_ina226_c,
                              INA226_REG_POWER,
                              &s_sample_c.pwr_raw,
                              INA226_SERVICE_READ_VBUS_A);
        break;

    case INA226_SERVICE_READ_VBUS_A:
        (void)INA226_ReadStep(&s_ina226_a,
                              INA226_REG_BUS_VOLTAGE,
                              &s_sample_a.vbus_raw,
                              INA226_SERVICE_READ_CURR_A);
        break;

    case INA226_SERVICE_READ_CURR_A:
        (void)INA226_ReadStep(&s_ina226_a,
                              INA226_REG_CURRENT,
                              &s_sample_a.curr_raw,
                              INA226_SERVICE_READ_PWR_A);
        break;

    case INA226_SERVICE_READ_PWR_A:
        (void)INA226_ReadStep(&s_ina226_a,
                              INA226_REG_POWER,
                              &s_sample_a.pwr_raw,
                              INA226_SERVICE_READ_VBUS_C);
        break;

    default:
        s_state = INA226_SERVICE_INIT_CFG_C;
        break;
    }
}

void INA226_Service_GetSamples(ina226_service_sample_t *channel_c,
                               ina226_service_sample_t *channel_a)
{
    if (channel_c != 0) {
        *channel_c = s_sample_c;
    }
    if (channel_a != 0) {
        *channel_a = s_sample_a;
    }
}
