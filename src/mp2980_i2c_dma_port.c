/**
 * @file mp2980_i2c_dma_port.c
 * @brief I2C DMA port implementation for MP2980 power management IC driver.
 * @author LuoXiFNT
 * @date 2026-06-19
 * @lastModified 2026-08-23
 */

#include "mp2980_i2c_dma_port.h"
#include "ch32l103_i2c_dma.h"

mp2980_t mp2980;
static ch32_i2c_dma_t *s_iic_dma = 0;
static uint16_t s_current_limit_ma = 0u;
static uint8_t s_current_limit_configured = 0u;

typedef enum {
    MP2980_PORT_SET_IDLE = 0u,
    MP2980_PORT_SET_ILIM,
    MP2980_PORT_WAIT_ILIM,
    MP2980_PORT_SET_VOUT,
    MP2980_PORT_WAIT_VOUT
} mp2980_port_set_state_t;

static mp2980_port_set_state_t s_set_state = MP2980_PORT_SET_IDLE;
static uint16_t s_set_target_vout_mv = 0u;
static uint16_t s_set_target_hw_limit_ma = 0u;
static uint16_t s_set_started_hw_limit_ma = 0u;
static uint16_t s_set_started_vout_mv = 0u;

/**
 * @brief Adapt the MP2980 write callback to the shared I2C DMA queue.
 * @param user Pointer to the shared I2C DMA context.
 * @param dev_addr_7bit Seven-bit MP2980 device address.
 * @param reg Register address to write.
 * @param data Register value to write.
 * @return int 0 if queued successfully, otherwise -1.
 */
static int MP2980_I2C_DMA_WriteReg(void *user, uint8_t dev_addr_7bit, uint8_t reg, uint8_t data)
{
    ch32_i2c_dma_t *iic_dma = (ch32_i2c_dma_t *)user;
    uint8_t frame[2];

    if (iic_dma == 0) {
        return -1;
    }

    frame[0] = reg;
    frame[1] = data;

    return (CH32_I2C_DMA_Write(iic_dma,
                               (uint16_t)(dev_addr_7bit << 1u),
                               frame,
                               (uint16_t)sizeof(frame)) != 0u) ? 0 : -1;
}

/**
 * @brief Adapt the MP2980 read callback to the shared I2C DMA queue.
 * @param user Pointer to the shared I2C DMA context.
 * @param dev_addr_7bit Seven-bit MP2980 device address.
 * @param reg Register address to read.
 * @param data Pointer to store the register value.
 * @return int 0 if queued successfully, otherwise -1.
 */
static int MP2980_I2C_DMA_ReadReg(void *user, uint8_t dev_addr_7bit, uint8_t reg, uint8_t *data)
{
    ch32_i2c_dma_t *iic_dma = (ch32_i2c_dma_t *)user;

    if ((iic_dma == 0) || (data == 0)) {
        return -1;
    }

    return (CH32_I2C_DMA_Read(iic_dma,
                                 (uint16_t)(dev_addr_7bit << 1u),
                                 reg,
                                 data,
                                 1u) != 0u) ? 0 : -1;
}

/**
 * @brief Return the shared I2C DMA idle state to the MP2980 driver.
 * @param user Pointer to the shared I2C DMA context.
 * @return uint8_t 1 if idle or no context is supplied, 0 if busy.
 */
static uint8_t MP2980_I2C_DMA_IsIdle(void *user)
{
    ch32_i2c_dma_t *iic_dma = (ch32_i2c_dma_t *)user;

    if (iic_dma == 0) {
        return 1u;
    }

    return CH32_I2C_DMA_IsIdle(iic_dma);
}

mp2980_status_t MP2980_I2C_DMA_Init(ch32_i2c_dma_t *iic_dma,
                                    uint8_t dev_addr_7bit,
                                    float vout_feedback_ratio,
                                    float avg_current_sense_mohm)
{
    mp2980_status_t st;

    s_iic_dma = iic_dma;
    s_current_limit_ma = 0u;
    s_current_limit_configured = 0u;

    st = mp2980_init(&mp2980,
                     dev_addr_7bit,
                     vout_feedback_ratio,
                     MP2980_I2C_DMA_WriteReg,
                     MP2980_I2C_DMA_ReadReg,
                     MP2980_I2C_DMA_IsIdle,
                     iic_dma);
    if (st != MP2980_OK) {
        return st;
    }

    mp2980_set_average_current_sense_resistor(&mp2980, avg_current_sense_mohm);
    return MP2980_OK;
}

mp2980_status_t MP2980_I2C_DMA_InitDefault(ch32_i2c_dma_t *iic_dma)
{
    mp2980_status_t st;

    st = MP2980_I2C_DMA_Init(iic_dma,
                             MP2980_DEFAULT_ADDRESS_7BIT,
                             MP2980_DEFAULT_VOUT_FEEDBACK_RATIO,
                             MP2980_DEFAULT_AVG_RSENSE_MOHM);
    if (st != MP2980_OK) {
        return st;
    }

    return MP2980_I2C_DMA_DisablePower();
}

mp2980_status_t MP2980_I2C_DMA_SetOutputVoltageMv(uint16_t vout_mv)
{
    return MP2980_I2C_DMA_SetOutputVoltageCurrentMv(vout_mv,
                                                    MP2980_DEFAULT_CURRENT_LIMIT_MA);
}

mp2980_status_t MP2980_I2C_DMA_SetOutputVoltageCurrentMv(uint16_t vout_mv, uint16_t current_limit_ma)
{
    mp2980_status_t st;
    uint16_t hw_limit_ma;

    if (current_limit_ma == 0u) {
        hw_limit_ma = MP2980_DEFAULT_CURRENT_LIMIT_MA;
    } else if (current_limit_ma >= (MP2980_DEFAULT_CURRENT_LIMIT_MA - MP2980_CURRENT_LIMIT_MARGIN_MA)) {
        hw_limit_ma = MP2980_DEFAULT_CURRENT_LIMIT_MA;
    } else {
        hw_limit_ma = (uint16_t)(current_limit_ma + MP2980_CURRENT_LIMIT_MARGIN_MA);
    }

    s_set_target_vout_mv = vout_mv;
    s_set_target_hw_limit_ma = hw_limit_ma;

    if (s_set_state == MP2980_PORT_SET_IDLE) {
        s_set_state = ((s_current_limit_configured == 0u) ||
                       (s_current_limit_ma != hw_limit_ma)) ?
                      MP2980_PORT_SET_ILIM :
                      MP2980_PORT_SET_VOUT;
    }

    switch (s_set_state) {
        case MP2980_PORT_SET_ILIM:
            if ((s_iic_dma != 0) && (CH32_I2C_DMA_IsIdle(s_iic_dma) == 0u)) {
                return MP2980_BUSY;
            }
            if (mp2980_is_busy(&mp2980) != 0u) {
                return MP2980_BUSY;
            }
            st = mp2980_set_current_limit_by_current_ma(&mp2980,
                                                        s_set_target_hw_limit_ma);
            if (st == MP2980_OK) {
                s_set_started_hw_limit_ma = s_set_target_hw_limit_ma;
                s_set_state = MP2980_PORT_WAIT_ILIM;
                return MP2980_BUSY;
            }
            if (st == MP2980_BUSY) {
                return MP2980_BUSY;
            }
            s_set_state = MP2980_PORT_SET_IDLE;
            return st;

        case MP2980_PORT_WAIT_ILIM:
            if (((s_iic_dma != 0) && (CH32_I2C_DMA_IsIdle(s_iic_dma) == 0u)) ||
                (mp2980_is_busy(&mp2980) != 0u)) {
                return MP2980_BUSY;
            }
            s_current_limit_configured = 1u;
            s_current_limit_ma = s_set_started_hw_limit_ma;
            s_set_state = (s_current_limit_ma != s_set_target_hw_limit_ma) ?
                          MP2980_PORT_SET_ILIM :
                          MP2980_PORT_SET_VOUT;
            return MP2980_BUSY;

        case MP2980_PORT_SET_VOUT:
            if (((s_iic_dma != 0) && (CH32_I2C_DMA_IsIdle(s_iic_dma) == 0u)) ||
                (mp2980_is_busy(&mp2980) != 0u)) {
                return MP2980_BUSY;
            }
            if ((s_current_limit_configured == 0u) ||
                (s_current_limit_ma != s_set_target_hw_limit_ma)) {
                s_set_state = MP2980_PORT_SET_ILIM;
                return MP2980_BUSY;
            }
            st = mp2980_set_output_voltage_mv(&mp2980, s_set_target_vout_mv);
            if (st == MP2980_OK) {
                s_set_started_vout_mv = s_set_target_vout_mv;
                s_set_state = MP2980_PORT_WAIT_VOUT;
                return MP2980_BUSY;
            }
            if (st == MP2980_BUSY) {
                return MP2980_BUSY;
            }
            s_set_state = MP2980_PORT_SET_IDLE;
            return st;

        case MP2980_PORT_WAIT_VOUT:
            if ((s_iic_dma != 0) && (CH32_I2C_DMA_IsIdle(s_iic_dma) == 0u)) {
                return MP2980_BUSY;
            }
            if ((s_set_started_vout_mv != s_set_target_vout_mv) ||
                (s_current_limit_ma != s_set_target_hw_limit_ma)) {
                s_set_state = MP2980_PORT_SET_VOUT;
                return MP2980_BUSY;
            }
            s_set_state = MP2980_PORT_SET_IDLE;
            return MP2980_OK;

        case MP2980_PORT_SET_IDLE:
        default:
            return MP2980_OK;
    }
}

mp2980_status_t MP2980_I2C_DMA_DisablePower(void)
{
    s_set_state = MP2980_PORT_SET_IDLE;
    return mp2980_disable_power(&mp2980);
}

mp2980_status_t MP2980_I2C_DMA_SetDischarge(bool enable)
{
    return mp2980_set_discharge(&mp2980, enable);
}

mp2980_status_t MP2980_I2C_DMA_WaitVoltageChangeDone(void)
{
    return mp2980_wait_voltage_change_done(&mp2980, 0u, 0);
}

uint8_t MP2980_I2C_DMA_IsBusy(void)
{
    if (s_set_state != MP2980_PORT_SET_IDLE) {
        return 1u;
    }
    if ((s_iic_dma != 0) && (CH32_I2C_DMA_IsIdle(s_iic_dma) == 0u)) {
        return 1u;
    }
    return mp2980_is_busy(&mp2980);
}

void MP2980_I2C_DMA_Task(void)
{
    if ((s_iic_dma != 0) && (CH32_I2C_DMA_HadError(s_iic_dma) != 0U)) {
        mp2980_abort(&mp2980);
        s_set_state = MP2980_PORT_SET_IDLE;
    }
    (void)mp2980_task(&mp2980);
}

void MP2980_I2C_DMA_Abort(void)
{
    mp2980_abort(&mp2980);
    s_set_state = MP2980_PORT_SET_IDLE;
    s_current_limit_configured = 0u;
    s_current_limit_ma = 0u;
    s_set_target_vout_mv = 0u;
    s_set_target_hw_limit_ma = 0u;
    s_set_started_hw_limit_ma = 0u;
    s_set_started_vout_mv = 0u;
}

mp2980_status_t MP2980_I2C_DMA_ReadInterruptStatus(mp2980_interrupt_status_t *status)
{
    return mp2980_get_interrupt_status(&mp2980, status);
}

mp2980_status_t MP2980_I2C_DMA_ClearInterruptStatus(void)
{
    return mp2980_clear_interrupt_status(&mp2980);
}
