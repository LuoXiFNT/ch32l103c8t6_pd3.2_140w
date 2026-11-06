/**
 * @file mp2980_i2c_dma_port.c
 * @brief I2C DMA port implementation for MP2980 power management IC driver.
 * @author LuoXiFNT
 * @date 2026-06-19
 * @lastModified 2026-11-06
 */

#include "mp2980_i2c_dma_port.h"

/** Device instance; only this port touches it. */
static mp2980_t mp2980;

/** Steps of the current-limit-then-voltage sequence. */
typedef enum {
    MP2980_PORT_SET_IDLE = 0u,  /**< No request in progress. */
    MP2980_PORT_SET_ILIM,       /**< Start programming the current limit. */
    MP2980_PORT_WAIT_ILIM,      /**< Wait for the ILIM write to drain. */
    MP2980_PORT_SET_VOUT,       /**< Start programming the output voltage. */
    MP2980_PORT_WAIT_VOUT       /**< Wait for the voltage change to drain. */
} mp2980_port_set_state_t;

/** Shared I2C DMA queue the driver callbacks operate on. */
static ch32_i2c_dma_t *s_iic_dma = 0;

/** Voltage/current sequence state. */
static mp2980_port_set_state_t s_set_state = MP2980_PORT_SET_IDLE;
/** Latest requested output voltage in mV. */
static uint16_t s_set_target_vout_mv = 0u;
/** Latest requested hardware current limit in mA (margin included). */
static uint16_t s_set_target_hw_limit_ma = 0u;
/** ILIM value written by the step that is currently in flight. */
static uint16_t s_set_started_hw_limit_ma = 0u;
/** Output voltage written by the step that is currently in flight. */
static uint16_t s_set_started_vout_mv = 0u;
/** Hardware current limit currently programmed, 0 when unknown. */
static uint16_t s_current_limit_ma = 0u;
/** VREF register value (mV) confirmed to be applied, 0 when unknown. */
static uint16_t s_set_applied_vref_mv = 0u;

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
 * @return true when the queue has drained.
 */
static bool MP2980_I2C_DMA_IsIdle(void *user)
{
    return CH32_I2C_DMA_IsIdle((ch32_i2c_dma_t *)user);
}

/**
 * @brief Check whether the shared queue has drained.
 * @return true when no transaction is queued or in flight.
 */
static bool MP2980_PortQueueDrained(void)
{
    return CH32_I2C_DMA_IsIdle(s_iic_dma);
}

/**
 * @brief Check whether a new sequence step may be started.
 * @return true when the queue has drained and the driver is idle.
 */
static bool MP2980_PortStepReady(void)
{
    return MP2980_PortQueueDrained() && !mp2980_is_busy(&mp2980);
}

/**
 * @brief Translate a negotiated current into the hardware ILIM target.
 * @param current_limit_ma Current limit requested by the PD contract.
 * @return uint16_t Hardware current limit in mA.
 *
 * @note The margin absorbs load transients. Requests at or above the default
 *       limit minus that margin are capped to the default limit.
 */
static uint16_t MP2980_PortHardwareCurrentLimit(uint16_t current_limit_ma)
{
    if ((current_limit_ma == 0u) ||
        (current_limit_ma >= (uint16_t)(MP2980_DEFAULT_CURRENT_LIMIT_MA -
                                        MP2980_CURRENT_LIMIT_MARGIN_MA))) {
        return MP2980_DEFAULT_CURRENT_LIMIT_MA;
    }

    return (uint16_t)(current_limit_ma + MP2980_CURRENT_LIMIT_MARGIN_MA);
}

mp2980_status_t MP2980_I2C_DMA_Init(ch32_i2c_dma_t *iic_dma,
                                    uint8_t dev_addr_7bit,
                                    float vout_feedback_ratio,
                                    float avg_current_sense_mohm)
{
    mp2980_status_t st;

    s_set_state = MP2980_PORT_SET_IDLE;
    s_current_limit_ma = 0u;
    s_set_applied_vref_mv = 0u;

    st = mp2980_init(&mp2980,
                     dev_addr_7bit,
                     vout_feedback_ratio,
                     MP2980_I2C_DMA_WriteReg,
                     MP2980_I2C_DMA_ReadReg,
                     MP2980_I2C_DMA_IsIdle,
                     iic_dma);
    if (st != MP2980_OK) {
        /* Do not publish a half-configured port. */
        s_iic_dma = 0;
        return st;
    }

    s_iic_dma = iic_dma;
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

/**
 * @brief Advance the ILIM-then-VOUT sequence by one step per call.
 *
 * Each call stores the newest request, waits until the previous step has
 * drained, and then starts or finishes exactly one step. A step that is still
 * in flight keeps its "started" value, so a request arriving mid-step is
 * detected on completion and the sequence restarts from the ILIM step.
 */
mp2980_status_t MP2980_I2C_DMA_SetOutputVoltageCurrentMv(uint16_t vout_mv,
                                                         uint16_t current_limit_ma)
{
    mp2980_status_t st;
    uint16_t hw_limit_ma;
    uint16_t vref_mv = 0u;

    if (s_iic_dma == 0) {
        return MP2980_ERROR;    /* port not initialized: no queue to talk through */
    }

    hw_limit_ma = MP2980_PortHardwareCurrentLimit(current_limit_ma);
    s_set_target_vout_mv = vout_mv;
    s_set_target_hw_limit_ma = hw_limit_ma;

    if (s_set_state == MP2980_PORT_SET_IDLE) {
        s_set_state = (s_current_limit_ma != hw_limit_ma) ?
                      MP2980_PORT_SET_ILIM : MP2980_PORT_SET_VOUT;
    }

    /* Wait for the previous step. The final step only needs the bus, the
     * others also need the driver, because they call into it directly. */
    if (s_set_state == MP2980_PORT_WAIT_VOUT) {
        if (!MP2980_PortQueueDrained()) {
            return MP2980_BUSY;
        }
    } else if (!MP2980_PortStepReady()) {
        return MP2980_BUSY;
    }

    switch (s_set_state) {
    case MP2980_PORT_SET_ILIM:
        st = mp2980_set_current_limit_by_current_ma(&mp2980, s_set_target_hw_limit_ma);
        if (st == MP2980_BUSY) {
            return MP2980_BUSY;
        }
        if (st != MP2980_OK) {
            s_set_state = MP2980_PORT_SET_IDLE;
            return st;
        }
        s_set_started_hw_limit_ma = s_set_target_hw_limit_ma;
        s_set_state = MP2980_PORT_WAIT_ILIM;
        return MP2980_BUSY;

    case MP2980_PORT_WAIT_ILIM:
        s_current_limit_ma = s_set_started_hw_limit_ma;
        if (s_current_limit_ma != s_set_target_hw_limit_ma) {
            s_set_state = MP2980_PORT_SET_ILIM;
            return MP2980_BUSY;
        }
        s_set_state = MP2980_PORT_SET_VOUT;
        /* fall through: the voltage step can run in the same tick */

    case MP2980_PORT_SET_VOUT:
        if (s_current_limit_ma != s_set_target_hw_limit_ma) {
            s_set_state = MP2980_PORT_SET_ILIM;
            return MP2980_BUSY;
        }
        /* GO also forces a 20ms output discharge, so skip the write when the
         * exact register value is already applied (PPS sub-LSB steps, repeat
         * requests, current-only updates). */
        if ((mp2980_convert_vout_mv_to_vref_mv(&mp2980, s_set_target_vout_mv, &vref_mv) ==
             MP2980_OK) &&
            (s_set_applied_vref_mv == vref_mv)) {
            s_set_state = MP2980_PORT_SET_IDLE;
            return MP2980_OK;
        }
        st = mp2980_set_output_voltage_mv(&mp2980, s_set_target_vout_mv);
        if (st == MP2980_BUSY) {
            return MP2980_BUSY;
        }
        if (st != MP2980_OK) {
            s_set_state = MP2980_PORT_SET_IDLE;
            return st;
        }
        s_set_started_vout_mv = s_set_target_vout_mv;
        s_set_state = MP2980_PORT_WAIT_VOUT;
        return MP2980_BUSY;

    case MP2980_PORT_WAIT_VOUT:
        /* A newer request may have replaced the target while the writes were
         * in flight; redo the sequence from the voltage step in that case. */
        if ((s_set_started_vout_mv != s_set_target_vout_mv) ||
            (s_current_limit_ma != s_set_target_hw_limit_ma)) {
            s_set_state = MP2980_PORT_SET_VOUT;
            return MP2980_BUSY;
        }
        /* The writes have drained, so the value is really in the device. */
        if (mp2980_convert_vout_mv_to_vref_mv(&mp2980, s_set_started_vout_mv, &vref_mv) ==
            MP2980_OK) {
            s_set_applied_vref_mv = vref_mv;
        }
        s_set_state = MP2980_PORT_SET_IDLE;
        return MP2980_OK;

    case MP2980_PORT_SET_IDLE:
    default:
        return MP2980_OK;
    }
}

/**
 * @brief Restart the sequence after a bus error, keeping what is programmed.
 *
 * The MP2980 keeps its registers across an I2C error, and s_current_limit_ma is
 * only committed once the ILIM write has drained, so that value is still
 * trustworthy: keeping it lets a PPS/AVS ramp continue without two extra
 * transactions. The VREF cache is dropped because a write that was in flight
 * when the error hit may never have landed.
 */
static void MP2980_PortRestartAfterError(void)
{
    mp2980_abort(&mp2980);
    s_set_state = MP2980_PORT_SET_IDLE;
    s_set_applied_vref_mv = 0u;
}

mp2980_status_t MP2980_I2C_DMA_DisablePower(void)
{
    s_set_state = MP2980_PORT_SET_IDLE;
    /* ENPWR 1-to-0 makes the device reset VREF to 0.5V. */
    s_set_applied_vref_mv = 0u;
    return mp2980_disable_power(&mp2980);
}

mp2980_status_t MP2980_I2C_DMA_SetDischarge(bool enable)
{
    return mp2980_set_discharge(&mp2980, enable);
}

bool MP2980_I2C_DMA_IsBusy(void)
{
    if (s_set_state != MP2980_PORT_SET_IDLE) {
        return true;
    }

    return MP2980_PortStepReady() == 0u;
}

void MP2980_I2C_DMA_Task(void)
{
    if ((s_iic_dma != 0) && (CH32_I2C_DMA_HadError(s_iic_dma))) {
        MP2980_PortRestartAfterError();
    }

    (void)mp2980_task(&mp2980);
}

void MP2980_I2C_DMA_Abort(void)
{
    mp2980_abort(&mp2980);
    s_set_state = MP2980_PORT_SET_IDLE;
    s_current_limit_ma = 0u;
    s_set_applied_vref_mv = 0u;
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
