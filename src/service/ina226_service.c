/**
 * @file ina226_service.c
 * @brief Dual-channel INA226 sampling service and converted board measurements.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-05
 */

#include "ina226_service.h"

#include "ina226_i2c_dma_port.h"

/*
 * INA226 ALERT output: intentionally not armed.
 *
 * Both channels route ALERT to the MCU (PB12 for channel A, PB13 for channel C)
 * and fault_service samples those pins, yet MASK_ENABLE and ALERT_LIMIT keep
 * their power-on values so the pin never asserts. The alert sources duplicate
 * what MP2980 (OCP/OVP/OTP/PNG) and CH211 (VBUS OV/OT) already protect, and a
 * latched alert is only released by reading MASK_ENABLE or writing the
 * configuration register, which adds I2C traffic inside the PD timing-critical
 * sequence. These two INA226s are monitors: the samples below feed display and
 * telemetry, and any real protection decision comes from the MP2980/CH211 paths.
 */

#define INA226_SERVICE_A_ADDR_7BIT       INA226_ADDRESS_A1_GND_A0_GND
#define INA226_SERVICE_C_ADDR_7BIT       INA226_ADDRESS_A1_GND_A0_VS
#define INA226_SERVICE_SHUNT_MOHM        10u

/* A channel charged with an aborted transfer is skipped for this many service
 * passes. One dead part would otherwise re-queue a failing transfer immediately
 * after every abort, and because an abort flushes the shared queue, the healthy
 * channel would never finish a read. */
#define INA226_SERVICE_FAIL_COOLDOWN     1000u

/** @brief Independent sampling step of one channel. */
typedef enum {
    INA226_CH_INIT_CFG = 0u, /**< Write the configuration register. */
    INA226_CH_INIT_CAL,      /**< Write the calibration register. */
    INA226_CH_READ_VBUS,     /**< Read bus voltage. */
    INA226_CH_READ_CURR,     /**< Read current. */
    INA226_CH_READ_PWR       /**< Read power, then wrap back to bus voltage. */
} ina226_ch_state_t;

/** @brief One INA226 channel: device, latest sample, and its own step state. */
typedef struct {
    ina226_t dev;                   /**< Driver instance bound to this channel. */
    ina226_service_sample_t sample; /**< Latest raw conversion results. */
    ina226_ch_state_t state;        /**< Next step to run. */
    uint16_t cooldown;              /**< Passes left before this channel retries. */
} ina226_channel_t;

static ina226_channel_t s_ch_c;
static ina226_channel_t s_ch_a;
static ch32_i2c_dma_t *s_iic_dma;

/* Alternating priority: whichever channel is stepped first takes the free bus, so
 * regardless of that, both channels get half of the transfer slots. Without it the
 * first channel in the call order would win every pass and the second would never
 * be sampled. */
static bool s_first_is_a;

/** Channel whose transfer was queued last. An I2C error is charged to it so only
 * the failing channel backs off; cleared once the shared queue has drained. */
static ina226_channel_t *s_bus_owner;

/**
 * @brief Check whether the shared queue can accept the next step.
 *
 * The driver's write path only queues, so without this gate a step that is
 * retried every main-loop pass would enqueue the same transfer repeatedly.
 *
 * @return uint8_t 1 when the queue is ready, otherwise 0.
 */
static bool INA226_Service_BusFree(void)
{
    if (s_iic_dma == 0) {
        return 0u;
    }
    return (CH32_I2C_DMA_IsIdle(s_iic_dma)) ? 1u : 0u;
}

/** @brief Reset one channel to the start of its sequence. */
static void INA226_ChannelReset(ina226_channel_t *ch)
{
    ch->sample.vbus_raw = 0u;
    ch->sample.curr_raw = 0u;
    ch->sample.pwr_raw = 0u;
    ch->state = INA226_CH_INIT_CFG;
    ch->cooldown = 0u;
}

void INA226_Service_Init(ch32_i2c_dma_t *iic_dma)
{
    s_iic_dma = iic_dma;
    s_bus_owner = 0;
    s_first_is_a = false;
    INA226_ChannelReset(&s_ch_c);
    INA226_ChannelReset(&s_ch_a);

    (void)INA226_I2C_DMA_InitDevice(&s_ch_c.dev,
                                    iic_dma,
                                    INA226_SERVICE_C_ADDR_7BIT,
                                    INA226_SERVICE_SHUNT_MOHM,
                                    INA226_SERVICE_CURRENT_LSB_UA);
    (void)INA226_I2C_DMA_InitDevice(&s_ch_a.dev,
                                    iic_dma,
                                    INA226_SERVICE_A_ADDR_7BIT,
                                    INA226_SERVICE_SHUNT_MOHM,
                                    INA226_SERVICE_CURRENT_LSB_UA);
}

void INA226_Service_I2C_DMA_Task(void)
{
    (void)INA226_I2C_DMA_Task(&s_ch_c.dev);
    (void)INA226_I2C_DMA_Task(&s_ch_a.dev);

    /* Nothing of ours is in flight once the shared queue drained again, so a later
     * bus error belongs to another client and must not back this service off. */
    if (INA226_Service_BusFree()) {
        s_bus_owner = 0;
    }
}

void INA226_Service_Abort(void)
{
    INA226_I2C_DMA_Abort(&s_ch_c.dev);
    INA226_I2C_DMA_Abort(&s_ch_a.dev);

    /* Charge the failure to the channel whose transfer was in flight and let it
     * back off. The other channel keeps its position in the sequence, so a single
     * dead device can no longer stop it from being sampled. */
    if (s_bus_owner != 0) {
        s_bus_owner->cooldown = INA226_SERVICE_FAIL_COOLDOWN;
        s_bus_owner = 0;
    }
}

/**
 * @brief Queue the transfer of one channel's current step.
 *
 * Busy and error returns leave the step unchanged, so it is retried on a later
 * pass. Each channel owns its own state, so both are advanced from the same call
 * and the shared bus is handed to whichever channel finds it free.
 *
 * @param ch Channel to advance.
 */
static void INA226_ChannelStep(ina226_channel_t *ch)
{
    ina226_status_t st;

    if (ch->cooldown != 0u) {
        ch->cooldown--;
        return;
    }

    if (!INA226_Service_BusFree()) {
        return;
    }

    switch (ch->state) {
    case INA226_CH_INIT_CFG:
        st = INA226_I2C_DMA_WriteRegister(&ch->dev,
                                          INA226_REG_CONFIGURATION,
                                          INA226_CONFIGURATION_DEFAULT);
        if (st == INA226_OK) {
            ch->state = INA226_CH_INIT_CAL;
            s_bus_owner = ch;
        }
        break;

    case INA226_CH_INIT_CAL:
        st = INA226_I2C_DMA_Calibrate(&ch->dev);
        if (st == INA226_OK) {
            ch->state = INA226_CH_READ_VBUS;
            s_bus_owner = ch;
        }
        break;

    case INA226_CH_READ_VBUS:
        st = INA226_I2C_DMA_ReadRegister(&ch->dev,
                                         INA226_REG_BUS_VOLTAGE,
                                         &ch->sample.vbus_raw);
        if (st == INA226_OK) {
            ch->state = INA226_CH_READ_CURR;
            s_bus_owner = ch;
        }
        break;

    case INA226_CH_READ_CURR:
        st = INA226_I2C_DMA_ReadRegister(&ch->dev,
                                         INA226_REG_CURRENT,
                                         &ch->sample.curr_raw);
        if (st == INA226_OK) {
            ch->state = INA226_CH_READ_PWR;
            s_bus_owner = ch;
        }
        break;

    case INA226_CH_READ_PWR:
        st = INA226_I2C_DMA_ReadRegister(&ch->dev,
                                         INA226_REG_POWER,
                                         &ch->sample.pwr_raw);
        if (st == INA226_OK) {
            ch->state = INA226_CH_READ_VBUS;
            s_bus_owner = ch;
        }
        break;

    default:
        ch->state = INA226_CH_INIT_CFG;
        break;
    }
}

void INA226_Service_UpdateSamples(void)
{
    if (!s_first_is_a) {
        INA226_ChannelStep(&s_ch_c);
        INA226_ChannelStep(&s_ch_a);
    } else {
        INA226_ChannelStep(&s_ch_a);
        INA226_ChannelStep(&s_ch_c);
    }

    s_first_is_a = !s_first_is_a;
}

void INA226_Service_GetSamples(ina226_service_sample_t *channel_c,
                               ina226_service_sample_t *channel_a)
{
    if (channel_c != 0) {
        *channel_c = s_ch_c.sample;
    }
    if (channel_a != 0) {
        *channel_a = s_ch_a.sample;
    }
}
