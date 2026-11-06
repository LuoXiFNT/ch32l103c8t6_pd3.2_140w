/**
 * @file ina226_service.h
 * @brief Public interface for scheduled INA226 channel sampling.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-05
 */

#ifndef INA226_SERVICE_H
#define INA226_SERVICE_H

#include "ch32_i2c_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Current conversion LSB shared by both monitored channels. */
#define INA226_SERVICE_CURRENT_LSB_UA 200u

/** @brief Raw measurement sample for one INA226 channel. */
typedef struct {
    uint16_t vbus_raw; /**< Raw bus-voltage register value. */
    uint16_t curr_raw; /**< Raw current register value. */
    uint16_t pwr_raw;  /**< Raw power register value. */
} ina226_service_sample_t;

/**
 * @brief Initialize both INA226 channel instances and arm the sampling sequence.
 *
 * Only the drivers are bound here; the configuration and calibration writes for
 * channel C and then channel A are performed by INA226_Service_UpdateSamples(),
 * one step per call. The initialization results are not checked because the
 * addresses, shunt value and current LSB are compile-time constants.
 *
 * @param iic_dma Pointer to the shared I2C DMA handle.
 */
void INA226_Service_Init(ch32_i2c_dma_t *iic_dma);

/**
 * @brief Advance both INA226 driver state machines.
 *
 * Call it every main-loop pass, next to the other port task functions.
 */
void INA226_Service_I2C_DMA_Task(void);

/**
 * @brief Schedule the next step of both channel state machines.
 *
 * Each channel runs its own initialize-then-measure sequence, and the shared bus
 * is handed to whichever channel finds it free, so the two are sampled
 * interleaved rather than one after the other. A step that a driver reports as
 * busy or failed is retried on a later pass.
 */
void INA226_Service_UpdateSamples(void);

/**
 * @brief Abort both channel transactions after a shared-bus error.
 *
 * The channel whose transfer was in flight is put on a short cooldown while the
 * other keeps its place in the sequence, so one dead device cannot starve the
 * remaining channel.
 */
void INA226_Service_Abort(void);

/**
 * @brief Copy the latest raw samples for channels C and A.
 * @param channel_c Destination for channel C; may be NULL.
 * @param channel_a Destination for channel A; may be NULL.
 */
void INA226_Service_GetSamples(ina226_service_sample_t *channel_c,
                               ina226_service_sample_t *channel_a);


#ifdef __cplusplus
}
#endif

#endif /* INA226_SERVICE_H */
