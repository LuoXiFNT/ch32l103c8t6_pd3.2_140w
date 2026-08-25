/**
 * @file ina226_service.h
 * @brief Public interface for scheduled INA226 channel sampling.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef INA226_SERVICE_H
#define INA226_SERVICE_H

#include "ch32l103_i2c_dma.h"

/** Current conversion LSB shared by both monitored channels. */
#define INA226_SERVICE_CURRENT_LSB_UA 200U

/** @brief Raw measurement sample for one INA226 channel. */
typedef struct {
    uint16_t vbus_raw; /**< Raw bus-voltage register value. */
    uint16_t curr_raw; /**< Raw current register value. */
    uint16_t pwr_raw;  /**< Raw power register value. */
} ina226_service_sample_t;

/** @brief Initialize both INA226 channel instances. */
void INA226_Service_Init(ch32_i2c_dma_t *iic_dma);
/** @brief Advance both INA226 driver state machines. */
void INA226_Service_I2C_DMA_Task(void);
/** @brief Schedule the next initialization or measurement step. */
void INA226_Service_UpdateSamples(void);
/** @brief Abort both channel transactions and restart initialization. */
void INA226_Service_Abort(void);
/** @brief Copy the latest raw samples for channels C and A. */
void INA226_Service_GetSamples(ina226_service_sample_t *channel_c,
                               ina226_service_sample_t *channel_a);

#endif /* INA226_SERVICE_H */
