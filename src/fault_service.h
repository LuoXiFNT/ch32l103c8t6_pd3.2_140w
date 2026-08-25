/**
 * @file fault_service.h
 * @brief Public interface for board fault monitoring and reporting.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef FAULT_SERVICE_H
#define FAULT_SERVICE_H

#include "board_link.h"

#include <stdint.h>

/** @brief Advance GPIO and I2C fault-monitor timers. */
void Fault_TickMs(uint16_t elapsed_ms);
/** @brief Sample, debounce, aggregate, and publish board faults. */
void Fault_Task(void);

#endif /* FAULT_SERVICE_H */
