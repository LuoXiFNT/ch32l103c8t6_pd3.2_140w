/**
 * @file display_service.h
 * @brief Public task interface for the board status display.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef DISPLAY_SERVICE_H
#define DISPLAY_SERVICE_H

#include "debug.h"

/** @brief Advance display refresh and boot-animation timers. */
void Display_TickMs(u16 elapsed_ms);
/** @brief Render and schedule the next LCD frame when due. */
void Display_Task(void);

#endif /* DISPLAY_SERVICE_H */
