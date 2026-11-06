/**
 * @file display_service.h
 * @brief Public task interface for the board status display.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-05
 */

#ifndef DISPLAY_SERVICE_H
#define DISPLAY_SERVICE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Advance display refresh and boot-animation timers. */
void Display_TickMs(uint16_t elapsed_ms);
/** @brief Render and schedule the next LCD frame when due. */
void Display_Task(void);


#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_SERVICE_H */
