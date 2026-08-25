/**
 * @file led_service.h
 * @brief LED control macros and periodic toggle interface.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef LED_SERVICE_H
#define LED_SERVICE_H

#include "ch32l103_init.h"

/** Drive the active-low status LED on. */
#define LED_ON()                      GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN)
/** Drive the active-low status LED off. */
#define LED_OFF()                     GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN)
/** Toggle the active-low status LED output. */
#define LED_TOGGLE()                  GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, \
                                       (GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_GPIO_PIN) == Bit_SET) \
                                       ? Bit_RESET : Bit_SET)

/** @brief Toggle the LED when the configured period expires.
 *  @param elapsed_ms Time elapsed since the previous call.
 *  @param toggle_period_ms LED toggle interval in milliseconds.
 */
void LED_RunTickMs(u16 elapsed_ms, u16 toggle_period_ms);

#endif /* LED_SERVICE_H */
