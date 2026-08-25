/**
 * @file led_service.c
 * @brief LED timing helper used by the board timer task.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "led_service.h"

static volatile u16 s_led_elapsed_ms = 0U;

void LED_RunTickMs(u16 elapsed_ms, u16 toggle_period_ms)
{
    u16 next_ms;

    if (toggle_period_ms == 0U) {
        return;
    }

    next_ms = (u16)(s_led_elapsed_ms + elapsed_ms);
    if (next_ms >= toggle_period_ms) {
        s_led_elapsed_ms = (u16)(next_ms - toggle_period_ms);
        LED_TOGGLE();
    } else {
        s_led_elapsed_ms = next_ms;
    }
}
