/**
 * @file display_service.c
 * @brief Main-board display scheduling, rendering, and refresh coordination.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "display_service.h"

#include "board_config.h"
#include "board_link.h"
#include "ina226.h"
#include "ina226_service.h"
#if BOARD_IS_MAIN

/* Display model, formatting, and asynchronous LCD refresh pipeline. */
#include "lcd.h"
#include "lcd_init.h"
#endif
#include <stdio.h>

#define LCD_REFRESH_PERIOD_MS   100U
#define LCD_BOOT_DURATION_MS    4000U
#if BOARD_IS_MAIN
typedef enum {
    LCD_DISPLAY_BOOT_FIRST = 0U,
    LCD_DISPLAY_BOOT_WAIT_ON,
    LCD_DISPLAY_BOOT_ANIM,
    LCD_DISPLAY_NORMAL
} lcd_display_state_t;
#endif

#if BOARD_IS_MAIN
static volatile u8 s_display_flag = 1U;
static volatile u16 s_lcd_refresh_counter = 0U;
static volatile u16 s_lcd_boot_ms = 0U;
static volatile u8 s_lcd_display_state = LCD_DISPLAY_BOOT_FIRST;
#endif

void Display_TickMs(u16 elapsed_ms)
{
#if BOARD_IS_MAIN
    if ((s_lcd_display_state == LCD_DISPLAY_BOOT_ANIM) &&
        (s_lcd_boot_ms < LCD_BOOT_DURATION_MS)) {
        s_lcd_boot_ms = (u16)(s_lcd_boot_ms + elapsed_ms);
        if (s_lcd_boot_ms > LCD_BOOT_DURATION_MS) {
            s_lcd_boot_ms = LCD_BOOT_DURATION_MS;
        }
    }

    s_lcd_refresh_counter = (u16)(s_lcd_refresh_counter + elapsed_ms);
    if (s_lcd_refresh_counter >= LCD_REFRESH_PERIOD_MS) {
        s_lcd_refresh_counter = (u16)(s_lcd_refresh_counter - LCD_REFRESH_PERIOD_MS);
        s_display_flag = 1U;
    }
#endif

}

#if BOARD_IS_MAIN
/*
 * 160x128 LCD layout:
 *   C1: left top     A1: right top
 *   C2: left bottom  A2: right bottom
 */
#define LCD_PANEL_W             80U
#define LCD_PANEL_H             64U
#define LCD_PANEL_INNER_W       (LCD_PANEL_W - 4U)
#define LCD_PANEL_INNER_H       (LCD_PANEL_H - 4U)
#define LCD_HEADER_H            18U
#define LCD_VALUE_FONT_H        12U
#define LCD_PROTO_FONT_H        12U
#define LCD_PAGE_BG             0x0842U
#define LCD_PANEL_BG            0x18E3U
#define LCD_HEADER_BG           0x2965U
#define LCD_BORDER              0x5AEFU
#define LCD_FAULT_TEXT          0xFBE0U

/** @brief Draw the frame and header area for one measurement panel. */
static void LCD_DrawInaPanel(u16 x, u16 y, u16 accent)
{
    u16 x0 = (u16)(x + 2U);
    u16 y0 = (u16)(y + 2U);
    u16 x1 = (u16)(x0 + LCD_PANEL_INNER_W - 1U);
    u16 y1 = (u16)(y0 + LCD_PANEL_INNER_H - 1U);

    LCD_Fill(x0, y0, (u16)(x0 + LCD_PANEL_INNER_W), (u16)(y0 + LCD_PANEL_INNER_H), LCD_PANEL_BG);
    LCD_Fill(x0, y0, (u16)(x0 + LCD_PANEL_INNER_W), (u16)(y0 + LCD_HEADER_H), LCD_HEADER_BG);
    LCD_Fill(x0, y0, (u16)(x0 + 5U), (u16)(y0 + LCD_PANEL_INNER_H), accent);
    LCD_DrawRectangle(x0, y0, x1, y1, LCD_BORDER);
}

/** @brief Render the fault indication inside a measurement panel. */
static void LCD_ShowFaultArea(u16 x, u16 y, u16 color)
{
    (void)color;
    LCD_ShowString((u16)(x + 24U),
                   (u16)(y + 34U),
                   (const u8 *)"FAULT",
                   LCD_FAULT_TEXT,
                   LCD_PANEL_BG,
                   LCD_VALUE_FONT_H,
                   0);
}

/** @brief Return the short display label for a board-link protocol. */
static const u8 *LCD_ProtocolText(u8 protocol)
{
    switch (protocol) {
    case BOARD_LINK_PROTOCOL_TYPEC:
        return (const u8 *)"TYPEC";
    case BOARD_LINK_PROTOCOL_SPR_FIXED:
        return (const u8 *)"S-FIX";
    case BOARD_LINK_PROTOCOL_EPR_FIXED:
        return (const u8 *)"E-FIX";
    case BOARD_LINK_PROTOCOL_SPR_PPS:
        return (const u8 *)"S-PPS";
    case BOARD_LINK_PROTOCOL_SPR_AVS:
        return (const u8 *)"S-AVS";
    case BOARD_LINK_PROTOCOL_EPR_AVS:
        return (const u8 *)"E-AVS";
    case BOARD_LINK_PROTOCOL_BC12:
        return (const u8 *)"BC1.2";
    default:
        return (const u8 *)"--";
    }
}

/** @brief Interpolate two RGB565 colors for the boot animation. */
static u16 LCD_BlendColor(u16 from, u16 to, u16 step, u16 max_step)
{
    u16 from_r = (u16)((from >> 11) & 0x1FU);
    u16 from_g = (u16)((from >> 5) & 0x3FU);
    u16 from_b = (u16)(from & 0x1FU);
    u16 to_r = (u16)((to >> 11) & 0x1FU);
    u16 to_g = (u16)((to >> 5) & 0x3FU);
    u16 to_b = (u16)(to & 0x1FU);
    u16 r;
    u16 g;
    u16 b;

    if (step > max_step) {
        step = max_step;
    }

    r = (u16)((from_r * (max_step - step) + to_r * step) / max_step);
    g = (u16)((from_g * (max_step - step) + to_g * step) / max_step);
    b = (u16)((from_b * (max_step - step) + to_b * step) / max_step);

    return (u16)((r << 11) | (g << 5) | b);
}

/** @brief Calculate the RGB565 boot-gradient color at one scanline. */
static u16 LCD_BootGradientColor(u16 y, u16 elapsed_ms)
{
    const u16 top_target = 0x39E7U;
    u16 band_mix;
    u16 target;

    if (y >= LCD_H) {
        y = (u16)(LCD_H - 1U);
    }

    band_mix = (u16)(((u32)y * LCD_BOOT_DURATION_MS) / (LCD_H - 1U));
    target = LCD_BlendColor(top_target, LCD_PAGE_BG, band_mix, LCD_BOOT_DURATION_MS);
    return LCD_BlendColor(WHITE, target, elapsed_ms, LCD_BOOT_DURATION_MS);
}

/** @brief Fill the LCD background with the current boot gradient. */
static void LCD_FillBootGradient(u16 elapsed_ms)
{
    u16 y;
    const u16 band_h = 8U;

    LCD_Fill(0, 0, LCD_W, LCD_H, LCD_BootGradientColor(0U, elapsed_ms));

    for (y = 0U; y < LCD_H; y = (u16)(y + band_h)) {
        u16 color = LCD_BootGradientColor(y, elapsed_ms);
        u16 y_end = (u16)(y + band_h);

        if (y_end > LCD_H) {
            y_end = LCD_H;
        }

        LCD_Fill(0, y, LCD_W, y_end, color);
    }
}

/** @brief Render the animated boot page and progress indicator. */
static void LCD_ShowBootPage(u16 elapsed_ms)
{
    u16 bar_x = 20U;
    u16 bar_y = 78U;
    u16 bar_w = 120U;
    u16 bar_h = 14U;
    u16 fill_w;
    u16 percent;
    u16 title;
    u16 border;
    u16 track;
    u16 fill;
    u8 buf[16];

    if (elapsed_ms > LCD_BOOT_DURATION_MS) {
        elapsed_ms = LCD_BOOT_DURATION_MS;
    }

    fill_w = (u16)(((u32)(bar_w - 4U) * elapsed_ms) / LCD_BOOT_DURATION_MS);
    percent = (u16)(((u32)100U * elapsed_ms) / LCD_BOOT_DURATION_MS);
    title = LCD_BlendColor(BLACK, GBLUE, elapsed_ms, LCD_BOOT_DURATION_MS);
    border = LCD_BlendColor(LGRAY, LCD_BORDER, elapsed_ms, LCD_BOOT_DURATION_MS);
    track = LCD_BlendColor(WHITE, LCD_PANEL_BG, elapsed_ms, LCD_BOOT_DURATION_MS);
    fill = LCD_BlendColor(LIGHTBLUE, GBLUE, elapsed_ms, LCD_BOOT_DURATION_MS);

    LCD_FillBootGradient(elapsed_ms);
    LCD_ShowString(40U, 38U, (const u8 *)"Loading...", title, LCD_BootGradientColor(38U, elapsed_ms), 16, 0);

    LCD_DrawRectangle(bar_x, bar_y, (u16)(bar_x + bar_w - 1U), (u16)(bar_y + bar_h - 1U), border);
    LCD_Fill((u16)(bar_x + 2U),
             (u16)(bar_y + 2U),
             (u16)(bar_x + bar_w - 2U),
             (u16)(bar_y + bar_h - 2U),
             track);
    if (fill_w != 0U) {
        LCD_Fill((u16)(bar_x + 2U),
                 (u16)(bar_y + 2U),
                 (u16)(bar_x + 2U + fill_w),
                 (u16)(bar_y + bar_h - 2U),
                 fill);
    }

    sprintf((char *)buf, "%3d%%", (int)percent);
    LCD_ShowString(68U, 98U, buf, title, LCD_BootGradientColor(98U, elapsed_ms), 12, 0);
}

/** @brief Render converted voltage, current, power, and status for one channel. */
static void LCD_ShowInaChannel(u16 x,
                               u16 y,
                               const u8 *name,
                               u8 protocol,
                               u16 color,
                               const board_link_channel_t *ch)
{
    s32 v_mv = (s32)ina226_convert_bus_voltage_mv(ch->vbus_raw);
    s32 i_ma = ina226_convert_current_ma(ch->curr_raw,
                                         INA226_SERVICE_CURRENT_LSB_UA);
    s32 p_mw = (s32)ina226_convert_power_mw(ch->pwr_raw,
                                            INA226_SERVICE_CURRENT_LSB_UA);
    u8 buf[32];
    u16 label_x = (u16)(x + 12U);
    u16 value_x = (u16)(label_x + 12U);
    u16 name_x = (u16)(x + 12U);
    u16 proto_x = (u16)(x + 44U);

    LCD_DrawInaPanel(x, y, color);
    LCD_ShowString(name_x, (u16)(y + 3U), name, color, LCD_HEADER_BG, 16, 0);
    LCD_ShowString(proto_x,
                   (u16)(y + 5U),
                   LCD_ProtocolText(protocol),
                   color,
                   LCD_HEADER_BG,
                   LCD_PROTO_FONT_H,
                   0);

    if (ch->fault != 0U) {
        LCD_ShowFaultArea(x, y, color);
        return;
    }

    LCD_ShowString(label_x, (u16)(y + 20U), (const u8 *)"P:", color, LCD_PANEL_BG, LCD_VALUE_FONT_H, 0);
    sprintf((char *)buf, "%d.%03dW", (int)(p_mw / 1000), (int)(p_mw % 1000));
    LCD_ShowString(value_x, (u16)(y + 20U), buf, color, LCD_PANEL_BG, LCD_VALUE_FONT_H, 0);

    LCD_ShowString(label_x, (u16)(y + 34U), (const u8 *)"V:", color, LCD_PANEL_BG, LCD_VALUE_FONT_H, 0);
    sprintf((char *)buf, "%d.%03dV", (int)(v_mv / 1000), (int)(v_mv % 1000));
    LCD_ShowString(value_x, (u16)(y + 34U), buf, color, LCD_PANEL_BG, LCD_VALUE_FONT_H, 0);

    {
        s32 frac = i_ma % 1000;
        if (frac < 0) {
            frac = -frac;
        }
        sprintf((char *)buf, "%d.%03dA", (int)(i_ma / 1000), (int)frac);
    }
    LCD_ShowString(label_x, (u16)(y + 48U), (const u8 *)"I:", color, LCD_PANEL_BG, LCD_VALUE_FONT_H, 0);
    LCD_ShowString(value_x, (u16)(y + 48U), buf, color, LCD_PANEL_BG, LCD_VALUE_FONT_H, 0);
}

/** @brief Render all local and remote INA226 measurement panels. */
static void LCD_ShowInaPage(void)
{
    board_link_channel_t c1;
    board_link_channel_t a1;
    board_link_remote_t remote;

    BoardLink_BuildChannels(&c1, &a1);
    BoardLink_GetRemoteData(&remote);

    LCD_Fill(0, 0, LCD_W, LCD_H, LCD_PAGE_BG);
    LCD_ShowInaChannel(0, 0, (const u8 *)"C1", c1.protocol, RED, &c1);
    LCD_ShowInaChannel(80, 0, (const u8 *)"A1", BOARD_LINK_PROTOCOL_BC12, GBLUE, &a1);
    LCD_ShowInaChannel(0, 64, (const u8 *)"C2", remote.c.protocol, GREEN, &remote.c);
    LCD_ShowInaChannel(80, 64, (const u8 *)"A2", BOARD_LINK_PROTOCOL_BC12, YELLOW, &remote.a);
}
#endif

void Display_Task(void)
{
#if BOARD_IS_MAIN
    if (s_display_flag != 0U && LCD_IsIdle() != 0U) {
        u8 display_state = s_lcd_display_state;

        s_display_flag = 0U;

        if (display_state == LCD_DISPLAY_BOOT_FIRST) {
            s_lcd_boot_ms = 0U;
            LCD_ShowBootPage(0U);
            s_lcd_display_state = LCD_DISPLAY_BOOT_WAIT_ON;
        } else if (display_state == LCD_DISPLAY_BOOT_WAIT_ON) {
            LCD_DisplayOn();
            s_lcd_display_state = LCD_DISPLAY_BOOT_ANIM;
            return;
        } else if (display_state == LCD_DISPLAY_BOOT_ANIM) {
            u16 boot_ms = s_lcd_boot_ms;

            if (boot_ms < LCD_BOOT_DURATION_MS) {
                LCD_ShowBootPage(boot_ms);
            } else {
                s_lcd_display_state = LCD_DISPLAY_NORMAL;
                LCD_ShowInaPage();
            }
        } else {
            LCD_ShowInaPage();
        }
        LCD_RequestRefresh();
    }
#endif
}
