/**
 * @file display_service.c
 * @brief Main-board display scheduling, rendering, and refresh coordination.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-11-05
 */

#include "display_service.h"

#include "board_config.h"
#include "board_link.h"
#include "ina226.h"
#include "ina226_service.h"
#if BOARD_IS_MAIN

/* Display model, formatting, and asynchronous LCD refresh pipeline. */
#include "lcd.h"
#include "lcd_pic.h"
#include "pd.h"
#include "st7735s.h"
#endif

/* Measurement page refresh period and boot page duration. The refresh period also
 * paces the boot state machine: each due slot draws at most one frame. */
#define DISPLAY_REFRESH_PERIOD_MS   100u
#define DISPLAY_BOOT_DURATION_MS    2000u

/* Wait this long between DISPON and switching the backlight on. While the panel
 * is display-off it outputs a blank white page, and DISPON only takes effect at
 * the next panel refresh, so lighting the backlight together with DISPON can
 * still show one white frame. */
#define DISPLAY_BACKLIGHT_SETTLE_MS 50u

/* Set to 1 to replace the remote C2/A2 panels with the PD diagnostic block. It stays behind
 * a switch because it is the only way to read the PD failure bookkeeping (policy state, last
 * failure code, GoodCRC stage, reset counts, frames lost between the PHY and the policy
 * layer) without a debugger, while normal operation shows all four channels. */
#define PD_DEBUG_PAGE 0

/* Boot page colors. The progress bar animates; the background and text colors
 * are fixed, so no gradient blending is needed. */
#define DISPLAY_BOOT_BG             DISPLAY_PAGE_BG
#define DISPLAY_BOOT_TEXT           GBLUE
#define DISPLAY_BOOT_BAR            DISPLAY_BORDER
#define DISPLAY_BOOT_BAR_TRACK      DISPLAY_PANEL_BG
#define DISPLAY_BOOT_BAR_FILL       LIGHTBLUE

/* Boot page layout on the 160x128 panel, top to bottom: a 60x60 logo, the
 * caption, the progress bar, and the percentage. Bands never overlap, so the
 * logo cannot cover the text: logo rows 6..65, caption 70..85, bar 94..107,
 * percentage 112..123. */
#define DISPLAY_BOOT_LOGO_W         60u
#define DISPLAY_BOOT_LOGO_H         60u
#define DISPLAY_BOOT_LOGO_X         ((LCD_WIDTH - DISPLAY_BOOT_LOGO_W) / 2u)
#define DISPLAY_BOOT_LOGO_Y         6u
#define DISPLAY_BOOT_TEXT_X         40u
#define DISPLAY_BOOT_TEXT_Y         70u
#define DISPLAY_BOOT_BAR_X          20u
#define DISPLAY_BOOT_BAR_Y          94u
#define DISPLAY_BOOT_BAR_W          120u
#define DISPLAY_BOOT_BAR_H          14u
#define DISPLAY_BOOT_PCT_X          68u
#define DISPLAY_BOOT_PCT_Y          112u

#if BOARD_IS_MAIN
/** @brief Boot sequence: dark page, panel on, backlight on, then measurements. */
typedef enum {
    DISPLAY_STATE_BOOT_FIRST = 0u,  /**< Draw the boot page while the panel is off. */
    DISPLAY_STATE_BOOT_WAIT_ON,     /**< Send DISPON, still with the backlight off. */
    DISPLAY_STATE_BOOT_LIGHT,       /**< Wait for the page to latch, light the LEDs. */
    DISPLAY_STATE_BOOT_ANIM,        /**< Run the progress bar for the boot duration. */
    DISPLAY_STATE_NORMAL            /**< Refresh the four measurement panels. */
} display_state_t;
#endif

#if BOARD_IS_MAIN
/* Boot sequence and refresh pacing. The tick handler runs in the TIM1 interrupt and
 * only touches these, while Display_Task() consumes them from the main loop; each one
 * is a single byte or halfword, so every access is atomic on this core. */
static volatile bool s_display_flag = true;                             /**< Set by the tick when a frame is due. */
static volatile uint16_t s_lcd_refresh_counter = 0u;                    /**< Milliseconds since the last slot. */
static volatile uint16_t s_lcd_boot_ms = 0u;                            /**< Boot progress, or backlight settle timer. */
static volatile uint8_t s_lcd_display_state = DISPLAY_STATE_BOOT_FIRST; /**< Boot state machine. */
#endif

void Display_TickMs(uint16_t elapsed_ms)
{
#if BOARD_IS_MAIN
    if (((s_lcd_display_state == DISPLAY_STATE_BOOT_ANIM) ||
         (s_lcd_display_state == DISPLAY_STATE_BOOT_LIGHT)) &&
         (s_lcd_boot_ms < DISPLAY_BOOT_DURATION_MS)) {
        s_lcd_boot_ms = (uint16_t)(s_lcd_boot_ms + elapsed_ms);
        if (s_lcd_boot_ms > DISPLAY_BOOT_DURATION_MS) {
            s_lcd_boot_ms = DISPLAY_BOOT_DURATION_MS;
        }
    }

    s_lcd_refresh_counter = (uint16_t)(s_lcd_refresh_counter + elapsed_ms);
    if (s_lcd_refresh_counter >= DISPLAY_REFRESH_PERIOD_MS) {
        s_lcd_refresh_counter = (uint16_t)(s_lcd_refresh_counter - DISPLAY_REFRESH_PERIOD_MS);
        s_display_flag = true;
    }
#endif

}

#if BOARD_IS_MAIN
/*
 * 160x128 LCD layout:
 *   C1: left top     A1: right top
 *   C2: left bottom  A2: right bottom
 */
#define DISPLAY_PANEL_W             80u
#define DISPLAY_PANEL_H             64u
#define DISPLAY_PANEL_INNER_W       (DISPLAY_PANEL_W - 4u)
#define DISPLAY_PANEL_INNER_H       (DISPLAY_PANEL_H - 4u)
#define DISPLAY_HEADER_H            18u
#define DISPLAY_VALUE_FONT_H        12u
#define DISPLAY_PROTO_FONT_H        12u
#define DISPLAY_PAGE_BG             0x0842u
#define DISPLAY_PANEL_BG            0x18E3u
#define DISPLAY_HEADER_BG           0x2965u
#define DISPLAY_BORDER              0x5AEFu
#define DISPLAY_FAULT_TEXT          0xFBE0u
#define DISPLAY_OFFLINE_TEXT        0xC618u

/** @brief Draw the frame and header area for one measurement panel. */
static void Display_DrawPanel(uint16_t x, uint16_t y, uint16_t accent)
{
    uint16_t x0 = (uint16_t)(x + 2u);
    uint16_t y0 = (uint16_t)(y + 2u);
    uint16_t x1 = (uint16_t)(x0 + DISPLAY_PANEL_INNER_W - 1u);
    uint16_t y1 = (uint16_t)(y0 + DISPLAY_PANEL_INNER_H - 1u);

    lcd_fill(x0, y0, (uint16_t)(x0 + DISPLAY_PANEL_INNER_W), (uint16_t)(y0 + DISPLAY_PANEL_INNER_H), DISPLAY_PANEL_BG);
    lcd_fill(x0, y0, (uint16_t)(x0 + DISPLAY_PANEL_INNER_W), (uint16_t)(y0 + DISPLAY_HEADER_H), DISPLAY_HEADER_BG);
    lcd_fill(x0, y0, (uint16_t)(x0 + 5u), (uint16_t)(y0 + DISPLAY_PANEL_INNER_H), accent);
    lcd_draw_rect(x0, y0, x1, y1, DISPLAY_BORDER);
}

/** @brief Render the fault indication inside a measurement panel. */
static void Display_ShowFault(uint16_t x, uint16_t y)
{
    lcd_show_string((uint16_t)(x + 24u),
                    (uint16_t)(y + 34u),
                    (const uint8_t *)"FAULT",
                    DISPLAY_FAULT_TEXT,
                    DISPLAY_PANEL_BG,
                    DISPLAY_VALUE_FONT_H,
                    0);
}

/**
 * @brief Render the link-loss indication inside a measurement panel.
 *
 * Used for the two remote channels: their measurements and protocol come from the
 * other board, so once frames stop arriving there is nothing truthful left to show
 * and the panel reports the disconnect instead of a stale reading. The word is kept
 * short enough to stay well inside the 76-pixel panel body at the 6-pixel font.
 */
static void Display_ShowOffline(uint16_t x, uint16_t y)
{
    lcd_show_string((uint16_t)(x + 19u),
                    (uint16_t)(y + 34u),
                    (const uint8_t *)"OFFLINE",
                    DISPLAY_OFFLINE_TEXT,
                    DISPLAY_PANEL_BG,
                    DISPLAY_VALUE_FONT_H,
                    0);
}

/** @brief Return the short display label for a board-link protocol. */
static const uint8_t *Display_ProtocolText(uint8_t protocol)
{
    switch (protocol) {
    case BOARD_LINK_PROTOCOL_TYPEC:
        return (const uint8_t *)"TYPEC";
    case BOARD_LINK_PROTOCOL_SPR_FIXED:
        return (const uint8_t *)"S-FIX";
    case BOARD_LINK_PROTOCOL_EPR_FIXED:
        return (const uint8_t *)"E-FIX";
    case BOARD_LINK_PROTOCOL_SPR_PPS:
        return (const uint8_t *)"S-PPS";
    case BOARD_LINK_PROTOCOL_SPR_AVS:
        return (const uint8_t *)"S-AVS";
    case BOARD_LINK_PROTOCOL_EPR_AVS:
        return (const uint8_t *)"E-AVS";
    case BOARD_LINK_PROTOCOL_BC12:
        return (const uint8_t *)"BC1.2";
    default:
        return (const uint8_t *)"--";
    }
}

/**
 * @brief Format a milli-unit value with three decimals, for example "-12.345A".
 *
 * Replaces the previous sprintf() calls: this is the only formatting in the
 * firmware, so the printf machinery can stay out of the image.
 *
 * @param buf Destination buffer, at least 14 bytes.
 * @param value Scaled value in thousandths of the displayed unit.
 * @param suffix Unit character appended after the digits.
 */
static void Display_FormatMilli(char *buf, int32_t value, char suffix)
{
    char digits[10];
    uint8_t count = 0u;
    uint8_t n = 0u;
    uint32_t magnitude;
    uint32_t whole;
    uint32_t frac;

    if (value < 0) {
        buf[n++] = '-';
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }

    whole = magnitude / 1000u;
    frac = magnitude % 1000u;

    do {
        digits[count++] = (char)('0' + (whole % 10u));
        whole /= 10u;
    } while (whole != 0u);

    while (count != 0u) {
        buf[n++] = digits[--count];
    }

    buf[n++] = '.';
    buf[n++] = (char)('0' + (frac / 100u));
    buf[n++] = (char)('0' + ((frac / 10u) % 10u));
    buf[n++] = (char)('0' + (frac % 10u));
    buf[n++] = suffix;
    buf[n] = '\0';
}

/**
 * @brief Format a percentage into three right-aligned columns, for example " 42%".
 * @param buf Destination buffer, at least 5 bytes.
 * @param percent Percentage value, clamped to 100.
 */
static void Display_FormatPercent(char *buf, uint16_t percent)
{
    if (percent > 100u) {
        percent = 100u;
    }

    if (percent >= 100u) {
        buf[0] = '1';
        buf[1] = '0';
        buf[2] = '0';
    } else if (percent >= 10u) {
        buf[0] = ' ';
        buf[1] = (char)('0' + (percent / 10u));
        buf[2] = (char)('0' + (percent % 10u));
    } else {
        buf[0] = ' ';
        buf[1] = ' ';
        buf[2] = (char)('0' + percent);
    }

    buf[3] = '%';
    buf[4] = '\0';
}

/** @brief Render the boot page and its progress indicator. */
static void Display_ShowBootPage(uint16_t elapsed_ms)
{
    uint16_t fill_w;
    uint16_t percent;
    uint8_t buf[16];

    if (elapsed_ms > DISPLAY_BOOT_DURATION_MS) {
        elapsed_ms = DISPLAY_BOOT_DURATION_MS;
    }

    fill_w = (uint16_t)(((uint32_t)(DISPLAY_BOOT_BAR_W - 4u) * elapsed_ms) / DISPLAY_BOOT_DURATION_MS);
    percent = (uint16_t)(((uint32_t)100u * elapsed_ms) / DISPLAY_BOOT_DURATION_MS);

    lcd_fill(0u, 0u, LCD_WIDTH, LCD_HEIGHT, DISPLAY_BOOT_BG);
    lcd_show_picture_indexed(DISPLAY_BOOT_LOGO_X, DISPLAY_BOOT_LOGO_Y,
                             DISPLAY_BOOT_LOGO_W, DISPLAY_BOOT_LOGO_H, gImage_boot_logo);
    lcd_show_string(DISPLAY_BOOT_TEXT_X, DISPLAY_BOOT_TEXT_Y,
                    (const uint8_t *)"Loading...",
                    DISPLAY_BOOT_TEXT, DISPLAY_BOOT_BG, 16, 0);

    lcd_draw_rect(DISPLAY_BOOT_BAR_X, DISPLAY_BOOT_BAR_Y,
                  (uint16_t)(DISPLAY_BOOT_BAR_X + DISPLAY_BOOT_BAR_W - 1u),
                  (uint16_t)(DISPLAY_BOOT_BAR_Y + DISPLAY_BOOT_BAR_H - 1u),
                  DISPLAY_BOOT_BAR);
    lcd_fill((uint16_t)(DISPLAY_BOOT_BAR_X + 2u),
             (uint16_t)(DISPLAY_BOOT_BAR_Y + 2u),
             (uint16_t)(DISPLAY_BOOT_BAR_X + DISPLAY_BOOT_BAR_W - 2u),
             (uint16_t)(DISPLAY_BOOT_BAR_Y + DISPLAY_BOOT_BAR_H - 2u),
             DISPLAY_BOOT_BAR_TRACK);
    if (fill_w != 0u) {
        lcd_fill((uint16_t)(DISPLAY_BOOT_BAR_X + 2u),
                 (uint16_t)(DISPLAY_BOOT_BAR_Y + 2u),
                 (uint16_t)(DISPLAY_BOOT_BAR_X + 2u + fill_w),
                 (uint16_t)(DISPLAY_BOOT_BAR_Y + DISPLAY_BOOT_BAR_H - 2u),
                 DISPLAY_BOOT_BAR_FILL);
    }

    Display_FormatPercent((char *)buf, percent);
    lcd_show_string(DISPLAY_BOOT_PCT_X, DISPLAY_BOOT_PCT_Y, buf,
                    DISPLAY_BOOT_TEXT, DISPLAY_BOOT_BG, 12, 0);
}

/**
 * @brief Render one measurement panel.
 * @param x Left edge of the panel.
 * @param y Top edge of the panel.
 * @param name Channel label shown in the header.
 * @param protocol Board-link protocol identifier, or NONE when unknown.
 * @param color Accent colour of the panel.
 * @param ch Latest channel snapshot.
 * @param offline Non-zero when the data source is a remote board that stopped
 *                sending frames; the values are then not shown at all.
 */
static void Display_ShowChannel(uint16_t x,
                                uint16_t y,
                                const uint8_t *name,
                                uint8_t protocol,
                                uint16_t color,
                                const board_link_channel_t *ch,
                                uint8_t offline)
{
    int32_t v_mv = (int32_t)ina226_convert_bus_voltage_mv(ch->vbus_raw);
    int32_t i_ma = ina226_convert_current_ma(ch->curr_raw,
                                             INA226_SERVICE_CURRENT_LSB_UA);
    int32_t p_mw = (int32_t)ina226_convert_power_mw(ch->pwr_raw,
                                                    INA226_SERVICE_CURRENT_LSB_UA);
    uint8_t buf[24];
    uint16_t text_x = (uint16_t)(x + 12u);
    uint16_t proto_x = (uint16_t)(x + 44u);

    Display_DrawPanel(x, y, color);
    lcd_show_string(text_x, (uint16_t)(y + 3u), name, color, DISPLAY_HEADER_BG, 16, 0);
    lcd_show_string(proto_x,
                    (uint16_t)(y + 5u),
                    Display_ProtocolText(offline != 0u ? BOARD_LINK_PROTOCOL_NONE : protocol),
                    color,
                    DISPLAY_HEADER_BG,
                    DISPLAY_PROTO_FONT_H,
                    0);

    if (offline != 0u) {
        Display_ShowOffline(x, y);
        return;
    }

    if (ch->fault != 0u) {
        Display_ShowFault(x, y);
        return;
    }

    /* Label and value share one string. "P:" plus two 6-pixel characters puts
     * the digits on the same pixels as two separate strings, but uses one queue
     * slot and one text pass per scan line instead of two. */
    buf[0] = 'P';
    buf[1] = ':';
    Display_FormatMilli((char *)&buf[2], p_mw, 'W');
    lcd_show_string(text_x, (uint16_t)(y + 20u), buf, color, DISPLAY_PANEL_BG, DISPLAY_VALUE_FONT_H, 0);

    buf[0] = 'V';
    buf[1] = ':';
    Display_FormatMilli((char *)&buf[2], v_mv, 'V');
    lcd_show_string(text_x, (uint16_t)(y + 34u), buf, color, DISPLAY_PANEL_BG, DISPLAY_VALUE_FONT_H, 0);

    buf[0] = 'I';
    buf[1] = ':';
    Display_FormatMilli((char *)&buf[2], i_ma, 'A');
    lcd_show_string(text_x, (uint16_t)(y + 48u), buf, color, DISPLAY_PANEL_BG, DISPLAY_VALUE_FONT_H, 0);
}

#if PD_DEBUG_PAGE
/** @brief Append a two-digit decimal number to a text buffer. */
static uint8_t Display_PutTwoDigits(uint8_t *buf, uint8_t pos, uint16_t value)
{
    value = (uint16_t)(value % 100u);
    buf[pos++] = (uint8_t)('0' + (uint8_t)(value / 10u));
    buf[pos++] = (uint8_t)('0' + (uint8_t)(value % 10u));
    return pos;
}

/** @brief Append a three-digit decimal number to a text buffer. */
static uint8_t Display_PutThreeDigits(uint8_t *buf, uint8_t pos, uint16_t value)
{
    value = (uint16_t)(value % 1000u);
    buf[pos++] = (uint8_t)('0' + (uint8_t)(value / 100u));
    buf[pos++] = (uint8_t)('0' + (uint8_t)((value / 10u) % 10u));
    buf[pos++] = (uint8_t)('0' + (uint8_t)(value % 10u));
    return pos;
}

/**
 * @brief Render the PD diagnostic block over the bottom half of the page.
 *
 * Replaces the two remote-board panels when PD_DEBUG_PAGE is set: the PD layer already keeps
 * the failure bookkeeping, and showing it on the panel attributes a surprise reset (local
 * Soft/Hard Reset, a partner Hard Reset, GoodCRC recovery, EPR traffic silence) without a
 * debugger. Layout, 12-pixel font:
 *
 *   PD DEBUG
 *   ST nn  EV nn  FL nn     policy state / last event / last failure code
 *   RST Rnn Snn Hnn         partner hard resets / own soft / own hard resets
 *   KC nnn  MX nnn          EPR keepalives received / peak traffic silence seen here
 *   RX nnn  MSG nnn         frames seen by the PHY / frames handed to the policy layer
 *
 * MX is sampled here on every refresh instead of being taken from the PD layer, so it also
 * records a silence that ends before the keepalive watchdog reacts.
 */
static void Display_ShowPdDebug(void)
{
    static uint16_t s_pd_age_peak;
    pd_status_t st;
    uint8_t buf[20];
    uint8_t n;
    uint16_t y = 78u;

    PD_GetStatus(&st);

    if (st.debug_epr_keepalive_age_ms > s_pd_age_peak) {
        s_pd_age_peak = st.debug_epr_keepalive_age_ms;
    }

    lcd_fill(0u, 64u, LCD_WIDTH, LCD_HEIGHT, DISPLAY_PAGE_BG);
    lcd_draw_rect(2u, 66u, (uint16_t)(LCD_WIDTH - 3u), (uint16_t)(LCD_HEIGHT - 3u),
                  DISPLAY_BORDER);
    lcd_show_string(6u, 68u, (const uint8_t *)"PD DEBUG", DISPLAY_FAULT_TEXT,
                    DISPLAY_PAGE_BG, DISPLAY_VALUE_FONT_H, 0);

    n = 0u;
    buf[n++] = 'S';
    buf[n++] = 'T';
    buf[n++] = ' ';
    n = Display_PutTwoDigits(buf, n, (uint16_t)st.state);
    buf[n++] = ' ';
    buf[n++] = ' ';
    buf[n++] = 'F';
    buf[n++] = 'L';
    buf[n++] = ' ';
    n = Display_PutTwoDigits(buf, n, st.debug_last_failure);
    buf[n++] = ' ';
    buf[n++] = ' ';
    buf[n++] = 'G';
    buf[n++] = 'T';
    buf[n++] = ' ';
    n = Display_PutTwoDigits(buf, n, st.debug_goodcrc_fail_msg_type);
    buf[n++] = ' ';
    buf[n++] = ' ';
    buf[n++] = 'G';
    buf[n++] = 'S';
    buf[n++] = ' ';
    n = Display_PutTwoDigits(buf, n, st.debug_goodcrc_fail_stage);
    buf[n] = '\0';
    lcd_show_string(6u, y, buf, DISPLAY_FAULT_TEXT, DISPLAY_PAGE_BG,
                    DISPLAY_VALUE_FONT_H, 0);
    y = (uint16_t)(y + 12u);

    n = 0u;
    buf[n++] = 'R';
    buf[n++] = 'S';
    buf[n++] = 'T';
    buf[n++] = ' ';
    buf[n++] = 'R';
    n = Display_PutTwoDigits(buf, n, st.debug_rx_hard_reset_count);
    buf[n++] = ' ';
    buf[n++] = 'S';
    n = Display_PutTwoDigits(buf, n, st.debug_soft_reset_count);
    buf[n++] = ' ';
    buf[n++] = 'H';
    n = Display_PutTwoDigits(buf, n, st.debug_hard_reset_count);
    buf[n] = '\0';
    lcd_show_string(6u, y, buf, RED, DISPLAY_PAGE_BG, DISPLAY_VALUE_FONT_H, 0);
    y = (uint16_t)(y + 12u);

    n = 0u;
    buf[n++] = 'K';
    buf[n++] = 'C';
    buf[n++] = ' ';
    n = Display_PutThreeDigits(buf, n, st.debug_keepalive_rx_count);
    buf[n++] = ' ';
    buf[n++] = ' ';
    buf[n++] = 'M';
    buf[n++] = 'X';
    buf[n++] = ' ';
    n = Display_PutThreeDigits(buf, n, s_pd_age_peak);
    buf[n] = '\0';
    lcd_show_string(6u, y, buf, GBLUE, DISPLAY_PAGE_BG, DISPLAY_VALUE_FONT_H, 0);
    y = (uint16_t)(y + 12u);

    /* Received frames versus frames handed to the policy layer, plus the deepest output
     * sag of the last window. A growing gap between RX and MSG means traffic is being lost
     * between the interrupt and pd_handle_message(). */
    n = 0u;
    buf[n++] = 'R';
    buf[n++] = 'X';
    buf[n++] = ' ';
    n = Display_PutThreeDigits(buf, n, st.debug_rx_frame_count);
    buf[n++] = ' ';
    buf[n++] = ' ';
    buf[n++] = 'M';
    buf[n++] = 'S';
    buf[n++] = 'G';
    buf[n++] = ' ';
    n = Display_PutThreeDigits(buf, n, st.debug_rx_msg_count);
    buf[n] = '\0';
    lcd_show_string(6u, y, buf, GREEN, DISPLAY_PAGE_BG, DISPLAY_VALUE_FONT_H, 0);
}
#endif /* PD_DEBUG_PAGE */

/**
 * @brief Render the measurement page.
 *
 * Top half is the local C1/A1 pair; the bottom half shows the remote C2/A2 pair, or the PD
 * diagnostic block instead when PD_DEBUG_PAGE is set.
 */
static void Display_ShowPage(void)
{
    board_link_channel_t c1;
    board_link_channel_t a1;

    BoardLink_BuildChannels(&c1, &a1);

    lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT, DISPLAY_PAGE_BG);
    Display_ShowChannel(0, 0, (const uint8_t *)"C1", c1.protocol, RED, &c1, 0u);
    Display_ShowChannel(80, 0, (const uint8_t *)"A1", BOARD_LINK_PROTOCOL_BC12, GBLUE, &a1, 0u);
#if PD_DEBUG_PAGE
    Display_ShowPdDebug();
#else
    {
        board_link_remote_t remote;
        uint8_t remote_online;

        BoardLink_GetRemoteData(&remote);
        remote_online = BoardLink_RemoteOnline();
        Display_ShowChannel(0, 64, (const uint8_t *)"C2", remote.c.protocol, GREEN, &remote.c,
                            (remote_online == 0u) ? 1u : 0u);
        Display_ShowChannel(80, 64, (const uint8_t *)"A2", BOARD_LINK_PROTOCOL_BC12, YELLOW,
                            &remote.a, (remote_online == 0u) ? 1u : 0u);
    }
#endif
}
#endif

void Display_Task(void)
{
#if BOARD_IS_MAIN
    if (s_display_flag && lcd_is_idle()) {
        uint8_t display_state = s_lcd_display_state;

        s_display_flag = false;

        if (display_state == DISPLAY_STATE_BOOT_FIRST) {
            s_lcd_boot_ms = 0u;
            Display_ShowBootPage(0u);
            s_lcd_display_state = DISPLAY_STATE_BOOT_WAIT_ON;
        } else if (display_state == DISPLAY_STATE_BOOT_WAIT_ON) {
            /* Enable the panel output, but keep the backlight off: while the panel
             * is display-off it drives a blank white page, and DISPON only takes
             * effect at the next panel refresh. */
            st7735s_display_on();
            s_lcd_boot_ms = 0u;
            s_lcd_display_state = DISPLAY_STATE_BOOT_LIGHT;
            return;
        } else if (display_state == DISPLAY_STATE_BOOT_LIGHT) {
            if (s_lcd_boot_ms >= DISPLAY_BACKLIGHT_SETTLE_MS) {
                /* The boot page is latched and visible now, light it up. */
                st7735s_set_backlight(1u);
                s_lcd_boot_ms = 0u;
                s_lcd_display_state = DISPLAY_STATE_BOOT_ANIM;
                Display_ShowBootPage(0u);
            } else {
                /* Not settled yet; the next refresh period re-enters here. */
                return;
            }
        } else if (display_state == DISPLAY_STATE_BOOT_ANIM) {
            uint16_t boot_ms = s_lcd_boot_ms;

            if (boot_ms < DISPLAY_BOOT_DURATION_MS) {
                Display_ShowBootPage(boot_ms);
            } else {
                s_lcd_display_state = DISPLAY_STATE_NORMAL;
                Display_ShowPage();
            }
        } else {
            Display_ShowPage();
        }
        lcd_request_refresh();
    }
#endif
}
