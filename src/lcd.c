/**
 * @file lcd.c
 * @brief Asynchronous LCD drawing queue and scan-line SPI refresh engine.
 * @author 中景园电子
 * @date 2018-10-31
 * @modder LuoXiFNT
 * @lastModified 2026-08-23
 */

#include "lcd.h"
#include "ch32l103_spi_dma.h"
#include "lcd_init.h"
#include "lcdfont.h"

#include <string.h>

#define LCD_DRAW_ITEM_MAX       64U
#define LCD_TEXT_ITEM_CHARS     20U
#define LCD_LINE_BYTES          (LCD_W * 2U)

#define LCD_ITEM_FILL           0U
#define LCD_ITEM_LINE           1U
#define LCD_ITEM_TEXT           2U
#define LCD_ITEM_IMAGE          3U

#define LCD_REFRESH_IDLE        0U
#define LCD_REFRESH_CASET_CMD   1U
#define LCD_REFRESH_CASET_DATA  2U
#define LCD_REFRESH_RASET_CMD   3U
#define LCD_REFRESH_RASET_DATA  4U
#define LCD_REFRESH_RAMWR_CMD   5U
#define LCD_REFRESH_LINE        6U

typedef struct {
    uint8_t type;
    uint8_t sizey;
    uint8_t mode;
    uint8_t reserved;
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
    uint16_t color;
    uint16_t bg_color;
    uint16_t width;
    uint16_t height;
    const uint8_t *image;
    char text[LCD_TEXT_ITEM_CHARS + 1U];
} lcd_draw_item_t;

static lcd_draw_item_t s_draw_items[LCD_DRAW_ITEM_MAX];
static lcd_draw_item_t s_active_items[LCD_DRAW_ITEM_MAX];
static uint8_t s_draw_count;
static uint8_t s_active_count;
static uint16_t s_draw_bg_color = WHITE;
static uint16_t s_active_bg_color = WHITE;

/* Asynchronous SPI refresh state and scan-line rendering helpers. */
static volatile uint8_t s_refresh_pending;
static uint8_t s_bus_selected;
static uint8_t s_refresh_state = LCD_REFRESH_IDLE;
static uint16_t s_refresh_y;
static uint8_t s_cmd_byte;
static uint8_t s_addr_bytes[4];
static uint8_t s_line_buf[LCD_LINE_BYTES];

/** @brief Reserve and initialize one pending drawing command. */
static lcd_draw_item_t *lcd_add_item(uint8_t type)
{
    lcd_draw_item_t *item;

    if (s_draw_count >= LCD_DRAW_ITEM_MAX) {
        return 0;
    }

    item = &s_draw_items[s_draw_count++];
    memset(item, 0, sizeof(*item));
    item->type = type;
    return item;
}

/** @brief Return the bitmap value of one glyph pixel. */
static uint8_t lcd_font_pixel(uint8_t ch, uint8_t sizey, uint8_t x, uint8_t y)
{
    uint8_t sizex;
    uint8_t bytes_per_row;
    uint16_t index;
    const unsigned char *font = 0;

    if (ch < ' ' || ch > '~') {
        ch = '?';
    }
    ch = (uint8_t)(ch - ' ');
    sizex = (uint8_t)(sizey / 2U);
    bytes_per_row = (uint8_t)((sizex + 7U) / 8U);
    index = (uint16_t)y * bytes_per_row + (uint16_t)(x / 8U);

    if (sizey == 12U) {
        font = ascii_1206[ch];
    } else if (sizey == 16U) {
        font = ascii_1608[ch];
    } else if (sizey == 24U) {
        font = ascii_2412[ch];
    } else if (sizey == 32U) {
        font = ascii_3216[ch];
    } else {
        return 0U;
    }

    return (font[index] & (uint8_t)(1U << (x & 7U))) ? 1U : 0U;
}

/** @brief Write one RGB565 pixel to the active scan-line buffer. */
static void lcd_line_put_color(uint16_t x, uint16_t color)
{
    uint16_t offset;

    if (x >= LCD_W) {
        return;
    }

    offset = (uint16_t)(x * 2U);
    s_line_buf[offset] = (uint8_t)(color >> 8U);
    s_line_buf[offset + 1U] = (uint8_t)(color & 0xFFU);
}

/** @brief Fill a clipped horizontal span in the active scan-line buffer. */
static void lcd_line_fill_span(int16_t x1, int16_t x2, uint16_t color)
{
    int16_t x;

    if (x2 < 0 || x1 >= (int16_t)LCD_W) {
        return;
    }
    if (x1 < 0) {
        x1 = 0;
    }
    if (x2 >= (int16_t)LCD_W) {
        x2 = (int16_t)LCD_W - 1;
    }

    for (x = x1; x <= x2; x++) {
        lcd_line_put_color((uint16_t)x, color);
    }
}

/** @brief Rasterize one filled-rectangle command on a scan line. */
static void lcd_render_fill_item(const lcd_draw_item_t *item, uint16_t y)
{
    if ((int16_t)y < item->y1 || (int16_t)y > item->y2) {
        return;
    }

    lcd_line_fill_span(item->x1, item->x2, item->color);
}

/** @brief Rasterize one line command on a scan line. */
static void lcd_render_line_item(const lcd_draw_item_t *item, uint16_t scan_y)
{
    int x0 = item->x1;
    int y0 = item->y1;
    int x1 = item->x2;
    int y1 = item->y2;
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = (y1 > y0) ? (y0 - y1) : (y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int e2;

    for (;;) {
        if (y0 == (int)scan_y) {
            lcd_line_put_color((uint16_t)x0, item->color);
        }
        if (x0 == x1 && y0 == y1) {
            break;
        }
        e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

/** @brief Rasterize one text command on a scan line. */
static void lcd_render_text_item(const lcd_draw_item_t *item, uint16_t scan_y)
{
    uint8_t sizex = (uint8_t)(item->sizey / 2U);
    uint8_t row;
    uint8_t i;
    uint8_t px;
    int16_t x;

    if (sizex == 0U) {
        return;
    }
    if ((int16_t)scan_y < item->y1 ||
        (int16_t)scan_y >= (int16_t)(item->y1 + item->sizey)) {
        return;
    }

    row = (uint8_t)((int16_t)scan_y - item->y1);
    for (i = 0U; item->text[i] != '\0'; i++) {
        x = (int16_t)(item->x1 + (int16_t)i * sizex);
        if (x >= (int16_t)LCD_W) {
            break;
        }
        if ((int16_t)(x + sizex) <= 0) {
            continue;
        }

        for (px = 0U; px < sizex; px++) {
            int16_t screen_x = (int16_t)(x + px);
            if (screen_x < 0 || screen_x >= (int16_t)LCD_W) {
                continue;
            }

            if (lcd_font_pixel((uint8_t)item->text[i], item->sizey, px, row)) {
                lcd_line_put_color((uint16_t)screen_x, item->color);
            } else if (item->mode == 0U) {
                lcd_line_put_color((uint16_t)screen_x, item->bg_color);
            }
        }
    }
}

/** @brief Rasterize one RGB565 image command on a scan line. */
static void lcd_render_image_item(const lcd_draw_item_t *item, uint16_t scan_y)
{
    uint16_t row;
    uint16_t ix;
    const uint8_t *src;

    if (item->image == 0 ||
        (int16_t)scan_y < item->y1 ||
        (int16_t)scan_y >= (int16_t)(item->y1 + item->height)) {
        return;
    }

    row = (uint16_t)((int16_t)scan_y - item->y1);
    src = item->image + ((uint32_t)row * item->width * 2U);

    for (ix = 0U; ix < item->width; ix++) {
        int16_t screen_x = (int16_t)(item->x1 + (int16_t)ix);
        uint16_t dst;

        if (screen_x < 0 || screen_x >= (int16_t)LCD_W) {
            continue;
        }

        dst = (uint16_t)screen_x * 2U;
        s_line_buf[dst] = src[ix * 2U];
        s_line_buf[dst + 1U] = src[ix * 2U + 1U];
    }
}

/** @brief Render every active drawing command into one LCD scan line. */
static void lcd_render_line(uint16_t y)
{
    uint16_t x;
    uint8_t i;

    for (x = 0U; x < LCD_W; x++) {
        lcd_line_put_color(x, s_active_bg_color);
    }

    for (i = 0U; i < s_active_count; i++) {
        const lcd_draw_item_t *item = &s_active_items[i];

        if (item->type == LCD_ITEM_FILL) {
            lcd_render_fill_item(item, y);
        } else if (item->type == LCD_ITEM_LINE) {
            lcd_render_line_item(item, y);
        } else if (item->type == LCD_ITEM_TEXT) {
            lcd_render_text_item(item, y);
        } else if (item->type == LCD_ITEM_IMAGE) {
            lcd_render_image_item(item, y);
        }
    }
}

/** @brief Encode one LCD address-window coordinate pair in big-endian order. */
static void lcd_window_bytes(uint16_t start, uint16_t end)
{
    s_addr_bytes[0] = (uint8_t)(start >> 8U);
    s_addr_bytes[1] = (uint8_t)(start & 0xFFU);
    s_addr_bytes[2] = (uint8_t)(end >> 8U);
    s_addr_bytes[3] = (uint8_t)(end & 0xFFU);
}

/** @brief Prepare the full-width column address command payload. */
static void lcd_full_window_col_bytes(void)
{
#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
    lcd_window_bytes(2U, (uint16_t)(LCD_W - 1U + 2U));
#else
    lcd_window_bytes(1U, (uint16_t)(LCD_W - 1U + 1U));
#endif
}

/** @brief Prepare the full-height row address command payload. */
static void lcd_full_window_row_bytes(void)
{
#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
    lcd_window_bytes(1U, (uint16_t)(LCD_H - 1U + 1U));
#else
    lcd_window_bytes(2U, (uint16_t)(LCD_H - 1U + 2U));
#endif
}

/** @brief Select the LCD bus mode and start one SPI DMA transaction. */
static uint8_t lcd_start_tx(const uint8_t *data, uint16_t len, uint8_t is_data)
{
    if (data == 0 || len == 0U ||
        CH32_SPI_DMA_IsIdle() == 0U || s_bus_selected != 0U) {
        return 0U;
    }

    if (is_data != 0U) {
        LCD_DC_Set();
    } else {
        LCD_DC_Clr();
    }
    LCD_CS_Clr();
    s_bus_selected = 1U;

    if (CH32_SPI_DMA_Write(data, len) == 0U) {
        LCD_CS_Set();
        LCD_DC_Set();
        s_bus_selected = 0U;
        return 0U;
    }

    return 1U;
}

/** @brief Latch pending commands and begin the asynchronous full-screen refresh. */
static void lcd_refresh_begin(void)
{
    s_active_bg_color = s_draw_bg_color;
    s_active_count = s_draw_count;
    if (s_active_count != 0U) {
        memcpy(s_active_items, s_draw_items,
               (uint16_t)s_active_count * sizeof(s_active_items[0]));
    }

    s_refresh_pending = 0U;
    s_refresh_y = 0U;
    s_refresh_state = LCD_REFRESH_CASET_CMD;
}

void LCD_AsyncInit(void)
{
    s_refresh_pending = 0U;
    s_bus_selected = 0U;
    s_refresh_state = LCD_REFRESH_IDLE;
    LCD_Fill(0U, 0U, LCD_W, LCD_H, WHITE);
}

uint8_t LCD_IsIdle(void)
{
    return (CH32_SPI_DMA_IsIdle() != 0U &&
            s_bus_selected == 0U &&
            s_refresh_state == LCD_REFRESH_IDLE &&
            s_refresh_pending == 0U) ? 1U : 0U;
}

uint8_t LCD_RequestRefresh(void)
{
    s_refresh_pending = 1U;
    return 1U;
}

void LCD_Task(void)
{
    CH32_SPI_DMA_Task();

    if (s_bus_selected != 0U) {
        if (CH32_SPI_DMA_IsIdle() == 0U) {
            return;
        }
        LCD_CS_Set();
        LCD_DC_Set();
        s_bus_selected = 0U;
    }

    if (s_refresh_state == LCD_REFRESH_IDLE) {
        if (s_refresh_pending != 0U) {
            lcd_refresh_begin();
        } else {
            return;
        }
    }

    switch (s_refresh_state) {
    case LCD_REFRESH_CASET_CMD:
        s_cmd_byte = 0x2AU;
        if (lcd_start_tx(&s_cmd_byte, 1U, 0U) != 0U) {
            s_refresh_state = LCD_REFRESH_CASET_DATA;
        }
        break;

    case LCD_REFRESH_CASET_DATA:
        lcd_full_window_col_bytes();
        if (lcd_start_tx(s_addr_bytes, sizeof(s_addr_bytes), 1U) != 0U) {
            s_refresh_state = LCD_REFRESH_RASET_CMD;
        }
        break;

    case LCD_REFRESH_RASET_CMD:
        s_cmd_byte = 0x2BU;
        if (lcd_start_tx(&s_cmd_byte, 1U, 0U) != 0U) {
            s_refresh_state = LCD_REFRESH_RASET_DATA;
        }
        break;

    case LCD_REFRESH_RASET_DATA:
        lcd_full_window_row_bytes();
        if (lcd_start_tx(s_addr_bytes, sizeof(s_addr_bytes), 1U) != 0U) {
            s_refresh_state = LCD_REFRESH_RAMWR_CMD;
        }
        break;

    case LCD_REFRESH_RAMWR_CMD:
        s_cmd_byte = 0x2CU;
        if (lcd_start_tx(&s_cmd_byte, 1U, 0U) != 0U) {
            s_refresh_state = LCD_REFRESH_LINE;
        }
        break;

    case LCD_REFRESH_LINE:
        if (s_refresh_y < LCD_H) {
            lcd_render_line(s_refresh_y);
            s_refresh_y++;
            (void)lcd_start_tx(s_line_buf, LCD_LINE_BYTES, 1U);
        } else {
            s_refresh_state = LCD_REFRESH_IDLE;
        }
        break;

    default:
        s_refresh_state = LCD_REFRESH_IDLE;
        break;
    }
}

void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color)
{
    lcd_draw_item_t *item;

    if (xsta >= xend || ysta >= yend) {
        return;
    }

    if (xsta == 0U && ysta == 0U && xend >= LCD_W && yend >= LCD_H) {
        s_draw_count = 0U;
        s_draw_bg_color = color;
        return;
    }

    item = lcd_add_item(LCD_ITEM_FILL);
    if (item == 0) {
        return;
    }

    item->x1 = (int16_t)xsta;
    item->y1 = (int16_t)ysta;
    item->x2 = (int16_t)(xend - 1U);
    item->y2 = (int16_t)(yend - 1U);
    item->color = color;
}

void LCD_DrawPoint(u16 x, u16 y, u16 color)
{
    LCD_Fill(x, y, (u16)(x + 1U), (u16)(y + 1U), color);
}

void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
    lcd_draw_item_t *item = lcd_add_item(LCD_ITEM_LINE);

    if (item == 0) {
        return;
    }

    item->x1 = (int16_t)x1;
    item->y1 = (int16_t)y1;
    item->x2 = (int16_t)x2;
    item->y2 = (int16_t)y2;
    item->color = color;
}

void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

void Draw_Circle(u16 x0, u16 y0, u8 r, u16 color)
{
    int a = 0;
    int b = r;

    while (a <= b) {
        LCD_DrawPoint((u16)(x0 - b), (u16)(y0 - a), color);
        LCD_DrawPoint((u16)(x0 + b), (u16)(y0 - a), color);
        LCD_DrawPoint((u16)(x0 - a), (u16)(y0 + b), color);
        LCD_DrawPoint((u16)(x0 - a), (u16)(y0 - b), color);
        LCD_DrawPoint((u16)(x0 + b), (u16)(y0 + a), color);
        LCD_DrawPoint((u16)(x0 + a), (u16)(y0 - b), color);
        LCD_DrawPoint((u16)(x0 + a), (u16)(y0 + b), color);
        LCD_DrawPoint((u16)(x0 - b), (u16)(y0 + a), color);
        a++;
        if ((a * a + b * b) > (r * r)) {
            b--;
        }
    }
}

void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    u8 text[2];

    text[0] = num;
    text[1] = '\0';
    LCD_ShowString(x, y, text, fc, bc, sizey, mode);
}

void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode)
{
    uint8_t sizex = (uint8_t)(sizey / 2U);

    if (p == 0 || sizex == 0U) {
        return;
    }

    while (*p != '\0') {
        lcd_draw_item_t *item = lcd_add_item(LCD_ITEM_TEXT);
        uint8_t i = 0U;

        if (item == 0) {
            return;
        }

        item->x1 = (int16_t)x;
        item->y1 = (int16_t)y;
        item->color = fc;
        item->bg_color = bc;
        item->sizey = sizey;
        item->mode = mode;

        while (p[i] != '\0' && i < LCD_TEXT_ITEM_CHARS) {
            item->text[i] = (char)p[i];
            i++;
        }
        item->text[i] = '\0';

        x = (u16)(x + (u16)i * sizex);
        p += i;
    }
}

u32 mypow(u8 m, u8 n)
{
    u32 result = 1U;

    while (n--) {
        result *= m;
    }

    return result;
}

void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey)
{
    u8 t;
    u8 temp;
    u8 enshow = 0U;
    u8 sizex = (u8)(sizey / 2U);

    for (t = 0U; t < len; t++) {
        temp = (u8)((num / mypow(10U, (u8)(len - t - 1U))) % 10U);
        if (enshow == 0U && t < (len - 1U)) {
            if (temp == 0U) {
                LCD_ShowChar((u16)(x + t * sizex), y, ' ', fc, bc, sizey, 0U);
                continue;
            }
            enshow = 1U;
        }
        LCD_ShowChar((u16)(x + t * sizex), y, (u8)(temp + '0'), fc, bc, sizey, 0U);
    }
}

void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey)
{
    u8 t;
    u8 temp;
    u8 sizex = (u8)(sizey / 2U);
    u16 num1 = (u16)(num * 100.0f);

    for (t = 0U; t < len; t++) {
        temp = (u8)((num1 / mypow(10U, (u8)(len - t - 1U))) % 10U);
        if (t == (len - 2U)) {
            LCD_ShowChar((u16)(x + (len - 2U) * sizex), y, '.', fc, bc, sizey, 0U);
            t++;
            len++;
        }
        LCD_ShowChar((u16)(x + t * sizex), y, (u8)(temp + '0'), fc, bc, sizey, 0U);
    }
}

void LCD_ShowPicture(u16 x, u16 y, u16 length, u16 width, const u8 pic[])
{
    lcd_draw_item_t *item;

    if (pic == 0 || length == 0U || width == 0U) {
        return;
    }

    item = lcd_add_item(LCD_ITEM_IMAGE);
    if (item == 0) {
        return;
    }

    item->x1 = (int16_t)x;
    item->y1 = (int16_t)y;
    item->width = length;
    item->height = width;
    item->image = pic;
}
