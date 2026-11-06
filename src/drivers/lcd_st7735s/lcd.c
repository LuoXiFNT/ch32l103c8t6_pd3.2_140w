/**
 * @file lcd.c
 * @brief Asynchronous drawing queue and scan-line rasterizer for the ST7735S panel.
 *
 * Drawing functions only queue commands into a small item list. lcd_task()
 * rasterizes one scan line into a line buffer and starts its SPI DMA transfer, so
 * a full refresh costs a few bytes of stack and never blocks the main loop.
 * Controller commands and the pixel window come from st7735s.c.
 *
 * Images can be queued as raw RGB565 (lcd_show_picture) or as a palette plus
 * packed indices (lcd_show_picture_indexed), which cuts the boot logo from 7200 to
 * 908 bytes of flash.
 *
 * @author LuoXiFNT
 * @date 2026-11-05
 * @lastModified 2026-11-05
 */

#include "lcd.h"
#include "st7735s.h"
#include "lcdfont.h"

#include <string.h>

/* Queue depth per frame. The four-panel measurement page uses 48 slots: each
 * panel is 3 fills + 4 border lines + name + protocol + 3 value strings. Keep a
 * margin here, because lcd_add_item() silently drops a command once the list is
 * full. */
#define LCD_DRAW_ITEM_MAX       64u
#define LCD_TEXT_ITEM_CHARS     20u
#define LCD_LINE_BYTES          (LCD_WIDTH * 2u)

/* Queued command types. Images carry a payload pointer: either raw RGB565 pixels
 * or a palette blob, the type selects how the rasterizer decodes them. */
#define LCD_ITEM_FILL           0u
#define LCD_ITEM_LINE           1u
#define LCD_ITEM_TEXT           2u
#define LCD_ITEM_IMAGE          3u
#define LCD_ITEM_IMAGE_IDX      4u

#define LCD_REFRESH_IDLE        0u
#define LCD_REFRESH_CASET_CMD   1u
#define LCD_REFRESH_CASET_DATA  2u
#define LCD_REFRESH_RASET_CMD   3u
#define LCD_REFRESH_RASET_DATA  4u
#define LCD_REFRESH_RAMWR_CMD   5u
#define LCD_REFRESH_LINE        6u

/** @brief One queued drawing command. */
typedef struct {
    uint8_t type;               /**< LCD_ITEM_* selector. */
    uint8_t sizey;              /**< Font height for text items. */
    uint8_t mode;               /**< 0 draws an opaque background. */
    int16_t x1;                 /**< Left edge or line start X. */
    int16_t y1;                 /**< Top edge or line start Y. */
    int16_t x2;                 /**< Right edge or line end X. */
    int16_t y2;                 /**< Bottom edge or line end Y. */
    uint16_t color;             /**< Foreground color. */
    uint16_t bg_color;          /**< Background color for text items. */
    uint16_t width;             /**< Image width in pixels. */
    uint16_t height;            /**< Image height in pixels. */
    const uint8_t *image;       /**< Raw RGB565 payload, or indexed blob. */
    char text[LCD_TEXT_ITEM_CHARS + 1u]; /**< Copied glyph line. */
} lcd_draw_item_t;

static lcd_draw_item_t s_draw_items[LCD_DRAW_ITEM_MAX];
static lcd_draw_item_t s_active_items[LCD_DRAW_ITEM_MAX];
static uint8_t s_draw_count;
static uint8_t s_active_count;
static uint16_t s_draw_bg_color = WHITE;
static uint16_t s_active_bg_color = WHITE;

/* Asynchronous SPI refresh state and scan-line buffers. */
static volatile uint8_t s_refresh_pending;
static uint8_t s_bus_selected;
static uint8_t s_refresh_state = LCD_REFRESH_IDLE;
static uint16_t s_refresh_y;
static uint8_t s_cmd_byte;
static uint8_t s_addr_bytes[4];
static uint8_t s_line_buf[LCD_LINE_BYTES];

/**
 * @brief Reserve and initialize one pending drawing command.
 * @param type LCD_ITEM_* selector.
 * @return lcd_draw_item_t* Command slot, or NULL when the list is full.
 */
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

/**
 * @brief Return the bitmap value of one glyph pixel.
 * @param ch ASCII character, printable range only.
 * @param sizey Font height in pixels.
 * @param x Pixel column inside the glyph.
 * @param y Pixel row inside the glyph.
 * @return uint8_t 1 for a lit pixel, 0 otherwise.
 */
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
    sizex = (uint8_t)(sizey / 2u);
    bytes_per_row = (uint8_t)((sizex + 7u) / 8u);
    index = (uint16_t)y * bytes_per_row + (uint16_t)(x / 8u);

    if (sizey == 12u) {
        font = ascii_1206[ch];
    } else if (sizey == 16u) {
        font = ascii_1608[ch];
    } else if (sizey == 24u) {
        font = ascii_2412[ch];
    } else if (sizey == 32u) {
        font = ascii_3216[ch];
    } else {
        return 0u;
    }

    return (font[index] & (uint8_t)(1u << (x & 7u))) ? 1u : 0u;
}

/**
 * @brief Write one RGB565 pixel to the active scan-line buffer.
 * @param x Pixel column.
 * @param color RGB565 color.
 */
static void lcd_line_put_color(uint16_t x, uint16_t color)
{
    uint16_t offset;

    if (x >= LCD_WIDTH) {
        return;
    }

    offset = (uint16_t)(x * 2u);
    s_line_buf[offset] = (uint8_t)(color >> 8u);
    s_line_buf[offset + 1u] = (uint8_t)(color & 0xFFu);
}

/**
 * @brief Fill a clipped horizontal span in the active scan-line buffer.
 * @param x1 First pixel column.
 * @param x2 Last pixel column.
 * @param color RGB565 color.
 */
static void lcd_line_fill_span(int16_t x1, int16_t x2, uint16_t color)
{
    int16_t x;

    if (x2 < 0 || x1 >= (int16_t)LCD_WIDTH) {
        return;
    }
    if (x1 < 0) {
        x1 = 0;
    }
    if (x2 >= (int16_t)LCD_WIDTH) {
        x2 = (int16_t)LCD_WIDTH - 1;
    }

    for (x = x1; x <= x2; x++) {
        lcd_line_put_color((uint16_t)x, color);
    }
}

/**
 * @brief Rasterize one filled-rectangle command on a scan line.
 * @param item Queued fill command.
 * @param y Scan line being rendered.
 */
static void lcd_render_fill_item(const lcd_draw_item_t *item, uint16_t y)
{
    if ((int16_t)y < item->y1 || (int16_t)y > item->y2) {
        return;
    }

    lcd_line_fill_span(item->x1, item->x2, item->color);
}

/**
 * @brief Rasterize one line command on a scan line.
 * @param item Queued line command.
 * @param scan_y Scan line being rendered.
 */
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

/**
 * @brief Rasterize one text command on a scan line.
 * @param item Queued text command.
 * @param scan_y Scan line being rendered.
 */
static void lcd_render_text_item(const lcd_draw_item_t *item, uint16_t scan_y)
{
    uint8_t sizex = (uint8_t)(item->sizey / 2u);
    uint8_t row;
    uint8_t i;
    uint8_t px;
    int16_t x;

    if (sizex == 0u) {
        return;
    }
    if ((int16_t)scan_y < item->y1 ||
        (int16_t)scan_y >= (int16_t)(item->y1 + item->sizey)) {
        return;
    }

    row = (uint8_t)((int16_t)scan_y - item->y1);
    for (i = 0u; item->text[i] != '\0'; i++) {
        x = (int16_t)(item->x1 + (int16_t)i * sizex);
        if (x >= (int16_t)LCD_WIDTH) {
            break;
        }
        if ((int16_t)(x + sizex) <= 0) {
            continue;
        }

        for (px = 0u; px < sizex; px++) {
            int16_t screen_x = (int16_t)(x + px);

            if (screen_x < 0 || screen_x >= (int16_t)LCD_WIDTH) {
                continue;
            }

            if (lcd_font_pixel((uint8_t)item->text[i], item->sizey, px, row)) {
                lcd_line_put_color((uint16_t)screen_x, item->color);
            } else if (item->mode == 0u) {
                lcd_line_put_color((uint16_t)screen_x, item->bg_color);
            }
        }
    }
}

/**
 * @brief Rasterize one RGB565 image command on a scan line.
 * @param item Queued image command.
 * @param scan_y Scan line being rendered.
 */
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
    src = item->image + ((uint32_t)row * item->width * 2u);

    for (ix = 0u; ix < item->width; ix++) {
        int16_t screen_x = (int16_t)(item->x1 + (int16_t)ix);
        uint16_t dst;

        if (screen_x < 0 || screen_x >= (int16_t)LCD_WIDTH) {
            continue;
        }

        dst = (uint16_t)screen_x * 2u;
        s_line_buf[dst] = src[ix * 2u];
        s_line_buf[dst + 1u] = src[ix * 2u + 1u];
    }
}

/**
 * @brief Rasterize one indexed-colour image command on a scan line.
 * @param item Queued image command; image points at the indexed blob.
 * @param scan_y Scan line being rendered.
 *
 * Blob layout: [0] bits per pixel, [1] palette entry count (0 means 2^bpp),
 * then the palette entries (2 bytes each, RGB565, high byte first), then the
 * packed indices. The palette count is stored explicitly so an image with fewer
 * colours than 2^bpp does not shift the index data.
 */
static void lcd_render_image_indexed_item(const lcd_draw_item_t *item, uint16_t scan_y)
{
    const uint8_t *blob;
    const uint8_t *data;
    uint8_t bpp;
    uint8_t entries;
    uint16_t mask;
    uint16_t row;
    uint16_t ix;

    if (item->image == 0 ||
        (int16_t)scan_y < item->y1 ||
        (int16_t)scan_y >= (int16_t)(item->y1 + item->height)) {
        return;
    }

    blob = item->image;
    bpp = blob[0];
    if ((bpp != 1u) && (bpp != 2u) && (bpp != 4u) && (bpp != 8u)) {
        return;
    }

    mask = (uint16_t)((1u << bpp) - 1u);
    entries = blob[1];
    if ((entries == 0u) || (entries > (uint8_t)(1u << bpp))) {
        entries = (uint8_t)(1u << bpp);
    }
    data = blob + 2u + (uint16_t)entries * 2u;
    row = (uint16_t)((int16_t)scan_y - item->y1);

    for (ix = 0u; ix < item->width; ix++) {
        uint32_t bit_index = ((uint32_t)row * item->width + ix) * bpp;
        uint8_t raw = data[bit_index >> 3];
        uint8_t shift = (uint8_t)(8u - bpp - (uint8_t)(bit_index & 7u));
        uint16_t entry = (uint16_t)((raw >> shift) & mask);
        uint16_t color;
        int16_t screen_x = (int16_t)(item->x1 + (int16_t)ix);

        if (entry >= (uint16_t)entries) {
            entry = 0u;
        }
        if (screen_x < 0 || screen_x >= (int16_t)LCD_WIDTH) {
            continue;
        }

        color = (uint16_t)(((uint16_t)blob[2u + entry * 2u] << 8) |
                           blob[2u + entry * 2u + 1u]);
        lcd_line_put_color((uint16_t)screen_x, color);
    }
}

/**
 * @brief Render every active drawing command into one LCD scan line.
 * @param y Scan line to rasterize.
 */
static void lcd_render_line(uint16_t y)
{
    uint16_t x;
    uint8_t i;

    for (x = 0u; x < LCD_WIDTH; x++) {
        lcd_line_put_color(x, s_active_bg_color);
    }

    for (i = 0u; i < s_active_count; i++) {
        const lcd_draw_item_t *item = &s_active_items[i];

        if (item->type == LCD_ITEM_FILL) {
            lcd_render_fill_item(item, y);
        } else if (item->type == LCD_ITEM_LINE) {
            lcd_render_line_item(item, y);
        } else if (item->type == LCD_ITEM_TEXT) {
            lcd_render_text_item(item, y);
        } else if (item->type == LCD_ITEM_IMAGE) {
            lcd_render_image_item(item, y);
        } else if (item->type == LCD_ITEM_IMAGE_IDX) {
            lcd_render_image_indexed_item(item, y);
        }
    }
}

/**
 * @brief Encode one LCD address-window coordinate pair in big-endian order.
 * @param start First coordinate of the window.
 * @param end Last coordinate of the window.
 */
static void lcd_window_bytes(uint16_t start, uint16_t end)
{
    s_addr_bytes[0] = (uint8_t)(start >> 8u);
    s_addr_bytes[1] = (uint8_t)(start & 0xFFu);
    s_addr_bytes[2] = (uint8_t)(end >> 8u);
    s_addr_bytes[3] = (uint8_t)(end & 0xFFu);
}

/** @brief Prepare the full-width column address payload of the active window. */
static void lcd_full_window_col_bytes(void)
{
    uint16_t first = st7735s_column_offset();

    lcd_window_bytes(first, (uint16_t)(first + LCD_WIDTH - 1u));
}

/** @brief Prepare the full-height row address payload of the active window. */
static void lcd_full_window_row_bytes(void)
{
    uint16_t first = st7735s_row_offset();

    lcd_window_bytes(first, (uint16_t)(first + LCD_HEIGHT - 1u));
}

/**
 * @brief Hand one transfer to the controller through the platform bus.
 * @param data Payload to transmit.
 * @param len Number of bytes.
 * @param is_data Non-zero for data (D/CX high), zero for a single command byte.
 * @return true when the transfer was accepted.
 */
static uint8_t lcd_start_tx(const uint8_t *data, uint16_t len, uint8_t is_data)
{
    if ((data == 0) || (len == 0u) || (s_bus_selected != 0u)) {
        return 0u;
    }

    if (is_data == 0u) {
        /* Command bytes are framed synchronously by the port, so nothing stays on
         * the bus and the caller may continue on the next pass. */
        st7735s_write_command(data[0]);
        return 1u;
    }

    if (!st7735s_bus_idle()) {
        return 0u;
    }
    if (!st7735s_start_pixels(data, len)) {
        return 0u;
    }

    s_bus_selected = 1u;
    return 1u;
}

/** @brief Latch pending commands and begin the asynchronous full-screen refresh. */
static void lcd_refresh_begin(void)
{
    s_active_bg_color = s_draw_bg_color;
    s_active_count = s_draw_count;
    if (s_active_count != 0u) {
        memcpy(s_active_items, s_draw_items,
               (uint16_t)s_active_count * sizeof(s_active_items[0]));
    }

    s_refresh_pending = 0u;
    s_refresh_y = 0u;
    s_refresh_state = LCD_REFRESH_CASET_CMD;
}

void lcd_init(void)
{
    s_refresh_pending = 0u;
    s_bus_selected = 0u;
    s_refresh_state = LCD_REFRESH_IDLE;
    lcd_fill(0u, 0u, LCD_WIDTH, LCD_HEIGHT, WHITE);
}

bool lcd_is_idle(void)
{
    return (st7735s_bus_idle() &&
            s_bus_selected == 0u &&
            s_refresh_state == LCD_REFRESH_IDLE &&
            s_refresh_pending == 0u) ? 1u : 0u;
}

bool lcd_request_refresh(void)
{
    s_refresh_pending = 1u;
    return true;
}

void lcd_task(void)
{
    st7735s_bus_task();

    if (s_bus_selected != 0u) {
        /* The port releases CS once the transfer has drained, so there is nothing
         * to do here but wait for it. */
        if (!st7735s_bus_idle()) {
            return;
        }
        s_bus_selected = 0u;
    }

    if (s_refresh_state == LCD_REFRESH_IDLE) {
        if (s_refresh_pending != 0u) {
            lcd_refresh_begin();
        } else {
            return;
        }
    }

    switch (s_refresh_state) {
    case LCD_REFRESH_CASET_CMD:
        s_cmd_byte = ST7735S_CMD_CASET;
        if (lcd_start_tx(&s_cmd_byte, 1u, 0u) != 0u) {
            s_refresh_state = LCD_REFRESH_CASET_DATA;
        }
        break;

    case LCD_REFRESH_CASET_DATA:
        lcd_full_window_col_bytes();
        if (lcd_start_tx(s_addr_bytes, sizeof(s_addr_bytes), 1u) != 0u) {
            s_refresh_state = LCD_REFRESH_RASET_CMD;
        }
        break;

    case LCD_REFRESH_RASET_CMD:
        s_cmd_byte = ST7735S_CMD_RASET;
        if (lcd_start_tx(&s_cmd_byte, 1u, 0u) != 0u) {
            s_refresh_state = LCD_REFRESH_RASET_DATA;
        }
        break;

    case LCD_REFRESH_RASET_DATA:
        lcd_full_window_row_bytes();
        if (lcd_start_tx(s_addr_bytes, sizeof(s_addr_bytes), 1u) != 0u) {
            s_refresh_state = LCD_REFRESH_RAMWR_CMD;
        }
        break;

    case LCD_REFRESH_RAMWR_CMD:
        s_cmd_byte = ST7735S_CMD_RAMWR;
        if (lcd_start_tx(&s_cmd_byte, 1u, 0u) != 0u) {
            s_refresh_state = LCD_REFRESH_LINE;
        }
        break;

    case LCD_REFRESH_LINE:
        if (s_refresh_y < LCD_HEIGHT) {
            lcd_render_line(s_refresh_y);
            /* Advance only after the line is really on the bus: a failed start
             * would otherwise skip a scan line and shift the rest of the frame. */
            if (lcd_start_tx(s_line_buf, LCD_LINE_BYTES, 1u) != 0u) {
                s_refresh_y++;
            }
        } else {
            s_refresh_state = LCD_REFRESH_IDLE;
        }
        break;

    default:
        s_refresh_state = LCD_REFRESH_IDLE;
        break;
    }
}

void lcd_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    lcd_draw_item_t *item;

    if (xsta >= xend || ysta >= yend) {
        return;
    }

    if (xsta == 0u && ysta == 0u && xend >= LCD_WIDTH && yend >= LCD_HEIGHT) {
        s_draw_count = 0u;
        s_draw_bg_color = color;
        return;
    }

    item = lcd_add_item(LCD_ITEM_FILL);
    if (item == 0) {
        return;
    }

    item->x1 = (int16_t)xsta;
    item->y1 = (int16_t)ysta;
    item->x2 = (int16_t)(xend - 1u);
    item->y2 = (int16_t)(yend - 1u);
    item->color = color;
}

void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    lcd_fill(x, y, (uint16_t)(x + 1u), (uint16_t)(y + 1u), color);
}

void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
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

void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    lcd_draw_line(x1, y1, x2, y1, color);
    lcd_draw_line(x1, y1, x1, y2, color);
    lcd_draw_line(x1, y2, x2, y2, color);
    lcd_draw_line(x2, y1, x2, y2, color);
}

void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a = 0;
    int b = r;

    while (a <= b) {
        lcd_draw_pixel((uint16_t)(x0 - b), (uint16_t)(y0 - a), color);
        lcd_draw_pixel((uint16_t)(x0 + b), (uint16_t)(y0 - a), color);
        lcd_draw_pixel((uint16_t)(x0 - a), (uint16_t)(y0 + b), color);
        lcd_draw_pixel((uint16_t)(x0 - a), (uint16_t)(y0 - b), color);
        lcd_draw_pixel((uint16_t)(x0 + b), (uint16_t)(y0 + a), color);
        lcd_draw_pixel((uint16_t)(x0 + a), (uint16_t)(y0 - b), color);
        lcd_draw_pixel((uint16_t)(x0 + a), (uint16_t)(y0 + b), color);
        lcd_draw_pixel((uint16_t)(x0 - b), (uint16_t)(y0 + a), color);
        a++;
        if ((a * a + b * b) > (r * r)) {
            b--;
        }
    }
}

void lcd_show_char(uint16_t x, uint16_t y, uint8_t num,
                   uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t text[2];

    text[0] = num;
    text[1] = '\0';
    lcd_show_string(x, y, text, fc, bc, sizey, mode);
}

void lcd_show_string(uint16_t x, uint16_t y, const uint8_t *p,
                     uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t sizex = (uint8_t)(sizey / 2u);

    if (p == 0 || sizex == 0u) {
        return;
    }

    while (*p != '\0') {
        lcd_draw_item_t *item = lcd_add_item(LCD_ITEM_TEXT);
        uint8_t i = 0u;

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

        x = (uint16_t)(x + (uint16_t)i * sizex);
        p += i;
    }
}

uint32_t lcd_pow10(uint8_t m, uint8_t n)
{
    uint32_t result = 1u;

    while (n != 0u) {
        result *= m;
        n--;
    }

    return result;
}

void lcd_show_uint(uint16_t x, uint16_t y, uint16_t num, uint8_t len,
                   uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t t;
    uint8_t temp;
    uint8_t enshow = 0u;
    uint8_t sizex = (uint8_t)(sizey / 2u);

    for (t = 0u; t < len; t++) {
        temp = (uint8_t)((num / lcd_pow10(10u, (uint8_t)(len - t - 1u))) % 10u);
        if (enshow == 0u && t < (uint8_t)(len - 1u)) {
            if (temp == 0u) {
                lcd_show_char((uint16_t)(x + (uint16_t)t * sizex), y, ' ',
                              fc, bc, sizey, 0u);
                continue;
            }
            enshow = 1u;
        }
        lcd_show_char((uint16_t)(x + (uint16_t)t * sizex), y, (uint8_t)(temp + '0'),
                      fc, bc, sizey, 0u);
    }
}

void lcd_show_fixed1(uint16_t x, uint16_t y, float num, uint8_t len,
                     uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t t;
    uint8_t digit;
    uint8_t sizex = (uint8_t)(sizey / 2u);
    uint8_t dot = (uint8_t)(len - 2u);
    uint16_t scaled = (uint16_t)(num * 100.0f);

    if (sizex == 0u) {
        return;
    }

    /* The reference driver advanced len/t inside the loop, so the digits after
     * the decimal point were scaled by the wrong power of ten. The decimal point
     * now occupies its own column and does not consume a digit slot. */
    for (t = 0u; t < len; t++) {
        if (t == dot) {
            lcd_show_char((uint16_t)(x + (uint16_t)t * sizex), y, '.', fc, bc, sizey, 0u);
            continue;
        }

        if (t < dot) {
            digit = (uint8_t)((scaled / lcd_pow10(10u, (uint8_t)(dot - t))) % 10u);
        } else {
            digit = (uint8_t)((scaled / lcd_pow10(10u, (uint8_t)((len - 1u) - t))) % 10u);
        }
        lcd_show_char((uint16_t)(x + (uint16_t)t * sizex), y, (uint8_t)(digit + '0'),
                      fc, bc, sizey, 0u);
    }
}

void lcd_show_picture(uint16_t x, uint16_t y, uint16_t pixel_w, uint16_t pixel_h,
                      const uint8_t pic[])
{
    lcd_draw_item_t *item;

    if (pic == 0 || pixel_w == 0u || pixel_h == 0u) {
        return;
    }

    item = lcd_add_item(LCD_ITEM_IMAGE);
    if (item == 0) {
        return;
    }

    item->x1 = (int16_t)x;
    item->y1 = (int16_t)y;
    item->width = pixel_w;
    item->height = pixel_h;
    item->image = pic;
}

void lcd_show_picture_indexed(uint16_t x, uint16_t y, uint16_t pixel_w, uint16_t pixel_h,
                              const uint8_t *blob)
{
    lcd_draw_item_t *item;

    if (blob == 0 || pixel_w == 0u || pixel_h == 0u) {
        return;
    }

    item = lcd_add_item(LCD_ITEM_IMAGE_IDX);
    if (item == 0) {
        return;
    }

    item->x1 = (int16_t)x;
    item->y1 = (int16_t)y;
    item->width = pixel_w;
    item->height = pixel_h;
    item->image = blob;
}
