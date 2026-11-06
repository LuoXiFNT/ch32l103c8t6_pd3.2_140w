/**
 * @file lcd.h
 * @brief Panel geometry, RGB565 palette, and the asynchronous drawing interface.
 *
 * This is the graphics half of the display stack: drawing calls only queue
 * commands, and lcd_task() rasterizes one scan line at a time and hands it to the
 * SPI DMA engine, so a refresh never blocks the main loop. The controller itself
 * (commands, init sequence, window programming) lives in st7735s.h.
 *
 * @author LuoXiFNT
 * @date 2026-11-05
 * @lastModified 2026-11-05
 */

#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Panel orientation chosen by the display layer: four write directions.
 *
 * 0 and 1 select the 128x160 portrait directions, 2 and 3 the 160x128 landscape
 * ones. The value is passed to st7735s_init() and also decides LCD_WIDTH and
 * LCD_HEIGHT below.
 *
 * Must stay a plain number: the geometry below is selected with #if, and the
 * preprocessor cannot resolve the st7735s_rotation_t names (an enum name silently
 * becomes zero there). The static assert at the end of this block catches that. */
#define LCD_PANEL_ROTATION             3

/** @brief Visible panel width in pixels, derived from the write direction. */
#if (LCD_PANEL_ROTATION == 0) || (LCD_PANEL_ROTATION == 1)
#define LCD_WIDTH               (128u)
#define LCD_HEIGHT              (160u)
#else
#define LCD_WIDTH               (160u)
#define LCD_HEIGHT              (128u)
#endif

/* Guard against a rotation/geometry mismatch. LCD_WIDTH comes from the
 * preprocessor, which turns an enum name into zero without any diagnostic, while
 * the same expression is evaluated correctly in C. Comparing both forms makes a
 * non-numeric LCD_PANEL_ROTATION fail the build instead of garbling the panel. */
#define LCD_EXPECTED_WIDTH  (((LCD_PANEL_ROTATION) <= 1) ? 128u : 160u)
typedef char lcd_geometry_matches_rotation[(LCD_WIDTH == LCD_EXPECTED_WIDTH) ? 1 : -1];

/** @name Task interface */
/** @{ */
/** @brief Reset the drawing queue and clear the panel background. */
void lcd_init(void);
/** @brief Advance the rasterizer and the SPI transfer state machine. */
void lcd_task(void);
/** @brief Return non-zero when the engine is idle. */
bool lcd_is_idle(void);
/** @brief Request transmission of the pending drawing list. */
bool lcd_request_refresh(void);
/** @} */

/** @name Primitive drawing, queued */
/** @{ */
/** @brief Queue a filled rectangle; a full-screen fill also sets the background. */
void lcd_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
/** @brief Queue one pixel. */
void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
/** @brief Queue one line segment. */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
/** @brief Queue one rectangle outline. */
void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
/** @brief Queue one circle outline. */
void lcd_draw_circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
/** @} */

/** @name Text, queued */
/** @{ */
/** @brief Queue one character; mode 0 draws an opaque background. */
void lcd_show_char(uint16_t x, uint16_t y, uint8_t num,
                   uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
/** @brief Queue a zero-terminated string; mode 0 draws an opaque background. */
void lcd_show_string(uint16_t x, uint16_t y, const uint8_t *p,
                     uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
/** @brief Integer power helper used by the numeric formatters. */
uint32_t lcd_pow10(uint8_t m, uint8_t n);
/** @brief Queue an unsigned decimal number of len digits, blanking leading zeros. */
void lcd_show_uint(uint16_t x, uint16_t y, uint16_t num, uint8_t len,
                   uint16_t fc, uint16_t bc, uint8_t sizey);
/** @brief Queue a fixed-point number with one decimal digit. */
void lcd_show_fixed1(uint16_t x, uint16_t y, float num, uint8_t len,
                     uint16_t fc, uint16_t bc, uint8_t sizey);
/** @} */

/** @name Bitmap, queued */
/** @{ */
/** @brief Queue an RGB565 image; pixel_w pixels per row, pixel_h rows. */
void lcd_show_picture(uint16_t x, uint16_t y, uint16_t pixel_w, uint16_t pixel_h,
                      const uint8_t pic[]);

/**
 * @brief Queue an indexed-colour image to save flash.
 *
 * Blob layout: [0] bits per pixel (1, 2, 4 or 8), [1] palette entry count (0 means
 * 2^bpp), then that many palette entries of 2 bytes (RGB565, high byte first),
 * then the packed indices row by row, most significant bits first.
 *
 * @param x Left edge.
 * @param y Top edge.
 * @param pixel_w Image width in pixels.
 * @param pixel_h Image height in pixels.
 * @param blob Indexed image blob described above.
 */
void lcd_show_picture_indexed(uint16_t x, uint16_t y, uint16_t pixel_w, uint16_t pixel_h,
                              const uint8_t *blob);
/** @} */

/* RGB565 palette used by the display layout. */
#define WHITE         0xFFFFu
#define BLACK         0x0000u
#define BLUE          0x001Fu
#define BRED          0xF81Fu
#define GRED          0xFFE0u
#define GBLUE         0x07FFu
#define RED           0xF800u
#define MAGENTA       0xF81Fu
#define GREEN         0x07E0u
#define CYAN          0x7FFFu
#define YELLOW        0xFFE0u
#define BROWN         0xBC40u
#define BRRED         0xFC07u
#define GRAY          0x8430u
#define DARKBLUE      0x01CFu
#define LIGHTBLUE     0x7D7Cu
#define GRAYBLUE      0x5458u
#define LIGHTGREEN    0x841Fu
#define LGRAY         0xC618u
#define LGRAYBLUE     0xA651u
#define LBBLUE        0x2B12u

#ifdef __cplusplus
}
#endif

#endif /* LCD_H */
