/**
 * @file lcd.h
 * @brief LCD drawing, text, image, and asynchronous refresh interfaces.
 * @author 中景园电子
 * @date 2018-10-31
 * @modder LuoXiFNT
 * @lastModified 2026-08-23
 */

#ifndef __LCD_H
#define __LCD_H

#include "debug.h"

/** @brief Queue a filled rectangle in the pending LCD drawing list. */
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color);
/** @brief Initialize the asynchronous LCD refresh state. */
void LCD_AsyncInit(void);
/** @brief Advance the asynchronous LCD renderer and SPI transfer state. */
void LCD_Task(void);
/** @brief Return non-zero when the LCD refresh engine is idle. */
u8 LCD_IsIdle(void);
/** @brief Request transmission of the pending drawing list. */
u8 LCD_RequestRefresh(void);

/** @brief Queue one pixel in the pending LCD drawing list. */
void LCD_DrawPoint(u16 x, u16 y, u16 color);
/** @brief Queue one line segment in the pending LCD drawing list. */
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
/** @brief Queue one rectangle outline in the pending LCD drawing list. */
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
/** @brief Queue one circle outline in the pending LCD drawing list. */
void Draw_Circle(u16 x0, u16 y0, u8 r, u16 color);

/** @brief Queue one character glyph for rendering. */
void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode);
/** @brief Queue a zero-terminated string for rendering. */
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode);
/** @brief Calculate an unsigned integer power for legacy text formatting. */
u32 mypow(u8 m, u8 n);
/** @brief Queue a decimal integer for rendering. */
void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey);
/** @brief Queue a one-decimal floating-point value for rendering. */
void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey);

/** @brief Queue an RGB565 image for rendering. */
void LCD_ShowPicture(u16 x, u16 y, u16 length, u16 width, const u8 pic[]);

// 画笔颜色
#define WHITE         0xFFFF
#define BLACK         0x0000
#define BLUE          0x001F
#define BRED          0XF81F
#define GRED          0XFFE0
#define GBLUE         0X07FF
#define RED           0xF800
#define MAGENTA       0xF81F
#define GREEN         0x07E0
#define CYAN          0x7FFF
#define YELLOW        0xFFE0
#define BROWN         0XBC40
#define BRRED         0XFC07
#define GRAY          0X8430
#define DARKBLUE      0X01CF
#define LIGHTBLUE     0X7D7C
#define GRAYBLUE      0X5458
#define LIGHTGREEN    0X841F
#define LGRAY         0XC618
#define LGRAYBLUE     0XA651
#define LBBLUE        0X2B12

#endif
