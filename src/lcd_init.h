/**
 * @file lcd_init.h
 * @brief LCD panel dimensions, pin macros, and controller primitives.
 * @author 中景园电子
 * @date 2018-10-31
 * @modder LuoXiFNT
 * @lastModified 2026-08-23
 */

#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "ch32l103_init.h"
#include "debug.h"

#define USE_HORIZONTAL 3  // 0或1为竖屏 2或3为横屏

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 128
#define LCD_H 160
#else
#define LCD_W 160
#define LCD_H 128
#endif

//-----------------LCD端口定义----------------
#define LCD_RES_Clr()  GPIO_ResetBits(LCD_RES_GPIO_PORT, LCD_RES_GPIO_PIN)
#define LCD_RES_Set()  GPIO_SetBits(LCD_RES_GPIO_PORT, LCD_RES_GPIO_PIN)

#define LCD_DC_Clr()   GPIO_ResetBits(LCD_DC_GPIO_PORT, LCD_DC_GPIO_PIN)
#define LCD_DC_Set()   GPIO_SetBits(LCD_DC_GPIO_PORT, LCD_DC_GPIO_PIN)

#define LCD_CS_Clr()   GPIO_ResetBits(LCD_CS_GPIO_PORT, LCD_CS_GPIO_PIN)
#define LCD_CS_Set()   GPIO_SetBits(LCD_CS_GPIO_PORT, LCD_CS_GPIO_PIN)

#define LCD_BLK_Clr()  GPIO_ResetBits(LCD_BLK_GPIO_PORT, LCD_BLK_GPIO_PIN)
#define LCD_BLK_Set()  GPIO_SetBits(LCD_BLK_GPIO_PORT, LCD_BLK_GPIO_PIN)

/** @brief Write one byte over the LCD SPI bus. */
void LCD_Writ_Bus(u8 dat);
/** @brief Write one 8-bit data value to the controller. */
void LCD_WR_DATA8(u8 dat);
/** @brief Write one 16-bit RGB565 data value to the controller. */
void LCD_WR_DATA(u16 dat);
/** @brief Write one controller command byte. */
void LCD_WR_REG(u8 dat);
/** @brief Set the controller drawing address window. */
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2);
/** @brief Initialize the LCD controller and panel orientation. */
void LCD_Init(void);
/** @brief Enable LCD panel output after initialization. */
void LCD_DisplayOn(void);

#endif
