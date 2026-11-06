/**
 * @file st7735s.h
 * @brief ST7735S display controller: command set, register fields, and byte access.
 *
 * Panel module: Zhong Jing Yuan ZJY180S0800TG01, 128RGBx160 dots, ST7735S driver,
 * 4-wire SPI, 2.8-3.3 V logic, 12 o'clock viewing direction. The module spec
 * (section 10.3) defers the whole command set to the ST7735S technical manual, so
 * the codes and field masks below follow that manual (V1.1, section 10
 * "COMMAND").
 *
 * The only thing the module adds to the bare controller is a low-side backlight
 * switch: BLK drives an S8050 NPN base through 1 kOhm with a 10 kOhm pull-down to
 * GND, so the LEDs stay dark until the MCU drives BLK high; the collector sinks
 * the LED string through 10 Ohm. The data lines go straight to the controller, so
 * the ST7735S sequence is what initializes the panel; the parameter values below
 * are the ones validated on this board, not a generic datasheet default table.
 *
 * This layer only knows about the controller: it resets it, runs the power-on
 * sequence, selects the write direction, and programs the pixel window. Drawing
 * lives in lcd.h. Everything that touches the MCU goes through st7735s_bus_t, which
 * the platform layer supplies to st7735s_init(); that port file is the only place to
 * touch when the platform changes.
 *
 * @author LuoXiFNT
 * @date 2026-11-05
 * @lastModified 2026-11-05
 */

#ifndef ST7735S_H
#define ST7735S_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @name System function commands (datasheet 10.1) */
/** @{ */
#define ST7735S_CMD_NOP               (0x00u) /**< No operation. */
#define ST7735S_CMD_SWRESET           (0x01u) /**< Software reset. */
#define ST7735S_CMD_RDDID             (0x04u) /**< Read display ID. */
#define ST7735S_CMD_RDDST             (0x09u) /**< Read display status. */
#define ST7735S_CMD_RDDPM             (0x0Au) /**< Read display power mode. */
#define ST7735S_CMD_RDDMADCTL         (0x0Bu) /**< Read display MADCTL. */
#define ST7735S_CMD_RDDCOLMOD         (0x0Cu) /**< Read display pixel format. */
#define ST7735S_CMD_RDDIM             (0x0Du) /**< Read display image mode. */
#define ST7735S_CMD_RDDSM             (0x0Eu) /**< Read display signal mode. */
#define ST7735S_CMD_RDDSDR            (0x0Fu) /**< Read display self-diagnostic result. */
#define ST7735S_CMD_SLPIN             (0x10u) /**< Sleep in. */
#define ST7735S_CMD_SLPOUT            (0x11u) /**< Sleep out. */
#define ST7735S_CMD_PTLON             (0x12u) /**< Partial display mode on. */
#define ST7735S_CMD_NORON             (0x13u) /**< Normal display mode on. */
#define ST7735S_CMD_INVOFF            (0x20u) /**< Display inversion off. */
#define ST7735S_CMD_INVON             (0x21u) /**< Display inversion on. */
#define ST7735S_CMD_GAMSET            (0x26u) /**< Gamma set. */
#define ST7735S_CMD_DISPOFF           (0x28u) /**< Display off. */
#define ST7735S_CMD_DISPON            (0x29u) /**< Display on. */
#define ST7735S_CMD_CASET             (0x2Au) /**< Column address set. */
#define ST7735S_CMD_RASET             (0x2Bu) /**< Row address set. */
#define ST7735S_CMD_RAMWR             (0x2Cu) /**< Memory write. */
#define ST7735S_CMD_RGBSET            (0x2Du) /**< Color setting for 4K/65K/262K. */
#define ST7735S_CMD_RAMRD             (0x2Eu) /**< Memory read. */
#define ST7735S_CMD_PTLAR             (0x30u) /**< Partial area. */
#define ST7735S_CMD_SCRLAR            (0x33u) /**< Scroll area set. */
#define ST7735S_CMD_TEOFF             (0x34u) /**< Tearing effect line off. */
#define ST7735S_CMD_TEON              (0x35u) /**< Tearing effect line on. */
#define ST7735S_CMD_MADCTL            (0x36u) /**< Memory data access control. */
#define ST7735S_CMD_VSCSAD            (0x37u) /**< Vertical scroll start address. */
#define ST7735S_CMD_IDMOFF            (0x38u) /**< Idle mode off. */
#define ST7735S_CMD_IDMON             (0x39u) /**< Idle mode on. */
#define ST7735S_CMD_COLMOD            (0x3Au) /**< Interface pixel format. */
#define ST7735S_CMD_RDID1             (0xDAu) /**< Read ID1. */
#define ST7735S_CMD_RDID2             (0xDBu) /**< Read ID2. */
#define ST7735S_CMD_RDID3             (0xDCu) /**< Read ID3. */
/** @} */

/** @name Panel and power commands (datasheet 10.2) */
/** @{ */
#define ST7735S_CMD_FRMCTR1           (0xB1u) /**< Frame rate, normal mode / full colors. */
#define ST7735S_CMD_FRMCTR2           (0xB2u) /**< Frame rate, idle mode / 8 colors. */
#define ST7735S_CMD_FRMCTR3           (0xB3u) /**< Frame rate, partial mode / full colors. */
#define ST7735S_CMD_INVCTR            (0xB4u) /**< Display inversion control. */
#define ST7735S_CMD_PWCTR1            (0xC0u) /**< Power control 1. */
#define ST7735S_CMD_PWCTR2            (0xC1u) /**< Power control 2. */
#define ST7735S_CMD_PWCTR3            (0xC2u) /**< Power control 3, normal mode. */
#define ST7735S_CMD_PWCTR4            (0xC3u) /**< Power control 4, idle mode. */
#define ST7735S_CMD_PWCTR5            (0xC4u) /**< Power control 5, partial mode. */
#define ST7735S_CMD_VMCTR1            (0xC5u) /**< VCOM control 1. */
#define ST7735S_CMD_VMOFCTR           (0xC7u) /**< VCOM offset control. */
#define ST7735S_CMD_WRID2             (0xD1u) /**< Write ID2 value. */
#define ST7735S_CMD_WRID3             (0xD2u) /**< Write ID3 value. */
#define ST7735S_CMD_NVFCTR1           (0xD9u) /**< NVM control status. */
#define ST7735S_CMD_GMCTRP1           (0xE0u) /**< Gamma correction, positive polarity. */
#define ST7735S_CMD_GMCTRN1           (0xE1u) /**< Gamma correction, negative polarity. */
#define ST7735S_CMD_GCV               (0xFCu) /**< Gate pump clock frequency. */
/** @} */

/** @name MADCTL (36h) field masks (datasheet 10.1.29) */
/** @{ */
#define ST7735S_MADCTL_MY             (0x80u) /**< Row address order. */
#define ST7735S_MADCTL_MX             (0x40u) /**< Column address order. */
#define ST7735S_MADCTL_MV             (0x20u) /**< Row/column exchange. */
#define ST7735S_MADCTL_ML             (0x10u) /**< Vertical refresh order. */
#define ST7735S_MADCTL_RGB            (0x08u) /**< RGB order (1 = BGR panel wiring). */
#define ST7735S_MADCTL_MH             (0x04u) /**< Horizontal refresh order. */
/** @} */

/** @name COLMOD (3Ah) interface pixel formats (datasheet 10.1.33) */
/** @{ */
#define ST7735S_COLMOD_12BIT          (0x03u) /**< 12-bit, 4K colors. */
#define ST7735S_COLMOD_16BIT          (0x05u) /**< 16-bit, 65K colors (RGB565). */
#define ST7735S_COLMOD_18BIT          (0x06u) /**< 18-bit, 262K colors. */
/** @} */

/** @brief Panel write direction, matching the MADCTL / address-offset pairs.
 *
 * The display layer owns the choice (LCD_PANEL_ROTATION in lcd.h) and passes it to
 * st7735s_init(); 0 and 1 are the 128x160 portrait directions, 2 and 3 the
 * 160x128 landscape ones. */
typedef enum {
    ST7735S_ROTATION_0 = 0, /**< Portrait 128x160, no mirroring. */
    ST7735S_ROTATION_1 = 1, /**< Portrait 128x160, mirrored. */
    ST7735S_ROTATION_2 = 2, /**< Landscape 160x128, mirrored. */
    ST7735S_ROTATION_3 = 3  /**< Landscape 160x128, used by this product. */
} st7735s_rotation_t;

/**
 * @brief Host interface supplied by the platform layer.
 *
 * The driver owns no pins and no timers: chip select, the D/CX level, the reset and
 * backlight lines, the SPI byte stream and the blocking delays all arrive here.
 * Implementing this struct is the whole porting job; a platform without DMA can
 * implement start_pixels() as a blocking write and return 0 from is_idle().
 *
 * Chip select is part of write_cmd(), write_data() and start_pixels(): the platform
 * frames each transaction, the driver never toggles CS itself.
 */
typedef struct {
    /** @brief Send one command byte with D/CX low. */
    void (*write_cmd)(void *user, uint8_t cmd);
    /** @brief Send data bytes with D/CX high; blocking, used during init. */
    void (*write_data)(void *user, const uint8_t *data, uint16_t len);
    /** @brief Start a pixel stream with D/CX high; return true when started. */
    bool (*start_pixels)(void *user, const uint8_t *data, uint16_t len);
    /** @brief Return non-zero when the bus is ready for a new transfer. */
    bool (*is_idle)(void *user);
    /** @brief Advance the transfer and release the bus when it has drained. */
    void (*task)(void *user);
    /** @brief Drive the panel reset line, 1 = released. */
    void (*set_reset)(void *user, uint8_t level);
    /** @brief Drive the backlight switch, 1 = LEDs on. */
    void (*set_backlight)(void *user, uint8_t level);
    /** @brief Block for the given number of milliseconds. */
    void (*delay_ms)(void *user, uint16_t ms);
    void *user; /**< Opaque pointer passed back to every callback. */
} st7735s_bus_t;

/**
 * @brief Write one command byte (D/CX low) with a chip-select pulse.
 * @param cmd Command code from the ST7735S command set.
 */
void st7735s_write_command(uint8_t cmd);

/**
 * @brief Write one data byte (D/CX high).
 * @param data Byte to send.
 */
void st7735s_write_data8(uint8_t data);

/**
 * @brief Write one 16-bit RGB565 value as two data bytes.
 * @param data Pixel value, sent most significant byte first.
 */
void st7735s_write_data16(uint16_t data);

/**
 * @name Power-on blocking delays
 *
 * Single source of truth for how long the panel keeps the MCU busy. The main
 * board spends exactly ST7735S_INIT_BLOCKING_MS inside st7735s_init(), and the sub
 * board waits the same amount before enabling I2C2 so both boards reach their main
 * loop at the same time. Change a value here and both sides follow.
 * @{
 */
#define ST7735S_RESET_LOW_MS        (100u) /**< RES held low. */
#define ST7735S_RESET_SETTLE_MS     (100u) /**< Wait after RES is released. */
#define ST7735S_SLEEP_OUT_MS        (120u) /**< Wait after SLPOUT. */
#define ST7735S_SLEEP_IN_MS         (120u) /**< Wait after SLPIN. */
#define ST7735S_INIT_BLOCKING_MS    (ST7735S_RESET_LOW_MS + \
                                     ST7735S_RESET_SETTLE_MS + \
                                     ST7735S_SLEEP_OUT_MS)
/** @} */

/** @brief Pulse the reset pin and wait for the controller to boot. */
void st7735s_reset(void);

/**
 * @brief Run the power-on sequence from the datasheet and apply the rotation.
 *
 * Uses the frame-rate, power-control and gamma values of this panel module. The
 * write direction is a display-layer decision, so the caller passes it in, and the
 * bus comes from the platform layer.
 *
 * @param bus Platform host interface; must stay valid for the lifetime of the driver.
 * @param rotation Write direction written to MADCTL, also selecting the
 *                 visible-area offsets.
 */
void st7735s_init(const st7735s_bus_t *bus, st7735s_rotation_t rotation);

/**
 * @brief Start one pixel-stream transfer on the bus.
 * @param data Line buffer to send; the platform must hold it until is_idle() is true.
 * @param len Number of bytes to transfer.
 * @return true when the transfer started.
 */
bool st7735s_start_pixels(const uint8_t *data, uint16_t len);

/** @brief Return non-zero when the bus has finished the current transfer. */
bool st7735s_bus_idle(void);

/** @brief Advance the bus transfer; call it every pass while a transfer is active. */
void st7735s_bus_task(void);

/** @brief Enter sleep mode. */
void st7735s_sleep_in(void);
/** @brief Leave sleep mode. */
void st7735s_sleep_out(void);
/** @brief Turn the panel output on. */
void st7735s_display_on(void);
/** @brief Turn the panel output off. */
void st7735s_display_off(void);

/**
 * @brief Switch the module backlight on or off.
 *
 * BLK drives a low-side NPN switch, so a high level sinks the LED string and the
 * 10 kOhm base pull-down keeps the LEDs dark while the pin is undriven (reset,
 * high impedance). The display service therefore leaves the backlight off during
 * the power-on sequence and switches it on only after the first frame is on the
 * panel; turning it on any earlier lights the controller's blank white page.
 *
 * The pin can also be PWM driven for dimming when the platform routes a timer
 * channel to it; this call only switches it fully on or off.
 *
 * @param on Non-zero to switch the LEDs on, zero to switch them off.
 */
void st7735s_set_backlight(uint8_t on);

/**
 * @brief Select the write direction written by the next st7735s_init() call.
 * @param rotation Write direction, also selects the visible-area offsets.
 */
void st7735s_set_rotation(st7735s_rotation_t rotation);
/** @brief Return the column offset of the visible area inside frame memory. */
uint16_t st7735s_column_offset(void);
/** @brief Return the row offset of the visible area inside frame memory. */
uint16_t st7735s_row_offset(void);

/**
 * @brief Program the drawing window and start a memory write.
 *
 * The coordinates are panel-relative; the visible-area offset is added here.
 *
 * @param x Left edge, zero-based.
 * @param y Top edge, zero-based.
 * @param w Window width in pixels.
 * @param h Window height in pixels.
 */
void st7735s_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

#ifdef __cplusplus
}
#endif

#endif /* ST7735S_H */
