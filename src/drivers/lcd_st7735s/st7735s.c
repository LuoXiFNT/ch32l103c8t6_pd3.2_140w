/**
 * @file st7735s.c
 * @brief ST7735S controller driver: power-on sequence, write direction, and window.
 *
 * Panel module: ZJY180S0800TG01 (128RGBx160, ST7735S driver, 4-wire SPI). The
 * module spec points at the ST7735S manual for the command set, so the sequence
 * below is table-driven with one step per datasheet command; the payload values
 * are the ones validated on this board. The module's only extra circuit is the
 * BLK low-side backlight switch (see st7735s_set_backlight()).
 *
 * The driver knows nothing about the MCU: every byte, pin and delay goes through
 * the st7735s_bus_t supplied to st7735s_init(). The platform layer implements it in
 * its own port file, which is the only place to touch when the platform changes. Initialization runs once, so it writes bytes blocking; the
 * pixel stream is started by lcd.c through st7735s_start_pixels() and drained
 * asynchronously by the bus task callback.
 *
 * The reset and sleep-out waits below dominate the boot time (about 320 ms, see
 * ST7735S_INIT_BLOCKING_MS). The backlight stays dark for that whole window: the
 * platform drives BLK low before anything else and the base pull-down keeps the
 * LEDs off until the display service enables them.
 *
 * @author LuoXiFNT
 * @date 2026-11-05
 * @lastModified 2026-11-05
 */

#include "st7735s.h"

/** @brief One step of the power-on sequence: command, payload, and settle time. */
typedef struct {
    uint8_t cmd;            /**< Command code. */
    uint8_t len;            /**< Number of payload bytes that follow. */
    const uint8_t *data;    /**< Payload bytes, NULL when len is zero. */
    uint16_t delay_ms;      /**< Settle time after the command, in milliseconds. */
} st7735s_init_step_t;

/** @brief Write direction: MADCTL value and the matching visible-area offsets. */
typedef struct {
    uint8_t madctl;         /**< MADCTL value for this direction. */
    uint8_t column_offset;  /**< First visible column inside the 132-column memory. */
    uint8_t row_offset;     /**< First visible row inside the 162-row memory. */
} st7735s_rotation_cfg_t;

static const uint8_t s_frmctr1[] = { 0x05u, 0x3Cu, 0x3Cu };
static const uint8_t s_frmctr2[] = { 0x05u, 0x3Cu, 0x3Cu };
static const uint8_t s_frmctr3[] = { 0x05u, 0x3Cu, 0x3Cu, 0x05u, 0x3Cu, 0x3Cu };
/* Line inversion for the 160x128 glass. */
static const uint8_t s_invctr[] = { 0x03u };
/* Booster and regulator voltages. */
static const uint8_t s_pwctr1[] = { 0x28u, 0x08u, 0x04u };
static const uint8_t s_pwctr2[] = { 0xC0u };
static const uint8_t s_pwctr3[] = { 0x0Du, 0x00u };
static const uint8_t s_pwctr4[] = { 0x8Du, 0x2Au };
static const uint8_t s_pwctr5[] = { 0x8Du, 0xEEu };
static const uint8_t s_vmctr1[] = { 0x1Au };
/* RGB565 pixel format. */
static const uint8_t s_colmod[] = { ST7735S_COLMOD_16BIT };
/* Gamma curves measured for this module. */
static const uint8_t s_gmctrp1[] = {
    0x04u, 0x22u, 0x07u, 0x0Au, 0x2Eu, 0x30u, 0x25u, 0x2Au,
    0x28u, 0x26u, 0x2Eu, 0x3Au, 0x00u, 0x01u, 0x03u, 0x13u
};
static const uint8_t s_gmctrn1[] = {
    0x04u, 0x16u, 0x06u, 0x0Du, 0x2Du, 0x26u, 0x23u, 0x27u,
    0x27u, 0x25u, 0x2Du, 0x3Bu, 0x00u, 0x01u, 0x04u, 0x13u
};

/** Write directions: MADCTL value and the matching visible-area offsets.
 *
 * The controller addresses 132x162 of frame memory while the module glass shows
 * 128x160 dots, so the visible area starts at (+2,+1) or (+1,+2) depending on the
 * write direction. The values come from the working reference driver for this
 * module, with MADCTL D3 (RGB/BGR) left at 0: this glass is RGB wired, so setting
 * the bit swaps the red and blue channels. */
static const st7735s_rotation_cfg_t s_rotations[] = {
    { 0x00u, 2u, 1u },                                                              /* rotation 0 */
    { (uint8_t)(ST7735S_MADCTL_MX | ST7735S_MADCTL_MY), 2u, 1u },                    /* rotation 1 */
    { (uint8_t)(ST7735S_MADCTL_MX | ST7735S_MADCTL_MV | ST7735S_MADCTL_ML), 1u, 2u },/* rotation 2 */
    { (uint8_t)(ST7735S_MADCTL_MY | ST7735S_MADCTL_MV), 1u, 2u }                     /* rotation 3 */
};

/* Steps that run before MADCTL: sleep out, frame rate, inversion, power, VCOM.
 * The settle time after SLPOUT is a panel requirement from the vendor example and
 * must not be shortened; it also feeds ST7735S_INIT_BLOCKING_MS. Adding another
 * delay to this table means adding it to that macro as well, otherwise the sub
 * board waits for the wrong amount of time. */
static const st7735s_init_step_t s_init_pre_madctl[] = {
    { ST7735S_CMD_SLPOUT,  0u, 0,        ST7735S_SLEEP_OUT_MS },
    { ST7735S_CMD_FRMCTR1, 3u, s_frmctr1,  0u },
    { ST7735S_CMD_FRMCTR2, 3u, s_frmctr2,  0u },
    { ST7735S_CMD_FRMCTR3, 6u, s_frmctr3,  0u },
    { ST7735S_CMD_INVCTR,  1u, s_invctr,   0u },
    { ST7735S_CMD_PWCTR1,  3u, s_pwctr1,   0u },
    { ST7735S_CMD_PWCTR2,  1u, s_pwctr2,   0u },
    { ST7735S_CMD_PWCTR3,  2u, s_pwctr3,   0u },
    { ST7735S_CMD_PWCTR4,  2u, s_pwctr4,   0u },
    { ST7735S_CMD_PWCTR5,  2u, s_pwctr5,   0u },
    { ST7735S_CMD_VMCTR1,  1u, s_vmctr1,   0u }
};

/* Steps that run after MADCTL: gamma curves and the RGB565 pixel format. */
static const st7735s_init_step_t s_init_post_madctl[] = {
    { ST7735S_CMD_GMCTRP1,  16u, s_gmctrp1, 0u },
    { ST7735S_CMD_GMCTRN1,  16u, s_gmctrn1, 0u },
    { ST7735S_CMD_COLMOD,    1u, s_colmod,  0u }
};

/* Bus supplied by the platform, and the write direction in effect. */
static const st7735s_bus_t *s_bus;
static uint8_t s_rotation = (uint8_t)ST7735S_ROTATION_3;

/** @brief Block for the given number of milliseconds through the platform bus. */
static void st7735s_delay(uint16_t ms)
{
    if (s_bus != 0) {
        s_bus->delay_ms(s_bus->user, ms);
    }
}

void st7735s_write_command(uint8_t cmd)
{
    if (s_bus != 0) {
        s_bus->write_cmd(s_bus->user, cmd);
    }
}

void st7735s_write_data8(uint8_t data)
{
    if (s_bus != 0) {
        s_bus->write_data(s_bus->user, &data, 1u);
    }
}

void st7735s_write_data16(uint16_t data)
{
    uint8_t bytes[2];
    uint16_t len = 2u;

    bytes[0] = (uint8_t)(data >> 8u);
    bytes[1] = (uint8_t)(data & 0xFFu);

    if (s_bus != 0) {
        s_bus->write_data(s_bus->user, bytes, len);
    }
}

void st7735s_reset(void)
{
    /* Module reset timing taken from the vendor example. These blocking delays
     * are part of the panel's power-on requirements; do not shorten them. */
    if (s_bus != 0) {
        s_bus->set_reset(s_bus->user, 0u);
    }
    st7735s_delay(ST7735S_RESET_LOW_MS);
    if (s_bus != 0) {
        s_bus->set_reset(s_bus->user, 1u);
    }
    st7735s_delay(ST7735S_RESET_SETTLE_MS);
}

/**
 * @brief Run one initialization table.
 * @param steps Pointer to the first step.
 * @param count Number of steps in the table.
 */
static void st7735s_run_sequence(const st7735s_init_step_t *steps, uint16_t count)
{
    uint16_t i;
    uint8_t n;

    for (i = 0u; i < count; i++) {
        st7735s_write_command(steps[i].cmd);
        for (n = 0u; n < steps[i].len; n++) {
            st7735s_write_data8(steps[i].data[n]);
        }
        if (steps[i].delay_ms != 0u) {
            st7735s_delay(steps[i].delay_ms);
        }
    }
}

void st7735s_init(const st7735s_bus_t *bus, st7735s_rotation_t rotation)
{
    s_bus = bus;
    st7735s_set_rotation(rotation);

    st7735s_reset();

    st7735s_run_sequence(s_init_pre_madctl,
                         (uint16_t)(sizeof(s_init_pre_madctl) / sizeof(s_init_pre_madctl[0])));

    st7735s_write_command(ST7735S_CMD_MADCTL);
    st7735s_write_data8(s_rotations[s_rotation].madctl);

    st7735s_run_sequence(s_init_post_madctl,
                         (uint16_t)(sizeof(s_init_post_madctl) / sizeof(s_init_post_madctl[0])));
}

bool st7735s_start_pixels(const uint8_t *data, uint16_t len)
{
    if (s_bus == 0) {
        return 0u;
    }
    return s_bus->start_pixels(s_bus->user, data, len);
}

bool st7735s_bus_idle(void)
{
    if (s_bus == 0) {
        return true;
    }
    return s_bus->is_idle(s_bus->user);
}

void st7735s_bus_task(void)
{
    if (s_bus != 0) {
        s_bus->task(s_bus->user);
    }
}

void st7735s_sleep_in(void)
{
    st7735s_write_command(ST7735S_CMD_SLPIN);
    st7735s_delay(ST7735S_SLEEP_IN_MS);
}

void st7735s_sleep_out(void)
{
    st7735s_write_command(ST7735S_CMD_SLPOUT);
    st7735s_delay(ST7735S_SLEEP_OUT_MS);
}

void st7735s_display_on(void)
{
    st7735s_write_command(ST7735S_CMD_DISPON);
}

void st7735s_display_off(void)
{
    st7735s_write_command(ST7735S_CMD_DISPOFF);
}

void st7735s_set_backlight(uint8_t on)
{
    if (s_bus != 0) {
        s_bus->set_backlight(s_bus->user, (on != 0u) ? 1u : 0u);
    }
}

void st7735s_set_rotation(st7735s_rotation_t rotation)
{
    if ((uint8_t)rotation < (uint8_t)(sizeof(s_rotations) / sizeof(s_rotations[0]))) {
        s_rotation = (uint8_t)rotation;
    }
}

uint16_t st7735s_column_offset(void)
{
    return (uint16_t)s_rotations[s_rotation].column_offset;
}

uint16_t st7735s_row_offset(void)
{
    return (uint16_t)s_rotations[s_rotation].row_offset;
}

void st7735s_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    uint16_t col = (uint16_t)(x + st7735s_column_offset());
    uint16_t row = (uint16_t)(y + st7735s_row_offset());

    st7735s_write_command(ST7735S_CMD_CASET);
    st7735s_write_data16(col);
    st7735s_write_data16((uint16_t)(col + w - 1u));
    st7735s_write_command(ST7735S_CMD_RASET);
    st7735s_write_data16(row);
    st7735s_write_data16((uint16_t)(row + h - 1u));
    st7735s_write_command(ST7735S_CMD_RAMWR);
}
