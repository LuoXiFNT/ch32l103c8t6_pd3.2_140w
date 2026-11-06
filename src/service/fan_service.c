/**
 * @file fan_service.c
 * @brief Main-board fan duty control derived from total measured power.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#include "fan_service.h"

#include "board_config.h"
#include "board_link.h"
#include "ch32l103_init.h"
#include "ina226.h"
#include "ina226_service.h"

#if BOARD_IS_MAIN
/* Total power <= ramp start: fixed duty. Then linearly ramp to max duty. */
#define FAN_RAMP_START_POWER_MW 100000ul
#define FAN_CONST_DUTY_PERMILLE 300u
#define FAN_FULL_SPEED_POWER_MW 300000ul
#define FAN_MAX_DUTY_PERMILLE   800u
#define FAN_DUTY_UNKNOWN        0xFFFFu

#if FAN_RAMP_START_POWER_MW >= FAN_FULL_SPEED_POWER_MW
#error "FAN_FULL_SPEED_POWER_MW must be greater than FAN_RAMP_START_POWER_MW"
#endif
#if (FAN_CONST_DUTY_PERMILLE > 1000u) || (FAN_MAX_DUTY_PERMILLE > 1000u)
#error "Fan duty permille must be 0..1000"
#endif

static uint16_t s_fan_last_permille = FAN_DUTY_UNKNOWN;

/**
 * @brief Sum the power of both channels on both boards.
 *
 * The remote channels only count while the other board is still sending frames: a
 * stale snapshot would otherwise hold the fan at its last duty forever after the link
 * goes down.
 */
static uint32_t Fan_TotalPowerMw(void)
{
    board_link_channel_t c1;
    board_link_channel_t a1;
    board_link_remote_t remote;
    uint32_t total_mw;

    BoardLink_BuildChannels(&c1, &a1);
    BoardLink_GetRemoteData(&remote);

    total_mw = ina226_convert_power_mw(c1.pwr_raw, INA226_SERVICE_CURRENT_LSB_UA) +
               ina226_convert_power_mw(a1.pwr_raw, INA226_SERVICE_CURRENT_LSB_UA);
    if (BoardLink_RemoteOnline()) {
        total_mw += ina226_convert_power_mw(remote.c.pwr_raw, INA226_SERVICE_CURRENT_LSB_UA) +
                    ina226_convert_power_mw(remote.a.pwr_raw, INA226_SERVICE_CURRENT_LSB_UA);
    }

    return total_mw;
}

/** @brief Convert total measured power to the configured fan duty. */
static uint16_t Fan_DutyFromPowerMw(uint32_t total_mw)
{
    uint32_t ramp_mw;
    uint32_t ramp_range_mw;
    uint32_t delta;

    if (total_mw <= FAN_RAMP_START_POWER_MW) {
        return FAN_CONST_DUTY_PERMILLE;
    }

    if (total_mw >= FAN_FULL_SPEED_POWER_MW) {
        return FAN_MAX_DUTY_PERMILLE;
    }

    ramp_mw = total_mw - FAN_RAMP_START_POWER_MW;
    ramp_range_mw = FAN_FULL_SPEED_POWER_MW - FAN_RAMP_START_POWER_MW;

#if (FAN_MAX_DUTY_PERMILLE >= FAN_CONST_DUTY_PERMILLE)
    delta = ((uint32_t)(FAN_MAX_DUTY_PERMILLE - FAN_CONST_DUTY_PERMILLE) * ramp_mw +
             (ramp_range_mw / 2u)) / ramp_range_mw;
    return (uint16_t)(FAN_CONST_DUTY_PERMILLE + delta);
#else
    delta = ((uint32_t)(FAN_CONST_DUTY_PERMILLE - FAN_MAX_DUTY_PERMILLE) * ramp_mw +
             (ramp_range_mw / 2u)) / ramp_range_mw;
    return (uint16_t)(FAN_CONST_DUTY_PERMILLE - delta);
#endif
}
#endif /* BOARD_IS_MAIN */

void Fan_Task(void)
{
#if BOARD_IS_MAIN
    uint16_t duty_permille = Fan_DutyFromPowerMw(Fan_TotalPowerMw());

    if (duty_permille != s_fan_last_permille) {
        s_fan_last_permille = duty_permille;
        CH32L103_FanSetDutyPermille(duty_permille);
    }
#endif
}
