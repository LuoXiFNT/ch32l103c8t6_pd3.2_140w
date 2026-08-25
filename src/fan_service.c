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
#define FAN_RAMP_START_POWER_MW 100000UL
#define FAN_CONST_DUTY_PERMILLE 300U
#define FAN_FULL_SPEED_POWER_MW 300000UL
#define FAN_MAX_DUTY_PERMILLE   800U
#define FAN_DUTY_UNKNOWN        0xFFFFU

#if FAN_RAMP_START_POWER_MW >= FAN_FULL_SPEED_POWER_MW
#error "FAN_FULL_SPEED_POWER_MW must be greater than FAN_RAMP_START_POWER_MW"
#endif
#if (FAN_CONST_DUTY_PERMILLE > 1000U) || (FAN_MAX_DUTY_PERMILLE > 1000U)
#error "Fan duty permille must be 0..1000"
#endif

static u16 s_fan_last_permille = FAN_DUTY_UNKNOWN;

/** @brief Convert one channel's raw INA226 power to milliwatts. */
static u32 Fan_ChannelPowerMw(const board_link_channel_t *ch)
{
    return ina226_convert_power_mw(ch->pwr_raw,
                                   INA226_SERVICE_CURRENT_LSB_UA);
}

/** @brief Sum power from both channels on both boards. */
static u32 Fan_TotalPowerMw(void)
{
    board_link_channel_t c1;
    board_link_channel_t a1;
    board_link_remote_t remote;

    BoardLink_BuildChannels(&c1, &a1);
    BoardLink_GetRemoteData(&remote);

    return Fan_ChannelPowerMw(&c1) +
           Fan_ChannelPowerMw(&a1) +
           Fan_ChannelPowerMw(&remote.c) +
           Fan_ChannelPowerMw(&remote.a);
}

/** @brief Convert total measured power to the configured fan duty. */
static u16 Fan_DutyFromPowerMw(u32 total_mw)
{
    u32 ramp_mw;
    u32 ramp_range_mw;
    u32 delta;

    if (total_mw <= FAN_RAMP_START_POWER_MW) {
        return FAN_CONST_DUTY_PERMILLE;
    }

    if (total_mw >= FAN_FULL_SPEED_POWER_MW) {
        return FAN_MAX_DUTY_PERMILLE;
    }

    ramp_mw = total_mw - FAN_RAMP_START_POWER_MW;
    ramp_range_mw = FAN_FULL_SPEED_POWER_MW - FAN_RAMP_START_POWER_MW;

    if (FAN_MAX_DUTY_PERMILLE >= FAN_CONST_DUTY_PERMILLE) {
        delta = ((u32)(FAN_MAX_DUTY_PERMILLE - FAN_CONST_DUTY_PERMILLE) * ramp_mw +
                 (ramp_range_mw / 2U)) / ramp_range_mw;
        return (u16)(FAN_CONST_DUTY_PERMILLE + delta);
    }

    delta = ((u32)(FAN_CONST_DUTY_PERMILLE - FAN_MAX_DUTY_PERMILLE) * ramp_mw +
             (ramp_range_mw / 2U)) / ramp_range_mw;
    return (u16)(FAN_CONST_DUTY_PERMILLE - delta);
}
#endif

void Fan_Task(void)
{
#if BOARD_IS_MAIN
    u16 duty_permille = Fan_DutyFromPowerMw(Fan_TotalPowerMw());

    if (duty_permille != s_fan_last_permille) {
        s_fan_last_permille = duty_permille;
        CH32L103_FanSetDutyPermille(duty_permille);
    }
#endif
}
