#include "heater.h"

#include "config.h"

static struct {
    uint64_t history; /* one bit per second: was the heater on? */
    bool     blink;
} h;

bool heater_decide(bool on, bool temp_valid, int16_t t, power_state_t power, bool *blink)
{
    if (power == POWER_CRITICAL)
        return false; /* the last energy goes to logging and the radio */
    if (!temp_valid) {
        *blink = !*blink;
        return *blink;
    }
    int16_t on_below  = (power == POWER_LOW) ? HEATER_LOW_ON_BELOW : HEATER_ON_BELOW_CDEG;
    int16_t off_above = (power == POWER_LOW) ? HEATER_LOW_OFF_ABOVE : HEATER_OFF_ABOVE_CDEG;
    if (t < on_below)
        return true;
    if (t > off_above)
        return false;
    return on; /* inside the band: keep doing what we were doing (hysteresis) */
}

void heater_init(void)
{
    h.history = 0;
    h.blink = false;
    V.heater_on = false;
    V.heater_duty_pct = 0;
    hal_heater(false);
}

void heater_task(uint32_t now)
{
    (void)now;
    /* board temperature: the barometer's sensor is the most accurate, IMU is the backup */
    V.board_temp_valid = true;
    if (V.baro_ok)
        V.board_temp_cdeg = V.baro_temp_cdeg;
    else if (V.imu_ok)
        V.board_temp_cdeg = V.imu_temp_cdeg;
    else
        V.board_temp_valid = false;

    V.heater_on = heater_decide(V.heater_on, V.board_temp_valid, V.board_temp_cdeg, V.power, &h.blink);
    hal_heater(V.heater_on);

    h.history = (h.history << 1) | (V.heater_on ? 1u : 0u);
    V.heater_duty_pct = (uint8_t)(__builtin_popcountll(h.history & ((1ULL << 60) - 1)) * 100 / 60);
}
