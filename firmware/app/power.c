#include "power.h"

#include <string.h>

#include "config.h"
#include "health.h"
#include "heater.h"
#include "ina226.h"
#include "telemetry.h"

static struct {
    dev_health_t h;
    power_sm_t   sm;
    uint32_t     charge_rem; /* mA x ms not yet counted as uAh */
} p;

power_state_t power_update_state(power_sm_t *sm, power_state_t cur, uint16_t mv, uint32_t now,
                                 bool allow_better)
{
    power_state_t down = (mv < VBAT_CRITICAL_MV) ? POWER_CRITICAL
                       : (mv < VBAT_LOW_MV)      ? POWER_LOW
                                                 : POWER_NORMAL;
    power_state_t up   = (mv >= VBAT_LOW_MV + VBAT_HYST_MV)      ? POWER_NORMAL
                       : (mv >= VBAT_CRITICAL_MV + VBAT_HYST_MV) ? POWER_LOW
                                                                 : POWER_CRITICAL;
    if (down > cur) { /* getting worse */
        sm->better_pending = false;
        if (!sm->worse_pending) {
            sm->worse_pending = true;
            sm->worse_since = now;
        }
        if (now - sm->worse_since >= POWER_WORSE_MS) {
            sm->worse_pending = false;
            return down;
        }
    } else if (allow_better && up < cur) { /* clearly better, past the hysteresis band */
        sm->worse_pending = false;
        if (!sm->better_pending) {
            sm->better_pending = true;
            sm->better_since = now;
        }
        if (now - sm->better_since >= POWER_BETTER_MS) {
            sm->better_pending = false;
            return up;
        }
    } else {
        sm->worse_pending = sm->better_pending = false;
    }
    return cur;
}

static bool bringup(void)
{
    return ina226_init(INA226_ADDR, SHUNT_MILLIOHM, VBAT_CRITICAL_MV) == HAL_OK;
}

void power_init(uint32_t now)
{
    memset(&p, 0, sizeof p);
    if (!bringup())
        dev_health_fail(&p.h, now);
    V.pwrmon_ok = !p.h.failed;
    V.power = POWER_NORMAL;
}

void power_task(uint32_t now)
{
    if (p.h.failed) {
        if (dev_health_retry_due(&p.h, now)) {
            hal_i2c_recover();
            if (bringup())
                dev_health_reset(&p.h);
        }
        V.pwrmon_ok = !p.h.failed;
        return;
    }

    uint16_t mv = 0;
    int16_t ma = 0;
    hal_status_t st = ina226_read(INA226_ADDR, &mv, &ma);
    if (st == HAL_OK) {
        V.vbat_mv = mv;
        V.ibat_ma = ma;
        if (ma > 0) { /* coulomb counting: 1 mA for 3.6 s = 1 uAh */
            p.charge_rem += (uint32_t)ma * POWER_PERIOD_MS;
            V.used_uah += p.charge_rem / 3600u;
            p.charge_rem %= 3600u;
        }
        bool on_ground = V.phase == PHASE_BOOT || V.phase == PHASE_PREFLIGHT || V.phase == PHASE_LANDED;
        power_state_t next = power_update_state(&p.sm, V.power, mv, now, on_ground);
        if (next != V.power) {
            V.power = next;
            telemetry_event(EVT_POWER_STATE, next);
            heater_task(now); /* shed the heater now, not at its next 1 Hz slot */
        }
    }
    dev_health_report(&p.h, st == HAL_OK, now);
    V.pwrmon_ok = !p.h.failed;
}
