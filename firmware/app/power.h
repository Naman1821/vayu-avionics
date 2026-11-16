/*
 * power.h - battery monitoring and the NORMAL / LOW / CRITICAL power state.
 *
 *   NORMAL    everything on
 *   LOW       heater only keeps the box above -20 C, telemetry slows to 0.5 Hz
 *   CRITICAL  heater off; logging and telemetry continue as long as possible
 */
#ifndef VAYU_POWER_H
#define VAYU_POWER_H

#include <stdbool.h>
#include <stdint.h>

#include "app.h"

typedef struct {
    bool     worse_pending, better_pending;
    uint32_t worse_since, better_since;
} power_sm_t;

/* Pure function, unit-tested: next state from battery voltage, with
 * hysteresis and time filters so a radio TX dip does not flip states.
 *
 * allow_better is false in flight. A battery does not recharge in the air,
 * and the voltage always "recovers" when we switch loads off: if that were
 * allowed, shedding the heater would bring us back to LOW, the heater would
 * come back on, the voltage would sag to CRITICAL again, and so on every
 * few seconds (the simulator caught exactly this). */
power_state_t power_update_state(power_sm_t *sm, power_state_t cur, uint16_t mv, uint32_t now,
                                 bool allow_better);

void power_init(uint32_t now);
void power_task(uint32_t now); /* 10 Hz */

#endif
