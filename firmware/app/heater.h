/*
 * heater.h - keeps the avionics box (and the battery inside it) warm.
 *
 * At 50,000 ft the air is -56.5 C. The electronics alone only warm the box
 * to about -44 C, below the -40 C rating of most parts, and a cold Li-ion
 * cell loses capacity. A thermostat with hysteresis drives a heater film.
 */
#ifndef VAYU_HEATER_H
#define VAYU_HEATER_H

#include <stdbool.h>
#include <stdint.h>

#include "app.h"

/* Pure function, unit-tested. *blink alternates when we have no temperature
 * at all, giving a blind 50 % duty instead of freezing or cooking the board. */
bool heater_decide(bool currently_on, bool temp_valid, int16_t temp_cdeg,
                   power_state_t power, bool *blink);

void heater_init(void);
void heater_task(uint32_t now); /* 1 Hz */

#endif
