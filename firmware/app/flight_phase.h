/*
 * flight_phase.h - which altitude to trust, how fast we climb, and which
 * phase of flight we are in:
 *
 *   BOOT -> PREFLIGHT -> ASCENT <-> CRUISE -> DESCENT -> LANDED
 *
 * Every transition must hold for a few seconds, so one noisy reading
 * cannot change the phase.
 */
#ifndef VAYU_FLIGHT_PHASE_H
#define VAYU_FLIGHT_PHASE_H

#include <stdint.h>

#include "logger.h"

void phase_init(const log_resume_t *resume); /* warm restart if the log says we were flying */
void phase_task(uint32_t now);               /* 10 Hz */
const char *phase_name(int phase);

#endif
