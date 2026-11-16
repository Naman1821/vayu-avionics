#ifndef VAYU_ALTITUDE_H
#define VAYU_ALTITUDE_H

#include <stdint.h>

/* Pressure (Pa) to altitude (m) with the International Standard Atmosphere,
 * valid from sea level to 20 km (two layers). */
float isa_altitude_m(float pressure_pa);

/* The common one-line formula, 44330 * (1 - (p/p0)^0.1903). It only models
 * the troposphere and is ~260 m wrong at 50,000 ft. Kept for comparison. */
float troposphere_only_altitude_m(float pressure_pa);

#endif
