/*
 * altitude.c - pressure to altitude, correct above 11 km.
 *
 * The standard atmosphere has layers. Up to 11 km the air cools 6.5 C per km.
 * From 11 to 20 km (where a 50,000 ft flight spends its time) the temperature
 * is constant at -56.5 C, so pressure falls exponentially instead. The usual
 * one-line formula only knows the first layer.
 */
#include "altitude.h"

#include <math.h>

#define P0      101325.0f  /* sea-level pressure, Pa           */
#define T0      288.15f    /* sea-level temperature, K         */
#define LAPSE   0.0065f    /* K per metre, troposphere         */
#define G0      9.80665f   /* m/s^2                            */
#define R_AIR   287.053f   /* J/(kg K)                         */
#define P11     22632.06f  /* pressure at 11 km, Pa            */
#define T11     216.65f    /* temperature from 11 to 20 km, K  */

float isa_altitude_m(float p)
{
    if (p >= P11) /* troposphere */
        return (T0 / LAPSE) * (1.0f - powf(p / P0, R_AIR * LAPSE / G0));
    /* lower stratosphere: isothermal, h = 11 km + H * ln(P11 / p) */
    return 11000.0f + (R_AIR * T11 / G0) * logf(P11 / p);
}

float troposphere_only_altitude_m(float p)
{
    return 44330.0f * (1.0f - powf(p / P0, 0.190295f));
}
