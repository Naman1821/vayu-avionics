/*
 * world.c - physics for the digital twin.
 *
 * Simple, documented models (not CFD): the point is to put the firmware
 * through the right conditions (thin cold air, a cold battery, a heater
 * that has to work for its living), not to predict temperatures to 0.1 C.
 */
#include "world.h"

#include <math.h>
#include <string.h>

world_t W;

#define RAMP_S     10.0
#define T_GROUND   300.0
#define CLIMB_MS   5.0
#define DESC_MS    8.0
#define T_LOITER   3600.0
#define T_AFTER    180.0
#define LOITER_AMP 10.0   /* m: gentle +/-10 m altitude hold oscillation */
#define LOITER_T   200.0  /* s: 18 full periods in the loiter */

static double t_climb, t_desc, t_climb_end, t_loiter_end, t_desc_end, t_end;
static bool   heater_connected;
static int    phys_div;

/* ---- standard atmosphere ---------------------------------------------- */
double isa_temperature_c(double h)
{
    return (h < 11000.0 ? 288.15 - 0.0065 * h : 216.65) - 273.15;
}

double isa_pressure_pa(double h)
{
    if (h < 11000.0)
        return 101325.0 * pow(1.0 - 0.0065 * h / 288.15, 5.25588);
    return 22632.06 * exp(-(h - 11000.0) * 9.80665 / (287.053 * 216.65));
}

/* ---- flight profile ------------------------------------------------------ */
static double trapezoid(double tau, double total, double v)
{
    if (tau < RAMP_S) return v * tau / RAMP_S;
    if (tau > total - RAMP_S) return v * (total - tau) / RAMP_S;
    return v;
}

static double target_vz(double t)
{
    if (t < T_GROUND) return 0.0;
    if (t < t_climb_end) return trapezoid(t - T_GROUND, t_climb, CLIMB_MS);
    if (t < t_loiter_end) {
        double w = 2.0 * M_PI / LOITER_T;
        return LOITER_AMP * w * cos(w * (t - t_climb_end)) * fmin(1.0, (t - t_climb_end) / 20.0);
    }
    if (t < t_desc_end) return -trapezoid(t - t_loiter_end, t_desc, DESC_MS);
    return 0.0;
}

double world_end_s(void) { return t_end; }

/* ---- battery (2S Li-ion) -------------------------------------------------- */
static double cell_ocv(double soc)
{
    static const double v[] = { 3.00, 3.30, 3.45, 3.55, 3.62, 3.68, 3.74, 3.81, 3.89, 3.97, 4.07, 4.18 };
    static const double s[] = { 0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00 };
    if (soc <= 0) return v[0];
    if (soc >= 1) return v[11];
    for (int i = 1; i < 12; i++)
        if (soc <= s[i])
            return v[i - 1] + (v[i] - v[i - 1]) * (soc - s[i - 1]) / (s[i] - s[i - 1]);
    return v[11];
}

/* fraction of capacity a Li-ion cell can deliver at temperature T (approximate) */
static double cold_capacity_factor(double t_c)
{
    if (t_c >= 25) return 1.0;
    if (t_c >= 0)  return 0.90 + 0.10 * t_c / 25.0;
    if (t_c >= -20) return 0.70 + 0.20 * (t_c + 20) / 20.0;
    if (t_c >= -40) return 0.45 + 0.25 * (t_c + 40) / 20.0;
    return 0.45;
}

/* pack internal resistance roughly doubles every 15 C colder */
static double pack_resistance(double t_c)
{
    double r = 0.06 * pow(2.0, (25.0 - t_c) / 15.0);
    return r > 2.0 ? 2.0 : r;
}

static void battery_and_heat(double dt)
{
    double p_elec = 0.0; /* electronics, battery side, W */
    if (W.powered)
        p_elec = 5.0 * (I_3V3_A + I_GPS_5V_A + I_RADIO_5V_A) / BUCK_EFFICIENCY;
    bool heater = W.powered && W.heater_cmd && heater_connected;

    double ocv = 2.0 * cell_ocv(W.soc);
    double r = pack_resistance(W.box_c);
    double k = heater ? 1.0 + r / HEATER_OHMS : 1.0;
    /* V = OCV - I R with I = P/V (regulated load) + V/R_heater: a quadratic in V */
    double disc = ocv * ocv - 4.0 * k * p_elec * r;
    double v = disc > 0 ? (ocv + sqrt(disc)) / (2.0 * k) : ocv / (2.0 * k);
    double i = (v > 0.1 ? p_elec / v : 0.0) + (heater ? v / HEATER_OHMS : 0.0);
    double p_heater = heater ? v * v / HEATER_OHMS : 0.0;

    W.vbat = v;
    W.ibat = i;
    /* a cold cell delivers less of its charge: the same current drains it faster */
    W.soc -= i * dt / (3600.0 * BATTERY_AH * cold_capacity_factor(W.box_c));
    if (W.soc < 0) W.soc = 0;
    W.energy_wh += v * i * dt / 3600.0;
    W.heater_wh += p_heater * dt / 3600.0;

    /* box: one thermal mass, insulation + outside convection that weakens in thin air */
    double h = H_CONV_SEA_LEVEL * pow(W.rho / 1.225, 0.6);
    double r_total = BOX_R_INSULATION + 1.0 / (h * BOX_AREA_M2);
    double p_in = p_elec + p_heater + i * i * r; /* every watt used ends up as heat in the box */
    W.box_c += dt * (p_in - (W.box_c - W.amb_c) / r_total) / BOX_HEAT_CAP_J_K;
}

/* ---- public ------------------------------------------------------------- */
void world_init(const scenario_t *sc)
{
    memset(&W, 0, sizeof W);
    t_climb = (TOP_ALT_M - GROUND_ALT_M) / CLIMB_MS + RAMP_S;
    t_desc = (TOP_ALT_M - GROUND_ALT_M) / DESC_MS + RAMP_S;
    t_climb_end = T_GROUND + t_climb;
    t_loiter_end = t_climb_end + T_LOITER;
    t_desc_end = t_loiter_end + t_desc;
    t_end = t_desc_end + T_AFTER;

    heater_connected = !sc->heater_disconnected;
    phys_div = 0;
    W.alt_m = GROUND_ALT_M;
    W.lat_deg = 34.30;   /* illustrative high-altitude launch site */
    W.lon_deg = 77.60;
    W.amb_c = isa_temperature_c(W.alt_m);
    W.p_pa = isa_pressure_pa(W.alt_m);
    W.rho = W.p_pa / (287.053 * (W.amb_c + 273.15));
    W.box_c = W.amb_c;   /* cold-soaked overnight */
    W.soc = sc->start_soc > 0 ? sc->start_soc : 1.0;
    W.vbat = 2.0 * cell_ocv(W.soc);
    W.powered = true;
    W.mcu_running = true;
}

void world_step(double dt)
{
    double vz_old = W.vz;
    W.t += dt;
    W.vz = target_vz(W.t);
    W.az = (W.vz - vz_old) / dt;
    W.alt_m += 0.5 * (vz_old + W.vz) * dt;

    if (W.alt_m > GROUND_ALT_M + 1.0) { /* drift with the wind once airborne */
        W.lon_deg += 8.0 * dt / (111320.0 * cos(W.lat_deg * M_PI / 180.0));
        W.lat_deg += 2.0 * dt / 110540.0;
    }

    if (++phys_div >= 10) { /* the slow physics runs at 100 Hz */
        phys_div = 0;
        W.amb_c = isa_temperature_c(W.alt_m);
        W.p_pa = isa_pressure_pa(W.alt_m);
        W.rho = W.p_pa / (287.053 * (W.amb_c + 273.15));
        battery_and_heat(dt * 10.0);
    }
}
