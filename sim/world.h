/*
 * world.h - the physics around the board: flight profile, standard
 * atmosphere, the avionics box's temperature, and the battery.
 */
#ifndef VAYU_WORLD_H
#define VAYU_WORLD_H

#include <stdbool.h>

#include "sim.h"

typedef struct {
    double t;            /* s since power-on                          */
    double alt_m;        /* true altitude above mean sea level        */
    double vz, az;       /* vertical speed m/s, acceleration m/s^2    */
    double lat_deg, lon_deg;
    double amb_c;        /* outside air temperature                   */
    double p_pa;         /* outside static pressure                   */
    double rho;          /* air density kg/m^3                        */
    double box_c;        /* avionics box (board + battery) temperature */
    bool   heater_cmd;   /* what the firmware asked for               */
    double soc;          /* battery state of charge 0..1              */
    double vbat, ibat;   /* battery terminal voltage, current         */
    double energy_wh;    /* drawn from the battery so far             */
    double heater_wh;
    bool   powered;      /* supply rails up                           */
    bool   mcu_running;  /* firmware executing                        */
} world_t;

extern world_t W;

void   world_init(const scenario_t *sc);
void   world_step(double dt);
double world_end_s(void);
double isa_pressure_pa(double h_m);
double isa_temperature_c(double h_m);

/* physical constants shared with tools/power_budget.py (keep in sync) */
#define GROUND_ALT_M       5486.4  /* 18,000 ft launch site */
#define TOP_ALT_M          15240.0 /* 50,000 ft             */
#define BOX_HEAT_CAP_J_K   200.0   /* board + 2 x 18650 + enclosure            */
#define BOX_R_INSULATION   10.0    /* K/W: 12 mm foam (k = 0.03) around a 10x10x5 cm box */
#define BOX_AREA_M2        0.04
#define H_CONV_SEA_LEVEL   30.0    /* W/m^2K, moving air on a flying UAV        */
#define HEATER_OHMS        11.0    /* ~5 W at 7.4 V                             */
#define BUCK_EFFICIENCY    0.88    /* battery -> 5 V                            */
#define I_3V3_A            0.065   /* MCU 45 mA, sensors, flash, CAN transceiver (via LDO) */
#define I_GPS_5V_A         0.030
#define I_RADIO_5V_A       0.045   /* average incl. TX bursts                   */
#define BATTERY_AH         3.0     /* 2S1P 18650: 7.2 V nominal, 21.6 Wh        */

#endif
