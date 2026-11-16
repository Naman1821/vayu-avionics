/*
 * scenarios.c - the flights the digital twin runs.
 *
 * Mission (all scenarios): power on at a launch site at 18,000 ft (-20 C),
 * 5 min on the ground, climb at 5 m/s to 50,000 ft (~33 min), 60 min loiter,
 * descend at 8 m/s, land, 3 min on the ground. About 2 hours.
 *
 *   t =    0 -  300 s  ground, 5.5 km
 *   t =  300 - 2261 s  climb (9 km at ~1000 s, 12 km at ~1600 s)
 *   t = 2261 - 5861 s  loiter at 15.2 km, -56.5 C
 *   t = 5861 - 7090 s  descent
 *   t = 7090 - 7270 s  ground
 */
#include <string.h>

#include "sim.h"

const scenario_t SCENARIOS[] = {
    { .name = "nominal",
      .about = "Full mission, no faults. Ground sends a ping on the ground and an erase command in flight (must be refused).",
      .uplink_tests = true },
    { .name = "baro_failure",
      .about = "Barometer stops answering for 60 s at ~9 km. Altitude must switch to GPS, then back.",
      .baro_fail_from = 1000, .baro_fail_to = 1060 },
    { .name = "i2c_bus_stuck",
      .about = "A slave holds SDA low at ~10 km. Firmware must clock it free (9 pulses + STOP).",
      .i2c_stuck_at = 1200 },
    { .name = "gps_outage",
      .about = "No GPS fix for 60 s while loitering at 50,000 ft (antenna or jamming).",
      .gps_outage_from = 2700, .gps_outage_to = 2760 },
    { .name = "brownout",
      .about = "Battery connector glitch at ~13.5 km: everything loses power for 300 ms.",
      .brownout_at = 1900 },
    { .name = "cpu_hang",
      .about = "Firmware stops at 50,000 ft (e.g. a radiation bit-flip). Only the watchdog can recover.",
      .cpu_hang_at = 3000 },
    { .name = "can_bus_off",
      .about = "CAN-H shorted for 10 s: controller goes bus-off and must recover by itself.",
      .can_fault_from = 2500, .can_fault_to = 2510 },
    { .name = "imu_stuck",
      .about = "IMU keeps answering but its data freezes at ~9 km.",
      .imu_stuck_at = 1000 },
    { .name = "gps_rejects_airborne",
      .about = "Receiver refuses the airborne-mode command. Shows why it matters: no fix above 12 km.",
      .gps_rejects_airborne = true },
    { .name = "no_heater",
      .about = "What-if: heater disconnected. The box drops below the -40 C rating at altitude.",
      .heater_disconnected = true },
    { .name = "low_battery",
      .about = "Battery starts at 40 % charge. Firmware must shed load (LOW, then CRITICAL) and keep logging to landing.",
      .start_soc = 0.40 },
};

const int N_SCENARIOS = (int)(sizeof SCENARIOS / sizeof SCENARIOS[0]);

const scenario_t *sim_find_scenario(const char *name)
{
    for (int i = 0; i < N_SCENARIOS; i++)
        if (strcmp(SCENARIOS[i].name, name) == 0)
            return &SCENARIOS[i];
    return 0;
}
