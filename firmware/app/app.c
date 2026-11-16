/*
 * app.c - start-up and the main loop.
 *
 * No RTOS: a cooperative scheduler. Each task runs at its own rate; none of
 * them blocks for long. Things that must react quickly (GPS bytes, ground
 * commands, CAN frames, flash writes) are checked on every pass.
 */
#include "app.h"

#include <string.h>

#include "canbus.h"
#include "config.h"
#include "flight_phase.h"
#include "health.h"
#include "heater.h"
#include "logger.h"
#include "power.h"
#include "sensors.h"
#include "telemetry.h"

vayu_t V;

static volatile bool power_fail_flag;

typedef struct {
    void (*fn)(uint32_t now);
    uint16_t period_ms;
    uint32_t last_ms;
} task_t;

static task_t tasks[] = {
    { imu_task,       IMU_PERIOD_MS,    0 },
    { baro_task,      BARO_PERIOD_MS,   0 },
    { power_task,     POWER_PERIOD_MS,  0 },
    { phase_task,     PHASE_PERIOD_MS,  0 },
    { health_task,    HEALTH_PERIOD_MS, 0 },
    { heater_task,    HEATER_PERIOD_MS, 0 },
    { logger_task,    LOG_PERIOD_MS,    0 },
    { telemetry_task, TELEM_PERIOD_MS,  0 },
    { canbus_task,    CAN_PERIOD_MS,    0 },
};
#define N_TASKS (sizeof tasks / sizeof tasks[0])

/* Supply is falling (PVD interrupt). Only set a flag: the SPI bus may be in
 * use by the main loop right now, so the flash write happens there. */
void app_power_fail_isr(void)
{
    power_fail_flag = true;
}

void app_init(void)
{
    memset(&V, 0, sizeof V);
    power_fail_flag = false;
    V.reset_cause = hal_reset_cause();

    uint32_t now = hal_millis();
    telemetry_init();
    health_init();
    sensors_init(now);
    power_init(now);

    log_resume_t resume;
    logger_init(&resume);
    phase_init(&resume); /* warm restart if the log says we were flying */

    heater_init();
    canbus_init(now);

    now = hal_millis();
    V.gps_last_msg_ms = now;
    V.gps_last_fix_ms = now;
    for (unsigned i = 0; i < N_TASKS; i++)
        tasks[i].last_ms = now;

    logger_event(REC_BOOT);
    telemetry_event(EVT_BOOT, (uint16_t)(V.reset_cause | (V.warm_restart ? 0x100 : 0)));

    hal_watchdog_start(); /* from here on, a stuck loop resets the MCU in ~0.5 s */
}

void app_step(void)
{
    uint32_t now = hal_millis();

    if (power_fail_flag) { /* a few ms before the supply is gone */
        power_fail_flag = false;
        logger_event(REC_POWER_FAIL);
        telemetry_event(EVT_POWER_FAIL, V.vbat_mv);
    }

    gps_task(now);
    telemetry_rx(now);
    canbus_rx(now);
    logger_service(now);

    for (unsigned i = 0; i < N_TASKS; i++) {
        if (now - tasks[i].last_ms >= tasks[i].period_ms) {
            tasks[i].last_ms = now;
            tasks[i].fn(now);
        }
    }

    /* Kicked only here, never in an interrupt: if the main loop stops,
     * the watchdog must fire even though interrupts keep running. */
    hal_watchdog_kick();
}
