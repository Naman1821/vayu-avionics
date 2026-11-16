/*
 * sim_core.c - runs one scenario: advances time 1 ms at a time, steps the
 * world and the chips, injects faults, plays the watchdog and the power
 * supply, and calls the firmware's app_step() whenever the MCU is running.
 *
 * Outputs (in out/<scenario>/):
 *   truth.csv     the real world once a second (what really happened)
 *   flash.bin     the W25Q128's contents after landing (the black box)
 *   downlink.bin  every byte the firmware sent to the radio
 *   events.csv    what the simulator did (faults, resets)
 *   boots.csv     when the MCU booted and why (to put log time on one axis)
 */
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "app.h"
#include "crc.h"
#include "models.h"
#include "sim.h"
#include "telemetry.h"
#include "world.h"

sim_faults_t  F;
uint64_t      sim_ms;
uint64_t      sim_boot_ms;
reset_cause_t sim_reset_cause;
bool          sim_wd_armed;
uint64_t      sim_wd_last_kick;
uint32_t      sim_i2c_recoveries;

static const scenario_t *SC;
static FILE *events_fp, *boots_fp, *truth_fp, *downlink_fp;
static int   boot_count;
static uint64_t rng = 0x9E3779B97F4A7C15ULL;

/* ---- helpers -------------------------------------------------------------- */
double sim_noise(double sigma)
{
    /* xorshift64 + Box-Muller: deterministic, so every run is identical */
    rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
    double u1 = ((rng >> 11) + 1.0) / 9007199254740993.0;
    rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
    double u2 = (rng >> 11) / 9007199254740992.0;
    return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

void sim_event(const char *fmt, ...)
{
    if (!events_fp)
        return;
    va_list ap;
    va_start(ap, fmt);
    fprintf(events_fp, "%.3f,\"", sim_ms / 1000.0);
    vfprintf(events_fp, fmt, ap);
    fprintf(events_fp, "\"\n");
    va_end(ap);
}

void sim_downlink_write(const uint8_t *data, uint16_t len)
{
    if (downlink_fp)
        fwrite(data, 1, len, downlink_fp);
}

static bool in_window(double t, double from, double to)
{
    return from > 0 && t >= from && t < to;
}

static void update_faults(void)
{
    double t = sim_ms / 1000.0;
    bool was;

    was = F.baro_nack;
    F.baro_nack = in_window(t, SC->baro_fail_from, SC->baro_fail_to);
    if (F.baro_nack != was) sim_event(F.baro_nack ? "fault: barometer stops answering" : "fault cleared: barometer answers again");

    was = F.gps_outage;
    F.gps_outage = in_window(t, SC->gps_outage_from, SC->gps_outage_to);
    if (F.gps_outage != was) sim_event(F.gps_outage ? "fault: GPS signal lost" : "fault cleared: GPS signal back");

    was = F.can_fault;
    F.can_fault = in_window(t, SC->can_fault_from, SC->can_fault_to);
    if (F.can_fault != was) sim_event(F.can_fault ? "fault: CAN-H shorted" : "fault cleared: CAN bus repaired");

    if (SC->imu_stuck_at > 0 && !F.imu_stuck && t >= SC->imu_stuck_at) {
        F.imu_stuck = true;
        sim_event("fault: IMU data frozen");
    }
}

void sim_advance(uint32_t ms)
{
    while (ms--) {
        sim_ms++;
        world_step(0.001);
        update_faults();
        w25q_model_step();
        ublox_model_step();
        can_model_step();
    }
}

static void boot(reset_cause_t cause)
{
    static const char *names[] = { "power-on", "brown-out", "watchdog", "software", "pin", "unknown" };
    sim_boot_ms = sim_ms;
    sim_reset_cause = cause;
    sim_wd_armed = false;
    W.mcu_running = true;
    sim_uart_clear(); /* MCU RAM is gone */
    if (boots_fp)
        fprintf(boots_fp, "%d,%llu,%s\n", boot_count, (unsigned long long)sim_ms, names[cause]);
    boot_count++;
    sim_event("MCU boots (%s reset)", names[cause]);
    app_init();
}

static void send_uplink(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t frame[TM_MAX_PAYLOAD + 6];
    sim_uart_push_rx(UART_RADIO, frame, tm_encode(type, payload, len, frame));
}

/* ---- the run ------------------------------------------------------------------- */
static FILE *open_out(const char *dir, const char *name, const char *mode)
{
    char path[512];
    snprintf(path, sizeof path, "%s/%s", dir, name);
    FILE *fp = fopen(path, mode);
    if (!fp)
        fprintf(stderr, "cannot write %s\n", path);
    return fp;
}

int sim_run(const scenario_t *sc, const char *outdir)
{
    SC = sc;
    memset(&F, 0, sizeof F);
    F.gps_rejects_airborne = sc->gps_rejects_airborne;
    F.heater_disconnected = sc->heater_disconnected;
    sim_ms = 0;
    sim_i2c_recoveries = 0;
    boot_count = 0;
    rng = 0x9E3779B97F4A7C15ULL;

    if (mkdir(outdir, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "cannot create %s\n", outdir);
        return 1;
    }
    events_fp = open_out(outdir, "events.csv", "w");
    boots_fp = open_out(outdir, "boots.csv", "w");
    truth_fp = open_out(outdir, "truth.csv", "w");
    downlink_fp = open_out(outdir, "downlink.bin", "wb");
    if (!events_fp || !boots_fp || !truth_fp || !downlink_fp)
        return 1;
    fprintf(events_fp, "t_s,event\n");
    fprintf(boots_fp, "boot,sim_ms,cause\n");
    fprintf(truth_fp, "t_s,alt_m,vz,amb_c,p_pa,box_c,heater_cmd,vbat,ibat,soc,energy_wh,heater_wh,gps_fix,gps_dyn,powered,mcu_running\n");

    world_init(sc);
    w25q_model_init();
    ms5611_model_reset();
    ina226_model_reset();
    icm_model_reset();
    can_model_reset();
    ublox_model_power(true, true);
    sim_event("scenario %s: %s", sc->name, sc->about);

    boot(RESET_POWER_ON);

    const uint64_t end_ms = (uint64_t)(world_end_s() * 1000.0);
    int bo_stage = 0, hang_done = 0, ping_done = 0, erase_done = 0, stuck_done = 0;
    uint64_t bo_t = 0;

    while (sim_ms < end_ms) {
        sim_advance(1);
        double t = sim_ms / 1000.0;

        if (sc->i2c_stuck_at > 0 && !stuck_done && t >= sc->i2c_stuck_at) {
            stuck_done = 1;
            F.i2c_stuck = true;
            sim_event("fault: an I2C slave is holding SDA low");
        }

        /* brown-out: early warning, a few ms of hold-up, then nothing for 300 ms */
        if (sc->brownout_at > 0) {
            if (bo_stage == 0 && t >= sc->brownout_at) {
                sim_event("fault: battery connector glitch, supply falling (PVD interrupt)");
                app_power_fail_isr();
                bo_stage = 1;
                bo_t = sim_ms + 15;
            } else if (bo_stage == 1 && sim_ms >= bo_t) {
                sim_event("supply gone: MCU and every chip unpowered");
                W.powered = false;
                W.mcu_running = false;
                sim_wd_armed = false;
                w25q_model_power_loss();
                ublox_model_power(false, false);
                ms5611_model_reset();
                icm_model_reset();
                ina226_model_reset();
                can_model_reset();
                sim_uart_clear();
                bo_stage = 2;
                bo_t = sim_ms + 300;
            } else if (bo_stage == 2 && sim_ms >= bo_t) {
                sim_event("supply back");
                W.powered = true;
                ublox_model_power(true, false); /* hot start, but factory settings again */
                bo_stage = 3;
                boot(RESET_BROWN_OUT);
            }
        }

        /* firmware hang: the main loop stops, only the watchdog can save us */
        if (sc->cpu_hang_at > 0 && !hang_done && t >= sc->cpu_hang_at) {
            hang_done = 1;
            W.mcu_running = false;
            sim_event("fault: firmware stops running (main loop hung)");
        }
        if (sim_wd_armed && sim_ms - sim_wd_last_kick > 500) {
            sim_event("independent watchdog expired: MCU reset");
            boot(RESET_WATCHDOG);
        }

        /* ground station commands */
        if (sc->uplink_tests && !ping_done && t >= 200.0) {
            ping_done = 1;
            sim_event("ground: PING");
            send_uplink(TC_PING, 0, 0);
        }
        if (sc->uplink_tests && !erase_done && t >= 1500.0) {
            erase_done = 1;
            uint8_t magic[4] = { 0xA5, 0xE5, 0xA5, 0xE5 }; /* TC_ERASE_MAGIC, little-endian */
            sim_event("ground: ERASE LOG sent while flying (must be refused)");
            send_uplink(TC_ERASE_LOG, magic, 4);
        }

        /* below ~5.6 V the 5 V regulator drops out: the board browns out for good */
        if (W.powered && W.vbat < 5.6) {
            sim_event("battery exhausted (%.2f V): board browns out", W.vbat);
            W.powered = false;
            W.mcu_running = false;
            sim_wd_armed = false;
        }

        if (W.mcu_running)
            app_step();

        if (sim_ms % 1000 == 0)
            fprintf(truth_fp, "%.0f,%.2f,%.3f,%.2f,%.1f,%.3f,%d,%.3f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d\n",
                    t, W.alt_m, W.vz, W.amb_c, W.p_pa, W.box_c, W.heater_cmd ? 1 : 0, W.vbat, W.ibat,
                    W.soc, W.energy_wh, W.heater_wh, ublox_model_has_fix() ? 1 : 0, ublox_model_dyn_model(),
                    W.powered ? 1 : 0, W.mcu_running ? 1 : 0);
    }

    sim_event("end of mission");
    char path[512];
    snprintf(path, sizeof path, "%s/flash.bin", outdir);
    w25q_model_save(path);
    fclose(events_fp);
    fclose(boots_fp);
    fclose(truth_fp);
    fclose(downlink_fp);
    events_fp = boots_fp = truth_fp = downlink_fp = 0;
    return 0;
}
