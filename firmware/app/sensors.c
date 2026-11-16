#include "sensors.h"

#include <string.h>

#include "altitude.h"
#include "app.h"
#include "config.h"
#include "health.h"
#include "icm42688.h"
#include "ms5611.h"
#include "ublox.h"

static ms5611_t baro;

static struct {
    dev_health_t baro_h, imu_h;
    /* MS5611 alternates: temperature (D2), then pressure (D1), then again */
    bool     waiting_pressure;
    bool     have_d2;
    uint32_t d2;
    bool     filt_init;
    int32_t  alt_filt_cm;
    /* IMU frozen-data detector */
    icm_raw_t last;
    uint16_t  same_count;
    /* GPS */
    uint32_t last_pvt_ms;
} s;

/* ---- barometer ------------------------------------------------------------ */
static bool baro_bringup(void)
{
    s.waiting_pressure = false;
    s.have_d2 = false;
    s.filt_init = false;
    if (ms5611_init(&baro, MS5611_ADDR) != HAL_OK)
        return false;
    return ms5611_start(&baro, false) == HAL_OK; /* temperature first */
}

static void baro_update(uint32_t d1, uint32_t d2)
{
    int32_t p, t;
    ms5611_compensate(baro.prom, d1, d2, &p, &t);
    V.pressure_pa = p;
    V.baro_temp_cdeg = (int16_t)t;
    if (p < 500 || p > 130000)
        return; /* garbage: don't turn it into an altitude */

    int32_t alt = (int32_t)(isa_altitude_m((float)p) * 100.0f);
    if (!s.filt_init) {
        s.alt_filt_cm = alt;
        s.filt_init = true;
    } else {
        /* light low-pass. At 50,000 ft one pascal is ~0.55 m, so raw noise is ~0.7 m */
        s.alt_filt_cm += (alt - s.alt_filt_cm) * 3 / 10;
    }
    V.baro_alt_cm = s.alt_filt_cm;
}

void baro_task(uint32_t now)
{
    if (s.baro_h.failed) {
        if (dev_health_retry_due(&s.baro_h, now)) {
            hal_i2c_recover(); /* in case a slave is holding SDA low */
            if (baro_bringup())
                dev_health_reset(&s.baro_h);
        }
        V.baro_ok = false; /* not until it delivers a fresh reading */
        return;
    }

    uint32_t raw = 0;
    hal_status_t st = ms5611_read_adc(&baro, &raw);
    if (st == HAL_OK) {
        if (!s.waiting_pressure) { /* that was a temperature result */
            s.d2 = raw;
            s.have_d2 = true;
            st = ms5611_start(&baro, true);
            s.waiting_pressure = true;
        } else {                   /* that was a pressure result */
            if (s.have_d2)
                baro_update(raw, s.d2);
            st = ms5611_start(&baro, false);
            s.waiting_pressure = false;
        }
    } else {
        (void)ms5611_start(&baro, false); /* start the cycle again */
        s.waiting_pressure = false;
    }
    dev_health_report(&s.baro_h, st == HAL_OK, now);
    /* only "ok" once a fresh altitude exists: after a recovery the old value
     * in V.baro_alt_cm is from before the outage and could be 300 m stale */
    V.baro_ok = !s.baro_h.failed && s.filt_init;
}

/* ---- IMU ------------------------------------------------------------------- */
void imu_task(uint32_t now)
{
    if (s.imu_h.failed) {
        if (dev_health_retry_due(&s.imu_h, now) && icm42688_init() == HAL_OK) {
            dev_health_reset(&s.imu_h);
            s.same_count = 0;
        }
        V.imu_ok = !s.imu_h.failed;
        return;
    }

    icm_raw_t r;
    hal_status_t st = icm42688_read(&r);
    if (st == HAL_OK) {
        for (int i = 0; i < 3; i++) {
            V.acc_mg[i] = icm_acc_mg(r.acc[i]);
            V.gyro_ddps[i] = icm_gyro_ddps(r.gyro[i]);
        }
        V.imu_temp_cdeg = icm_temp_cdeg(r.temp);

        /* A working MEMS sensor always has a little noise. A run of
         * bit-identical samples means the chip froze while still answering. */
        bool same = memcmp(r.acc, s.last.acc, sizeof r.acc) == 0 &&
                    memcmp(r.gyro, s.last.gyro, sizeof r.gyro) == 0;
        s.same_count = same ? (uint16_t)(s.same_count < 60000 ? s.same_count + 1 : s.same_count) : 0;
        s.last = r;
        V.imu_stuck = s.same_count >= IMU_STUCK_SAMPLES;
    }
    dev_health_report(&s.imu_h, st == HAL_OK, now);
    V.imu_ok = !s.imu_h.failed;
}

/* ---- GPS ------------------------------------------------------------------- */
void gps_task(uint32_t now)
{
    ublox_poll(now);

    ubx_pvt_t p;
    if (ublox_take_pvt(&p)) {
        s.last_pvt_ms = now;
        V.gps_fix_type = p.fix_type;
        V.gps_num_sv = p.num_sv;
        V.gps_fix = (p.fix_type == 3 || p.fix_type == 4) && (p.flags & 0x01);
        if (V.gps_fix) {
            V.lat_e7 = p.lat_e7;
            V.lon_e7 = p.lon_e7;
            V.gps_alt_cm = p.hmsl_mm / 10;
            V.gps_last_fix_ms = now;
        }
    }
    V.gps_last_msg_ms = ublox_last_msg_ms();

    if (now - s.last_pvt_ms > 1000)
        V.gps_fix = false; /* a stale fix is not a fix */

    if (now - V.gps_last_msg_ms > GPS_LINK_TIMEOUT_MS && !ublox_config_busy()) {
        /* The GPS went quiet: it probably rebooted after a power glitch and
         * forgot our settings (back to NMEA, 1 Hz, "portable"). Configure again. */
        ublox_restart_config(now);
    }
}

/* ---- start-up --------------------------------------------------------------- */
void sensors_init(uint32_t now)
{
    memset(&s, 0, sizeof s);
    s.last_pvt_ms = now;
    if (!baro_bringup())
        dev_health_fail(&s.baro_h, now);
    if (icm42688_init() != HAL_OK)
        dev_health_fail(&s.imu_h, now);
    V.baro_ok = false; /* becomes true with the first pressure reading (~40 ms) */
    V.imu_ok = !s.imu_h.failed;
    ublox_init(now);
}
