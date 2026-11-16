#include "health.h"

#include "app.h"
#include "config.h"
#include "logger.h"
#include "telemetry.h"
#include "ublox.h"

void dev_health_reset(dev_health_t *h)
{
    h->errors = 0;
    h->failed = false;
    h->next_retry_ms = 0;
}

void dev_health_fail(dev_health_t *h, uint32_t now)
{
    h->failed = true;
    h->next_retry_ms = now + SENSOR_RETRY_MS;
}

void dev_health_report(dev_health_t *h, bool ok, uint32_t now)
{
    if (ok) {
        h->errors = 0;
        return;
    }
    if (h->errors < 255)
        h->errors++;
    if (!h->failed && h->errors >= SENSOR_ERR_LIMIT)
        dev_health_fail(h, now);
}

bool dev_health_retry_due(dev_health_t *h, uint32_t now)
{
    if (!h->failed || (int32_t)(now - h->next_retry_ms) < 0)
        return false;
    h->next_retry_ms = now + SENSOR_RETRY_MS;
    return true;
}

void health_init(void)
{
    V.faults = 0;
}

void health_task(uint32_t now)
{
    uint16_t f = 0;

    if (!V.baro_ok) f |= FAULT_BARO;
    if (V.baro_ok && (V.pressure_pa < 1000 || V.pressure_pa > 120000 ||
                      V.baro_temp_cdeg < BOARD_TEMP_MIN_CDEG))
        f |= FAULT_BARO_RANGE; /* MS5611 is rated 10..1200 mbar, -40..+85 C */
    if (!V.imu_ok) f |= FAULT_IMU;
    if (V.imu_stuck) f |= FAULT_IMU_STUCK;

    if (now - V.gps_last_msg_ms > GPS_LINK_TIMEOUT_MS) f |= FAULT_GPS_LINK;
    if (now - V.gps_last_fix_ms > GPS_FIX_TIMEOUT_MS) f |= FAULT_GPS_NOFIX;
    if (ublox_config_failed()) f |= FAULT_GPS_CONFIG;

    if (V.power == POWER_LOW) f |= FAULT_POWER_LOW;
    if (V.power == POWER_CRITICAL) f |= FAULT_POWER_CRIT;
    if (!V.pwrmon_ok) f |= FAULT_PWRMON;

    if (!logger_ok()) f |= FAULT_FLASH;
    if (logger_full()) f |= FAULT_LOG_FULL;

    can_health_t ch;
    hal_can_health(&ch);
    if (ch.bus_off || ch.error_passive) f |= FAULT_CAN;
    if (now - V.payload_last_ms > PAYLOAD_TIMEOUT_MS) f |= FAULT_PAYLOAD;

    if (V.board_temp_valid && V.board_temp_cdeg < BOARD_TEMP_MIN_CDEG) f |= FAULT_TEMP_LOW;
    if (hal_clock_fallback()) f |= FAULT_CLOCK;

    if (f != V.faults)
        telemetry_event(EVT_FAULTS, f); /* the ground hears about every change */
    V.faults = f;
}
