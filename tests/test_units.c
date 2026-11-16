/*
 * test_units.c - unit tests for the pieces that are easy to get subtly wrong.
 * Known answers come from datasheets and standards, not from this code.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "altitude.h"
#include "app.h"
#include "crc.h"
#include "flight_phase.h"
#include "heater.h"
#include "models.h"
#include "ms5611.h"
#include "power.h"
#include "sim.h"
#include "telemetry.h"
#include "ublox.h"

static int checks, failures;

#define CHECK(cond, ...)                                              \
    do {                                                              \
        checks++;                                                     \
        if (!(cond)) {                                                \
            failures++;                                               \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);             \
            printf(__VA_ARGS__);                                      \
            printf("\n");                                             \
        }                                                             \
    } while (0)

/* ------------------------------------------------------------------------ */
static void test_crc(void)
{
    puts("crc16_ccitt");
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1, "standard check value");
}

static void test_ms5611_datasheet_example(void)
{
    puts("ms5611: datasheet worked example");
    const uint16_t prom[8] = { 0, 40127, 36924, 23317, 23282, 33464, 28312, 0 };
    int32_t p, t;
    ms5611_compensate(prom, 9085466, 8569150, &p, &t);
    CHECK(t == 2007, "TEMP = 2007 (20.07 C), got %d", (int)t);
    CHECK(p == 100009, "P = 100009 (1000.09 mbar), got %d", (int)p);
}

static void test_ms5611_crc4(void)
{
    puts("ms5611: PROM CRC-4");
    uint16_t prom[8];
    ms5611_model_prom(prom);
    CHECK(ms5611_crc4(prom) == (prom[7] & 0xF), "CRC of a good PROM matches");
    prom[3] ^= 0x0100;
    CHECK(ms5611_crc4(prom) != (prom[7] & 0xF), "one flipped bit is detected");
}

static void test_ms5611_cold(void)
{
    puts("ms5611: second-order compensation at the -20 C launch site");
    uint16_t prom[8];
    ms5611_model_prom(prom);
    uint32_t d1, d2;
    int32_t p2, t2, p1, t1;
    ms5611_model_encode(50600.0, -20.0, &d1, &d2); /* 18,000 ft, -20 C */
    ms5611_compensate(prom, d1, d2, &p2, &t2);
    ms5611_compensate_first_order(prom, d1, d2, &p1, &t1);
    CHECK(abs(p2 - 50600) <= 1 && abs(t2 + 2000) <= 1, "round trip: %d Pa, %d cC", (int)p2, (int)t2);
    double err_m = isa_altitude_m((float)p1) - isa_altitude_m((float)p2);
    printf("  skipping the second-order step: %+d Pa, %+.0f m altitude error\n", (int)(p1 - p2), err_m);
    CHECK(fabs(err_m) > 20.0, "first-order only is badly wrong when cold");
}

static void test_isa(void)
{
    puts("altitude: two-layer standard atmosphere");
    double h_ref = 11000.0 + 287.053 * 216.65 / 9.80665 * log(22632.06 / 11600.0);
    CHECK(fabsf(isa_altitude_m(101325.0f)) < 0.5f, "sea level -> 0 m");
    CHECK(fabsf(isa_altitude_m(22632.06f) - 11000.0f) < 1.0f, "22632 Pa -> 11,000 m");
    CHECK(fabs(isa_altitude_m(11600.0f) - h_ref) < 2.0, "11600 Pa -> %.0f m (50,000 ft)", h_ref);
    double tropo_err = troposphere_only_altitude_m(11600.0f) - h_ref;
    printf("  one-line formula at 50,000 ft: %+.0f m off\n", tropo_err);
    CHECK(fabs(tropo_err) > 200.0, "the one-layer formula is wrong in the stratosphere");
}

static void test_ubx(void)
{
    puts("ublox: checksum and parser");
    const uint8_t rate[6] = { 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00 };
    uint8_t f[16];
    uint16_t n = ubx_frame(0x06, 0x08, rate, 6, f);
    CHECK(n == 14 && f[12] == 0xDE && f[13] == 0x6A, "CFG-RATE 5 Hz ends DE 6A (u-blox docs)");

    /* NMEA junk, a corrupted frame, then a good NAV-PVT */
    sim_uart_clear();
    ublox_init(0);
    const char *nmea = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    sim_uart_push_rx(UART_GPS, (const uint8_t *)nmea, (uint16_t)strlen(nmea));
    uint8_t pvt[92] = { 0 }, frame[100];
    pvt[20] = 3; pvt[21] = 1; pvt[23] = 14;
    int32_t lon = 776000000, lat = 343000000, hmsl = 15240000;
    memcpy(&pvt[24], &lon, 4);
    memcpy(&pvt[28], &lat, 4);
    memcpy(&pvt[36], &hmsl, 4);
    n = ubx_frame(0x01, 0x07, pvt, 92, frame);
    frame[50] ^= 0xFF; /* corrupt */
    sim_uart_push_rx(UART_GPS, frame, n);
    frame[50] ^= 0xFF; /* good copy */
    sim_uart_push_rx(UART_GPS, frame, n);
    ublox_poll(1);
    ubx_pvt_t out;
    int got = 0;
    while (ublox_take_pvt(&out)) got++;
    CHECK(got == 1, "only the good frame is accepted (got %d)", got);
    CHECK(out.fix_type == 3 && out.num_sv == 14 && out.lat_e7 == lat && out.lon_e7 == lon &&
          out.hmsl_mm == hmsl, "NAV-PVT fields decoded");
}

static void test_telemetry_frame(void)
{
    puts("telemetry: frame encoding");
    uint8_t payload[3] = { 1, 2, 3 }, f[16];
    uint16_t n = tm_encode(TM_EVENT, payload, 3, f);
    uint16_t crc = (uint16_t)(f[7] | (f[8] << 8));
    CHECK(n == 9 && f[0] == 0xAA && f[1] == 0x55 && f[2] == 3 && f[3] == TM_EVENT, "header");
    CHECK(crc == crc16_ccitt(&f[2], 5), "CRC covers len, type and payload");
}

static void test_flight_phase(void)
{
    puts("flight phase: synthetic flight with +/-0.5 m altitude noise");
    log_resume_t none = { 0 };
    memset(&V, 0, sizeof V);
    phase_init(&none);
    V.baro_ok = true;

    const double ground = 5486.0;
    double alt = ground;
    uint32_t t_launch = 0, t_cruise = 0, t_desc = 0, t_land = 0, now = 0;
    phase_t last = PHASE_BOOT;
    for (int i = 0; i < 50000; i++, now += 100) {
        double t = now / 1000.0, vz;
        if (t < 120) vz = 0;            /* on the ground */
        else if (t < 1120) vz = 5;      /* climb 5 km   */
        else if (t < 1300) vz = 0;      /* level        */
        else if (alt > ground) vz = -8; /* descend      */
        else vz = 0;
        alt += vz * 0.1;
        if (alt < ground) alt = ground;
        double noise = ((i * 7919) % 200 - 100) / 200.0; /* deterministic +/-0.5 m */
        V.baro_alt_cm = (int32_t)((alt + noise) * 100);
        phase_task(now);
        if (V.phase != last) {
            if (V.phase == PHASE_ASCENT) t_launch = now;
            if (V.phase == PHASE_CRUISE) t_cruise = now;
            if (V.phase == PHASE_DESCENT) t_desc = now;
            if (V.phase == PHASE_LANDED) t_land = now;
            last = V.phase;
        }
    }
    CHECK(t_launch >= 120000 && t_launch < 126000, "launch seen 0-6 s after it happened (%u ms)", t_launch);
    CHECK(t_cruise > 1120000 && t_cruise < 1160000, "cruise ~30 s after levelling off (%u ms)", t_cruise);
    CHECK(t_desc > 1300000 && t_desc < 1306000, "descent seen within 6 s (%u ms)", t_desc);
    CHECK(t_land > 0 && V.phase == PHASE_LANDED, "landed");
}

static void test_power_state(void)
{
    puts("power: hysteresis, time filters, no recovery in flight");
    power_sm_t sm = { 0 };
    power_state_t s = POWER_NORMAL;
    uint32_t t = 0;
    for (; t < 1000; t += 100) s = power_update_state(&sm, s, 6500, t, false); /* 1 s TX dip */
    for (; t < 5000; t += 100) s = power_update_state(&sm, s, 7400, t, false);
    CHECK(s == POWER_NORMAL, "a 1 s dip is ignored");
    for (; t < 8000; t += 100) s = power_update_state(&sm, s, 6500, t, false);
    CHECK(s == POWER_LOW, "3 s below 6.6 V -> LOW");
    for (; t < 80000; t += 100) s = power_update_state(&sm, s, 7200, t, false);
    CHECK(s == POWER_LOW, "voltage recovering in flight does not undo LOW");
    for (; t < 150000; t += 100) s = power_update_state(&sm, s, 7200, t, true);
    CHECK(s == POWER_NORMAL, "on the ground, 60 s of good voltage -> NORMAL");
    for (; t < 153000; t += 100) s = power_update_state(&sm, s, 6100, t, false);
    CHECK(s == POWER_CRITICAL, "below 6.2 V -> CRITICAL");
}

static void test_heater(void)
{
    puts("heater: thermostat");
    bool blink = false;
    CHECK(heater_decide(false, true, -100, POWER_NORMAL, &blink), "on below 0 C");
    CHECK(heater_decide(true, true, 300, POWER_NORMAL, &blink), "stays on inside the band");
    CHECK(!heater_decide(true, true, 600, POWER_NORMAL, &blink), "off above 5 C");
    CHECK(!heater_decide(false, true, 300, POWER_NORMAL, &blink), "stays off inside the band");
    CHECK(!heater_decide(false, true, -1000, POWER_LOW, &blink), "low power: -10 C is acceptable");
    CHECK(heater_decide(false, true, -2100, POWER_LOW, &blink), "low power: still protects below -20 C");
    CHECK(!heater_decide(true, true, -3000, POWER_CRITICAL, &blink), "critical: always off");
    bool a = heater_decide(false, false, 0, POWER_NORMAL, &blink);
    bool b = heater_decide(a, false, 0, POWER_NORMAL, &blink);
    CHECK(a != b, "no temperature sensor: blind 50 %% duty");
}

int main(void)
{
    test_crc();
    test_ms5611_datasheet_example();
    test_ms5611_crc4();
    test_ms5611_cold();
    test_isa();
    test_ubx();
    test_telemetry_frame();
    test_flight_phase();
    test_power_state();
    test_heater();
    printf("\n%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
