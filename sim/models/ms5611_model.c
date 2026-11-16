/*
 * MS5611 model. The chip does not output pressure: it outputs two raw 24-bit
 * ADC values (D1, D2) that only make sense with its factory coefficients.
 * So for a true pressure and temperature, we search for the D1/D2 that the
 * datasheet formulas map back to them. The firmware then has to do the
 * compensation right, including the cold-weather second-order terms.
 */
#include <string.h>

#include "models.h"
#include "ms5611.h"
#include "sim.h"
#include "world.h"

/* coefficients from the datasheet's worked example */
static uint16_t prom[8] = { 0x0000, 40127, 36924, 23317, 23282, 33464, 28312, 0x4A30 };

static struct {
    enum { PTR_NONE, PTR_ADC, PTR_PROM } ptr;
    int      prom_word;
    bool     converting;
    uint64_t ready_ms;
    uint32_t result;
} m;

void ms5611_model_prom(uint16_t out[8])
{
    prom[7] = (uint16_t)((prom[7] & 0xFFF0) | ms5611_crc4(prom));
    memcpy(out, prom, sizeof prom);
}

void ms5611_model_reset(void)
{
    memset(&m, 0, sizeof m);
    prom[7] = (uint16_t)((prom[7] & 0xFFF0) | ms5611_crc4(prom));
}

void ms5611_model_encode(double p_pa, double t_c, uint32_t *d1, uint32_t *d2)
{
    int32_t want_t = (int32_t)(t_c * 100.0 + (t_c >= 0 ? 0.5 : -0.5));
    int32_t want_p = (int32_t)(p_pa + 0.5);
    int32_t p, t;

    /* temperature depends only on D2 and rises with it: binary search */
    uint32_t lo = 0, hi = 0xFFFFFF;
    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        ms5611_compensate(prom, 0, mid, &p, &t);
        if (t < want_t) lo = mid + 1; else hi = mid;
    }
    *d2 = lo;

    /* for that D2, pressure rises with D1 */
    lo = 0;
    hi = 0xFFFFFF;
    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        ms5611_compensate(prom, mid, *d2, &p, &t);
        if (p < want_p) lo = mid + 1; else hi = mid;
    }
    *d1 = lo;
}

static uint32_t convert(bool pressure)
{
    double p_true = W.p_pa + sim_noise(1.2);   /* OSR 4096 noise ~0.012 mbar */
    double t_true = W.box_c + sim_noise(0.01); /* sensor sits on the board */
    if (t_true < -45.0)
        t_true = -45.0; /* below its rating the real part gets unreliable; clamp */
    uint32_t d1, d2;
    ms5611_model_encode(p_true, t_true, &d1, &d2);
    return pressure ? d1 : d2;
}

hal_status_t ms5611_model_write(const uint8_t *d, uint16_t n)
{
    if (n != 1)
        return HAL_NACK;
    uint8_t cmd = d[0];
    if (cmd == 0x1E) { /* reset */
        memset(&m, 0, sizeof m);
    } else if (cmd >= 0x40 && cmd <= 0x58 && (cmd & 0x01) == 0) {
        bool pressure = cmd < 0x50;
        m.converting = true;
        m.result = convert(pressure);
        m.ready_ms = sim_ms + 9; /* OSR 4096: 9.04 ms max */
        m.ptr = PTR_NONE;
    } else if (cmd == 0x00) {
        m.ptr = PTR_ADC;
    } else if (cmd >= 0xA0 && cmd <= 0xAE && (cmd & 0x01) == 0) {
        m.ptr = PTR_PROM;
        m.prom_word = (cmd - 0xA0) / 2;
    } else {
        return HAL_NACK;
    }
    return HAL_OK;
}

hal_status_t ms5611_model_read(uint8_t *d, uint16_t n)
{
    if (m.ptr == PTR_PROM && n == 2) {
        d[0] = (uint8_t)(prom[m.prom_word] >> 8);
        d[1] = (uint8_t)prom[m.prom_word];
        return HAL_OK;
    }
    if (m.ptr == PTR_ADC && n == 3) {
        /* datasheet: reading before the conversion finished (or twice) gives 0 */
        uint32_t v = (m.converting && sim_ms >= m.ready_ms) ? m.result : 0;
        m.converting = false;
        d[0] = (uint8_t)(v >> 16);
        d[1] = (uint8_t)(v >> 8);
        d[2] = (uint8_t)v;
        return HAL_OK;
    }
    memset(d, 0, n);
    return HAL_OK;
}
