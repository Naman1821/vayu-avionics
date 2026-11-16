/* INA226 model: register file with a pointer register, values from the battery model. */
#include <math.h>
#include <string.h>

#include "ina226.h"
#include "models.h"
#include "world.h"

#define SHUNT_OHMS 0.010

static struct {
    uint8_t  ptr;
    uint16_t config, cal, mask, alert;
} r;

void ina226_model_reset(void)
{
    memset(&r, 0, sizeof r);
    r.config = 0x4127; /* power-on default */
}

static int16_t shunt_raw(void)
{
    double i = W.ibat + sim_noise(0.002);
    return (int16_t)lround(i * SHUNT_OHMS / 2.5e-6); /* 2.5 uV per bit */
}

static uint16_t reg_value(uint8_t reg)
{
    switch (reg) {
    case INA226_REG_CONFIG: return r.config;
    case INA226_REG_SHUNT: return (uint16_t)shunt_raw();
    case INA226_REG_BUS: {
        double v = W.vbat + sim_noise(0.002);
        return (uint16_t)lround((v < 0 ? 0 : v) / 1.25e-3); /* 1.25 mV per bit */
    }
    case INA226_REG_CURRENT: return (uint16_t)(int16_t)((int32_t)shunt_raw() * r.cal / 2048);
    case INA226_REG_POWER: {
        int32_t cur = (int32_t)shunt_raw() * r.cal / 2048;
        int32_t bus = (int32_t)lround(W.vbat / 1.25e-3);
        return (uint16_t)(cur * bus / 20000);
    }
    case INA226_REG_CAL: return r.cal;
    case INA226_REG_MASK: return r.mask;
    case INA226_REG_ALERT: return r.alert;
    case INA226_REG_MFG_ID: return 0x5449;
    case 0xFF: return 0x2260; /* die ID */
    default: return 0;
    }
}

hal_status_t ina226_model_write(const uint8_t *d, uint16_t n)
{
    if (n < 1)
        return HAL_NACK;
    r.ptr = d[0];
    if (n == 3) {
        uint16_t v = (uint16_t)((d[1] << 8) | d[2]);
        switch (d[0]) {
        case INA226_REG_CONFIG:
            if (v & 0x8000) ina226_model_reset(); /* RST bit */
            else r.config = v;
            break;
        case INA226_REG_CAL: r.cal = v & 0x7FFF; break;
        case INA226_REG_MASK: r.mask = v; break;
        case INA226_REG_ALERT: r.alert = v; break;
        default: return HAL_NACK;
        }
    }
    return HAL_OK;
}

hal_status_t ina226_model_read(uint8_t *d, uint16_t n)
{
    uint16_t v = reg_value(r.ptr);
    if (n >= 1) d[0] = (uint8_t)(v >> 8);
    if (n >= 2) d[1] = (uint8_t)v;
    return HAL_OK;
}
