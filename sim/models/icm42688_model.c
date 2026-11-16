/* ICM-42688-P model: SPI register file. Data registers are generated from the
 * true motion at read time, scaled by the configured full-scale range. */
#include <math.h>
#include <string.h>

#include "icm42688.h"
#include "models.h"
#include "sim.h"
#include "world.h"

static uint8_t regs[128];
static uint8_t frozen[14];
static bool    have_frozen;

void icm_model_reset(void)
{
    memset(regs, 0, sizeof regs);
    regs[ICM_REG_WHO_AM_I] = ICM_WHO_AM_I_VALUE;
    regs[ICM_REG_GYRO_CONFIG0] = 0x06;  /* power-on defaults */
    regs[ICM_REG_ACCEL_CONFIG0] = 0x06;
    have_frozen = false;
}

static void put16(uint8_t *p, double v)
{
    long x = lround(v);
    if (x > 32767) x = 32767;
    if (x < -32768) x = -32768;
    p[0] = (uint8_t)((uint16_t)x >> 8);
    p[1] = (uint8_t)x;
}

static void sample(uint8_t out[14])
{
    if ((regs[ICM_REG_PWR_MGMT0] & 0x0F) == 0) { /* sensors off: "invalid" marker */
        for (int i = 0; i < 14; i += 2) {
            out[i] = 0x80;
            out[i + 1] = 0x00;
        }
        return;
    }
    static const double acc_lsb[]  = { 2048, 4096, 8192, 16384 };    /* 16, 8, 4, 2 g */
    static const double gyro_lsb[] = { 16.4, 32.8, 65.5, 131, 262 }; /* 2000 .. 125 dps */
    int afs = (regs[ICM_REG_ACCEL_CONFIG0] >> 5) & 3;
    int gfs = (regs[ICM_REG_GYRO_CONFIG0] >> 5) & 7;
    if (gfs > 4) gfs = 4;

    bool cruising = fabs(W.vz) < 1.0 && W.alt_m > GROUND_ALT_M + 1000.0;
    bool flying = W.alt_m > GROUND_ALT_M + 1.0;
    double vib = flying ? 0.05 : 0.005; /* g, propeller vibration */
    double ax = sim_noise(0.01 + vib), ay = sim_noise(0.01 + vib);
    double az = 1.0 + W.az / 9.80665 + sim_noise(0.01 + vib);
    double gx = sim_noise(0.3), gy = sim_noise(0.3);
    double gz = (cruising ? 1.5 : 0.0) + sim_noise(0.3); /* slow orbit while loitering */
    double temp_c = W.box_c + 2.0 + sim_noise(0.05);     /* self-heating */

    put16(&out[0], (temp_c - 25.0) * 132.48);
    put16(&out[2], ax * acc_lsb[afs]);
    put16(&out[4], ay * acc_lsb[afs]);
    put16(&out[6], az * acc_lsb[afs]);
    put16(&out[8], gx * gyro_lsb[gfs]);
    put16(&out[10], gy * gyro_lsb[gfs]);
    put16(&out[12], gz * gyro_lsb[gfs]);
}

void icm_model_xfer(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    if (tx_len == 0)
        return;
    uint8_t addr = tx[0] & 0x7F;
    if (!(tx[0] & 0x80)) { /* write: following bytes go to consecutive registers */
        for (uint16_t i = 1; i < tx_len; i++, addr++) {
            if (addr == ICM_REG_DEVICE_CONFIG && (tx[i] & 0x01)) {
                icm_model_reset(); /* soft reset */
                return;
            }
            if (addr != ICM_REG_WHO_AM_I)
                regs[addr & 0x7F] = tx[i];
        }
        return;
    }
    uint8_t data[14];
    bool data_read = addr <= ICM_REG_TEMP_DATA1 + 13 && addr + rx_len > ICM_REG_TEMP_DATA1;
    if (data_read) {
        if (F.imu_stuck) {
            if (!have_frozen) {
                sample(frozen);
                have_frozen = true;
            }
            memcpy(data, frozen, sizeof data);
        } else {
            sample(data);
        }
    }
    for (uint16_t i = 0; i < rx_len; i++) {
        uint8_t a = (uint8_t)(addr + i);
        rx[i] = (a >= ICM_REG_TEMP_DATA1 && a < ICM_REG_TEMP_DATA1 + 14) ? data[a - ICM_REG_TEMP_DATA1]
                                                                        : regs[a & 0x7F];
    }
}
