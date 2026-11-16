#include "ms5611.h"

#define CMD_RESET     0x1E
#define CMD_D1_4096   0x48 /* convert pressure,    OSR 4096 */
#define CMD_D2_4096   0x58 /* convert temperature, OSR 4096 */
#define CMD_ADC_READ  0x00
#define CMD_PROM_READ 0xA0 /* + 2 * word index */

static hal_status_t command(ms5611_t *dev, uint8_t cmd)
{
    return hal_i2c_write(dev->addr, &cmd, 1);
}

hal_status_t ms5611_init(ms5611_t *dev, uint8_t addr)
{
    dev->addr = addr;
    hal_status_t st = command(dev, CMD_RESET);
    if (st != HAL_OK)
        return st;
    hal_delay_ms(3); /* reset reloads the PROM: 2.8 ms */

    for (int i = 0; i < 8; i++) {
        uint8_t buf[2];
        st = command(dev, (uint8_t)(CMD_PROM_READ + 2 * i));
        if (st == HAL_OK)
            st = hal_i2c_read(dev->addr, buf, 2);
        if (st != HAL_OK)
            return st;
        dev->prom[i] = (uint16_t)((buf[0] << 8) | buf[1]);
    }
    if (ms5611_crc4(dev->prom) != (dev->prom[7] & 0x000F))
        return HAL_ERROR; /* corrupted calibration would give wrong altitude */
    return HAL_OK;
}

hal_status_t ms5611_start(ms5611_t *dev, bool pressure)
{
    return command(dev, pressure ? CMD_D1_4096 : CMD_D2_4096);
}

hal_status_t ms5611_read_adc(ms5611_t *dev, uint32_t *raw)
{
    uint8_t buf[3];
    hal_status_t st = command(dev, CMD_ADC_READ);
    if (st == HAL_OK)
        st = hal_i2c_read(dev->addr, buf, 3);
    if (st != HAL_OK)
        return st;
    *raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return (*raw == 0) ? HAL_ERROR : HAL_OK;
}

static void compensate(const uint16_t c[8], uint32_t d1, uint32_t d2, bool second_order,
                       int32_t *p_pa, int32_t *t_cdeg)
{
    /* Straight from the datasheet. 64-bit maths because SENS * D1 overflows 32 bits. */
    int64_t dT   = (int64_t)d2 - ((int64_t)c[5] << 8);
    int64_t temp = 2000 + (dT * c[6]) / (1LL << 23);
    int64_t off  = ((int64_t)c[2] << 16) + ((int64_t)c[4] * dT) / (1LL << 7);
    int64_t sens = ((int64_t)c[1] << 15) + ((int64_t)c[3] * dT) / (1LL << 8);

    if (second_order && temp < 2000) {           /* below 20 C */
        int64_t t2    = (dT * dT) / (1LL << 31);
        int64_t d     = temp - 2000;
        int64_t off2  = 5 * d * d / 2;
        int64_t sens2 = 5 * d * d / 4;
        if (temp < -1500) {                      /* below -15 C: our launch site is -20 C */
            int64_t e = temp + 1500;
            off2  += 7 * e * e;
            sens2 += 11 * e * e / 2;
        }
        temp -= t2;
        off  -= off2;
        sens -= sens2;
    }
    *t_cdeg = (int32_t)temp;
    *p_pa   = (int32_t)((((int64_t)d1 * sens) / (1LL << 21) - off) / (1LL << 15));
}

void ms5611_compensate(const uint16_t prom[8], uint32_t d1, uint32_t d2,
                       int32_t *p_pa, int32_t *t_cdeg)
{
    compensate(prom, d1, d2, true, p_pa, t_cdeg);
}

void ms5611_compensate_first_order(const uint16_t prom[8], uint32_t d1, uint32_t d2,
                                   int32_t *p_pa, int32_t *t_cdeg)
{
    compensate(prom, d1, d2, false, p_pa, t_cdeg);
}

uint8_t ms5611_crc4(const uint16_t prom[8])
{
    uint16_t n[8];
    for (int i = 0; i < 8; i++)
        n[i] = prom[i];
    n[7] &= 0xFF00; /* the CRC itself is not part of the calculation */

    uint16_t rem = 0;
    for (int cnt = 0; cnt < 16; cnt++) {
        rem ^= (cnt & 1) ? (uint16_t)(n[cnt >> 1] & 0x00FF) : (uint16_t)(n[cnt >> 1] >> 8);
        for (int bit = 8; bit > 0; bit--)
            rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000) : (uint16_t)(rem << 1);
    }
    return (uint8_t)((rem >> 12) & 0x000F);
}
