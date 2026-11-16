#include "icm42688.h"

static hal_status_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), value }; /* bit 7 = 0: write */
    return hal_spi_txrx(SPI_IMU, tx, 2, 0, 0);
}

static hal_status_t read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t tx = (uint8_t)(reg | 0x80); /* bit 7 = 1: read, address auto-increments */
    return hal_spi_txrx(SPI_IMU, &tx, 1, buf, len);
}

hal_status_t icm42688_init(void)
{
    hal_status_t st = write_reg(ICM_REG_DEVICE_CONFIG, 0x01); /* soft reset */
    if (st != HAL_OK)
        return st;
    hal_delay_ms(2);

    uint8_t who = 0;
    if ((st = read_regs(ICM_REG_WHO_AM_I, &who, 1)) != HAL_OK)
        return st;
    if (who != ICM_WHO_AM_I_VALUE)
        return HAL_ERROR; /* wrong chip, or MISO stuck */

    if ((st = write_reg(ICM_REG_BANK_SEL, 0)) != HAL_OK) return st;
    if ((st = write_reg(ICM_REG_GYRO_CONFIG0, (0 << 5) | 0x08)) != HAL_OK) return st;  /* 2000 dps, 100 Hz */
    if ((st = write_reg(ICM_REG_ACCEL_CONFIG0, (0 << 5) | 0x08)) != HAL_OK) return st; /* 16 g, 100 Hz */
    if ((st = write_reg(ICM_REG_PWR_MGMT0, 0x0F)) != HAL_OK) return st; /* gyro + accel low-noise mode */
    hal_delay_ms(1);
    return HAL_OK;
}

hal_status_t icm42688_read(icm_raw_t *out)
{
    uint8_t b[14];
    hal_status_t st = read_regs(ICM_REG_TEMP_DATA1, b, sizeof b);
    if (st != HAL_OK)
        return st;
    out->temp = (int16_t)((b[0] << 8) | b[1]); /* big-endian */
    for (int i = 0; i < 3; i++) {
        out->acc[i]  = (int16_t)((b[2 + 2 * i] << 8) | b[3 + 2 * i]);
        out->gyro[i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]);
    }
    /* -32768 everywhere means the sensors are off: the chip reset itself */
    if (out->acc[0] == -32768 && out->acc[1] == -32768 && out->acc[2] == -32768)
        return HAL_ERROR;
    return HAL_OK;
}

int16_t icm_acc_mg(int16_t raw)    { return (int16_t)((int32_t)raw * 1000 / ICM_ACC_LSB_PER_G); }
int16_t icm_gyro_ddps(int16_t raw) { return (int16_t)((int32_t)raw * 100 / 164); }
int16_t icm_temp_cdeg(int16_t raw) { return (int16_t)((int32_t)raw * 10000 / 13248 + 2500); }
