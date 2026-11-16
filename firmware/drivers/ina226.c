#include "ina226.h"

static hal_status_t write_reg(uint8_t addr, uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)value };
    return hal_i2c_write(addr, buf, 3);
}

static hal_status_t read_reg(uint8_t addr, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    hal_status_t st = hal_i2c_write(addr, &reg, 1); /* set the register pointer */
    if (st == HAL_OK)
        st = hal_i2c_read(addr, buf, 2);
    if (st == HAL_OK)
        *value = (uint16_t)((buf[0] << 8) | buf[1]);
    return st;
}

hal_status_t ina226_init(uint8_t addr, uint16_t shunt_milliohm, uint16_t alert_below_mv)
{
    uint16_t id = 0;
    hal_status_t st = read_reg(addr, INA226_REG_MFG_ID, &id);
    if (st != HAL_OK)
        return st;
    if (id != 0x5449)
        return HAL_ERROR;

    /* average 16 samples, 1.1 ms conversions, shunt + bus continuous */
    const uint16_t config = 0x4000 | (0x2 << 9) | (0x4 << 6) | (0x4 << 3) | 0x7;
    /* CAL = 0.00512 / (current_lsb * R_shunt). With 1 mA per bit: 5120 / R in mOhm */
    const uint16_t cal = (uint16_t)(5120u / shunt_milliohm);
    /* bus voltage is 1.25 mV per bit */
    const uint16_t alert_limit = (uint16_t)((uint32_t)alert_below_mv * 4u / 5u);

    if ((st = write_reg(addr, INA226_REG_CONFIG, config)) != HAL_OK) return st;
    if ((st = write_reg(addr, INA226_REG_CAL, cal)) != HAL_OK) return st;
    if ((st = write_reg(addr, INA226_REG_ALERT, alert_limit)) != HAL_OK) return st;
    return write_reg(addr, INA226_REG_MASK, 1u << 12); /* BUL: bus under-voltage alert */
}

hal_status_t ina226_read(uint8_t addr, uint16_t *bus_mv, int16_t *current_ma)
{
    uint16_t bus = 0, cur = 0;
    hal_status_t st = read_reg(addr, INA226_REG_BUS, &bus);
    if (st == HAL_OK)
        st = read_reg(addr, INA226_REG_CURRENT, &cur);
    if (st != HAL_OK)
        return st;
    *bus_mv = (uint16_t)((uint32_t)bus * 5u / 4u); /* 1.25 mV per bit */
    *current_ma = (int16_t)cur;                    /* 1 mA per bit, signed */
    return HAL_OK;
}
