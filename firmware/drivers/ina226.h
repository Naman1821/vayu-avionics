/*
 * INA226 current / voltage monitor over I2C, on the battery line.
 * With a 10 mOhm shunt and 1 mA per bit it reads up to 8.19 A.
 */
#ifndef VAYU_INA226_H
#define VAYU_INA226_H

#include <stdint.h>

#include "hal.h"

#define INA226_REG_CONFIG  0x00
#define INA226_REG_SHUNT   0x01
#define INA226_REG_BUS     0x02
#define INA226_REG_POWER   0x03
#define INA226_REG_CURRENT 0x04
#define INA226_REG_CAL     0x05
#define INA226_REG_MASK    0x06
#define INA226_REG_ALERT   0x07
#define INA226_REG_MFG_ID  0xFE /* reads 0x5449, "TI" */

/* Check the ID, set averaging and calibration, and arm the hardware
 * under-voltage ALERT pin so a dying battery is flagged even if the
 * firmware is busy. */
hal_status_t ina226_init(uint8_t addr, uint16_t shunt_milliohm, uint16_t alert_below_mv);

hal_status_t ina226_read(uint8_t addr, uint16_t *bus_mv, int16_t *current_ma);

#endif
