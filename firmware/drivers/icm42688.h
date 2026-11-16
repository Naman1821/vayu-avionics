/*
 * ICM-42688-P 6-axis IMU over SPI (mode 3).
 * Set to +/-16 g and +/-2000 dps: the widest ranges, so engine and prop
 * vibration does not clip the accelerometer.
 */
#ifndef VAYU_ICM42688_H
#define VAYU_ICM42688_H

#include <stdint.h>

#include "hal.h"

#define ICM_REG_DEVICE_CONFIG 0x11
#define ICM_REG_TEMP_DATA1    0x1D /* first of 14 data bytes: temp, accel xyz, gyro xyz */
#define ICM_REG_PWR_MGMT0     0x4E
#define ICM_REG_GYRO_CONFIG0  0x4F
#define ICM_REG_ACCEL_CONFIG0 0x50
#define ICM_REG_WHO_AM_I      0x75
#define ICM_REG_BANK_SEL      0x76
#define ICM_WHO_AM_I_VALUE    0x47

#define ICM_ACC_LSB_PER_G     2048  /* +/-16 g   */
#define ICM_GYRO_LSB_PER_DPS  16.4f /* +/-2000 dps */

typedef struct {
    int16_t temp;
    int16_t acc[3];
    int16_t gyro[3];
} icm_raw_t;

hal_status_t icm42688_init(void);
hal_status_t icm42688_read(icm_raw_t *out);

/* unit conversions */
int16_t icm_acc_mg(int16_t raw);
int16_t icm_gyro_ddps(int16_t raw);   /* deci-degrees per second */
int16_t icm_temp_cdeg(int16_t raw);   /* centi-degrees C          */

#endif
