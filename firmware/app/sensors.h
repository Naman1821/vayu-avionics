/*
 * sensors.h - the barometer, IMU and GPS tasks: they turn driver calls into
 * numbers in V and keep track of each sensor's health.
 */
#ifndef VAYU_SENSORS_H
#define VAYU_SENSORS_H

#include <stdint.h>

void sensors_init(uint32_t now);
void baro_task(uint32_t now); /* 50 Hz */
void imu_task(uint32_t now);  /* 100 Hz */
void gps_task(uint32_t now);  /* every loop: drains the UART buffer */

#endif
