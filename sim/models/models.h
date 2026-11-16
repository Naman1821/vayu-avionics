/*
 * models.h - software models of the chips on the board.
 *
 * Each model answers the firmware's bus transactions the way the real part
 * does (register maps, command bytes, timing, quirks like the MS5611
 * returning 0 if you read before a conversion is done), using the "true"
 * world state from world.c plus realistic noise.
 */
#ifndef VAYU_MODELS_H
#define VAYU_MODELS_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

/* MS5611 barometer (I2C 0x77) */
void         ms5611_model_reset(void);
hal_status_t ms5611_model_write(const uint8_t *d, uint16_t n);
hal_status_t ms5611_model_read(uint8_t *d, uint16_t n);
void         ms5611_model_prom(uint16_t prom[8]);
void         ms5611_model_encode(double p_pa, double t_c, uint32_t *d1, uint32_t *d2);

/* INA226 power monitor (I2C 0x40) */
void         ina226_model_reset(void);
hal_status_t ina226_model_write(const uint8_t *d, uint16_t n);
hal_status_t ina226_model_read(uint8_t *d, uint16_t n);

/* ICM-42688-P IMU (SPI1) */
void icm_model_reset(void);
void icm_model_xfer(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len);

/* W25Q128 flash (SPI2) */
void     w25q_model_init(void);
void     w25q_model_xfer(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len);
void     w25q_model_step(void);
void     w25q_model_power_loss(void);
uint8_t *w25q_model_memory(void);
int      w25q_model_save(const char *path);

/* u-blox M8 GPS (USART1) */
void ublox_model_power(bool on, bool cold_start);
void ublox_model_rx(const uint8_t *d, uint16_t n); /* bytes from the MCU */
void ublox_model_step(void);                        /* every ms: may send bytes */
bool ublox_model_has_fix(void);
int  ublox_model_dyn_model(void);

/* CAN bus: our controller + the payload node */
void can_model_reset(void);
bool can_model_send(const can_frame_t *f);
bool can_model_receive(can_frame_t *f);
void can_model_health(can_health_t *h);
void can_model_step(void);

/* shared: gaussian noise */
double sim_noise(double sigma);

#endif
