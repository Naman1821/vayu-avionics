/*
 * MS5611-01BA03 barometer over I2C.
 *
 * Why this part: it measures 10 to 1200 mbar. The popular BMP280 stops at
 * 300 mbar (about 9 km), well below a 50,000 ft (15.2 km, ~116 mbar) ceiling.
 */
#ifndef VAYU_MS5611_H
#define VAYU_MS5611_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

typedef struct {
    uint8_t  addr;
    uint16_t prom[8]; /* prom[1]..prom[6] are the factory calibration C1..C6 */
} ms5611_t;

/* Reset, read the 8 PROM words and check their CRC-4. */
hal_status_t ms5611_init(ms5611_t *dev, uint8_t addr);

/* Start a conversion at OSR 4096 (takes up to 9.04 ms).
 * pressure = true: D1 (pressure), false: D2 (temperature). */
hal_status_t ms5611_start(ms5611_t *dev, bool pressure);

/* Read the 24-bit result. The chip returns 0 if the conversion is not done:
 * we report that as HAL_ERROR rather than using a bogus zero. */
hal_status_t ms5611_read_adc(ms5611_t *dev, uint32_t *raw);

/* Datasheet compensation, including the second-order terms that matter
 * below 20 C and again below -15 C. Output: Pa and centi-degrees C. */
void ms5611_compensate(const uint16_t prom[8], uint32_t d1, uint32_t d2,
                       int32_t *p_pa, int32_t *t_cdeg);

/* Same, but first-order only. Exists to measure what skipping step 2 costs. */
void ms5611_compensate_first_order(const uint16_t prom[8], uint32_t d1, uint32_t d2,
                                   int32_t *p_pa, int32_t *t_cdeg);

/* CRC-4 from TE application note AN520, stored in the low nibble of word 7. */
uint8_t ms5611_crc4(const uint16_t prom[8]);

#endif
