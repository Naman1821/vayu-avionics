/*
 * W25Q128JV 16 MB SPI NOR flash: the black-box log.
 *
 * Soldered flash instead of a microSD card: no socket to shake loose under
 * vibration, and no file system to corrupt when power drops mid-write.
 */
#ifndef VAYU_W25Q_H
#define VAYU_W25Q_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#define W25Q_PAGE_SIZE 256u

hal_status_t w25q_read_id(uint8_t id[3]);             /* expect EF 40 18 */
hal_status_t w25q_read(uint32_t addr, uint8_t *buf, uint16_t len);
bool         w25q_busy(void);                         /* program/erase in progress */
/* Program up to 256 bytes inside one page. Returns at once; the chip stays
 * busy for ~0.7 ms (3 ms max). Only 1 -> 0 bit changes are possible. */
hal_status_t w25q_program(uint32_t addr, const uint8_t *data, uint16_t len);
/* Whole chip back to 0xFF. Takes ~40 s. Only ever done on the ground. */
hal_status_t w25q_erase_chip(void);

#endif
