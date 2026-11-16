/*
 * logger.h - the black box.
 *
 * Fixed 64-byte records, appended to the W25Q128 one at a time as they are
 * made, so a crash loses at most the record being written. Each record has
 * a CRC. The chip is never erased in flight: erasing is a ground command.
 *
 * At boot we binary-search the flash for the first empty slot and read the
 * last good record. That tells us where to continue and, after a reboot in
 * the air, what phase we were in.
 */
#ifndef VAYU_LOGGER_H
#define VAYU_LOGGER_H

#include <stdbool.h>
#include <stdint.h>

#define LOG_MAGIC 0x5641 /* "VA" */

enum { REC_PERIODIC = 1, REC_BOOT = 2, REC_POWER_FAIL = 3 };

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t crc;             /* CRC-16 of bytes 4..63 */
    uint32_t seq;             /* never repeats, continues across reboots */
    uint32_t t_ms;            /* ms since this boot */
    uint8_t  type;            /* REC_* */
    uint8_t  phase;
    uint16_t faults;
    int32_t  alt_cm;          /* altitude the flight logic used */
    int32_t  gps_alt_cm;
    int32_t  lat_e7;
    int32_t  lon_e7;
    int32_t  pressure_pa;
    int32_t  ground_alt_cm;   /* lets a reboot in the air carry on */
    int16_t  board_temp_cdeg;
    uint16_t vbat_mv;
    int16_t  ibat_ma;
    int16_t  acc_mg[3];
    int16_t  gyro_ddps[3];
    int16_t  climb_cms;
    uint8_t  heater_duty;
    uint8_t  gps_num_sv;
    uint8_t  gps_fix;
    uint8_t  info;            /* bits 0-1 alt source, 2-4 reset cause, 5 heater on, 6-7 power state */
} log_record_t;

_Static_assert(sizeof(log_record_t) == 64, "log records must be 64 bytes (4 per flash page)");

typedef struct {
    bool     valid;
    uint8_t  phase;
    int32_t  ground_alt_cm;
    uint32_t seq;
} log_resume_t;

void     logger_init(log_resume_t *resume);
void     logger_task(uint32_t now);    /* 10 Hz: decides when to log */
void     logger_service(uint32_t now); /* every loop: moves queued records to flash */
void     logger_event(uint8_t type);   /* REC_BOOT, REC_POWER_FAIL */
bool     logger_ok(void);
bool     logger_full(void);
bool     logger_erase(void);           /* ground only: caller checks the phase */
uint32_t logger_next_slot(void);

#endif
