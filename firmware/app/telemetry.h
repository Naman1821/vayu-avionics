/*
 * telemetry.h - radio link to the ground station (UART2 -> 433 MHz modem).
 *
 * Frame:  0xAA 0x55 | len | type | payload (len bytes) | CRC-16 (LE)
 * The CRC covers len, type and payload. Same framing both directions.
 */
#ifndef VAYU_TELEMETRY_H
#define VAYU_TELEMETRY_H

#include <stdint.h>

#define TM_SYNC1 0xAA
#define TM_SYNC2 0x55
#define TM_MAX_PAYLOAD 64

/* downlink packet types */
enum { TM_STATUS = 0x01, TM_EVENT = 0x02 };
/* uplink command types */
enum { TC_PING = 0x10, TC_ERASE_LOG = 0x11 };
#define TC_ERASE_MAGIC 0xE5A5E5A5u /* an erase command must carry this */

/* event codes (TM_EVENT) */
enum {
    EVT_BOOT = 1,         /* arg: reset cause | warm-restart << 8 */
    EVT_PHASE = 2,        /* arg: new phase                       */
    EVT_FAULTS = 3,       /* arg: new fault bits                  */
    EVT_POWER_STATE = 4,  /* arg: new power state                 */
    EVT_PONG = 5,
    EVT_ERASE_STARTED = 6,
    EVT_ERASE_REFUSED = 7,/* arg: phase we were in                */
    EVT_POWER_FAIL = 8,   /* arg: last battery mV                 */
    EVT_CAN_BUS_OFF = 9,  /* arg: bus-off count                   */
    EVT_GPS_CONFIG = 10,  /* arg: 1 = airborne mode confirmed     */
};

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    uint8_t  phase, power;
    uint16_t faults;
    int32_t  alt_cm;
    int16_t  climb_cms;
    int32_t  gps_alt_cm;
    int32_t  lat_e7, lon_e7;
    uint8_t  gps_num_sv, gps_fix;
    int16_t  board_temp_cdeg;
    uint8_t  heater_duty;
    uint16_t vbat_mv;
    int16_t  ibat_ma;
    uint16_t used_mah;
    uint8_t  reset_cause;
    uint8_t  alt_src;
} tm_status_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    uint8_t  code;
    uint16_t arg;
} tm_event_t;

/* Build a frame into out (needs len + 6 bytes). Returns the frame length. */
uint16_t tm_encode(uint8_t type, const void *payload, uint8_t len, uint8_t *out);

void telemetry_init(void);
void telemetry_task(uint32_t now);    /* 2 Hz status (0.5 Hz on low power) */
void telemetry_rx(uint32_t now);      /* every loop: ground commands       */
void telemetry_event(uint8_t code, uint16_t arg);

#endif
