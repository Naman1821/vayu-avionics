/*
 * u-blox M8 GPS over UART, UBX binary protocol.
 *
 * Two jobs:
 *  1. parse NAV-PVT (position, velocity, time) messages, byte by byte;
 *  2. configure the receiver at every boot, without blocking the main loop:
 *     UBX output only, 5 Hz, NAV-PVT on, and the "airborne <1g" dynamic model.
 *     The default "portable" model refuses to give a fix above 12 km.
 *     After configuring, we read the setting back to prove it stuck.
 */
#ifndef VAYU_UBLOX_H
#define VAYU_UBLOX_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t  fix_type;     /* 0 none, 2 = 2D, 3 = 3D, 4 = GNSS + dead reckoning */
    uint8_t  flags;        /* bit 0: gnssFixOK                                   */
    uint8_t  num_sv;       /* satellites used                                    */
    int32_t  lon_e7, lat_e7;
    int32_t  hmsl_mm;      /* height above mean sea level                        */
    uint32_t h_acc_mm, v_acc_mm;
    int32_t  vel_down_mms;
} ubx_pvt_t;

void     ublox_init(uint32_t now);
void     ublox_poll(uint32_t now);          /* call every loop */
bool     ublox_take_pvt(ubx_pvt_t *out);    /* true once per new NAV-PVT */
uint32_t ublox_last_msg_ms(void);           /* time of the last valid UBX frame */
bool     ublox_configured(void);            /* airborne model read back OK */
bool     ublox_config_failed(void);
bool     ublox_config_busy(void);
void     ublox_restart_config(uint32_t now);

/* Exposed so the tests and the simulator can build frames. */
void     ubx_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);
uint16_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *out);

#define UBX_CLS_NAV     0x01
#define UBX_CLS_ACK     0x05
#define UBX_CLS_CFG     0x06
#define UBX_NAV_PVT     0x07
#define UBX_ACK_NAK     0x00
#define UBX_ACK_ACK     0x01
#define UBX_CFG_PRT     0x00
#define UBX_CFG_MSG     0x01
#define UBX_CFG_RATE    0x08
#define UBX_CFG_NAV5    0x24
#define UBX_MAX_PAYLOAD 100

#endif
