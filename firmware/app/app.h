/*
 * app.h - the shared picture of the world ("blackboard").
 *
 * V holds everything the board knows right now. Each task updates its own
 * part (the baro task writes pressure, the power task writes battery volts,
 * ...) and reads whatever else it needs. It is single-threaded: only the
 * main loop touches V, never an interrupt.
 */
#ifndef VAYU_APP_H
#define VAYU_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

typedef enum {
    PHASE_BOOT = 0,
    PHASE_PREFLIGHT,
    PHASE_ASCENT,
    PHASE_CRUISE,
    PHASE_DESCENT,
    PHASE_LANDED
} phase_t;

typedef enum { POWER_NORMAL = 0, POWER_LOW, POWER_CRITICAL } power_state_t;
typedef enum { ALT_NONE = 0, ALT_BARO, ALT_GPS } alt_source_t;

/* One bit per thing that can go wrong. Sent on the radio, CAN and in the log. */
#define FAULT_BARO        (1u << 0)  /* barometer not answering             */
#define FAULT_BARO_RANGE  (1u << 1)  /* barometer outside its rated range   */
#define FAULT_IMU         (1u << 2)  /* IMU not answering                   */
#define FAULT_IMU_STUCK   (1u << 3)  /* IMU answers but data is frozen      */
#define FAULT_GPS_LINK    (1u << 4)  /* no messages from the GPS            */
#define FAULT_GPS_NOFIX   (1u << 5)  /* GPS talks but has no 3D fix         */
#define FAULT_GPS_CONFIG  (1u << 6)  /* airborne mode not confirmed         */
#define FAULT_POWER_LOW   (1u << 7)
#define FAULT_POWER_CRIT  (1u << 8)
#define FAULT_PWRMON      (1u << 9)  /* INA226 not answering                */
#define FAULT_FLASH       (1u << 10) /* log chip missing or write failed    */
#define FAULT_LOG_FULL    (1u << 11)
#define FAULT_CAN         (1u << 12) /* bus-off or error-passive            */
#define FAULT_PAYLOAD     (1u << 13) /* payload node silent                 */
#define FAULT_TEMP_LOW    (1u << 14) /* board colder than -40 C             */
#define FAULT_CLOCK       (1u << 15) /* crystal failed, on internal RC      */

typedef struct {
    /* barometer (MS5611) */
    bool     baro_ok;
    int32_t  pressure_pa;
    int16_t  baro_temp_cdeg;   /* centi-degrees C */
    int32_t  baro_alt_cm;      /* ISA altitude from pressure, filtered */

    /* IMU (ICM-42688-P) */
    bool     imu_ok;
    bool     imu_stuck;
    int16_t  acc_mg[3];
    int16_t  gyro_ddps[3];     /* deci-degrees per second */
    int16_t  imu_temp_cdeg;

    /* GPS (u-blox) */
    bool     gps_fix;          /* 3D fix right now */
    uint8_t  gps_fix_type;
    uint8_t  gps_num_sv;
    int32_t  lat_e7, lon_e7;   /* degrees x 1e7 */
    int32_t  gps_alt_cm;       /* height above mean sea level */
    uint32_t gps_last_msg_ms;
    uint32_t gps_last_fix_ms;

    /* power (INA226) */
    bool     pwrmon_ok;
    uint16_t vbat_mv;
    int16_t  ibat_ma;
    uint32_t used_uah;         /* charge drawn since boot, micro-amp-hours */
    power_state_t power;

    /* altitude and flight phase */
    alt_source_t alt_src;
    int32_t  alt_cm;           /* best altitude: baro, else GPS */
    int16_t  climb_cms;        /* vertical speed, cm/s */
    int32_t  ground_alt_cm;    /* launch-site altitude */
    phase_t  phase;
    bool     warm_restart;     /* rebooted in the air and resumed */

    /* thermal */
    bool     board_temp_valid;
    int16_t  board_temp_cdeg;
    bool     heater_on;
    uint8_t  heater_duty_pct;  /* % of the last 60 s */

    /* CAN */
    uint32_t payload_last_ms;
    uint8_t  can_bus_off_count;

    /* health */
    uint16_t faults;
    reset_cause_t reset_cause;
} vayu_t;

extern vayu_t V;

void app_init(void);
void app_step(void); /* call forever from main() */

#endif
