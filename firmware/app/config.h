/*
 * config.h - every tunable number in one place.
 */
#ifndef VAYU_CONFIG_H
#define VAYU_CONFIG_H

/* ---- task periods (ms) -------------------------------------------------- */
#define IMU_PERIOD_MS      10   /* 100 Hz                                       */
#define BARO_PERIOD_MS     20   /* 50 Hz, alternating pressure and temperature  */
#define POWER_PERIOD_MS    100  /* 10 Hz                                        */
#define PHASE_PERIOD_MS    100
#define HEALTH_PERIOD_MS   100
#define HEATER_PERIOD_MS   1000
#define LOG_PERIOD_MS      100  /* 10 Hz in the air, every 10th call on ground  */
#define TELEM_PERIOD_MS    500  /* 2 Hz status downlink                         */
#define CAN_PERIOD_MS      200  /* 5 Hz altitude frame, 1 Hz heartbeat + power  */

/* ---- I2C addresses ---------------------------------------------------- */
#define MS5611_ADDR 0x77 /* CSB pin tied low  */
#define INA226_ADDR 0x40 /* A0 = A1 = GND     */

/* ---- power (2S Li-ion pack) ------------------------------------------- */
#define SHUNT_MILLIOHM    10
#define VBAT_LOW_MV       6600 /* 3.30 V per cell: start shedding load     */
#define VBAT_CRITICAL_MV  6200 /* 3.10 V per cell: keep only logging alive */
#define VBAT_HYST_MV      100
#define POWER_WORSE_MS    2000 /* must stay low this long (ignore TX dips)  */
#define POWER_BETTER_MS   60000 /* on the ground only: new battery, charger */

/* ---- thermal (centi-degrees C) ------------------------------------------ */
#define HEATER_ON_BELOW_CDEG     0     /* normal: hold the box at 0..5 C        */
#define HEATER_OFF_ABOVE_CDEG    500
#define HEATER_LOW_ON_BELOW     -2000  /* low battery: only keep it above -20 C */
#define HEATER_LOW_OFF_ABOVE    -1500
#define BOARD_TEMP_MIN_CDEG     -4000  /* parts are rated down to -40 C         */

/* ---- flight phase detection ------------------------------------------- */
#define CLIMB_HISTORY_S        2      /* climb rate = altitude change over 2 s */
#define LAUNCH_CLIMB_CMS       200    /* > +2 m/s ...                          */
#define LAUNCH_CONFIRM_MS      3000   /* ... for 3 s means we launched         */
#define LAUNCH_HEIGHT_CM       5000   /* or 50 m above the launch site         */
#define DESCENT_SINK_CMS      -200    /* < -2 m/s for 3 s means descending     */
#define DESCENT_CONFIRM_MS     3000
#define LEVEL_BAND_CMS         100    /* |climb| < 1 m/s for 30 s means cruise */
#define LEVEL_CONFIRM_MS       30000
#define STILL_BAND_CMS         50     /* |climb| < 0.5 m/s for 10 s ...        */
#define STILL_CONFIRM_MS       10000
#define NEAR_GROUND_CM         30000  /* ... within 300 m of launch = landed   */

/* ---- sensor health ------------------------------------------------------ */
#define SENSOR_ERR_LIMIT     5     /* consecutive bus errors before "failed" */
#define SENSOR_RETRY_MS      2000  /* re-initialise a failed sensor this often */
#define IMU_STUCK_SAMPLES    100   /* 1 s of bit-identical samples at 100 Hz   */
#define GPS_LINK_TIMEOUT_MS  2000  /* no UBX message at all                    */
#define GPS_FIX_TIMEOUT_MS   5000  /* no 3D fix                                */
#define PAYLOAD_TIMEOUT_MS   3000  /* no heartbeat from the payload node       */

/* ---- GPS (u-blox M8, UBX protocol) -------------------------------------- */
#define GPS_RATE_MS          200   /* 5 Hz NAV-PVT                              */
#define GPS_DYN_AIRBORNE_1G  6     /* valid to 50 km. Default "portable" stops
                                      giving fixes above 12 km                  */
#define GPS_CFG_RETRIES      3
#define GPS_ACK_TIMEOUT_MS   500
#define GPS_CFG_RETRY_MS     10000 /* if configuration failed, start over      */

/* ---- CAN IDs -------------------------------------------------------------- */
#define CAN_ID_HEARTBEAT   0x100
#define CAN_ID_ALT_TEMP    0x101
#define CAN_ID_POWER       0x102
#define CAN_ID_PAYLOAD_HB  0x180 /* sent by the payload node */

/* ---- black-box log ------------------------------------------------------ */
#define FLASH_SIZE_BYTES (16u * 1024u * 1024u) /* W25Q128: 16 MB */
#define LOG_RECORD_SIZE  64u
#define LOG_SLOTS        (FLASH_SIZE_BYTES / LOG_RECORD_SIZE)
#define LOG_QUEUE_LEN    8

#endif
