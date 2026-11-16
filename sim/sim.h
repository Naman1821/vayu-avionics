/*
 * sim.h - the digital twin: simulated time, scenarios and fault switches.
 *
 * The real firmware (firmware/app + firmware/drivers) is compiled for the PC
 * and linked against sim/hal_sim.c instead of the STM32 HAL. Every I2C, SPI,
 * UART and CAN transaction it makes lands in a model of the real chip
 * (sim/models), which answers byte for byte like the datasheet says.
 */
#ifndef VAYU_SIM_H
#define VAYU_SIM_H

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

typedef struct {
    const char *name;
    const char *about;
    /* fault windows, seconds after power-on (0 = not used) */
    double baro_fail_from, baro_fail_to;   /* barometer stops acknowledging            */
    double i2c_stuck_at;                   /* a slave holds SDA low until recovered     */
    double gps_outage_from, gps_outage_to; /* no satellites (antenna, jamming)          */
    double brownout_at;                    /* connector glitch: 300 ms without power    */
    double cpu_hang_at;                    /* firmware stops running                    */
    double can_fault_from, can_fault_to;   /* CAN-H shorted: every frame fails          */
    double imu_stuck_at;                   /* IMU answers but its data freezes          */
    bool   gps_rejects_airborne;           /* receiver NAKs the airborne-mode command   */
    bool   heater_disconnected;            /* what-if: no heater                        */
    double start_soc;                      /* battery charge at start (0 = full)        */
    bool   uplink_tests;                   /* send ground commands during the flight    */
} scenario_t;

extern const scenario_t SCENARIOS[];
extern const int N_SCENARIOS;

/* live fault switches, read by hal_sim.c and the chip models */
typedef struct {
    bool baro_nack;
    bool i2c_stuck;
    bool gps_outage;
    bool can_fault;
    bool imu_stuck;
    bool gps_rejects_airborne;
    bool heater_disconnected;
} sim_faults_t;
extern sim_faults_t F;

extern uint64_t      sim_ms;          /* ms since power-on      */
extern uint64_t      sim_boot_ms;     /* when the MCU last booted */
extern reset_cause_t sim_reset_cause;
extern bool          sim_wd_armed;
extern uint64_t      sim_wd_last_kick;
extern uint32_t      sim_i2c_recoveries;

void sim_advance(uint32_t ms);                /* time passes, firmware not running */
void sim_event(const char *fmt, ...);
int  sim_run(const scenario_t *sc, const char *outdir);
const scenario_t *sim_find_scenario(const char *name);

/* UART plumbing between hal_sim.c and the models */
void sim_uart_push_rx(uart_port_t port, const uint8_t *data, uint16_t len); /* bytes reaching the MCU */
void sim_uart_clear(void);                                                 /* MCU reset: buffers lost */
void sim_downlink_write(const uint8_t *data, uint16_t len);                /* radio bytes leaving the MCU */

#endif
