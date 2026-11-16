/*
 * hal.h - the only door between Vayu's firmware and the hardware.
 *
 * Everything in firmware/app and firmware/drivers is plain C: no registers,
 * no vendor libraries. It talks to the outside world only through the
 * functions below. There are two implementations of this file:
 *
 *   firmware/target/hal_stm32f405.c  real STM32F405 registers (the board)
 *   sim/hal_sim.c                    simulated chips (the digital twin)
 *
 * Because both implement the same functions, the exact same app and driver
 * code runs on the board and in the simulation.
 */
#ifndef VAYU_HAL_H
#define VAYU_HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERROR,   /* bus error or unexpected state                      */
    HAL_TIMEOUT, /* a flag never came; we gave up instead of hanging   */
    HAL_NACK     /* I2C device did not acknowledge its address or data */
} hal_status_t;

/* ---- time ---------------------------------------------------------- */
uint32_t hal_millis(void);          /* milliseconds since this boot    */
void     hal_delay_ms(uint32_t ms); /* only used during start-up       */

/* ---- I2C1 @ 400 kHz: MS5611 barometer, INA226 power monitor -------- */
hal_status_t hal_i2c_write(uint8_t addr7, const uint8_t *data, uint16_t len);
hal_status_t hal_i2c_read(uint8_t addr7, uint8_t *data, uint16_t len);
/* A slave that lost power mid-byte can hold SDA low forever. This clocks
 * SCL nine times so it finishes its byte, sends a STOP and resets I2C1. */
void hal_i2c_recover(void);

/* ---- SPI: chip-select is held low for the whole call ---------------- */
typedef enum { SPI_IMU = 0, SPI_FLASH = 1 } spi_dev_t;
/* Send tx_len bytes, then clock in rx_len bytes (sending 0xFF). */
hal_status_t hal_spi_txrx(spi_dev_t dev, const uint8_t *tx, uint16_t tx_len,
                          uint8_t *rx, uint16_t rx_len);

/* ---- UART: RX is interrupt-driven into a ring buffer ---------------- */
typedef enum { UART_GPS = 0, UART_RADIO = 1 } uart_port_t;
void hal_uart_write(uart_port_t port, const uint8_t *data, uint16_t len);
int  hal_uart_read(uart_port_t port); /* next received byte, or -1 if none */

/* ---- CAN1 @ 500 kbit/s, standard 11-bit IDs ------------------------- */
typedef struct {
    uint16_t id;
    uint8_t  len;
    uint8_t  data[8];
} can_frame_t;

typedef struct {
    uint8_t tx_errors;  /* transmit error counter (TEC) */
    uint8_t rx_errors;  /* receive error counter (REC)  */
    bool    error_passive;
    bool    bus_off;
} can_health_t;

bool hal_can_send(const can_frame_t *f); /* false: no free mailbox or bus-off */
bool hal_can_receive(can_frame_t *f);    /* false: nothing waiting           */
void hal_can_health(can_health_t *h);

/* ---- outputs ---------------------------------------------------------- */
void hal_heater(bool on); /* MOSFET driving the battery/board heater film */
void hal_led(bool on);

/* ---- reliability ------------------------------------------------------ */
typedef enum {
    RESET_POWER_ON = 0,
    RESET_BROWN_OUT,
    RESET_WATCHDOG,
    RESET_SOFTWARE,
    RESET_PIN,
    RESET_UNKNOWN
} reset_cause_t;

reset_cause_t hal_reset_cause(void);
bool hal_clock_fallback(void);  /* true: crystal failed, running on internal RC */
void hal_watchdog_start(void);  /* ~0.5 s independent watchdog, cannot be stopped */
void hal_watchdog_kick(void);

/* Called by the HAL from the supply monitor (PVD) interrupt when VDD starts
 * to fall. Implemented in app.c. It only sets a flag. */
void app_power_fail_isr(void);

#endif
