/*
 * hal_sim.c - hal.h implemented on simulated chips. This file is the
 * "wiring diagram" of the digital twin: which bus and address goes where.
 */
#include <string.h>

#include "config.h"
#include "hal.h"
#include "models.h"
#include "sim.h"
#include "world.h"

#define RX_BUF 512 /* same size as on the STM32 */

typedef struct {
    uint16_t head, tail;
    uint8_t  buf[RX_BUF];
    uint32_t dropped;
} ring_t;

static ring_t rings[2];

/* ---- time -------------------------------------------------------------- */
uint32_t hal_millis(void)          { return (uint32_t)(sim_ms - sim_boot_ms); }
void     hal_delay_ms(uint32_t ms) { sim_advance(ms); }

/* ---- I2C1 ----------------------------------------------------------------- */
hal_status_t hal_i2c_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
    if (F.i2c_stuck)
        return HAL_TIMEOUT; /* SDA held low: BUSY never clears */
    if (addr7 == MS5611_ADDR)
        return F.baro_nack ? HAL_NACK : ms5611_model_write(data, len);
    if (addr7 == INA226_ADDR)
        return ina226_model_write(data, len);
    return HAL_NACK;
}

hal_status_t hal_i2c_read(uint8_t addr7, uint8_t *data, uint16_t len)
{
    if (F.i2c_stuck)
        return HAL_TIMEOUT;
    if (addr7 == MS5611_ADDR)
        return F.baro_nack ? HAL_NACK : ms5611_model_read(data, len);
    if (addr7 == INA226_ADDR)
        return ina226_model_read(data, len);
    return HAL_NACK;
}

void hal_i2c_recover(void)
{
    sim_i2c_recoveries++;
    if (F.i2c_stuck) {
        F.i2c_stuck = false; /* the 9 clocks let the slave finish its byte */
        sim_event("I2C: bus recovered by 9 clock pulses + STOP");
    }
}

/* ---- SPI ------------------------------------------------------------------ */
hal_status_t hal_spi_txrx(spi_dev_t dev, const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    if (dev == SPI_IMU)
        icm_model_xfer(tx, tx_len, rx, rx_len);
    else
        w25q_model_xfer(tx, tx_len, rx, rx_len);
    return HAL_OK;
}

/* ---- UART -------------------------------------------------------------------- */
void sim_uart_push_rx(uart_port_t port, const uint8_t *data, uint16_t len)
{
    ring_t *r = &rings[port];
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((r->head + 1) % RX_BUF);
        if (next == r->tail) { /* full: byte lost, like the real ISR */
            r->dropped++;
            continue;
        }
        r->buf[r->head] = data[i];
        r->head = next;
    }
}

void sim_uart_clear(void)
{
    memset(rings, 0, sizeof rings);
}

int hal_uart_read(uart_port_t port)
{
    ring_t *r = &rings[port];
    if (r->head == r->tail)
        return -1;
    uint8_t b = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1) % RX_BUF);
    return b;
}

void hal_uart_write(uart_port_t port, const uint8_t *data, uint16_t len)
{
    if (port == UART_GPS)
        ublox_model_rx(data, len);
    else
        sim_downlink_write(data, len);
}

/* ---- CAN ---------------------------------------------------------------------- */
bool hal_can_send(const can_frame_t *f)    { return can_model_send(f); }
bool hal_can_receive(can_frame_t *f)       { return can_model_receive(f); }
void hal_can_health(can_health_t *h)       { can_model_health(h); }

/* ---- outputs and reliability ------------------------------------------------------ */
void hal_heater(bool on) { W.heater_cmd = on; }
void hal_led(bool on)    { (void)on; }

reset_cause_t hal_reset_cause(void) { return sim_reset_cause; }
bool hal_clock_fallback(void)       { return false; }

void hal_watchdog_start(void)
{
    sim_wd_armed = true;
    sim_wd_last_kick = sim_ms;
}

void hal_watchdog_kick(void)
{
    sim_wd_last_kick = sim_ms;
}
