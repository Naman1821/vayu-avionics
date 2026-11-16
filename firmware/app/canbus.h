/*
 * canbus.h - CAN link to the payload node.
 *
 *   0x100 heartbeat  1 Hz  phase, power state, faults, uptime, reset cause, counter
 *   0x101 alt/temp   5 Hz  altitude (cm), board temperature, climb rate
 *   0x102 power      1 Hz  battery mV, mA, mAh used, heater duty, power state
 *   0x180 payload heartbeat (received): if it stops for 3 s -> FAULT_PAYLOAD
 *
 * Bus-off is handled by the controller's automatic recovery (ABOM); we count
 * the events and report them.
 */
#ifndef VAYU_CANBUS_H
#define VAYU_CANBUS_H

#include <stdint.h>

void canbus_init(uint32_t now);
void canbus_rx(uint32_t now);   /* every loop */
void canbus_task(uint32_t now); /* 5 Hz */

#endif
