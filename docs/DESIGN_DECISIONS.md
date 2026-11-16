# Design decisions

Each choice, what it was chosen over, and why.

## Parts

**STM32F405 (Cortex-M4F, 168 MHz).** Chosen over an STM32G4 or an H7. It is the MCU inside a large
share of drone flight controllers, it has a hardware FPU for the altitude maths, and it has bxCAN,
3 SPI, 3 I2C and 6 UARTs. Its peripherals and errata are very well documented. An H7 would be
overkill for a 10 Hz logger.

**MS5611 barometer** over BMP280 / BMP388 / DPS310. Those stop at 300 mbar (~9 km). The MS5611 is
rated 10 to 1200 mbar, so it works to ~31 km. The cost is more complex maths: you get raw ADC counts
and must apply the factory coefficients, including second-order terms when cold.

**ICM-42688-P IMU at ±16 g / ±2000 dps.** Widest ranges, so prop and engine vibration does not clip
the accelerometer. Resolution is not the limit for a logger.

**W25Q128 soldered NOR flash** over a microSD card. A card socket can shake open, a card's
controller can corrupt a FAT file system on power loss, and a card draws more current. NOR flash
plus fixed-size CRC'd records has no file system to corrupt. The cost: data comes off through a
dump tool (`tools/decode_log.py`) instead of a card reader.

**INA226 on the battery line.** It measures the bus voltage (up to 36 V) and the shunt current in
one chip. The hardware ALERT pin flags under-voltage even if the firmware is busy.

**u-blox M8 over UBX** instead of parsing NMEA. Binary, checksummed, one message (NAV-PVT) has
everything, and configuration gets ACK/NAK. The M8 uses `CFG-NAV5`. An M9/M10 would use `CFG-VALSET`
with `CFG-NAVSPG-DYNMODEL` instead: same idea, different message.

**Airborne <1g dynamic model.** It allows 50 km altitude. <2g and <4g also do, and are for more
aggressive flying. A high-altitude ISR aircraft flies gently, and the tighter model gives better
filtering.

**GPS stays at 9600 baud, with NMEA turned off.** At 9600 baud the link carries ~960 bytes/s.
NAV-PVT at 5 Hz is 500 bytes/s (52 %). NMEA on top would push it to ~95 %. Raising the baud also
works, but the ACK for that change comes back at the new baud. Turning NMEA off was the simpler fix.

**CAN at 500 kbit/s with ABOM.** Standard for short UAV harnesses. Automatic bus-off management
means the controller rejoins the bus by itself once the fault clears. The firmware only counts and
reports it.

## Software

**Bare metal, register level, no vendor HAL.** Every register access is visible and explainable,
the binary is 13.9 KB, and nothing is hidden behind generated code. The cost is more care, which
is what the RM0090 sequences in the I2C code are for.

**No RTOS.** Nine periodic tasks at 1 to 100 Hz, none of which blocks for long, fit a cooperative
loop. An RTOS adds stacks, priorities and locking for no gain at this size. It would come in with
a 1 kHz IMU + FIFO + estimator.

**One shared struct (`V`), only touched by the main loop.** No locks and no races. Interrupts only
fill ring buffers or set flags.

**Timeouts on every hardware wait.** A dead chip or a stuck bus returns an error. It never hangs
the loop. The watchdog is the last line of defence, not the first.

**Watchdog kicked only at the end of the main loop.** A kick inside a timer interrupt keeps
running while the main loop is stuck, which defeats the purpose.

**Hysteresis and "must hold for N seconds" everywhere.** Phases, power states, the thermostat and
sensor health. One noisy reading must not change what the board does.

**Climb rate from averaged ends of a 2 s window.** About half the noise of "last minus first" for
the same lag. Found necessary when landing detection failed with 1 m of altitude noise.

**Two-layer standard atmosphere.** The one-line formula is 257 m wrong at 50,000 ft. The two-layer
version is exact for the standard atmosphere up to 20 km and costs one `logf`.

**Logging: one record at a time, never erase in flight, CRC + sequence per record.** At most one
record is lost to a crash. A torn record is detected and skipped. Sequence numbers prove nothing
is missing.

**Resume from the log after a reboot.** After a brown-out or watchdog reset in the air, the RAM is
gone. The newest log record says what phase we were in and where the ground is. Without this, a
reboot at 13 km would decide it was on the ground.

**Power state only gets worse in flight.** Load shedding makes the voltage recover, and if that
counted, the board oscillates (seen in the simulator: every 9 s). A battery does not recharge in
the air.

**Heater: on/off thermostat with a 5 °C band**, not PID. The box's thermal time constant is about
40 minutes. A PID controller would add tuning and nothing else. The band keeps switching to a few
cycles per hour.

**Reconfigure the GPS at every boot, then read the setting back.** A GPS without a backup supply
returns to factory settings after any power glitch. "Command sent" and "command applied" are
different things: the read-back is what proves it.

**Digital twin with byte-level chip models** over a simple "fake sensor returns a number" stub.
Models that speak the real protocols exercise the drivers: command bytes, register pointers,
PROM CRC, the MS5611's raw ADC counts, UBX ACK/NAK. A stub would only exercise the app logic.
