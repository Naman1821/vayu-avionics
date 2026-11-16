# FAQ: questions a reviewer would ask

**Has this flown, or run on a real board?**
No. The firmware cross-compiles for the STM32F405, and its application and driver code has flown
11 simulated two-hour missions against byte-level chip models. The register-level layer
(`firmware/target/`) is written from RM0090 and compiles, but has not executed on silicon. The
first job on a real board is bring-up against these same checks.

**Then how do you know the drivers are right?**
The drivers only speak through `hal.h`, and the chip models answer as the datasheets specify. The
models return raw MS5611 ADC counts, require write-enable before a flash program, ignore commands
while busy, NAK bad UBX settings, and so on. Datasheet examples are unit tests: the MS5611 worked
example gives 1000.09 mbar and 20.07 °C, and the u-blox 5 Hz command must end `DE 6A`. What
simulation cannot catch: electrical problems, timing on real buses, and silicon errata. That is
what bring-up is for.

**What happens if the I2C bus locks up?**
Every I2C wait has a timeout, so the call returns an error instead of hanging. After 5 errors in a
row the sensor is marked failed. Every 2 s the firmware runs bus recovery: SCL and SDA become GPIOs,
SCL is clocked up to 9 times until the slave releases SDA, then a STOP is sent and the peripheral is
reset. Then the sensor is re-initialised. In `i2c_bus_stuck` both I2C chips are back in 2.4 s.

**Why does a slave hold SDA low?**
If the master resets or glitches in the middle of a read, the slave is still part-way through
sending a byte. If the bit it is sending is a 0, it holds SDA low and waits for clocks that never
come. Nine clocks are enough to finish any byte.

**How does the watchdog work here, and where is it kicked?**
The independent watchdog runs from its own ~32 kHz oscillator, times out in ~0.5 s, and cannot be
stopped once started. It is kicked once per main-loop pass and never in an interrupt. In `cpu_hang`
the main loop stops at 50,000 ft. The MCU resets 0.50 s later, finds from the log that it was in
CRUISE, and carries on with a 0.6 s gap in the log.

**What is a brown-out, and how does the board handle one?**
The supply dips low enough that the MCU cannot run reliably. The PVD gives an interrupt at 2.9 V, a
few ms before the reset. The firmware uses that time to write a POWER_FAIL record. On reboot it reads
the reset cause (brown-out), resumes the flight phase from the log and re-configures the GPS, which
lost its settings too. In `brownout` no records are lost, and airborne mode is back 1.0 s after reboot.

**Why does the GPS need configuring at every boot?**
Without a backup supply, a u-blox receiver returns to factory settings after any power loss:
NMEA output, 1 Hz, and the "portable" dynamic model, which refuses to fix above 12 km. So the
firmware sends the settings at every boot (and whenever the GPS goes quiet for 2 s), then polls
CFG-NAV5 to confirm. `gps_rejects_airborne` shows what happens without it: no fix above 12 km.

**Explain the MS5611 second-order compensation.**
The first-order formulas correct pressure for temperature linearly. Below 20 °C the sensor's error
is non-linear, so the datasheet adds quadratic terms. Below -15 °C a second set is added. Skipping
them costs 82 m of altitude at the -20 °C launch site and about 280 m at -40 °C.

**Why does altitude need two formulas?**
The usual formula assumes temperature falls 6.5 °C per km all the way up. That is true only to
11 km. From 11 to 20 km the standard atmosphere is isothermal (-56.5 °C), so pressure falls
exponentially. The one-line formula is 257 m wrong at 50,000 ft.

**What are TEC, REC and bus-off on CAN?**
Every CAN node counts transmit and receive errors. A failed transmission adds 8 to TEC and a good
one subtracts 1. At 128 the node becomes error-passive (it can no longer send active error flags).
Above 255 it goes bus-off and stops driving the bus, so one broken node cannot jam the bus for
everyone. With ABOM set, the controller rejoins after seeing 128 x 11 recessive bits. In
`can_bus_off` the fault is flagged in 0.11 s and the bus is back 1.0 s after repair.

**Why is the heater 84 % of the energy?**
At 50,000 ft the air is -56.5 °C. Through 12 mm of foam the box loses about 4.7 W to hold +2.5 °C.
The electronics provide only 0.8 W of that. The heater makes up the remaining 3.9 W, which is more
than everything else on the board uses. See POWER_BUDGET.md.

**How did you size the heater?**
It must hold 0 °C at 50,000 ft even at the 6.6 V LOW threshold. An 11 ohm film gives 6.6² / 11 =
4.0 W at that voltage against 3.9 W needed. Below LOW, the firmware deliberately lets the box cool
to -20 °C.

**What happens when the battery runs low?**
LOW (6.6 V for 2 s): the heater only protects the battery (-20 °C) and telemetry drops to 0.5 Hz.
CRITICAL (6.2 V): the heater is off at once and logging plus radio continue. In flight the state
never improves, because shedding load makes the voltage recover and that would cause oscillation.
In `low_battery` (starting at 40 %) it goes LOW at 74 min and CRITICAL at 100 min, and logs to
landing at 6.18 V minimum.

**What was the hardest bug?**
Two, both found by the digital twin. (1) When the barometer came back after an outage, the
firmware used its last altitude from *before* the outage for one sample, which was 300 m stale.
The fix: a sensor is "ok" only once it has produced a fresh reading. (2) With a low battery the
power state flapped between LOW and CRITICAL every 9 s, because turning the heater off let the
voltage recover. The fix: in flight, the power state can only get worse.

**How is the black box protected against power loss?**
Records are written one at a time as they are made, so at most one is in flight. Each has a CRC and
a sequence number. The decoder skips torn records and proves nothing is missing. The chip is never
erased in flight. An erase command is refused unless the board is on the ground, and it also needs
a magic number.

**Why not use an SD card?**
Vibration can unseat it, FAT can corrupt on power loss, and it draws more current. NOR flash with
fixed records has none of those problems.

**What would you change for the real board?**
DMA for UART transmit (removes an ~8 ms stall per status frame). IMU at 1 kHz with its FIFO and a
proper filter. The mAh counter in battery-backed SRAM so it survives a reboot. BOR level 3 set in
the option bytes at provisioning. Board side: a 4-layer PCB with a solid ground plane, JST-GH
locking connectors, a TVS diode on the battery input, enough hold-up capacitance for the PVD
warning, and conformal coating.

**How would you test it on hardware?**
Bring-up order: power rails, clocks (scope the MCO pin), UART console, then each bus with a logic
analyser against the expected command bytes from the models. Then the same scenarios on the bench:
a freezer for the cold, a bench supply to cause brown-outs, a shorted CAN pair, a GPS simulator or
the airborne-mode read-back, and a vibration table or a drone flight. The pass criteria already
exist in `tests/test_scenarios.py`.
