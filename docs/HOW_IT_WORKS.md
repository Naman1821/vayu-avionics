# How Vayu works

A walk through the code in the order you should read it. Each section names the file, says what
it does in plain words, and lists the details worth remembering.

---

## 1. The big picture: three layers

```
            firmware/app/        what the board decides (tasks, flight phases, faults)
                 |
            firmware/drivers/    how to talk to each chip (commands, registers, maths)
                 |
            firmware/hal/hal.h   the only door to hardware: i2c, spi, uart, can, time, watchdog
           /                  \
 firmware/target/          sim/hal_sim.c + sim/models/
 real STM32F405            simulated chips (the digital twin)
 registers
```

- **app** and **drivers** never touch a register. They only call functions in `hal.h`.
- `hal.h` has two implementations. On the board, `hal_stm32f405.c` writes real registers. In the
  digital twin, `hal_sim.c` sends every transaction to a model of the chip.
- So **the same app and driver code** runs on the board and in the simulation. That is why the
  simulation tests mean something.

---

## 2. Boot: what happens at power-on

1. `target/startup.c` runs first: it copies initialised variables from flash to RAM, zeroes the
   rest, **turns on the FPU** (the altitude maths uses `logf`/`powf`) and calls `main()`.
2. `target/main.c` calls `hal_init()`, then `app_init()`, then loops forever on `app_step()`.
3. `hal_init()` (in `hal_stm32f405.c`):
   - reads **why we reset** (power-on, brown-out, watchdog...) from `RCC->CSR`, then clears it;
   - sets the clock to **168 MHz** from the 8 MHz crystal. If the crystal does not start (cold,
     cracked), it uses the internal 16 MHz RC instead and reports `FAULT_CLOCK`. It never hangs;
   - starts the 1 ms SysTick, configures pins, I2C, SPI, both UARTs, CAN, and the supply monitor (PVD).
4. `app_init()` (in `app/app.c`) starts every module, reads the black box to find out **where we
   were** (see section 6), and finally **starts the watchdog**.

---

## 3. The main loop: a cooperative scheduler, no RTOS

`app/app.c` has a small table of tasks, each with a period:

| Task | Rate | What it does |
|---|---|---|
| `imu_task` | 100 Hz | read accel, gyro, temperature; spot frozen data |
| `baro_task` | 50 Hz | MS5611: alternate temperature and pressure conversions, compute altitude |
| `power_task` | 10 Hz | INA226: battery V and I, mAh used, NORMAL/LOW/CRITICAL |
| `phase_task` | 10 Hz | choose the altitude source, climb rate, flight phase |
| `health_task` | 10 Hz | turn everything into fault bits, report changes |
| `logger_task` | 10 Hz | write a black-box record (10 Hz flying, 1 Hz on the ground) |
| `canbus_task` | 5 Hz | altitude frame at 5 Hz, heartbeat and power at 1 Hz |
| `telemetry_task` | 2 Hz | status frame to the radio (0.5 Hz when the battery is low) |
| `heater_task` | 1 Hz | thermostat |

Things that need a fast reaction run **every pass**: parsing GPS bytes, ground commands, CAN
frames, and moving queued log records into flash.

At the end of every pass the loop **kicks the watchdog**. It is never kicked in an interrupt: if
the main loop hangs, interrupts may still run, and the watchdog must still fire.

---

## 4. `V`: the blackboard

`app/app.h` defines one struct, `V`, holding everything the board knows right now: pressure,
altitude, GPS fix, battery volts, phase, fault bits, and so on. Each task writes its own part and
reads what it needs. Only the main loop touches it (no interrupts), so no locking is needed.

Fault bits (`FAULT_BARO`, `FAULT_GPS_NOFIX`, `FAULT_CAN` ...) are defined in the same file. The
same 16 bits go into the log, the radio and CAN.

---

## 5. The tasks, one by one

### Sensors (`app/sensors.c`)
- **Barometer.** The MS5611 needs about 9 ms per conversion at its best resolution (OSR 4096). The task
  runs every 20 ms: read the last result, start the next conversion. It alternates temperature
  (D2) and pressure (D1). Pressure goes through the ISA formula, then a light low-pass filter.
  `V.baro_ok` becomes true only when a **fresh** altitude exists (bug found by the simulator: after an
  outage the old altitude was 300 m stale).
- **IMU.** Burst-reads 14 bytes. A real MEMS sensor always has a bit of noise, so **100 identical
  samples in a row (1 s)** means it froze while still answering: `FAULT_IMU_STUCK`.
- **GPS.** Feeds UART bytes to the u-blox driver and copies new fixes into `V`. If the GPS is
  silent for 2 s, it assumes the GPS rebooted (power glitch) and lost its settings, so it
  **configures it again**.
- **Failed sensors** (5 bus errors in a row) are re-initialised every 2 s, with an I2C bus
  recovery first. A sensor that comes back is used again.

### Flight phase (`app/flight_phase.c`)
- **Altitude source:** barometer first, GPS if the barometer is out, otherwise keep the last value.
- **Climb rate:** the last 0.5 s of altitude minus the 0.5 s from 1.6 s earlier, divided by 1.6 s.
  Averaging the ends makes it about half as noisy as "last minus first".
- **Phases:** BOOT, PREFLIGHT, ASCENT, CRUISE, DESCENT, LANDED. Each transition has to hold for
  a few seconds (climb above 2 m/s for 3 s, level for 30 s, still for 10 s near the ground). One
  noisy reading cannot change the phase.
- The launch-site altitude is tracked only while still, so it cannot drift up with the climb.

### Health (`app/health.c`)
Builds the 16 fault bits every 100 ms from the sensor flags and the timeouts: GPS silent for 2 s,
no fix for 5 s, no payload heartbeat for 3 s. It also checks the CAN error state, the box
temperature against -40 °C, and whether the barometer is outside its 10 to 1200 mbar rating. Every
change is sent to the ground as an event.

### Power (`app/power.c`)
- INA226 every 100 ms: voltage, current, and **coulomb counting** (1 mA for 3.6 s = 1 µAh).
- **NORMAL / LOW (6.6 V) / CRITICAL (6.2 V).** It has to be below a threshold for 2 s to move
  down, so a radio TX dip is ignored.
- **In flight the state only gets worse.** Turning the heater off lets the voltage recover, and
  if that counted, the board would flip between LOW and CRITICAL forever. The simulator showed
  exactly that every 9 s. Moving back up is allowed only on the ground, after 60 s.
- When the state changes, the heater is re-evaluated **immediately**, not at its next 1 Hz slot.

### Heater (`app/heater.c`)
Thermostat with hysteresis: on below 0 °C, off above 5 °C. In LOW it only protects the battery
(-20 °C). In CRITICAL it is always off. If both temperature sensors are dead, it runs a blind 50 %
duty cycle instead of freezing or cooking. The box takes about 40 minutes to change temperature,
so a simple on/off thermostat is all it needs (see DESIGN_DECISIONS).

### Black box (`app/logger.c`)
See section 6.

### Telemetry (`app/telemetry.c`)
- Frame: `AA 55 | len | type | payload | CRC-16`. Status (39 bytes) at 2 Hz, events as they happen.
- Ground commands use the same framing. **PING** gets an answer. **ERASE_LOG** must carry a magic
  number *and* the board must be on the ground. An erase sent in flight is refused, which
  `nominal` tests.

### CAN (`app/canbus.c`)
Sends three frames (heartbeat 0x100, altitude/temperature 0x101, power 0x102). Watches the
payload node's heartbeat (0x180), counts bus-off events and reports them.

---

## 6. The black box, and how the board survives a reset in the air

- **64-byte records, appended one at a time** to the W25Q128 as soon as they are made. A crash
  loses at most the record being written. Four records fit exactly in a 256-byte flash page.
- Each record has a magic number, a **sequence number** that never repeats, and a **CRC-16**. The
  decoder skips any torn record (power lost mid-write).
- NOR flash can only turn 1s into 0s, and erasing takes 40 s. So **the chip is never erased in
  flight**. Erase is a ground command.
- **At boot, binary search** finds the first empty slot. Written slots are contiguous, so this
  takes about 18 reads for 262,144 slots. The firmware then reads the newest good record.
- **Warm restart:** if that record says ASCENT, CRUISE or DESCENT, we rebooted in the air. The
  firmware restores the phase and the launch-site altitude instead of calling the current altitude
  "the ground". The `brownout` and `cpu_hang` scenarios test this.
- **Brown-out warning:** the PVD interrupt fires when the 3.3 V rail falls below 2.9 V, a few ms
  before the reset. The interrupt only sets a flag. The main loop then writes a POWER_FAIL record.
  It does not write flash from the interrupt, because the main loop might be using the SPI bus at
  that moment.

---

## 7. The drivers (`firmware/drivers/`)

| Driver | What to remember |
|---|---|
| `ms5611.c` | Commands: reset `0x1E`, convert D1 `0x48` / D2 `0x58` (OSR 4096), ADC read `0x00`, PROM `0xA0 + 2i`. Reading the ADC before a conversion is done returns **0**, so the driver treats 0 as an error. PROM has a **CRC-4** (TE app note AN520). The compensation uses **64-bit maths** (SENS x D1 overflows 32 bits) and the **second-order terms** below 20 °C and below -15 °C. Unit-tested against the datasheet example (1000.09 mbar, 20.07 °C). |
| `ina226.c` | Register pointer, then 2 bytes big-endian. CAL = 0.00512 / (1 mA x 10 mOhm) = **512**. Bus voltage is 1.25 mV per bit. Also arms the hardware under-voltage ALERT. |
| `icm42688.c` | SPI mode 3. Bit 7 of the first byte = read. Soft reset, check WHO_AM_I = `0x47`, set ±16 g / ±2000 dps / 100 Hz, enable low-noise mode, burst-read 14 bytes from `0x1D`. All axes at -32768 means the chip reset itself. |
| `w25q.c` | JEDEC ID `EF 40 18`. Write-enable before every program. Page program up to 256 bytes without crossing a page (otherwise it wraps around). Poll the BUSY bit instead of waiting a fixed time. |
| `ublox.c` | UBX frame `B5 62 class id len payload ckA ckB` (8-bit Fletcher checksum). A byte-by-byte parser that skips NMEA text and bad checksums. A **non-blocking** configuration state machine: UBX-only output (NMEA would use ~45 % of 9600 baud), 5 Hz, NAV-PVT on, airborne model. Each step waits for ACK/NAK with 3 retries. Then it **polls CFG-NAV5 to read the setting back**: "sent" is not the same as "applied". |

---

## 8. The STM32 layer (`firmware/target/hal_stm32f405.c`)

- **Clocks:** 8 MHz crystal, PLL M=8 N=336 P=2, giving 168 MHz (Q=7 gives 48 MHz). AHB 168 MHz,
  APB1 42 MHz, APB2 84 MHz, 5 flash wait states. Every wait has a timeout, with the internal RC as fallback.
- **UART:** RX interrupt puts bytes in a 512-byte ring buffer (dropped when full). TX is polled.
  Baud register = clock / baud: 84 MHz / 9600 = 8750.
- **SPI1** at 10.5 MHz mode 3 (IMU) and **SPI2** at 21 MHz mode 0 (flash). Software chip-select.
- **I2C1** at 400 kHz (CCR = 35, TRISE = 13) using RM0090's special read sequences for 1, 2 and
  3+ bytes: the STM32F4 I2C peripheral needs the NACK and STOP set at exact moments.
- **I2C bus recovery:** take SCL/SDA as GPIOs, clock SCL up to 9 times until the slave releases
  SDA, make a STOP, reset the peripheral.
- **CAN (bxCAN):** 42 MHz / 6 = 7 MHz time quanta, 1 + 11 + 2 = 14 tq per bit, so 500 kbit/s with an
  86 % sample point. One filter in mask mode with mask 0 accepts everything. **ABOM** rejoins the
  bus automatically after bus-off.
- **Watchdog (IWDG):** LSI ~32 kHz / 32 with reload 500 gives about 0.5 s. Once started it cannot be stopped.
- **PVD** at 2.9 V on EXTI line 16, rising edge (the PVDO flag rises when VDD falls).
- **Reset cause:** check the flags in order. A power-on also sets the brown-out flag, and a
  watchdog reset also sets the pin flag.

---

## 9. The digital twin (`sim/`)

- `sim_core.c` advances time **1 ms at a time**. Each millisecond it steps the world physics, the
  chip models and the fault schedule, then calls `app_step()` (if the MCU is running). It also acts
  as the watchdog and the power supply.
- `world.c`: the flight profile, standard atmosphere, the box's temperature (one thermal mass,
  foam insulation, convection that weakens in thin air, heater, electronics heat) and the 2S
  battery (voltage vs charge, internal resistance that doubles every 15 °C colder, reduced
  capacity when cold).
- `models/`: each chip answers like its datasheet says, with realistic noise. The MS5611 model
  turns the true pressure and temperature into raw D1/D2 counts by searching the compensation
  formulas backwards. The u-blox model starts in factory settings (NMEA, 1 Hz, portable mode)
  and refuses to fix above 12 km unless it was put in airborne mode. The flash model can only
  clear bits, ignores commands while busy, and tears a record if power is lost mid-write.
- `scenarios.c`: the 11 flights and their faults.
- **Outputs** in `out/<scenario>/`: `truth.csv` (what really happened), `flash.bin` (the black
  box), `downlink.bin` (every radio byte), `events.csv` and `boots.csv`.
- `tests/test_scenarios.py` decodes the black box and the radio stream and checks them against
  the truth. Nothing peeks inside the firmware.

---

## 10. Numbers worth remembering

| | |
|---|---|
| 50,000 ft | 15,240 m, 116 mbar, -56.5 °C, air density 0.19 kg/m³ |
| 18,000 ft launch site | 5,486 m, 506 mbar, -20.7 °C |
| Firmware size | 13.9 KB flash, 2.3 KB RAM |
| Altitude formula error if you use the one-layer formula | 257 m at 50,000 ft |
| MS5611 error without second-order compensation | 82 m at -20 °C |
| u-blox portable mode limit | 12 km |
| Electronics power | 0.80 W at the battery (65 mA at 3.3 V, 140 mA at 5 V) |
| Heater needed at 50,000 ft | 3.9 W. It is 84 % of the mission energy |
| Battery | 2S 18650, 21.6 Wh, 14.5 Wh usable, 1.5x margin, 3.1 h at 50,000 ft |
| Watchdog | 0.5 s. A hang is recovered in 0.5 s with a 0.6 s gap in the log |
| CAN | 500 kbit/s, bus-off after TEC > 255, recovery after 128 x 11 recessive bits |
