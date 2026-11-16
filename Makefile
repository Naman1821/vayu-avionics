# Vayu - build the STM32F405 firmware, the digital twin, and run the tests.
#
#   make firmware    arm-none-eabi-gcc -> build/vayu.elf, vayu.bin, size report
#   make sim         host build of the same firmware + simulated chips
#   make unit        unit tests (datasheet examples, ISA altitude, UBX, CRC ...)
#   make scenarios   simulated flights with injected faults, pass/fail checks
#   make plots       figures for the README (docs/img)
#   make budget      power and thermal budget (docs/POWER_BUDGET.md)
#   make all         everything above

BUILD   := build
CC      ?= gcc
ARMCC   := arm-none-eabi-gcc
PYTHON  ?= python3

FW_SRC  := $(wildcard firmware/app/*.c) $(wildcard firmware/drivers/*.c)
FW_HDR  := $(wildcard firmware/app/*.h) $(wildcard firmware/drivers/*.h) firmware/hal/hal.h
INC     := -Ifirmware/app -Ifirmware/drivers -Ifirmware/hal
WARN    := -Wall -Wextra -Wshadow -Wno-unused-parameter

# ---- target --------------------------------------------------------------
TGT_SRC := $(FW_SRC) $(wildcard firmware/target/*.c)
ARMFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
            -std=c11 -Os -g $(WARN) -ffunction-sections -fdata-sections
ARMLD    := -T firmware/target/stm32f405.ld -Wl,--gc-sections -Wl,-Map=$(BUILD)/vayu.map \
            --specs=nano.specs --specs=nosys.specs -nostartfiles -lm

# ---- host ------------------------------------------------------------------
SIM_SRC  := $(FW_SRC) $(filter-out sim/sim_main.c,$(wildcard sim/*.c)) $(wildcard sim/models/*.c)
SIM_HDR  := $(wildcard sim/*.h) $(wildcard sim/models/*.h)
HOSTFLAGS := -std=c11 -O2 -g $(WARN) -D_DEFAULT_SOURCE -Isim -Isim/models

.PHONY: all firmware sim unit scenarios plots budget clean

all: firmware sim unit scenarios budget plots

firmware: $(BUILD)/vayu.elf

$(BUILD)/vayu.elf: $(TGT_SRC) $(FW_HDR) $(wildcard firmware/target/*.h) firmware/target/stm32f405.ld
	@mkdir -p $(BUILD)
	$(ARMCC) $(ARMFLAGS) $(INC) -Ifirmware/target $(TGT_SRC) $(ARMLD) -o $@
	arm-none-eabi-objcopy -O binary $@ $(BUILD)/vayu.bin
	arm-none-eabi-size $@

sim: $(BUILD)/vayu_sim

$(BUILD)/vayu_sim: $(SIM_SRC) sim/sim_main.c $(FW_HDR) $(SIM_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(HOSTFLAGS) $(INC) $(SIM_SRC) sim/sim_main.c -lm -o $@

$(BUILD)/test_units: $(SIM_SRC) tests/test_units.c $(FW_HDR) $(SIM_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(HOSTFLAGS) $(INC) $(SIM_SRC) tests/test_units.c -lm -o $@

unit: $(BUILD)/test_units
	./$(BUILD)/test_units

scenarios: sim
	$(PYTHON) tests/test_scenarios.py

budget:
	$(PYTHON) tools/power_budget.py

plots: sim
	$(PYTHON) tools/plot_flight.py

clean:
	rm -rf $(BUILD) out
