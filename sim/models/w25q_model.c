/* W25Q128JV model: 16 MB of NOR flash. Programming can only clear bits,
 * the chip is busy (and ignores commands) while programming or erasing, and
 * losing power mid-program leaves a half-written ("torn") record. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "models.h"
#include "sim.h"

static uint8_t *mem;
static struct {
    bool     wel, busy;
    uint64_t busy_until;
    enum { OP_NONE, OP_PROGRAM, OP_SECTOR, OP_CHIP } op;
    uint32_t addr;
    uint16_t len;
    uint8_t  data[256];
} f;

static void apply_program(uint16_t count)
{
    uint32_t page = f.addr & ~0xFFu;
    for (uint16_t i = 0; i < count; i++) {
        uint32_t a = page | ((f.addr + i) & 0xFFu); /* wraps inside the page, like the real chip */
        mem[a] &= f.data[i];                        /* NOR flash: 1 -> 0 only */
    }
}

static void finish(void)
{
    if (f.op == OP_PROGRAM) apply_program(f.len);
    else if (f.op == OP_SECTOR) memset(&mem[f.addr & ~0xFFFu], 0xFF, 4096);
    else if (f.op == OP_CHIP) memset(mem, 0xFF, FLASH_SIZE_BYTES);
    f.op = OP_NONE;
    f.busy = false;
}

void w25q_model_init(void)
{
    if (!mem)
        mem = malloc(FLASH_SIZE_BYTES);
    memset(mem, 0xFF, FLASH_SIZE_BYTES);
    memset(&f, 0, sizeof f);
}

void w25q_model_step(void)
{
    if (f.busy && sim_ms >= f.busy_until)
        finish();
}

void w25q_model_power_loss(void)
{
    if (f.busy && f.op == OP_PROGRAM) {
        apply_program(f.len / 2); /* torn write: only part of the record made it */
        sim_event("flash: power lost during a page program, record torn");
    }
    f.op = OP_NONE;
    f.busy = false;
    f.wel = false;
}

static void start(int op, uint32_t addr, uint64_t duration_ms)
{
    f.op = op;
    f.addr = addr;
    f.busy = true;
    f.busy_until = sim_ms + duration_ms;
    f.wel = false;
}

void w25q_model_xfer(const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    if (rx_len)
        memset(rx, 0xFF, rx_len);
    if (tx_len == 0)
        return;
    uint8_t cmd = tx[0];
    uint32_t addr = tx_len >= 4 ? ((uint32_t)tx[1] << 16 | (uint32_t)tx[2] << 8 | tx[3]) : 0;

    if (f.busy && cmd != 0x05)
        return; /* busy: everything but "read status" is ignored */

    switch (cmd) {
    case 0x9F: /* JEDEC ID: Winbond, W25Q128JV */
        if (rx_len >= 3) { rx[0] = 0xEF; rx[1] = 0x40; rx[2] = 0x18; }
        break;
    case 0x05:
        if (rx_len >= 1) rx[0] = (uint8_t)((f.busy ? 0x01 : 0) | (f.wel ? 0x02 : 0));
        break;
    case 0x06: f.wel = true; break;
    case 0x04: f.wel = false; break;
    case 0x03:
        for (uint16_t i = 0; i < rx_len; i++)
            rx[i] = mem[(addr + i) % FLASH_SIZE_BYTES];
        break;
    case 0x02:
        if (!f.wel || tx_len < 5) break;
        f.len = (uint16_t)(tx_len - 4 > 256 ? 256 : tx_len - 4);
        memcpy(f.data, &tx[4], f.len);
        start(OP_PROGRAM, addr % FLASH_SIZE_BYTES, 1); /* 0.7 ms typical */
        break;
    case 0x20:
        if (f.wel) start(OP_SECTOR, addr % FLASH_SIZE_BYTES, 45);
        break;
    case 0xC7:
    case 0x60:
        if (f.wel) start(OP_CHIP, 0, 40000); /* 40 s typical */
        break;
    default:
        break;
    }
}

uint8_t *w25q_model_memory(void) { return mem; }

int w25q_model_save(const char *path)
{
    /* only the used part: everything after the last non-0xFF byte is erased */
    long last = FLASH_SIZE_BYTES - 1;
    while (last >= 0 && mem[last] == 0xFF)
        last--;
    FILE *fp = fopen(path, "wb");
    if (!fp)
        return -1;
    fwrite(mem, 1, (size_t)(last + 1), fp);
    fclose(fp);
    return 0;
}
