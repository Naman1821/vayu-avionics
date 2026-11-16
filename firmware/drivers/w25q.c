#include "w25q.h"

#include <string.h>

#define CMD_WRITE_ENABLE 0x06
#define CMD_READ_STATUS1 0x05
#define CMD_READ_DATA    0x03
#define CMD_PAGE_PROGRAM 0x02
#define CMD_CHIP_ERASE   0xC7
#define CMD_JEDEC_ID     0x9F
#define STATUS_BUSY      0x01

static hal_status_t simple(uint8_t cmd)
{
    return hal_spi_txrx(SPI_FLASH, &cmd, 1, 0, 0);
}

hal_status_t w25q_read_id(uint8_t id[3])
{
    uint8_t cmd = CMD_JEDEC_ID;
    return hal_spi_txrx(SPI_FLASH, &cmd, 1, id, 3);
}

hal_status_t w25q_read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t cmd[4] = { CMD_READ_DATA, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    return hal_spi_txrx(SPI_FLASH, cmd, 4, buf, len);
}

bool w25q_busy(void)
{
    uint8_t cmd = CMD_READ_STATUS1, status = STATUS_BUSY;
    if (hal_spi_txrx(SPI_FLASH, &cmd, 1, &status, 1) != HAL_OK)
        return true; /* can't tell: assume busy, try again later */
    return (status & STATUS_BUSY) != 0;
}

hal_status_t w25q_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > W25Q_PAGE_SIZE || (addr % W25Q_PAGE_SIZE) + len > W25Q_PAGE_SIZE)
        return HAL_ERROR; /* would wrap around inside the page */

    hal_status_t st = simple(CMD_WRITE_ENABLE);
    if (st != HAL_OK)
        return st;

    uint8_t buf[4 + W25Q_PAGE_SIZE];
    buf[0] = CMD_PAGE_PROGRAM;
    buf[1] = (uint8_t)(addr >> 16);
    buf[2] = (uint8_t)(addr >> 8);
    buf[3] = (uint8_t)addr;
    memcpy(&buf[4], data, len);
    return hal_spi_txrx(SPI_FLASH, buf, (uint16_t)(4 + len), 0, 0);
}

hal_status_t w25q_erase_chip(void)
{
    hal_status_t st = simple(CMD_WRITE_ENABLE);
    if (st == HAL_OK)
        st = simple(CMD_CHIP_ERASE);
    return st;
}
