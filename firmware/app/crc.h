#ifndef VAYU_CRC_H
#define VAYU_CRC_H

#include <stdint.h>

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF. crc16("123456789") = 0x29B1.
 * Used on every log record and every radio frame. */
uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);

#endif
