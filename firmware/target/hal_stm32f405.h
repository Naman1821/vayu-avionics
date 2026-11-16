#ifndef VAYU_HAL_STM32F405_H
#define VAYU_HAL_STM32F405_H

/*
 * Pin map (STM32F405RG, 8 MHz crystal)
 *
 *   I2C1   PB6 SCL, PB7 SDA (AF4)            MS5611 0x77, INA226 0x40
 *   SPI1   PA5 SCK, PA6 MISO, PA7 MOSI (AF5) ICM-42688-P, CS = PA4, mode 3, 10.5 MHz
 *   SPI2   PB13 SCK, PB14 MISO, PB15 MOSI    W25Q128,     CS = PB12, mode 0, 21 MHz
 *   USART1 PA9 TX, PA10 RX (AF7)             u-blox GPS, 9600 baud
 *   USART2 PA2 TX, PA3 RX (AF7)              433 MHz telemetry modem, 57600 baud
 *   CAN1   PB8 RX, PB9 TX (AF9)              SN65HVD230 transceiver, 500 kbit/s
 *   PB0    heater MOSFET gate
 *   PC13   status LED
 */
void hal_init(void);

#endif
