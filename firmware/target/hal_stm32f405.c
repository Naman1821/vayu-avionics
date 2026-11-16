/*
 * hal_stm32f405.c - hal.h on real STM32F405 registers (bare metal, no vendor HAL).
 *
 * Rules followed everywhere:
 *  - every wait on a status flag has a timeout, so a dead chip or a stuck bus
 *    returns an error instead of hanging the main loop;
 *  - interrupts only move bytes into ring buffers or set flags.
 */
#include "hal_stm32f405.h"

#include "hal.h"
#include "stm32f405.h"

#define SYSCLK_HZ   168000000u
#define APB1_HZ     42000000u
#define APB2_HZ     84000000u
#define GPS_BAUD    9600u
#define RADIO_BAUD  57600u
#define SPIN_LIMIT  200000u /* polling loops: roughly 1-5 ms at 168 MHz */

static volatile uint32_t ms_ticks;
static reset_cause_t     reset_cause;
static bool              on_internal_clock;

/* ===================================================================== */
/* clocks                                                                 */
/* ===================================================================== */

/* 168 MHz from the 8 MHz crystal. A crystal can fail to start (cold, cracked,
 * bad solder joint): then we run the same PLL from the 16 MHz internal RC and
 * report FAULT_CLOCK instead of hanging in a loop forever. */
static bool clock_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWR;
    PWR->CR |= PWR_CR_VOS; /* regulator scale 1, needed for 168 MHz */

    RCC->CR |= RCC_CR_HSEON;
    for (uint32_t n = 0; !(RCC->CR & RCC_CR_HSERDY) && n < SPIN_LIMIT; n++) {
    }
    bool hse_ok = (RCC->CR & RCC_CR_HSERDY) != 0;

    uint32_t pll;
    if (hse_ok) /* 8 MHz / M=8 * N=336 / P=2 = 168 MHz, / Q=7 = 48 MHz */
        pll = 8u | (336u << 6) | (0u << 16) | RCC_PLLCFGR_SRC_HSE | (7u << 24);
    else {      /* 16 MHz HSI / M=16, same N, P, Q */
        RCC->CR &= ~RCC_CR_HSEON;
        pll = 16u | (336u << 6) | (0u << 16) | (7u << 24);
    }
    RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_MASK) | pll;

    FLASHR->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | 5u; /* 5 wait states */
    RCC->CFGR = (0u << 4) | (5u << 10) | (4u << 13); /* AHB /1, APB1 /4 = 42 MHz, APB2 /2 = 84 MHz */

    RCC->CR |= RCC_CR_PLLON;
    for (uint32_t n = 0; !(RCC->CR & RCC_CR_PLLRDY) && n < SPIN_LIMIT; n++) {
    }
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    for (uint32_t n = 0; ((RCC->CFGR >> RCC_CFGR_SWS_SHIFT) & 3u) != RCC_CFGR_SW_PLL && n < SPIN_LIMIT; n++) {
    }
    return hse_ok;
}

void SysTick_Handler(void)
{
    ms_ticks++;
}

uint32_t hal_millis(void)
{
    return ms_ticks;
}

void hal_delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while (ms_ticks - start < ms) {
    }
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT, cycles = us * (SYSCLK_HZ / 1000000u);
    while (DWT_CYCCNT - start < cycles) {
    }
}

/* ===================================================================== */
/* GPIO helpers                                                           */
/* ===================================================================== */
static void pin_af(GPIO_TypeDef *g, uint32_t pin, uint32_t af, bool open_drain)
{
    g->MODER   = (g->MODER & ~(3u << (pin * 2))) | (2u << (pin * 2));
    g->OTYPER  = open_drain ? (g->OTYPER | (1u << pin)) : (g->OTYPER & ~(1u << pin));
    g->OSPEEDR = (g->OSPEEDR & ~(3u << (pin * 2))) | (2u << (pin * 2));
    g->PUPDR   = (g->PUPDR & ~(3u << (pin * 2))) | ((open_drain ? 1u : 0u) << (pin * 2));
    g->AFR[pin >> 3] = (g->AFR[pin >> 3] & ~(0xFu << ((pin & 7u) * 4))) | (af << ((pin & 7u) * 4));
}

static void pin_output(GPIO_TypeDef *g, uint32_t pin, bool open_drain)
{
    g->MODER  = (g->MODER & ~(3u << (pin * 2))) | (1u << (pin * 2));
    g->OTYPER = open_drain ? (g->OTYPER | (1u << pin)) : (g->OTYPER & ~(1u << pin));
}

static inline void pin_set(GPIO_TypeDef *g, uint32_t pin, bool high)
{
    g->BSRR = high ? (1u << pin) : (1u << (pin + 16));
}

/* ===================================================================== */
/* UART: interrupt RX into ring buffers, polled TX                         */
/* ===================================================================== */
#define RX_BUF 512u

typedef struct {
    volatile uint16_t head, tail;
    uint8_t buf[RX_BUF];
} ring_t;

static ring_t rx_ring[2];
static USART_TypeDef *const uart_regs[2] = { USART1, USART2 };

static void uart_init(USART_TypeDef *u, uint32_t pclk, uint32_t baud)
{
    u->CR1 = 0;
    u->BRR = (pclk + baud / 2) / baud; /* oversampling by 16 */
    u->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
}

static void uart_isr(int i)
{
    USART_TypeDef *u = uart_regs[i];
    uint32_t sr = u->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) { /* reading DR after SR also clears ORE */
        uint8_t b = (uint8_t)u->DR;
        ring_t *r = &rx_ring[i];
        uint16_t next = (uint16_t)((r->head + 1) % RX_BUF);
        if (next != r->tail) { /* full: drop the byte, the parsers resync */
            r->buf[r->head] = b;
            r->head = next;
        }
    }
}

void USART1_IRQHandler(void) { uart_isr(UART_GPS); }
void USART2_IRQHandler(void) { uart_isr(UART_RADIO); }

int hal_uart_read(uart_port_t port)
{
    ring_t *r = &rx_ring[port];
    if (r->head == r->tail)
        return -1;
    uint8_t b = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1) % RX_BUF);
    return b;
}

void hal_uart_write(uart_port_t port, const uint8_t *data, uint16_t len)
{
    /* Blocking TX keeps this simple: a 45-byte status frame takes ~8 ms at
     * 57600 baud. DMA would remove that stall (next step on real hardware). */
    USART_TypeDef *u = uart_regs[port];
    while (len--) {
        for (uint32_t n = 0; !(u->SR & USART_SR_TXE); n++)
            if (n > SPIN_LIMIT)
                return;
        u->DR = *data++;
    }
}

/* ===================================================================== */
/* SPI1 (IMU) and SPI2 (flash)                                             */
/* ===================================================================== */
static SPI_TypeDef *spi_regs(spi_dev_t d) { return d == SPI_IMU ? SPI1 : SPI2; }

static void spi_cs(spi_dev_t d, bool active)
{
    if (d == SPI_IMU)
        pin_set(GPIOA, 4, !active);
    else
        pin_set(GPIOB, 12, !active);
}

static hal_status_t spi_byte(SPI_TypeDef *s, uint8_t out, uint8_t *in)
{
    for (uint32_t n = 0; !(s->SR & SPI_SR_TXE); n++)
        if (n > SPIN_LIMIT) return HAL_TIMEOUT;
    *(__IO uint8_t *)&s->DR = out;
    for (uint32_t n = 0; !(s->SR & SPI_SR_RXNE); n++)
        if (n > SPIN_LIMIT) return HAL_TIMEOUT;
    uint8_t v = *(__IO uint8_t *)&s->DR;
    if (in)
        *in = v;
    return HAL_OK;
}

hal_status_t hal_spi_txrx(spi_dev_t dev, const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    SPI_TypeDef *s = spi_regs(dev);
    hal_status_t st = HAL_OK;
    spi_cs(dev, true);
    for (uint16_t i = 0; i < tx_len && st == HAL_OK; i++)
        st = spi_byte(s, tx[i], 0);
    for (uint16_t i = 0; i < rx_len && st == HAL_OK; i++)
        st = spi_byte(s, 0xFF, &rx[i]);
    for (uint32_t n = 0; (s->SR & SPI_SR_BSY) && n < SPIN_LIMIT; n++) {
    }
    spi_cs(dev, false);
    return st;
}

static void spi_init(void)
{
    /* SPI1 on APB2 (84 MHz) / 8 = 10.5 MHz, mode 3 for the ICM-42688-P */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (2u << SPI_CR1_BR_SHIFT) |
                SPI_CR1_CPOL | SPI_CR1_CPHA;
    SPI1->CR1 |= SPI_CR1_SPE;
    /* SPI2 on APB1 (42 MHz) / 2 = 21 MHz, mode 0 for the W25Q128 */
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (0u << SPI_CR1_BR_SHIFT);
    SPI2->CR1 |= SPI_CR1_SPE;
}

/* ===================================================================== */
/* I2C1, 400 kHz, polled, following the RM0090 master-receiver sequences  */
/* ===================================================================== */
static hal_status_t i2c_wait(uint32_t flag)
{
    for (uint32_t n = 0; !(I2C1->SR1 & flag); n++) {
        if (I2C1->SR1 & I2C_SR1_AF) { /* no ACK from the slave */
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return HAL_NACK;
        }
        if (I2C1->SR1 & (I2C_SR1_BERR | I2C_SR1_ARLO)) {
            I2C1->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO);
            return HAL_ERROR;
        }
        if (n > SPIN_LIMIT)
            return HAL_TIMEOUT;
    }
    return HAL_OK;
}

static void i2c_peripheral_init(void)
{
    pin_af(GPIOB, 6, 4, true);
    pin_af(GPIOB, 7, 4, true);
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;
    I2C1->CR2 = APB1_HZ / 1000000u;   /* 42 */
    I2C1->CCR = I2C_CCR_FS | 35u;     /* fast mode: 42 MHz / (3 * 35) = 400 kHz */
    I2C1->TRISE = 13u;                /* 300 ns * 42 MHz + 1 */
    I2C1->CR1 = I2C_CR1_PE;
}

static hal_status_t i2c_start(uint8_t addr7, bool read)
{
    for (uint32_t n = 0; (I2C1->SR2 & I2C_SR2_BUSY); n++)
        if (n > SPIN_LIMIT)
            return HAL_TIMEOUT; /* SDA held low: health will call hal_i2c_recover() */
    I2C1->CR1 |= I2C_CR1_START;
    hal_status_t st = i2c_wait(I2C_SR1_SB);
    if (st != HAL_OK)
        return st;
    I2C1->DR = (uint8_t)((addr7 << 1) | (read ? 1u : 0u));
    return i2c_wait(I2C_SR1_ADDR);
}

hal_status_t hal_i2c_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
    hal_status_t st = i2c_start(addr7, false);
    if (st == HAL_OK) {
        (void)I2C1->SR1;
        (void)I2C1->SR2; /* reading SR1 then SR2 clears ADDR */
        for (uint16_t i = 0; i < len && st == HAL_OK; i++) {
            st = i2c_wait(I2C_SR1_TXE);
            if (st == HAL_OK)
                I2C1->DR = data[i];
        }
        if (st == HAL_OK)
            st = i2c_wait(I2C_SR1_BTF);
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    return st;
}

hal_status_t hal_i2c_read(uint8_t addr7, uint8_t *data, uint16_t len)
{
    if (len == 0)
        return HAL_OK;
    I2C1->CR1 &= ~I2C_CR1_POS;
    I2C1->CR1 |= I2C_CR1_ACK;
    hal_status_t st = i2c_start(addr7, true);
    if (st != HAL_OK) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return st;
    }

    if (len == 1) { /* NACK must be set before ADDR is cleared */
        I2C1->CR1 &= ~I2C_CR1_ACK;
        irq_disable();
        (void)I2C1->SR1;
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
        irq_enable();
        st = i2c_wait(I2C_SR1_RXNE);
        if (st == HAL_OK)
            data[0] = (uint8_t)I2C1->DR;
        return st;
    }

    if (len == 2) { /* POS: the NACK applies to the second byte */
        I2C1->CR1 |= I2C_CR1_POS;
        irq_disable();
        (void)I2C1->SR1;
        (void)I2C1->SR2;
        I2C1->CR1 &= ~I2C_CR1_ACK;
        irq_enable();
        st = i2c_wait(I2C_SR1_BTF); /* byte 1 in DR, byte 2 in the shift register */
        I2C1->CR1 |= I2C_CR1_STOP;
        if (st == HAL_OK) {
            data[0] = (uint8_t)I2C1->DR;
            data[1] = (uint8_t)I2C1->DR;
        }
        I2C1->CR1 &= ~I2C_CR1_POS;
        return st;
    }

    /* 3 or more bytes */
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    uint16_t i = 0;
    while (len - i > 3 && st == HAL_OK) {
        st = i2c_wait(I2C_SR1_RXNE);
        if (st == HAL_OK)
            data[i++] = (uint8_t)I2C1->DR;
    }
    /* last three: wait for BTF so the NACK lands exactly on the final byte */
    if (st == HAL_OK)
        st = i2c_wait(I2C_SR1_BTF);
    if (st == HAL_OK) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        data[i++] = (uint8_t)I2C1->DR;          /* byte N-2 */
        st = i2c_wait(I2C_SR1_BTF);
    }
    if (st == HAL_OK) {
        I2C1->CR1 |= I2C_CR1_STOP;
        data[i++] = (uint8_t)I2C1->DR;          /* byte N-1 */
        st = i2c_wait(I2C_SR1_RXNE);
    }
    if (st == HAL_OK)
        data[i] = (uint8_t)I2C1->DR;            /* byte N */
    else
        I2C1->CR1 |= I2C_CR1_STOP;
    return st;
}

void hal_i2c_recover(void)
{
    /* Take the pins away from the peripheral and drive them by hand. */
    I2C1->CR1 &= ~I2C_CR1_PE;
    pin_output(GPIOB, 6, true);
    pin_output(GPIOB, 7, true);
    pin_set(GPIOB, 6, true);
    pin_set(GPIOB, 7, true);
    delay_us(5);

    /* Up to 9 clocks: a slave stuck mid-byte shifts out its remaining bits
     * and lets go of SDA. */
    for (int i = 0; i < 9 && !(GPIOB->IDR & (1u << 7)); i++) {
        pin_set(GPIOB, 6, false);
        delay_us(5);
        pin_set(GPIOB, 6, true);
        delay_us(5);
    }
    /* STOP condition: SDA low -> high while SCL is high */
    pin_set(GPIOB, 7, false);
    delay_us(5);
    pin_set(GPIOB, 6, true);
    delay_us(5);
    pin_set(GPIOB, 7, true);
    delay_us(5);

    i2c_peripheral_init(); /* also resets the peripheral's internal state */
}

/* ===================================================================== */
/* CAN1 (bxCAN), 500 kbit/s                                                */
/* ===================================================================== */
static void can_init(void)
{
    pin_af(GPIOB, 8, 9, false);
    pin_af(GPIOB, 9, 9, false);

    CAN1->MCR = CAN_MCR_INRQ; /* leave sleep, enter initialisation */
    for (uint32_t n = 0; !(CAN1->MSR & CAN_MSR_INAK) && n < SPIN_LIMIT; n++) {
    }
    /* ABOM: automatic bus-off recovery. TXFP: send in the order queued. */
    CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM | CAN_MCR_TXFP;
    /* 42 MHz / 6 = 7 MHz time quanta; 1 + 11 + 2 = 14 tq per bit = 500 kbit/s;
     * sample point (1 + 11) / 14 = 86 % */
    CAN1->BTR = (0u << 24) | (1u << 20) | (10u << 16) | 5u;

    /* one filter bank, 32-bit mask mode, mask 0: accept everything into FIFO 0 */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~1u;
    CAN1->FS1R |= 1u;
    CAN1->FM1R &= ~1u;
    CAN1->FFA1R &= ~1u;
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->FA1R |= 1u;
    CAN1->FMR &= ~CAN_FMR_FINIT;

    CAN1->MCR &= ~CAN_MCR_INRQ; /* join the bus (needs 11 recessive bits) */
    for (uint32_t n = 0; (CAN1->MSR & CAN_MSR_INAK) && n < SPIN_LIMIT; n++) {
    }
}

bool hal_can_send(const can_frame_t *f)
{
    if (CAN1->ESR & CAN_ESR_BOFF)
        return false;
    uint32_t tsr = CAN1->TSR;
    int mb = (tsr & CAN_TSR_TME0) ? 0 : (tsr & CAN_TSR_TME1) ? 1 : (tsr & CAN_TSR_TME2) ? 2 : -1;
    if (mb < 0)
        return false;
    const uint8_t *d = f->data;
    CAN1->sTxMailBox[mb].TDTR = f->len & 0xFu;
    CAN1->sTxMailBox[mb].TDLR = (uint32_t)d[0] | (uint32_t)d[1] << 8 | (uint32_t)d[2] << 16 | (uint32_t)d[3] << 24;
    CAN1->sTxMailBox[mb].TDHR = (uint32_t)d[4] | (uint32_t)d[5] << 8 | (uint32_t)d[6] << 16 | (uint32_t)d[7] << 24;
    CAN1->sTxMailBox[mb].TIR = ((uint32_t)(f->id & 0x7FFu) << 21) | CAN_TIR_TXRQ;
    return true;
}

bool hal_can_receive(can_frame_t *f)
{
    while (CAN1->RF0R & CAN_RF0R_FMP0) {
        uint32_t rir = CAN1->sFIFOMailBox[0].RIR;
        uint32_t lo = CAN1->sFIFOMailBox[0].RDLR, hi = CAN1->sFIFOMailBox[0].RDHR;
        uint8_t len = (uint8_t)(CAN1->sFIFOMailBox[0].RDTR & 0xFu);
        CAN1->RF0R |= CAN_RF0R_RFOM0; /* release the FIFO slot */
        if (rir & CAN_RIR_IDE)
            continue; /* extended IDs are not used on this bus */
        f->id = (uint16_t)((rir >> 21) & 0x7FFu);
        f->len = len > 8 ? 8 : len;
        for (int i = 0; i < 4; i++) {
            f->data[i] = (uint8_t)(lo >> (8 * i));
            f->data[4 + i] = (uint8_t)(hi >> (8 * i));
        }
        return true;
    }
    return false;
}

void hal_can_health(can_health_t *h)
{
    uint32_t esr = CAN1->ESR;
    h->tx_errors = (uint8_t)(esr >> 16);
    h->rx_errors = (uint8_t)(esr >> 24);
    h->error_passive = (esr & CAN_ESR_EPVF) != 0;
    h->bus_off = (esr & CAN_ESR_BOFF) != 0;
}

/* ===================================================================== */
/* outputs, watchdog, reset cause, supply monitor                          */
/* ===================================================================== */
void hal_heater(bool on) { pin_set(GPIOB, 0, on); }
void hal_led(bool on)    { pin_set(GPIOC, 13, !on); } /* LED to VDD: active low */

void hal_watchdog_start(void)
{
    /* LSI ~32 kHz / 32 = 1 kHz, reload 500 -> ~0.5 s (0.34-0.94 s over LSI tolerance) */
    IWDG->KR = 0x5555; /* unlock PR and RLR */
    IWDG->PR = 3;
    IWDG->RLR = 500;
    IWDG->KR = 0xAAAA; /* reload */
    IWDG->KR = 0xCCCC; /* start: cannot be stopped until the next reset */
}

void hal_watchdog_kick(void) { IWDG->KR = 0xAAAA; }

static void read_reset_cause(void)
{
    uint32_t csr = RCC->CSR;
    /* order matters: a power-on also sets BORRSTF, a watchdog also sets PINRSTF */
    if (csr & RCC_CSR_IWDGRSTF)      reset_cause = RESET_WATCHDOG;
    else if (csr & RCC_CSR_SFTRSTF)  reset_cause = RESET_SOFTWARE;
    else if (csr & RCC_CSR_PORRSTF)  reset_cause = RESET_POWER_ON;
    else if (csr & RCC_CSR_BORRSTF)  reset_cause = RESET_BROWN_OUT;
    else if (csr & RCC_CSR_PINRSTF)  reset_cause = RESET_PIN;
    else                             reset_cause = RESET_UNKNOWN;
    RCC->CSR |= RCC_CSR_RMVF; /* clear for next time */
}

reset_cause_t hal_reset_cause(void) { return reset_cause; }
bool hal_clock_fallback(void)       { return on_internal_clock; }

static void pvd_init(void)
{
    /* Programmable voltage detector at 2.9 V on the 3.3 V rail: an early
     * warning that power is going, a few ms before the brown-out reset. */
    PWR->CR = (PWR->CR & ~(7u << PWR_CR_PLS_SHIFT)) | (7u << PWR_CR_PLS_SHIFT) | PWR_CR_PVDE;
    EXTI->IMR |= 1u << 16;  /* PVD is EXTI line 16 */
    EXTI->RTSR |= 1u << 16; /* rising PVDO = VDD falling below the threshold */
    nvic_enable(PVD_IRQn);
}

void PVD_IRQHandler(void)
{
    EXTI->PR = 1u << 16;
    app_power_fail_isr();
}

/* ===================================================================== */
/* bring-up                                                                */
/* ===================================================================== */
void hal_init(void)
{
    read_reset_cause();
    on_internal_clock = !clock_init();

    SYST_RVR = SYSCLK_HZ / 1000u - 1u; /* 1 ms tick */
    SYST_CVR = 0;
    SYST_CSR = 7u;                     /* processor clock, interrupt, enable */

    DEMCR |= 1u << 24;                 /* enable the cycle counter for delay_us */
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1u;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOA | RCC_AHB1ENR_GPIOB | RCC_AHB1ENR_GPIOC;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2 | RCC_APB1ENR_USART2 | RCC_APB1ENR_I2C1 | RCC_APB1ENR_CAN1;
    RCC->APB2ENR |= RCC_APB2ENR_USART1 | RCC_APB2ENR_SPI1 | RCC_APB2ENR_SYSCFG;

    /* chip selects idle high, heater off, LED off */
    pin_set(GPIOA, 4, true);
    pin_output(GPIOA, 4, false);
    pin_set(GPIOB, 12, true);
    pin_output(GPIOB, 12, false);
    pin_set(GPIOB, 0, false);
    pin_output(GPIOB, 0, false);
    pin_set(GPIOC, 13, true);
    pin_output(GPIOC, 13, false);

    pin_af(GPIOA, 5, 5, false);   /* SPI1 */
    pin_af(GPIOA, 6, 5, false);
    pin_af(GPIOA, 7, 5, false);
    pin_af(GPIOB, 13, 5, false);  /* SPI2 */
    pin_af(GPIOB, 14, 5, false);
    pin_af(GPIOB, 15, 5, false);
    pin_af(GPIOA, 9, 7, false);   /* USART1 */
    pin_af(GPIOA, 10, 7, false);
    pin_af(GPIOA, 2, 7, false);   /* USART2 */
    pin_af(GPIOA, 3, 7, false);

    i2c_peripheral_init();
    spi_init();
    uart_init(USART1, APB2_HZ, GPS_BAUD);
    uart_init(USART2, APB1_HZ, RADIO_BAUD);
    can_init();
    pvd_init();

    nvic_enable(USART1_IRQn);
    nvic_enable(USART2_IRQn);
    irq_enable();
}
