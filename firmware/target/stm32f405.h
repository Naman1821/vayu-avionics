/*
 * stm32f405.h - just the STM32F405 registers this project uses.
 *
 * Written by hand from the reference manual (RM0090) instead of pulling in
 * the vendor HAL, so every register access in hal_stm32f405.c is visible
 * and explainable. Addresses and bit positions are from RM0090.
 */
#ifndef VAYU_STM32F405_H
#define VAYU_STM32F405_H

#include <stdint.h>

#define __IO volatile

typedef struct { __IO uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2]; } GPIO_TypeDef;

typedef struct {
    __IO uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR;
    uint32_t RESERVED0;
    __IO uint32_t APB1RSTR, APB2RSTR;
    uint32_t RESERVED1[2];
    __IO uint32_t AHB1ENR, AHB2ENR, AHB3ENR;   /* 0x30 */
    uint32_t RESERVED2;
    __IO uint32_t APB1ENR, APB2ENR;            /* 0x40, 0x44 */
    uint32_t RESERVED3[2];
    __IO uint32_t AHB1LPENR, AHB2LPENR, AHB3LPENR;
    uint32_t RESERVED4;
    __IO uint32_t APB1LPENR, APB2LPENR;
    uint32_t RESERVED5[2];
    __IO uint32_t BDCR, CSR;                   /* 0x70, 0x74 */
} RCC_TypeDef;

typedef struct { __IO uint32_t ACR, KEYR, OPTKEYR, SR, CR, OPTCR; } FLASH_TypeDef;
typedef struct { __IO uint32_t CR, CSR; } PWR_TypeDef;
typedef struct { __IO uint32_t KR, PR, RLR, SR; } IWDG_TypeDef;
typedef struct { __IO uint32_t IMR, EMR, RTSR, FTSR, SWIER, PR; } EXTI_TypeDef;
typedef struct { __IO uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR; } USART_TypeDef;
typedef struct { __IO uint32_t CR1, CR2, SR, DR, CRCPR, RXCRCR, TXCRCR, I2SCFGR, I2SPR; } SPI_TypeDef;
typedef struct { __IO uint32_t CR1, CR2, OAR1, OAR2, DR, SR1, SR2, CCR, TRISE, FLTR; } I2C_TypeDef;

typedef struct { __IO uint32_t TIR, TDTR, TDLR, TDHR; } CAN_TxMailBox_TypeDef;
typedef struct { __IO uint32_t RIR, RDTR, RDLR, RDHR; } CAN_FIFOMailBox_TypeDef;
typedef struct { __IO uint32_t FR1, FR2; } CAN_FilterRegister_TypeDef;
typedef struct {
    __IO uint32_t MCR, MSR, TSR, RF0R, RF1R, IER, ESR, BTR;  /* 0x000 - 0x01C */
    uint32_t RESERVED0[88];
    CAN_TxMailBox_TypeDef   sTxMailBox[3];                   /* 0x180 */
    CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];                 /* 0x1B0 */
    uint32_t RESERVED1[12];
    __IO uint32_t FMR, FM1R;                                 /* 0x200 */
    uint32_t RESERVED2;
    __IO uint32_t FS1R;
    uint32_t RESERVED3;
    __IO uint32_t FFA1R;
    uint32_t RESERVED4;
    __IO uint32_t FA1R;
    uint32_t RESERVED5[8];
    CAN_FilterRegister_TypeDef sFilterRegister[28];          /* 0x240 */
} CAN_TypeDef;

/* ---- base addresses ---------------------------------------------------------- */
#define GPIOA  ((GPIO_TypeDef *)0x40020000u)
#define GPIOB  ((GPIO_TypeDef *)0x40020400u)
#define GPIOC  ((GPIO_TypeDef *)0x40020800u)
#define RCC    ((RCC_TypeDef *)0x40023800u)
#define FLASHR ((FLASH_TypeDef *)0x40023C00u)
#define PWR    ((PWR_TypeDef *)0x40007000u)
#define IWDG   ((IWDG_TypeDef *)0x40003000u)
#define EXTI   ((EXTI_TypeDef *)0x40013C00u)
#define USART1 ((USART_TypeDef *)0x40011000u)
#define USART2 ((USART_TypeDef *)0x40004400u)
#define SPI1   ((SPI_TypeDef *)0x40013000u)
#define SPI2   ((SPI_TypeDef *)0x40003800u)
#define I2C1   ((I2C_TypeDef *)0x40005400u)
#define CAN1   ((CAN_TypeDef *)0x40006400u)

/* ---- Cortex-M4 core ---------------------------------------------------------- */
#define SYST_CSR   (*(__IO uint32_t *)0xE000E010u)
#define SYST_RVR   (*(__IO uint32_t *)0xE000E014u)
#define SYST_CVR   (*(__IO uint32_t *)0xE000E018u)
#define NVIC_ISER  ((__IO uint32_t *)0xE000E100u)
#define SCB_CPACR  (*(__IO uint32_t *)0xE000ED88u)
#define DEMCR      (*(__IO uint32_t *)0xE000EDFCu)
#define DWT_CTRL   (*(__IO uint32_t *)0xE0001000u)
#define DWT_CYCCNT (*(__IO uint32_t *)0xE0001004u)

/* ---- RCC bits ------------------------------------------------------------------- */
#define RCC_CR_HSEON        (1u << 16)
#define RCC_CR_HSERDY       (1u << 17)
#define RCC_CR_PLLON        (1u << 24)
#define RCC_CR_PLLRDY       (1u << 25)
#define RCC_PLLCFGR_MASK    0x0F437FFFu /* PLLM, PLLN, PLLP, PLLSRC, PLLQ fields */
#define RCC_PLLCFGR_SRC_HSE (1u << 22)
#define RCC_CFGR_SW_PLL     2u
#define RCC_CFGR_SWS_SHIFT  2
#define RCC_AHB1ENR_GPIOA   (1u << 0)
#define RCC_AHB1ENR_GPIOB   (1u << 1)
#define RCC_AHB1ENR_GPIOC   (1u << 2)
#define RCC_APB1ENR_SPI2    (1u << 14)
#define RCC_APB1ENR_USART2  (1u << 17)
#define RCC_APB1ENR_I2C1    (1u << 21)
#define RCC_APB1ENR_CAN1    (1u << 25)
#define RCC_APB1ENR_PWR     (1u << 28)
#define RCC_APB2ENR_USART1  (1u << 4)
#define RCC_APB2ENR_SPI1    (1u << 12)
#define RCC_APB2ENR_SYSCFG  (1u << 14)
#define RCC_CSR_RMVF        (1u << 24)
#define RCC_CSR_BORRSTF     (1u << 25)
#define RCC_CSR_PINRSTF     (1u << 26)
#define RCC_CSR_PORRSTF     (1u << 27)
#define RCC_CSR_SFTRSTF     (1u << 28)
#define RCC_CSR_IWDGRSTF    (1u << 29)

#define FLASH_ACR_PRFTEN    (1u << 8)
#define FLASH_ACR_ICEN      (1u << 9)
#define FLASH_ACR_DCEN      (1u << 10)

#define PWR_CR_PVDE         (1u << 4)
#define PWR_CR_PLS_SHIFT    5
#define PWR_CR_VOS          (1u << 14)

/* ---- USART ------------------------------------------------------------------------ */
#define USART_SR_ORE    (1u << 3)
#define USART_SR_RXNE   (1u << 5)
#define USART_SR_TXE    (1u << 7)
#define USART_CR1_RE    (1u << 2)
#define USART_CR1_TE    (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_UE    (1u << 13)

/* ---- SPI -------------------------------------------------------------------------- */
#define SPI_CR1_CPHA    (1u << 0)
#define SPI_CR1_CPOL    (1u << 1)
#define SPI_CR1_MSTR    (1u << 2)
#define SPI_CR1_BR_SHIFT 3
#define SPI_CR1_SPE     (1u << 6)
#define SPI_CR1_SSI     (1u << 8)
#define SPI_CR1_SSM     (1u << 9)
#define SPI_SR_RXNE     (1u << 0)
#define SPI_SR_TXE      (1u << 1)
#define SPI_SR_BSY      (1u << 7)

/* ---- I2C -------------------------------------------------------------------------- */
#define I2C_CR1_PE      (1u << 0)
#define I2C_CR1_START   (1u << 8)
#define I2C_CR1_STOP    (1u << 9)
#define I2C_CR1_ACK     (1u << 10)
#define I2C_CR1_POS     (1u << 11)
#define I2C_CR1_SWRST   (1u << 15)
#define I2C_SR1_SB      (1u << 0)
#define I2C_SR1_ADDR    (1u << 1)
#define I2C_SR1_BTF     (1u << 2)
#define I2C_SR1_RXNE    (1u << 6)
#define I2C_SR1_TXE     (1u << 7)
#define I2C_SR1_BERR    (1u << 8)
#define I2C_SR1_ARLO    (1u << 9)
#define I2C_SR1_AF      (1u << 10)
#define I2C_SR2_BUSY    (1u << 1)
#define I2C_CCR_FS      (1u << 15)

/* ---- bxCAN ------------------------------------------------------------------------ */
#define CAN_MCR_INRQ    (1u << 0)
#define CAN_MCR_TXFP    (1u << 2)
#define CAN_MCR_ABOM    (1u << 6)
#define CAN_MSR_INAK    (1u << 0)
#define CAN_TSR_TME0    (1u << 26)
#define CAN_TSR_TME1    (1u << 27)
#define CAN_TSR_TME2    (1u << 28)
#define CAN_TIR_TXRQ    (1u << 0)
#define CAN_RIR_IDE     (1u << 2)
#define CAN_RF0R_FMP0   (3u << 0)
#define CAN_RF0R_RFOM0  (1u << 5)
#define CAN_ESR_EPVF    (1u << 1)
#define CAN_ESR_BOFF    (1u << 2)
#define CAN_FMR_FINIT   (1u << 0)

/* ---- interrupt numbers ------------------------------------------------------------ */
#define PVD_IRQn     1
#define USART1_IRQn  37
#define USART2_IRQn  38

static inline void irq_disable(void) { __asm volatile("cpsid i" ::: "memory"); }
static inline void irq_enable(void)  { __asm volatile("cpsie i" ::: "memory"); }
static inline void nvic_enable(int irq) { NVIC_ISER[irq >> 5] = 1u << (irq & 31); }

#endif
