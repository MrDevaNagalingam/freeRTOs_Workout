#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

/* ===================== Pin / Feature Map =====================
   HC-SR04: TRIG -> PA0 (GPIO out), ECHO -> PA1 (TIM2 CH2 input capture)
   Bluetooth (HC-05): USART1 9600, TX->PA9, RX->PA10
   Debug terminal (ST-Link VCP): USART2 115200, TX->PA2
   LED: PA5
   Clock: SYSCLK 84 MHz (HSI + PLL), APB1=42 MHz, APB2=84 MHz
   TIM2: 1 MHz counter (PSC = 84-1), ARR=0xFFFF
   ============================================================ */

/* ==================== Compile-time options ==================== */
#define RX_BUFFER_SIZE 64

/* ==================== Globals ==================== */
static volatile char    g_rxBuf[RX_BUFFER_SIZE];
static volatile uint8_t g_rxIdx = 0;
static volatile char    g_rxByte;

static volatile uint32_t ic_val1=0, ic_val2=0, ic_diff=0;
static volatile uint8_t  ic_first = 0;
static volatile float    g_distance_cm = 0.0f;

/* Keep SystemCoreClock for FreeRTOS tick setup */
//uint32_t SystemCoreClock = 84000000u;

/* ==================== Prototypes ==================== */
static void clock_init_84MHz(void);
static void gpio_init(void);
static void usart1_init_9600(void);
static void usart2_init_115200_tx(void);
static void tim2_ic_init(void);

static void hc_sr04_trigger(void);
static float hc_sr04_read_cm(void);
static void process_cmd(const char *cmd);

/* FreeRTOS tasks */
static void UltrasonicTask(void *arg);
static void BluetoothTask(void *arg);

/* ==================== SWV + UART helpers ==================== */
static inline void SWV_Print(const char *s) {
    while (*s) ITM_SendChar(*s++);
}

/* Bare-metal USART write: blocking */
static inline void usart_write_str(USART_TypeDef *U, const char *s) {
    while (*s) {
        while (!(U->SR & USART_SR_TXE));
        U->DR = (uint16_t)(*s++);
    }
    while (!(U->SR & USART_SR_TC));
}

/* Redirect printf() to SWV console */
int _write(int file, char *ptr, int len) {
    (void)file;
    for (int i = 0; i < len; ++i) ITM_SendChar((int)*ptr++);
    return len;
}

/* ==================== Main ==================== */
int main(void) {
    /* Clocks and low-level inits */
    clock_init_84MHz();
    gpio_init();
    usart2_init_115200_tx(); /* ST-Link VCP TX on PA2 */
    usart1_init_9600();      /* HC-05 on PA9/PA10    */
    tim2_ic_init();

    /* NVIC: USART1 RX IRQ (Bluetooth) */
    USART1->CR1 |= USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 6);   /* below or equal to configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY */
    NVIC_EnableIRQ(USART1_IRQn);

    /* NVIC: TIM2 CC IRQ (Echo capture) */
    NVIC_SetPriority(TIM2_IRQn, 7);
    NVIC_EnableIRQ(TIM2_IRQn);

    /* Enable input capture on CH2 */
    TIM2->CCER |= TIM_CCER_CC2E;

    /* Announce over UART2 */
    usart_write_str(USART2, "Bluetooth Chat Ready\r\n");

    /* FreeRTOS tasks */
    xTaskCreate(UltrasonicTask, "US", 256, NULL, 1, NULL);
    xTaskCreate(BluetoothTask,  "BT", 256, NULL, 2, NULL);

    vTaskStartScheduler();
    while (1) { /* Should not return */ }
}

/* ==================== Clock: 84 MHz from HSI+PLL ==================== */
static void clock_init_84MHz(void) {
    /* Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* PLL config: HSI 16 MHz /16 * 336 /4 = 84 MHz ; PLLQ=7 */
    RCC->PLLCFGR = (16u)                       /* PLLM */
                 | (336u << 6)                 /* PLLN */
                 | (1u << 16)                  /* PLLP: 01 -> /4 */
                 | RCC_PLLCFGR_PLLSRC_HSI
                 | (7u << 24);                 /* PLLQ */

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* AHB=/1, APB1=/2 (42 MHz), APB2=/1 (84 MHz) */
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_HPRE_DIV1;

    /* Flash latency 2 WS + caches */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;

    /* Switch SYSCLK to PLL */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while (((RCC->CFGR >> 2) & 3u) != 2u);

    SystemCoreClock = 84000000u;
}

/* ==================== GPIO ==================== */
static void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA5 LED */
    GPIOA->MODER &= ~(3u << (5*2));
    GPIOA->MODER |=  (1u << (5*2));
    GPIOA->OTYPER &= ~(1u << 5);
    GPIOA->OSPEEDR |= (2u << (5*2));

    /* PA0 TRIG: output low */
    GPIOA->MODER &= ~(3u << (0*2));
    GPIOA->MODER |=  (1u << (0*2));
    GPIOA->OTYPER &= ~(1u << 0);
    GPIOA->OSPEEDR |= (2u << (0*2));
    GPIOA->BSRR = (1u << (0+16)); /* low */

    /* PA1 ECHO: AF1 TIM2_CH2 */
    GPIOA->MODER &= ~(3u << (1*2));
    GPIOA->MODER |=  (2u << (1*2));         /* AF */
    GPIOA->AFR[0] &= ~(0xFu << (1*4));
    GPIOA->AFR[0] |=  (1u   << (1*4));      /* AF1 */

    /* USART1: PA9 (TX), PA10 (RX) AF7 */
    GPIOA->MODER &= ~((3u<<(9*2)) | (3u<<(10*2)));
    GPIOA->MODER |=  ((2u<<(9*2)) | (2u<<(10*2)));
    GPIOA->AFR[1] &= ~((0xFu<<((9-8)*4)) | (0xFu<<((10-8)*4)));
    GPIOA->AFR[1] |=  ((7u << ((9-8)*4)) | (7u << ((10-8)*4)));
    GPIOA->PUPDR &= ~((3u<<(9*2)) | (3u<<(10*2)));
    GPIOA->PUPDR |=  (1u << (10*2));  /* RX pull-up */

    /* USART2: PA2 (TX) AF7 */
    GPIOA->MODER &= ~(3u << (2*2));
    GPIOA->MODER |=  (2u << (2*2));
    GPIOA->AFR[0] &= ~(0xFu << (2*4));
    GPIOA->AFR[0] |=  (7u   << (2*4));
    GPIOA->OSPEEDR |= (2u << (2*2));
}

/* ==================== USART1 9600 (APB2=84 MHz) ==================== */
static void usart1_init_9600(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    /* 84e6/(16*9600)=546.875 => BRR mant=546 (0x222), frac=14 (0xE) -> 0x222E */
    USART1->BRR = 0x222E;

    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/* ==================== USART2 115200 (APB1=42 MHz) ==================== */
static void usart2_init_115200_tx(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    /* 42e6/(16*115200)=22.786 -> mant=22 (0x16) frac≈13 (0xD): 0x016D works well */
    USART2->BRR = 0x016D;

    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

/* ==================== TIM2 CH2 input capture @1MHz ==================== */
static void tim2_ic_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 84 - 1;  /* 84 MHz / 84 = 1 MHz => 1 us per tick */
    TIM2->ARR = 0xFFFF;

    /* CC2 -> TI2, no prescaler, no filter */
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_IC2PSC | TIM_CCMR1_IC2F);
    TIM2->CCMR1 |=  (1u << 8);      /* CC2S=01 -> TI2 */

    /* Start with rising edge */
    TIM2->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP);

    TIM2->DIER |= TIM_DIER_CC2IE;   /* enable CC2 interrupt */
    TIM2->CR1  |= TIM_CR1_CEN;      /* counter on */
}

/* ==================== Simple ~us delay (busy loop) ==================== */
static inline void delay_us(uint32_t us) {
    /* Very rough: ~84 cycles/us; each loop body ~8 NOPs + loop overhead.
       For 10us trigger this is fine. */
    for (volatile uint32_t i = 0; i < us * 10; ++i) {
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    }
}

/* ==================== HC-SR04 helpers ==================== */
static void hc_sr04_trigger(void) {
    GPIOA->BSRR = (1u << 0);          /* PA0 high */
    delay_us(10);                     /* 10 us pulse */
    GPIOA->BSRR = (1u << (0+16));     /* PA0 low */
}

static float hc_sr04_read_cm(void) {
    /* Arm capture on rising, enable IRQ; ISR will compute distance */
    ic_first = 0;
    TIM2->CCER &= ~TIM_CCER_CC2P;     /* rising edge */
    TIM2->DIER |= TIM_DIER_CC2IE;     /* enable CC2 IRQ */

    hc_sr04_trigger();

    vTaskDelay(pdMS_TO_TICKS(60));    /* wait measurement window */
    return g_distance_cm;
}

/* ==================== Command parser ==================== */
static void process_cmd(const char *cmd) {
    if (strcmp(cmd, "led on") == 0) {
        GPIOA->BSRR = (1u << 5);
        usart_write_str(USART1, "LED turned ON\r\n");   /* to phone */
    } else if (strcmp(cmd, "led off") == 0) {
        GPIOA->BSRR = (1u << (5+16));
        usart_write_str(USART1, "LED turned OFF\r\n");  /* to phone */
    } else {
        usart_write_str(USART1, "Unknown command\r\n");
    }
}

/* ==================== FreeRTOS tasks ==================== */
static void BluetoothTask(void *arg) {
    (void)arg;
    /* Everything RX-related handled in IRQ; we just idle here */
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

static void UltrasonicTask(void *arg) {
    (void)arg;
    char line[64];
    for (;;) {
        float d = hc_sr04_read_cm();
        /* Print ONLY to SWV per your requirement */
        int n = snprintf(line, sizeof(line), "Distance = %.2f cm\r\n", d);
        (void)n;
        SWV_Print(line);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ==================== IRQ Handlers ==================== */
void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) {
        g_rxByte = (char)(USART1->DR & 0xFF);

        if (g_rxByte == '\r' || g_rxByte == '\n') {
            if (g_rxIdx > 0) {
                g_rxBuf[g_rxIdx] = '\0';

                /* Echo the received line to PC terminal (USART2) */
                usart_write_str(USART2, "Received: ");
                usart_write_str(USART2, (const char*)g_rxBuf);
                usart_write_str(USART2, "\r\n");

                /* Process and send feedback to phone (USART1) */
                process_cmd((const char*)g_rxBuf);
                g_rxIdx = 0;
            }
        } else {
            if (g_rxIdx < (RX_BUFFER_SIZE-1)) {
                g_rxBuf[g_rxIdx++] = g_rxByte;
            } else {
                g_rxIdx = 0; /* overflow guard */
            }
        }
    }
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_CC2IF) {
        uint32_t cap = TIM2->CCR2;

        if (!ic_first) {
            ic_val1 = cap;
            ic_first = 1;
            /* switch to falling edge */
            TIM2->CCER |= TIM_CCER_CC2P;
        } else {
            ic_val2 = cap;

            if (ic_val2 >= ic_val1) ic_diff = ic_val2 - ic_val1;
            else                    ic_diff = (0xFFFF - ic_val1) + ic_val2;

            /* time(us) * 0.0343 cm/us / 2 */
            g_distance_cm = (float)ic_diff * 0.0343f * 0.5f;

            /* re-arm: rising edge next, mask until next trigger */
            TIM2->CCER &= ~TIM_CCER_CC2P;
            TIM2->DIER &= ~TIM_DIER_CC2IE;
            ic_first = 0;
        }
        TIM2->SR &= ~TIM_SR_CC2IF;
    }
}
