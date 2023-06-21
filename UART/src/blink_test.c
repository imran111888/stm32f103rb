#include "stm32f1xx.h"

void delay_ms(uint32_t milliseconds) {
    for (volatile uint32_t i = 0; i < (milliseconds * 8000); ++i) {
        // Delay loop
    }
}

int blink_main() {
    // Enable GPIOA peripheral clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // Configure pin 5 (PA5) as output
    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
    GPIOA->CRL |= GPIO_CRL_MODE5_0;

    while (1) {
        // Toggle the state of pin 5 (PA5)
        GPIOA->ODR ^= GPIO_ODR_ODR5;

        // Delay for 500 milliseconds
        delay_ms(100);
    }

  
}

