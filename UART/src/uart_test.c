#include "stm32f1xx.h"

#define Perpher_CLK 8000000
#define Baudrate    115200

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
    return ((PeriphClk + (BaudRate / 2U)) / BaudRate);
}

static void uart_set_baudrate(USART_TypeDef* USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}

void uart2_write(int ch)
{
    /* Make sure the transmit data register is empty */
    while (!(USART2->SR & USART_SR_TXE))
    {
    }

    /* Write to transmit data register */
    USART2->DR = (ch & 0xFF);
}

void uart2_send_string(const char* str)
{
    while (*str != '\0')
    {
        uart2_write(*str++);
    }
}

int uart_main(void)
{
    /* UART2 Pin configures */

    // Enable clock access to GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // Enable clock access to alternate function
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    /* Configure PA2 as output maximum speed to 50MHz
     * and alternate output push-pull mode */
    GPIOA->CRL |= GPIO_CRL_MODE2;
    GPIOA->CRL |= GPIO_CRL_CNF2_1;
    GPIOA->CRL &= ~GPIO_CRL_CNF2_0;

    /* Don't remap the pins */
    AFIO->MAPR &= ~AFIO_MAPR_USART2_REMAP;

    /* USART2 configuration */

    // Enable clock access to USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Transmit Enable
    USART2->CR1 |= USART_CR1_TE;

    /* Set baudrate */
    uart_set_baudrate(USART2, Perpher_CLK, Baudrate);
    // Enable UART
    USART2->CR1 |= USART_CR1_UE;

    while (1)
    {
        const char sentence[] = "Hello, Serial Monitor!\r\n";
        uart2_send_string(sentence);
    }
}

