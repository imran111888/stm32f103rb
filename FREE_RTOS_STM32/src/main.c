
#include "FreeRTOS.h"
#include "task.h"


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

void uart2_send(const char* str)
{
    while (*str != '\0')
    {
        uart2_write(*str++);
    }
}
// Task 1: Transmit Task
void vTask1(void *pvParameters)
{
    const char message1[] = "\r\nTask 1: Hello,Serial Monitor!\n";

    while (1)
    {
        uart2_send(message1);
        vTaskDelay(1000);
    }
}

// Task 2: Transmit Task
void vTask2(void *pvParameters)
{
    const char message2[] = "\r\nTask 2: welcome to 5g\n";

    while (1)
    {
        uart2_send(message2);
        vTaskDelay(1000);
    }
}
//task 3
void vTask3(void *pvParameters)
{
   const char message3[] = "\r\n Task 3 : Thank you!\n";
   while(1)
   {
   uart2_send(message3);
   vTaskDelay(1000);
   }
}

int main(void)


{
  

    // Enable clock access to GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // Enable clock access to alternate function
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    // Configure PA2 as output maximum speed to 50MHz
    // and alternate output push-pull mode
    GPIOA->CRL |= GPIO_CRL_MODE2;
    GPIOA->CRL |= GPIO_CRL_CNF2_1;
    GPIOA->CRL &= ~GPIO_CRL_CNF2_0;

    // Don't remap the pins
    AFIO->MAPR &= ~AFIO_MAPR_USART2_REMAP;

    // Enable clock access to USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Transmit Enable
    USART2->CR1 |= USART_CR1_TE;
    // Receive Enable
    USART2->CR1 |= USART_CR1_RE;

    // Set baud rate
    uart_set_baudrate(USART2, Perpher_CLK, Baudrate);
    // Enable UART
    USART2->CR1 |= USART_CR1_UE;
    

    // Create the transmit task 1
    xTaskCreate(vTask1, "TransmitTask1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    // Create the transmit task 2
    xTaskCreate(vTask2, "TransmitTask2", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
   //create th transmit task 3
    xTaskCreate(vTask3, "TransmitTask3", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
   
    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (1);

    return 0;
}

