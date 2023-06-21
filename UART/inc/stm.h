
#include<stdio.h>

/** 
  * @brief General Purpose I/O
  */

typedef struct
{
   volatile uint32_t CRL;
   volatile uint32_t CRH;
   volatile uint32_t IDR;
   volatile uint32_t ODR;
   volatile uint32_t BSRR;
   volatile uint32_t BRR;
   volatile uint32_t LCKR;
} GPIO_TypeDef;

/**
 * @brief USART universal synchronous  asynchronous receiver transmitter
*/
typedef struct
{
   volatile uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
   volatile uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
   volatile uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
   volatile uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
   volatile uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
   volatile uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
   volatile uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;

//reset and clock control
typedef struct
{
   volatile uint32_t CR;
   volatile uint32_t CFGR;
   volatile uint32_t CIR;
   volatile uint32_t APB2RSTR;
   volatile uint32_t APB1RSTR;
   volatile uint32_t AHBENR;
   volatile uint32_t APB2ENR;
   volatile uint32_t APB1ENR;
   volatile uint32_t BDCR;
   volatile uint32_t CSR;


} RCC_TypeDef;



//periperal memory map

#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x00020000UL)
#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000UL)


#define RCC                 ((RCC_TypeDef *)RCC_BASE)

#define USART2_BASE           (APB1PERIPH_BASE + 0x00004400UL)
#define USART2              ((USART_TypeDef *)USART2_BASE)
#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)

#define RCC_APB1ENR_USART2EN_Pos             (17U)                             
#define RCC_APB1ENR_USART2EN_Msk             (0x1UL << RCC_APB1ENR_USART2EN_Pos) /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN                 RCC_APB1ENR_USART2EN_Msk  


#define RCC_APB1ENR_USART2EN_Pos             (17U)                             
#define RCC_APB1ENR_USART2EN_Msk             (0x1UL << RCC_APB1ENR_USART2EN_Pos) /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN                 RCC_APB1ENR_USART2EN_Msk          /*!< USART 2 clock enable */

#define RCC_APB2ENR_IOPAEN_Pos               (2U)                              
#define RCC_APB2ENR_IOPAEN_Msk               (0x1UL << RCC_APB2ENR_IOPAEN_Pos)  /*!< 0x00000004 */
#define RCC_APB2ENR_IOPAEN                   RCC_APB2ENR_IOPAEN_Msk            /*!< I/O port A clock enable */
