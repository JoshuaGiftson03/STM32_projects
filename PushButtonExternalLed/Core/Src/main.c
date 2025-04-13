#include <stdint.h>

#define RCC_BASE 0x40023800
#define RCC_AHB1ENR *((volatile unsigned int*)(RCC_BASE + 0x30))
#define RCC_APB2ENR *((volatile unsigned int*)(RCC_BASE + 0x44))

#define GPIOA_BASE 0x40020000

#define GPIOA_MODER *((volatile unsigned int*)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER *((volatile unsigned int*)(GPIOA_BASE + 0x04))
#define GPIOA_OSPEEDR *((volatile unsigned int*)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR *((volatile unsigned int*)(GPIOA_BASE + 0x0C))
#define GPIOA_IDR *((volatile unsigned int*)(GPIOA_BASE + 0x10))
#define GPIOA_ODR *((volatile unsigned int*)(GPIOA_BASE + 0x14))

#define SYSCFG_BASE 0x40013800
#define SYSCFG_EXTICR1 *((volatile unsigned int*)(SYSCFG_BASE + 0x8))

#define EXTI_BASE 0x40013C00
#define EXTI_IMR *((volatile unsigned int*)(EXTI_BASE + 0x00))
#define EXTI_RTSR *((volatile unsigned int*)(EXTI_BASE + 0x08))
#define EXTI_PR *((volatile unsigned int*)(EXTI_BASE + 0x14))

#define NVIC_ISER0 *((volatile unsigned int*)0xE000E100)
volatile uint8_t flag = 0;

int main()
{
	RCC_AHB1ENR |= 0x1;
	RCC_APB2ENR |= 0x4000;          //SYSCFG for External interrupt

	GPIOA_MODER &= ~0xC0003;
	GPIOA_MODER |= 0x40000; //PA9 as output
	GPIOA_OSPEEDR |= 0xC0000;
	GPIOA_OTYPER &= ~(1 << 9); // Set PA9 as push-pull (default)

	SYSCFG_EXTICR1 &= ~0xF;  // Map PA0 to EXTI0
	EXTI_IMR |= 0x1;         // Enable interrupt for EXTI0
	EXTI_RTSR |= 0x1;        // Enable rising edge trigger


	NVIC_ISER0 |= (1 << 6);  // Enable EXTI0 interrupt in NVIC

	while(1)
	{

	}

}

void EXTI0_IRQHandler(void)
{
	if(!flag){
	GPIOA_ODR |= 0x200;
	flag = 1;
	}
	else
	{
		GPIOA_ODR &= ~0x200;
		flag = 0;
	}
	EXTI_PR |= 0x1;

}




