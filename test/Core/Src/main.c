#include <stdint.h>

#define RCC_BASE 0x40023800
#define RCC_AHB1ENR *((volatile unsigned int*)(RCC_BASE + 0x30))
#define RCC_APB1ENR *((volatile unsigned int*)(RCC_BASE + 0x40))
#define RCC_APB2ENR *((volatile unsigned int*)(RCC_BASE + 0x44))

#define TIM2_BASE 0x40000000
#define TIM2_CR1 *((volatile unsigned int*)(TIM2_BASE + 0x0))
#define TIM2_DIER *((volatile unsigned int*)(TIM2_BASE + 0x0C))
#define TIM2_SR *((volatile unsigned int*)(TIM2_BASE + 0x10))
#define TIM2_CNT *((volatile unsigned int*)(TIM2_BASE + 0x24))
#define TIM2_PSC *((volatile unsigned int*)(TIM2_BASE + 0x28))
#define TIM2_ARR *((volatile unsigned int*)(TIM2_BASE + 0x2C))


#define SYSTICK  0xE000E010
#define SYSTICK_CTRL *((volatile unsigned int*)(SYSTICK + 0x0))
#define SYSTICK_LOAD *((volatile unsigned int*)(SYSTICK + 0x4))
#define SYSTICK_VAL *((volatile unsigned int*)(SYSTICK + 0x8))

#define GPIOA_BASE 0x40020000
#define GPIOD_BASE 0x40020C00

#define GPIOA_MODER *((volatile unsigned int*)(GPIOA_BASE + 0x00))
#define GPIOA_PUPDR *((volatile unsigned int*)(GPIOA_BASE + 0x0C))
#define GPIOA_IDR *((volatile unsigned int*)(GPIOA_BASE + 0x10))

#define GPIOD_MODER *((volatile unsigned int*)(GPIOD_BASE + 0x00))
#define GPIOD_ODR *((volatile unsigned int*)(GPIOD_BASE + 0x14))

#define SYSCFG_BASE 0x40013800
#define SYSCFG_EXTICR1 *((volatile unsigned int*)(SYSCFG_BASE + 0x08))

#define EXTI_BASE 0x40013C00
#define EXTI_IMR *((volatile unsigned int*)(EXTI_BASE + 0x00))
#define EXTI_RTSR *((volatile unsigned int*)(EXTI_BASE + 0x08))
#define EXTI_PR *((volatile unsigned int*)(EXTI_BASE + 0x14))

#define NVIC_ISER0 *((volatile unsigned int*)0xE000E100)
#define NVIC_IPR1  *((volatile unsigned int*)0xE000E404)

void SYSTICKdelay(int ms);
void clockwise(int start);
void anticlockwise(int start);
void EXTI0_IRQHandler(void);

#define DELAY 50
volatile int flag = 1;

int main()
{
    // Enable clocks for GPIOA, GPIOD, and SYSCFG
    RCC_AHB1ENR |= 0x9;
    RCC_APB1ENR |= 0x1;             // TIM2
    RCC_APB2ENR |= 0x4000;          //SYSCFG for External interrupt

    // Configure GPIOD pins 12-15 as output
    GPIOD_MODER &= ~0xFF000000;
    GPIOD_MODER |= 0x55000000;

    // Configure GPIOA pin 0 as input with pull-up
    GPIOA_MODER &= ~0x3;
    //GPIOA_PUPDR |= 0x1;

    // Configure EXTI0 interrupt for PA0
    SYSCFG_EXTICR1 &= ~0xF;  // Map PA0 to EXTI0
    EXTI_IMR |= 0x1;         // Enable interrupt for EXTI0
    EXTI_RTSR |= 0x1;        // Enable rising edge trigger


    NVIC_ISER0 |= (1 << 6);  // Enable EXTI0 interrupt in NVIC
    NVIC_IPR1 |= (5 << 20);  // Shift left by 20 to set EXTI0 priority


    while (1)
    {
        if (flag) clockwise(0);
        else anticlockwise(3);
    }
}

void clockwise(int start)
{
    for (int i = start; i < 4; i++) {
        GPIOD_ODR = (0x1000 << i);
        SYSTICKdelay(DELAY);
    }
}

void anticlockwise(int start)
{
    for (int i = start; i >= 0; i--) {
        GPIOD_ODR = (0x1000 << i);
        SYSTICKdelay(DELAY);
    }
}

void EXTI0_IRQHandler(void)
{
        flag = !flag;  // Toggle direction
        EXTI_PR |= 0x1; // Clear interrupt flag
}

void TIM2delay(int ms)
{
	TIM2_PSC = 16000 - 1;
	TIM2_ARR = ms ;
	TIM2_CNT = 0;
	TIM2_DIER |= 0x1; // Enable update interrupt
	TIM2_CR1 |= 0x1;   // Start Timer
}
void SYSTICKdelay(int ms)
{
    SYSTICK_LOAD = (42000 * ms) - 1;  // Assuming 16MHz clock, 1ms per tick
	//SYSTICK_LOAD = 0xFFFFF;
    SYSTICK_VAL = 0;  // Clear current value
    SYSTICK_CTRL = 0x5;  // Enable SysTick with system clock

    while (!(SYSTICK_CTRL & (1 << 16)));  // Wait until COUNTFLAG is set
}
