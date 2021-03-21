
#include "stm32f407xx.h"
#include <stdbool.h> 
void SystemClock(void);
void PWM_Config();
void Tim6Config();
void msdelay(uint16_t ms);
void EXTI_config();

unsigned long ticks = 0 ;
bool state = 0;

int main(void)
{
  //SystemClock();
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN ;
 // RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  GPIOB->MODER = 0x5;
  GPIOB->ODR = 0x0002;
  PWM_Config();
  Tim6Config();
 // GPIOD->MODER = 0x55000000;
  

  while (1)
  {
     TIM4->CCR1 = 200;
     msdelay(2000);
     TIM4->CCR1 = 400;
     msdelay(2000);
     TIM4->CCR1 = 600;
     msdelay(2000);
     TIM4->CCR1 = 800;
     msdelay(2000);

     /* if(state)
      GPIOD->ODR = 0xf000;
      else
      GPIOD->ODR = 0x0000;*/

  }
  return 0 ; 
}

void PWM_Config()
{
  //1. active the clock 
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  //2. GPIOD PIN_12 Alternate function 
  GPIOD->MODER = 0 ;
  GPIOD->MODER |= (1<<25);

  //3.AF TIM4
  GPIOD->AFR[1] = 0x00020000;    

  // Enable channel 1 compare register
  TIM4->CCER |= (1<<0);
  TIM4->CR1  |= TIM_CR1_ARPE;

  //4. PWM Mode 1 
  TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |  TIM_CCMR1_OC1PE;

  // 50 hz frq          
  TIM4->PSC  = 320;       //16000000/320*1000
  TIM4->ARR  = 1000;     

  //duty cycle: 0--1000; 
  TIM4->CCR1 = 0;   // 0 degree
  
  TIM4->EGR |= TIM_EGR_UG;
  TIM4->CR1 |= TIM_CR1_CEN;

}

void EXTI_config()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->PUPDR &= !3U << 0;
  GPIOA->PUPDR |= 2U << 0;
  NVIC_EnableIRQ(EXTI0_IRQn);
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->EXTICR[0] =(0x0 << 0);
  EXTI->IMR |= EXTI_IMR_IM0;
  EXTI->RTSR |= EXTI_RTSR_TR0;
}
void EXTI0_IRQHandler(void)
{
  if((EXTI->PR & EXTI_PR_PR0) != 0)
  {
    ticks++;
    EXTI->PR |= EXTI_PR_PR0;
  }
}
// fct config timer 
void Tim6Config()
{
 //1. Enable the clock for TIM6
  RCC->APB1ENR |= (1<<4);

 //2. Set the prescaler to 15 to run the counter with 1MHZ freq so 1 us for each tick
  TIM6->PSC = 15;

 //3. Max value 
  TIM6->ARR = 0xffff;

 //4. Active counter up 
  TIM6->CR1 |= (1<<0);
  while(!(TIM6->SR & (1<<0)));
}

void usdelay(uint16_t us)
{
 TIM6->CNT = 0;
 while(TIM6->CNT < us);
}

void msdelay(uint16_t ms)
{
  for(uint16_t i = 0; i<ms; i++)
  {
    usdelay(1000);
  }  
}