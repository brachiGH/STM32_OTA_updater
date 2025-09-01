#ifndef _BOOT_GPIO_H_
#define _BOOT_GPIO_H_

#define GPIOAEN (1U << 0)
#define GPIOCEN (1U << 2)

#define LED_PIN (1U << 5)
#define BTN_PIN (1U << 13)

#define TIM2EN (1U << 0)
#define CR1_CEN (1U << 0)
#define OC_TOGGLE ((1U << 4) | (1U << 5))
#define CCER_CC1E (1U << 0)
#define AFR5_TIM (1U << 20)

static void BOOT_GPIO_init()
{
  /*Enable clock access to GPIOA and GPIOC*/
  RCC->AHB1ENR |= GPIOAEN;
  RCC->AHB1ENR |= GPIOCEN;

  /*Set PA5 as output pin*/
  GPIOA->MODER |= (1U << 10);
  GPIOA->MODER &= ~(1U << 11);

  /*Set PC13 as input pin*/
  GPIOC->MODER &= ~(1U << 26);
  GPIOC->MODER &= ~(1U << 27);
}

static void tim2_20hzLedBinkBegin(void)
{
  /*Enable clock access to GPIOA*/
  RCC->AHB1ENR |= GPIOAEN;

  /*Set PA5 mode to alternate function*/
  GPIOA->MODER &= ~(1U << 10);
  GPIOA->MODER |= (1U << 11);

  /*Set PA5 alternate function type to TIM2_CH1 (AF01)*/
  GPIOA->AFR[0] |= AFR5_TIM;

  /*Enable clock access to tim2*/
  RCC->APB1ENR |= TIM2EN;

  /*Set prescaler value*/
  TIM2->PSC = (SystemCoreClock / 1000) - 1;

  /*Set auto-reload value*/
  TIM2->ARR = 500 - 1;

  /*Set output compare toggle mode*/
  TIM2->CCMR1 = OC_TOGGLE;

  /*Enable tim2 ch1 in compare mode*/
  TIM2->CCER |= CCER_CC1E;

  /*Clear counter*/
  TIM2->CNT = 0;

  /*Enable timer*/
  TIM2->CR1 = CR1_CEN;
}

#endif