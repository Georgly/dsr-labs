#include <stm32f4xx.h>

#include "main.h"

/*
 * @brief 
 * Using STM32 hardware timers
 */

#define USER_LEDS    (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15)
#define PLL_N        200 
#define PLL_P        2
#define PLL_Q        4
#define PLL_M        8


int main(void)
{
  clock_init();
  set_clock();
  gpio_init();
  tim_init();
  TIM_Cmd(TIM4, ENABLE);

  uint8_t i = 0, pushed = 0;
  int8_t direction = 1;
  
  while (1)
  {
    int state = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
    if (pushed == 0)
    {
      if (state == 1)
      {
        pushed = 1;
        direction = -direction;
      }
    }
    else
    {
      if (!state)
      {
        pushed = 0;
      }
    }
    //blink_led(GPIOD, GPIO_Pin_12 << i);
    turn_on_led(GPIOD, GPIO_Pin_12 << i);
    i = (i + (4 - direction)) % 4;
  }
}

void gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable peripheral clock for LEDs and buttons port */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  /* Init LEDs */
  GPIO_InitStructure.GPIO_Pin   = USER_LEDS;//GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Init User_Button */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Turn all leds off*/
  GPIOD->ODR &= ~USER_LEDS;
//GPIO_ResetBits(GPIOD, USER_LEDS);// GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}

void tim_init(void)
{
  TIM_TimeBaseInitTypeDef tim_struct;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_TimeBaseStructInit(&tim_struct);
  tim_struct.TIM_Prescaler = 762;
  tim_struct.TIM_ClockDivision = 0;
  tim_struct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &tim_struct);
  TIM4->SR &= (uint16_t)((uint16_t)~(TIM_SR_UIF));
}

void clock_init(void)
{
  /* Reset RCC_CR register */
  RCC->CR = 0x00000081;
  /* Reset RCC_CFGR register*/
  RCC->CFGR = 0x00000000;
  RCC->CR &= (uint32_t)0xFEF6FFFF;
  /* Reset RCC_PLLCFGR register*/
  RCC->PLLCFGR = 0x24003010;
  RCC->CR &= (uint32_t)0xFFFBFFFF;
}

void set_clock(void)
{
  uint32_t StartUpCounter = 0, Status = 1, Pll_src = RCC_PLLCFGR_PLLSRC_HSE;
 
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
    
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  { 
    Status = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  } while((Status == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));
  
  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  { 
    Status = (uint32_t)0x01;
  }
  else
  { 
    Status = (uint32_t)0x00;
  }

  if (Status == (uint32_t)0x01)
  {

    /* Select regulator voltage output Scale 1 mode */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;
 
    /* HCLK = SYSCLK / 1*/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
 
    /* PCLK1 = HCLK / 4*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;


    /* Configure the main PLL */
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | ((PLL_P >> 2) << 16) |
                   (Pll_src) | (PLL_Q << 24); 

    /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;
    
    /* Wait till the main PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }
    
    /* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait till the main PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
    {
    }

  }
  else
  {
    GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);
  }
}

void blink_led(GPIO_TypeDef * port, uint16_t pins)
{
  port->ODR &= ~USER_LEDS;
  port->ODR |= (~pins & USER_LEDS);
  while ((TIM4->SR & TIM_SR_UIF) != 1);
  TIM4->SR &= (uint16_t)((uint16_t)~(TIM_SR_UIF));
}

void turn_on_led(GPIO_TypeDef * port, uint16_t pins)
{
  port->ODR &= ~USER_LEDS;
  port->ODR |= pins;
  while ((TIM4->SR & TIM_SR_UIF) != 1);
  TIM4->SR &= (uint16_t)((uint16_t)~(TIM_SR_UIF));
}
