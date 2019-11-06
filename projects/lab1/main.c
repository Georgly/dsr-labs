#include <stm32f4xx.h>

/*
 * @brief Simple lab -- blink leds in R->B->G order, on button press
 *        LEDS blink white once for a short period of time and then
 *        blink in a reverse order G->B->R.
 */

#define SWITCH_DELAY    ((uint32_t)200000)

typedef enum blink_mul_e
{
  BLINK_MUL_FAST = 1,
  BLINK_MUL_LONG = 3,
} blink_mul_t;

void blink_led(GPIO_TypeDef * port, uint16_t pins, blink_mul_t multiplier);

void set_clock(uint32_t pins)
{
    RCC->CR |= pins;
}

void config_hse_pll()
{
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
  RCC->PLLCFGR |= 
}

void switch_clock()
{
    if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE)
    {
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= RCC_CFGR_SW_HSI;
        return;
    }
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_HSE;
}

int main(void)
{
  while( (RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY);
  set_clock(RCC_CR_HSEON);
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable peripheral clock for LEDs and buttons port */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  /* Init LEDs */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Turn all the leds off */
  GPIO_SetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10);

  int8_t i = 0;
  int8_t direction = 1;

  while (1)
  {
    int state = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0);
    if (!state)
    {
        switch_clock();
      //direction = -direction;

      //blink_led(GPIOA, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10, BLINK_MUL_FAST);
    }

    blink_led(GPIOA, GPIO_Pin_8 << i, BLINK_MUL_LONG);

    //i = (3 + (i - direction) % 3) % 3;
    i = (i + 1) % 3;
  }
}

void blink_led(GPIO_TypeDef * port, uint16_t pins, blink_mul_t multiplier)
{
  uint32_t k = 0;

  GPIO_ResetBits(port, pins);
  for (k = 0; k < SWITCH_DELAY * multiplier; ++k);
  GPIO_SetBits(port, pins);
}
