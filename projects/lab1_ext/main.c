#include <stm32f4xx.h>

#include "main.h"
/*
 * @brief 
 *        
 *        
 */
#define BUTTON_PORT   GPIOE
#define LEDS_PORT     GPIOA
#define USER_LEDS     (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15)
#define SWITCH_DELAY  ((uint32_t)400000)
#define PLL_N         200
#define PLL_P         2
#define PLL_Q         4
#define PLL_M_HSE     8
#define PLL_M_HSI     16
#define L3GD20_ADR    0x00D4
#define L3GD20_CTR3R  0x0022

typedef enum blink_mul_e
{
  BLINK_MUL_FAST = 1,
  BLINK_MUL_LONG = 3,
} blink_mul_t;

void blink_led(GPIO_TypeDef * port, uint16_t pins, blink_mul_t multiplier);

void turn_on_led(GPIO_TypeDef * port, uint16_t pins, blink_mul_t multiplier);

void init_gpio(void);

void init_af_gpio(GPIO_InitTypeDef *GPIO_InitStructure);

void init_i2c1(void);

int i2c_write_l3gd20(void);

int main(void)
{
  clock_init();
  set_clock(1);
  init_gpio();

  //init_i2c1();
  //i2c_write_l3gd20();

  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable peripheral clock for LEDs and buttons port */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  //init_af_gpio(&GPIO_InitStructure);
  //GPIO_PinAFConfig(GPIOA, GPIO_Pin_5 | GPIO_Pin_7, GPIO_AF_I2C1);
  init_i2c1();
  i2c_write_l3gd20();

  /* Init LEDs */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(LEDS_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);
  /* Turn all the leds off */
  GPIO_ResetBits(LEDS_PORT, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10);
  //GPIOD->ODR &= ~USER_LEDS;

  uint8_t i = 0, pushed = 0;
  int8_t direction = 1;
    
  while (1)
  {
    int state = GPIO_ReadInputDataBit(BUTTON_PORT, GPIO_Pin_1);
    if (pushed == 0)
    {
      if (state == 1)
      {
        pushed = 1;
//        switch_clock();
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
    //turn_on_led(GPIOD, GPIO_Pin_12 << i, BLINK_MUL_LONG);
    i = (i + (4 - direction)) % 4;
GPIO_SetBits(LEDS_PORT, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10);
if (state == 1)
{
GPIO_ResetBits(LEDS_PORT, GPIO_Pin_8);
}
else 
{
GPIO_ResetBits(LEDS_PORT, GPIO_Pin_9);
}
  }
}

void init_i2c1(void)
{
  I2C_InitTypeDef i2c_struct;

  RCC_APB1PeriphClockCmd (RCC_APB1Periph_I2C1, ENABLE);
  GPIO_PinAFConfig(GPIOA, GPIO_Pin_5 | GPIO_Pin_7, GPIO_AF_I2C1);

  I2C1->CR1 &= (uint16_t)((uint16_t)~(I2C_CR1_PE));
  I2C_StructInit (&i2c_struct); 
  i2c_struct.I2C_ClockSpeed = 400000;
  I2C_Init (I2C1, &i2c_struct);
  I2C_Cmd(I2C1, ENABLE);
}

int i2c_write_l3gd20(void)
{
  uint16_t value = 0x0038; uint32_t timer = 4000000;
  /* START */
  GPIOD->ODR |= GPIO_Pin_15;
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
    if (timer == 0x00)
    {
      GPIOD->ODR &= ~USER_LEDS;
      return 1;
    }
    timer--;
  }
  GPIOD->ODR &= ~USER_LEDS;
  /* l3gd20-address + write-bit */
  I2C1->DR = L3GD20_ADR;
  GPIO_SetBits(GPIOD, GPIO_Pin_12); /* 
  //while (((I2C1->SR1 & I2C_SR1_ADDR) == 0) && ((I2C1->SR1 & I2C_SR1_TIMEOUT) == 0));
  
  I2C1->DR = L3GD20_CTR3R;
  GPIO_SetBits(GPIOD, GPIO_Pin_13); 
  //while (((I2C1->SR1 & I2C_SR1_BTF) == 0) && ((I2C1->SR1 & I2C_SR1_TIMEOUT) == 0));

  
  I2C1->DR = value;
  GPIO_SetBits(GPIOD, GPIO_Pin_14); */
  //while (((I2C1->SR1 & I2C_SR1_BTF) == 0) && ((I2C1->SR1 & I2C_SR1_TIMEOUT) == 0));
  I2C1->CR1 |= I2C_CR1_STOP;
  return 0;
}

void init_af_gpio(GPIO_InitTypeDef *GPIO_InitStructure)
{
  GPIO_InitStructure->GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure->GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure->GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure->GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure->GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIOA->AFR[0] |= 0x40400000;
  GPIO_Init(GPIOA, GPIO_InitStructure);
}

void init_gpio(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 
  /* Init LEDs */
  GPIO_InitStructure.GPIO_Pin   = USER_LEDS;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
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

void set_clock(int8_t clock)
{
  uint32_t StartUpCounter = 0, Status = 1, Pll_src = RCC_PLLCFGR_PLLSRC_HSE, Pll_m = PLL_M_HSE;
  
  if(clock)
  {
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

  }
  else
  {
    Pll_src = RCC_PLLCFGR_PLLSRC_HSI;
    Pll_m = PLL_M_HSI;
    
    if ((RCC->CR & RCC_CR_HSIRDY) != RESET)
    {
      Status = (uint32_t)0x01;
    }
    else
    {
      Status = (uint32_t)0x00;
    }
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
    RCC->PLLCFGR = Pll_m | (PLL_N << 6) | ((PLL_P >> 2) << 16) |
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
    GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
  }
}

void switch_clock()
{
  if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)
  {
    clock_init();
    set_clock(0);
  }
  else if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)
  {
    clock_init();
    set_clock(1);
  }
}

void blink_led(GPIO_TypeDef * port, uint16_t pins, blink_mul_t multiplier)
{
  uint32_t k = 0;
  port->ODR &= ~USER_LEDS;
  port->ODR |= (~pins & USER_LEDS);
  for (k = 0; k < SWITCH_DELAY * multiplier; ++k);
}

void turn_on_led(GPIO_TypeDef * port, uint16_t pins, blink_mul_t multiplier)
{
  uint32_t k = 0;
  port->ODR &= ~USER_LEDS;
  port->ODR |= pins;
  for (k = 0; k < SWITCH_DELAY * multiplier; ++k);
}
