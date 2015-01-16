
#include "stm32f407_LedDrv.h"

void InitPinsForLeds(void)
{
  //Enable the GPIOD Clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  // GPIOD Configuration
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void SetOrangeLed(uint8_t u8LedStatus)
{
  GPIO_WriteBit(GPIOD, GPIO_Pin_13, u8LedStatus);
}

void ToggleOrangeLed()
{
  GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
}

void SetGreenLed(uint8_t u8LedStatus)
{
  GPIO_WriteBit(GPIOD, GPIO_Pin_12, u8LedStatus);
}

void ToggleGreenLed()
{
  GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
}

void SetBlueLed(uint8_t u8LedStatus)
{
  GPIO_WriteBit(GPIOD, GPIO_Pin_15, u8LedStatus);
}

void ToggleBlueLed()
{
  GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
}

void SetRedLed(uint8_t u8LedStatus)
{
  GPIO_WriteBit(GPIOD, GPIO_Pin_14, u8LedStatus);
}

void ToggleRedLed()
{
  GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
}
