#include "Compass.h"
#include "math.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_i2c.h"
#include <stdio.h>


//**
//* Static variables
//**
static char magInfo[20];
static uint8_t  dataMag[6] = {0, 0, 0, 0, 0, 0}; //Readed data from magnetomer registers
static float heading = 0;
static lsm303MagData magData;

//**
//* Local constants
//**
static const float PI = 3.14159;

//**
//* Static functions
//**
static void ReadMagnetomerData(void);
static void PrintMagnetomerData(void);
static void PrintHeadingDegress(void);

void CompassInit(void)
{
  TM_I2C_Init(I2C1, TM_I2C_PinsPack_1, 50000);
  TM_I2C_Write(I2C1, MAG_ADDRESS, 0x02, 0); //Enable Continuous-conversion mode Register MR_REG_M (02h) = 0x00 (default 0x03 Sleep-mode. Device is placed in sleep-mode)
}

static void ReadMagnetomerData(void)
{
  dataMag[0] = TM_I2C_Read(I2C1, MAG_ADDRESS, 0x03); // OUT_X_H_M
  dataMag[1] = TM_I2C_Read(I2C1, MAG_ADDRESS, 0x04); // OUT_X_L_M
  dataMag[2] = TM_I2C_Read(I2C1, MAG_ADDRESS, 0x05); // OUT_Z_H_M
  dataMag[3] = TM_I2C_Read(I2C1, MAG_ADDRESS, 0x06); // OUT_Z_L_M
  dataMag[4] = TM_I2C_Read(I2C1, MAG_ADDRESS, 0x07); // OUT_Y_H_M
  dataMag[5] = TM_I2C_Read(I2C1, MAG_ADDRESS, 0x08); // OUT_Y_L_M
}

static void PrintMagnetomerData(void)
{
  sprintf(magInfo, "OUT_X_H_M: %X", dataMag[0]);
  TM_USART_Puts(USART3, magInfo);
  sprintf(magInfo, "OUT_X_L_M: %X", dataMag[1]);
  TM_USART_Puts(USART3, magInfo);
  
  sprintf(magInfo, "OUT_Z_H_M: %X", dataMag[2]);
  TM_USART_Puts(USART3, magInfo);
  sprintf(magInfo, "OUT_Z_L_M: %X", dataMag[3]);
  TM_USART_Puts(USART3, magInfo);
  
  sprintf(magInfo, "OUT_Y_H_M: %X", dataMag[4]);
  TM_USART_Puts(USART3, magInfo);
  sprintf(magInfo, "OUT_Y_L_M: %X", dataMag[5]);
  TM_USART_Puts(USART3, magInfo);
}

static void PrintHeadingDegress(void)
{
  sprintf(magInfo, "Compass heading: %f\n", heading);
  TM_USART_Puts(USART3, magInfo);
}

void CompassProcess(void)
{
  ReadMagnetomerData();

  magData.x = (int16_t)(dataMag[1] | ((int16_t)dataMag[0] << 8));
  magData.y = (int16_t)(dataMag[5] | ((int16_t)dataMag[4] << 8));
  magData.z = (int16_t)(dataMag[3] | ((int16_t)dataMag[2] << 8));
  
  heading = (atan2(magData.y, magData.x) * 180) / PI;
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  
  heading = 360 - heading;

  PrintHeadingDegress();
}
