#include "stm32f4xx.h"
#include "defines.h"


// Defines of types and constants
#ifndef MAG_ADDRESS
#define MAG_ADDRESS           0x3C  // 0011 110b   // b - 1 read; b - 0 write;
#endif

typedef struct lsm303MagData_s
{
  float x;
  float y;
  float z;
}lsm303MagData;

void CompassProcess(void);
void CompassInit(void);
