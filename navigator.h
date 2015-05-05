#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "defines.h"
#include "tm_stm32f4_pwm.h"
#include "tm_stm32f4_timer_properties.h"
#include "tm_stm32f4_fatfs.h"

#ifndef SYMBOLS_IN_LINE
#define SYMBOLS_IN_LINE 26        // 4239.4193,N,02316.2724,E\r\n
#endif

#ifndef MAX_PARSED_TRAJ_POINT
#define MAX_PARSED_TRAJ_POINT 20
#endif

#ifndef READ_BUFFER_SIZE
#define READ_BUFFER_SIZE  ((uint16_t) (SYMBOLS_IN_LINE * MAX_PARSED_TRAJ_POINT))
#endif

#ifndef MIN_WHEEL_DEGREES
#define MIN_WHEEL_DEGREES 50
#endif

#ifndef MID_WHEEL_DEGREES
#define MID_WHEEL_DEGREES 95
#endif

#ifndef MAX_WHEEL_DEGREES
#define MAX_WHEEL_DEGREES 140
#endif

#ifndef MAX_TURN_DEGREES_IN_DIR
#define MAX_TURN_DEGREES_IN_DIR 45
#endif

#ifndef LEFT_TURN
#define LEFT_TURN 0
#endif

#ifndef RIGHT_TURN
#define RIGHT_TURN 1
#endif

typedef struct TrajectoryInfo
{
  int32_t i32Latitude;    // Latitude
  int32_t i32Longitude;   // Longitude
} tTrajectoryInfo;

typedef struct NaviInfo
{
  uint8_t  u8Speed;
  uint8_t  u8Direction;
  uint16_t u16Degrees;
} tNaviInfo;

/**
 * Get trajectory points from SD card
 * 
 * Parameters: Pointer of the file that is stored in the SD card
 * 
 * Return value: 0 - error during parsing information
 *               1 - parsing is successfully
 */
void GetTrajectoryFromSDCard(FIL * fileTrajectory);
void NaviProcess(tNaviInfo * pNaviInfo);
void NaviInit(void);

