#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "defines.h"
#include "tm_stm32f4_pwm.h"
#include "tm_stm32f4_timer_properties.h"
#include "tm_stm32f4_fatfs.h"

#define SYMBOLS_IN_LINE 26        // 4239.4193,N,02316.2724,E\r\n
#define MAX_PARSED_TRAJ_POINT 20
#define READ_BUFFER_SIZE  ((uint16_t) (SYMBOLS_IN_LINE * MAX_PARSED_TRAJ_POINT))

typedef struct TrajectoryInfo
{
    int32_t i32Latitude;    // Latitude
    int32_t i32Longitude;   // Longitude
} tTrajectoryInfo;

/**
 * Get trajectory points from SD card
 * 
 * Parameters: Pointer of the file that is stored in the SD card
 * 
 * Return value: 0 - error during parsing information
 *               1 - parsing is successfully
 */
void GetTrajectoryFromSDCard(FIL * fileTrajectory);

