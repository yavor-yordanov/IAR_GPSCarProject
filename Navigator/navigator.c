
#include "navigator.h"
#include "ComParser.h"
#include "math.h"

tTrajectoryInfo asTrajectoryPoints[MAX_PARSED_TRAJ_POINT];
static uint8_t u8NumTrajectoryPoints = 0;      //Total points from trajectory
static uint8_t u8TargetPoint = 0;              //Current target point
static uint8_t u8MotorSpeedOld = 0;
static uint8_t u8WheelDegreesOld = MID_WHEEL_DEGREES;
static uint8_t u8WheelDirectionOld = CCW_Dir;

//Definition of static functions
static uint16_t CalculateDistance(int32_t i32CurrLatitude, int32_t i32CurrLongitude, 
                                  int32_t i32TargLatitude, int32_t i32TargLongitude);
static double CalculateBearing(int32_t i32CurrLatitude, int32_t i32CurrLongitude, 
                               int32_t i32TargLatitude, int32_t i32TargLongitude);

void NaviProcess(tNaviInfo * pNaviInfo)
{
  tGPSInfo CurrentGPSInfo;
  uint16_t u16Distance = 0;
  double   dDirCurrentTarget = 0;
  double   dDiffBearing = 0;
  //uint8_t  u8MotorSpeed = 0;
  uint8_t  u8WheelDegrees = MID_WHEEL_DEGREES;
  //uint8_t  u8WheelDirection = CCW_Dir;

  GetGPSInfoData(&CurrentGPSInfo);

  if((u8TargetPoint <= u8NumTrajectoryPoints) && (VALID_GPS_DATA == CurrentGPSInfo.u8ValidityFlag))
  {
    u16Distance = CalculateDistance(CurrentGPSInfo.i32Latitude, CurrentGPSInfo.i32Longitude, 
                                    asTrajectoryPoints[u8TargetPoint].i32Latitude, asTrajectoryPoints[u8TargetPoint].i32Longitude);
    dDirCurrentTarget = CalculateBearing(CurrentGPSInfo.i32Latitude, CurrentGPSInfo.i32Longitude, 
                                    asTrajector;yPoints[u8TargetPoint].i32Latitude, asTrajectoryPoints[u8TargetPoint].i32Longitude);
    
    //TODO: Calculation of u8WheelDegrees
    if(u16Distance > 5)
    {
      // If distance to target point is bigger than 5 x 18.5 = 92.5 cm then 
      // we should to continue with calculation of navigation

      dDiffBearing = dDirCurrentTarget - CurrentGPSInfo.dDirection;
      
      if()
      {
        
      }
      
      PWM_Motor_SetMode(CCW_Dir);
      PWM_Motor_SetDuty(&Motor, 50);
      TM_SERVO_SetDegrees(&Servo, u8WheelDegrees);
    }
    else
    {
      // If distance to target point is less than 5 x 18.5 = 92.5 cm then 
      // we should to continue with navigation to the next target point from trajectory
      u8TargetPoint++;
    }
  }
  else
  {
    PWM_Motor_SetDuty(&Motor, 0);
    TM_SERVO_SetDegrees(&Servo, MID_WHEEL_DEGREES);
    //Car has passed through all setpoints
  }

  /*
  PWM_Motor_SetMode(CCW_Dir);
  PWM_Motor_SetDuty(&Motor, 10);
  TM_SERVO_SetDegrees(&Servo, 50);
  */
}

static uint16_t CalculateDistance(int32_t i32CurrLatitude, int32_t i32CurrLongitude, 
                                  int32_t i32TargLatitude, int32_t i32TargLongitude)
{
  return (uint16_t) sqrt(((i32CurrLatitude - i32TargLatitude)*(i32CurrLatitude - i32TargLatitude)) +
                         ((i32CurrLongitude - i32TargLongitude)*(i32CurrLongitude - i32TargLongitude)));
}

static double CalculateBearing(int32_t i32CurrLatitude, int32_t i32CurrLongitude, 
                               int32_t i32TargLatitude, int32_t i32TargLongitude)
{
  
}

void GetTrajectoryFromSDCard(FIL * fileTrajectory)
{
  uint16_t u16ReadBufferIndex = 0;
  uint8_t  u8ParseBufferIndex = 0;
  int16_t  i16TempCalc = 0;
  u8NumTrajectoryPoints = 0;
  UINT uReadBytes = 0;
  char readBuffer[READ_BUFFER_SIZE];
  char parseBuffer[30];
    
  f_read(fileTrajectory, readBuffer, READ_BUFFER_SIZE, &uReadBytes);
  
  for(u16ReadBufferIndex = 0; ((u16ReadBufferIndex < uReadBytes) && (u16ReadBufferIndex < READ_BUFFER_SIZE)); u16ReadBufferIndex++)
  {
    if(readBuffer[u16ReadBufferIndex] != '\n')
    {
      parseBuffer[u8ParseBufferIndex++] = readBuffer[u16ReadBufferIndex];
    }
    else
    {
      //TODO: parsed algorithm
      asTrajectoryPoints[u8NumTrajectoryPoints].i32Latitude = 60*((10*(parseBuffer[0]-'0')) + (parseBuffer[1]-'0')) + 
                                                                  ((10*(parseBuffer[2]-'0')) + (parseBuffer[3]-'0'));
      asTrajectoryPoints[u8NumTrajectoryPoints].i32Latitude *= 10000;
      
      i16TempCalc = (parseBuffer[5]-'0') * 10;
      i16TempCalc += (parseBuffer[6]-'0');
      i16TempCalc *= 10;
      i16TempCalc += (parseBuffer[7]-'0');
      i16TempCalc *= 10;
      i16TempCalc += (parseBuffer[8]-'0');
      
      asTrajectoryPoints[u8NumTrajectoryPoints].i32Latitude += i16TempCalc;
        
      if('S' == parseBuffer[10])
      {
        asTrajectoryPoints[u8NumTrajectoryPoints].i32Latitude *= -1;
      }

      asTrajectoryPoints[u8NumTrajectoryPoints].i32Longitude = 60*((100*(parseBuffer[12]-'0')) + (10*(parseBuffer[13]-'0')) + (parseBuffer[14]-'0')) + 
                                                                  ((10*(parseBuffer[15]-'0'))  + (parseBuffer[16]-'0'));
      asTrajectoryPoints[u8NumTrajectoryPoints].i32Longitude *= 10000;
      
      i16TempCalc = (parseBuffer[18]-'0') * 10;
      i16TempCalc += (parseBuffer[19]-'0');
      i16TempCalc *= 10;
      i16TempCalc += (parseBuffer[20]-'0');
      i16TempCalc *= 10;
      i16TempCalc += (parseBuffer[21]-'0');

      asTrajectoryPoints[u8NumTrajectoryPoints].i32Longitude += i16TempCalc;

      if('W' == parseBuffer[23])
      {
        asTrajectoryPoints[u8NumTrajectoryPoints].i32Longitude *= -1;
      }

      u8NumTrajectoryPoints++;  // increment number of trajectory points
      u8ParseBufferIndex = 0;   // set index for parse buffer to 0
    }
  }
}

