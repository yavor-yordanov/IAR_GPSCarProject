
#include "pwm_motor_control.h"
#include "tm_stm32f4_servo.h"
#include "navigator.h"
#include "ComParser.h"
#include "math.h"

tTrajectoryInfo asTrajectoryPoints[MAX_PARSED_TRAJ_POINT];
static uint8_t u8NumTrajectoryPoints = 0;      //Total points from trajectory
static uint8_t u8TargetPoint = 0;              //Current target point
//static uint8_t u8MotorSpeedOld = 0;
static uint8_t u8WheelDegreesOld = MID_WHEEL_DEGREES;
//static uint8_t u8WheelDirectionOld = CCW_Dir;
//Servo object
TM_SERVO_t Servo;
// Motor object
Motor_t Motor;

//Definition of static functions
static uint16_t CalculateDistance(int32_t i32CurrLatitude, int32_t i32CurrLongitude, 
                                  int32_t i32TargLatitude, int32_t i32TargLongitude);
static double CalculateBearing(int32_t i32CurrLatitude, int32_t i32CurrLongitude, 
                               int32_t i32TargLatitude, int32_t i32TargLongitude);
static void CalculateWheelAngle(uint8_t u8TurnDirection, double dTurnAngle);

void NaviInit(void)
{
  /* Initialize pwn servo, TIM2, Channel 3, Pinspack 2 = PB10 */
  TM_SERVO_Init(&Servo, TIM2, TM_PWM_Channel_3, TM_PWM_PinsPack_2);
  TM_SERVO_SetDegrees(&Servo, MID_WHEEL_DEGREES);
  
  /* Initialize pwm motor, TIM3, Channel 2, Pinspack 1 = PA7 */
  PWM_Motor_Init(&Motor, TIM3, TM_PWM_Channel_2, TM_PWM_PinsPack_1);
  PWM_Motor_SetMode(CCW_Dir);
  PWM_Motor_SetDuty(&Motor, 0);
}

void NaviProcess(tNaviInfo * pNaviInfo)
{
  tGPSInfo CurrentGPSInfo;
  uint16_t u16Distance = 0;
  double   dDirCurrentTarget = 0;
  double   dDiffBearing = 0;
  //uint8_t  u8MotorSpeed = 0;
  uint8_t  u8WheelDegrees = MID_WHEEL_DEGREES;
  uint8_t  u8TurnDirection;
  double   dTurnAngle;
  //uint8_t  u8WheelDirection = CCW_Dir;

  GetGPSInfoData(&CurrentGPSInfo);

  if((u8TargetPoint <= u8NumTrajectoryPoints) && (VALID_GPS_DATA == CurrentGPSInfo.u8ValidityFlag))
  {
    u16Distance = CalculateDistance(CurrentGPSInfo.i32Latitude, CurrentGPSInfo.i32Longitude, 
                                    asTrajectoryPoints[u8TargetPoint].i32Latitude, asTrajectoryPoints[u8TargetPoint].i32Longitude);
    dDirCurrentTarget = CalculateBearing(CurrentGPSInfo.i32Latitude, CurrentGPSInfo.i32Longitude, 
                                    asTrajectoryPoints[u8TargetPoint].i32Latitude, asTrajectoryPoints[u8TargetPoint].i32Longitude);
    
    //TODO: Calculation of u8WheelDegrees
    if(abs(u16Distance) > 5)
    {
      // If distance to target point is bigger than 5 x 18.5 = 92.5 cm then 
      // we should to continue with calculation of navigation

      dDiffBearing = dDirCurrentTarget - CurrentGPSInfo.dDirection;
      
      //Calculate turn direction
      if(0 < dDiffBearing)
      {
        //left turn
        u8TurnDirection = LEFT_TURN;
        dTurnAngle = dDiffBearing;
      }
      else
      {
        if(fabs(dDiffBearing) > 180)
        {
          //left turn
          u8TurnDirection = LEFT_TURN;
          dTurnAngle = 360 - fabs(dDiffBearing);
        }
        else
        {
          //right turn
          u8TurnDirection = RIGHT_TURN;
          dTurnAngle = fabs(dDiffBearing);
        }
      }
      
      CalculateWheelAngle(u8TurnDirection, dTurnAngle);

      PWM_Motor_SetMode(CCW_Dir);
      PWM_Motor_SetDuty(&Motor, 50);
      TM_SERVO_SetDegrees(&Servo, u8WheelDegreesOld); //u8WheelDegrees);
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
    //Car has passed through all setpoints
    PWM_Motor_SetDuty(&Motor, 0);
    TM_SERVO_SetDegrees(&Servo, MID_WHEEL_DEGREES);
  }
}

static void CalculateWheelAngle(uint8_t u8TurnDirection, double dTurnAngle)
{
  double  dPercentOfTurn;
  double  dWheelDegreesTurn;

  dPercentOfTurn = dTurnAngle/180;
  dWheelDegreesTurn = dPercentOfTurn * MAX_TURN_DEGREES_IN_DIR;
  
  if(LEFT_TURN == u8TurnDirection)
  {
    u8WheelDegreesOld -= dWheelDegreesTurn;
  }
  else
  {
    u8WheelDegreesOld += dWheelDegreesTurn;
  }
  
  if(u8WheelDegreesOld > MAX_WHEEL_DEGREES)
  {
    u8WheelDegreesOld = MAX_WHEEL_DEGREES;
  }
  else
  if(u8WheelDegreesOld < MIN_WHEEL_DEGREES)
  {
    u8WheelDegreesOld = MIN_WHEEL_DEGREES;
  }
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
  float   fAngleATAN;
  double  returnValue;
  int32_t i32Diff_X_Longitude = i32TargLongitude - i32CurrLongitude;
  int32_t i32Diff_Y_Latitude = i32TargLatitude - i32CurrLatitude;
  
  
  if((0 == i32Diff_X_Longitude) && (0 == i32Diff_Y_Latitude))
  {
    //Current position is the same as the target position
    returnValue = 0;
  }
  else
  {
    if(0 == i32Diff_Y_Latitude)
    {
      if(i32TargLongitude > i32CurrLongitude)
      {
        //direction is equal to x - axis
        returnValue = 90;
      }
      else
      {
        //direction is equal to -x - axis
        returnValue = 270;
      }
    }
    else
    if(0 == i32Diff_X_Longitude)
    {
      if(i32TargLatitude > i32CurrLatitude)
      {
        //direction is equal to y - axis
        returnValue = 0;
      }
      else
      {
        //direction is equal to -y - axis
        returnValue = 180;
      }
    }
    else
    {
      fAngleATAN = atan(abs(i32Diff_X_Longitude)/abs(i32Diff_Y_Latitude));
      
      if(i32Diff_X_Longitude > 0)
      {
        if(i32Diff_Y_Latitude > 0)
        {
          returnValue = fAngleATAN;
        }
        else
        {
          returnValue = 180 - fAngleATAN;
        }
      }
      else
      {
        if(i32Diff_Y_Latitude > 0)
        {
          returnValue = 360 - fAngleATAN;
        }
        else
        {
          returnValue = 180 + fAngleATAN;
        }
      }
    }
  }
  
  return returnValue;
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

