
#include "navigator.h"



tTrajectoryInfo asTrajectoryPoints[MAX_PARSED_TRAJ_POINT];
uint8_t u8NumTrajectoryPoints = 0;

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

