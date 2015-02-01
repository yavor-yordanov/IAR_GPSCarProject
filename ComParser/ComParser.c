#include "ComParser.h"

// Led commands
const char G_LED_ON[LENGTH_COMMAND_9] = "#G_LED ON";
const char G_LED_OFF[LENGTH_COMMAND_10] = "#G_LED OFF";
const char O_LED_ON[LENGTH_COMMAND_9] = "#O_LED ON";
const char O_LED_OFF[LENGTH_COMMAND_10] = "#O_LED OFF";
const char R_LED_ON[LENGTH_COMMAND_9] = "#R_LED ON";
const char R_LED_OFF[LENGTH_COMMAND_10] = "#R_LED OFF";
const char B_LED_ON[LENGTH_COMMAND_9] = "#B_LED ON";
const char B_LED_OFF[LENGTH_COMMAND_10] = "#B_LED OFF";
// Mount SD card command
const char MOUNT_SD_CARD[LENGTH_COMMAND_9] = "#MOUNT SD";
// Unmount SD card command
const char UNMOUNT_SD_CARD[LENGTH_COMMAND_11] = "#UNMOUNT SD";
// Start save of GPS data
const char SAVE_GPS_DATA_ON[LENGTH_COMMAND_12] = "#SAVE_GPS ON";
// Stop save of GPS data
const char SAVE_GPS_DATA_OFF[LENGTH_COMMAND_13] = "#SAVE_GPS OFF";
// Start auto navigation
const char START_AUTO_NAVIGATION[LENGTH_COMMAND_13] = "#AUTO_NAVI ON";
// Stop auto navigation
const char STOP_AUTO_NAVIGATION[LENGTH_COMMAND_14] = "#AUTO_NAVI OFF";
// Manual control on
const char MANUAL_CONTROL_ON[LENGTH_COMMAND_13] = "#MAN_CONTR ON";
// Manual control off
const char MANUAL_CONTROL_OFF[LENGTH_COMMAND_14] = "#MAN_CONTR OFF";
// Rotation degrees
const char ROTATION_DEGREES[LENGTH_COMMAND_12] = "#ROT_DEG 000";
// RMC message ID
const char RMC_MESSAGE_ID[LENGTH_RMC_ID] = "$GPRMC";
// GGA message ID
const char GGA_MESSAGE_ID[LENGTH_GGA_ID] = "$GPGGA";


static uint8_t u8USART3_BufferIndex = 0;
static uint8_t u8USART2_BufferIndex = 0;
static uint8_t u8ReadyFlag = NOT_READY;
static uint8_t u8GPSReadyFlag = NOT_READY;
volatile char USART3_DynamicBuffer[MAX_COMMAND_LENGTH];
volatile char USART2_DynamicBuffer[MAX_NMEA_LENGTH];
volatile char USART3_Buffer[MAX_COMMAND_LENGTH];
volatile char USART2_Buffer[MAX_NMEA_LENGTH];
uint8_t u8RMC_Mess_Ready = 0;
char RMC_MESSAGE[MAX_NMEA_LENGTH];
uint8_t u8GGA_Mess_Ready = 0;
char GGA_MESSAGE[MAX_NMEA_LENGTH];
tGPSInfo sGPSInfo;

static uint8_t isDigit(char cChar);
static uint8_t isHex(char cChar);
static uint8_t HexToDec(char cChar);
uint8_t  u8CurrentServo1Degrees = 50;
volatile uint8_t u8Index = 0;

static uint8_t isDigit(char cChar)
{
  if(('0' <= cChar) && ('9' >= cChar))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

static uint8_t isHex(char cChar)
{
  if((('0' <= cChar) && ('9' >= cChar)) ||
     (('A' <= cChar) && ('F' >= cChar)))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

static uint8_t HexToDec(char cChar)
{
  if(('0' <= cChar) && ('9' >= cChar))
  {
    return cChar - '0';
  }
  else
  {
    return 10 + (cChar - 'A');
  }
}

void USART2_ReceivedChar(char cChar)
{ 
  if(cChar == '\n')
  {
    u8USART2_BufferIndex = 0;
    
    // ***** This block should be atomic *****
    memcpy((void *)USART2_Buffer, (void const *)USART2_DynamicBuffer, MAX_NMEA_LENGTH);
    memset((void *)USART2_DynamicBuffer, 0, MAX_NMEA_LENGTH);
    /*
    for(u8Index = 0; u8Index < MAX_NMEA_LENGTH; ++u8Index)
    {
      USART2_DynamicBuffer[u8Index] = 0;
    }
    */
    // ***************************************
    
    u8GPSReadyFlag = READY;
  }
  else
  {
    if(MAX_NMEA_LENGTH > u8USART2_BufferIndex)
    {
      USART2_DynamicBuffer[u8USART2_BufferIndex++] = cChar;
    }
  }
}

void USART3_ReceivedChar(char cChar)
{
  if(cChar == '\n')
  {
    u8USART3_BufferIndex = 0;
    
    // ***** This block should be atomic *****
    memcpy((void *)USART3_Buffer, (void const *)USART3_DynamicBuffer, MAX_COMMAND_LENGTH);
    memset((void *)USART3_DynamicBuffer, 0, MAX_COMMAND_LENGTH);
    // ***************************************
    
    u8ReadyFlag = READY;
  }
  else
  {
    if(MAX_COMMAND_LENGTH > u8USART3_BufferIndex)
    {
      USART3_DynamicBuffer[u8USART3_BufferIndex++] = cChar;
    }
  }
}

void GetCurrentServoDegrees(uint8_t *pu8Degrees)
{
  *pu8Degrees = u8CurrentServo1Degrees;
}

extern void GetGPSInfoData(tGPSInfo const * pGPSData)
{
  memcpy((void *)pGPSData, (void const *)&sGPSInfo, sizeof(tGPSInfo));
}

uint8_t GetRMCStatus(void)
{
  return u8RMC_Mess_Ready;
}

void GetRMCMessage(char string[])
{
  uint8_t u8Index = 0;

  memcpy((void *)string, (void const *)RMC_MESSAGE, MAX_NMEA_LENGTH);
  //memset((void *)RMC_MESSAGE, 0, MAX_NMEA_LENGTH);

  u8RMC_Mess_Ready = 0;
}

uint8_t GetGGAStatus(void)
{
  return u8GGA_Mess_Ready;
}

void GetGGAMessage(char string[])
{
  uint8_t u8Index = 0;

  memcpy((void *)string, (void const *)GGA_MESSAGE, MAX_NMEA_LENGTH);
  memset((void *)GGA_MESSAGE, 0, MAX_NMEA_LENGTH);

  u8GGA_Mess_Ready = 0;
}

void ParseGPSData(void)
{
  uint8_t u8Length = 0;
  
  if(READY == u8GPSReadyFlag)
  {
    while(('\r' != USART2_Buffer[u8Length]) && ((MAX_NMEA_LENGTH - 1) > u8Length))
    {
      u8Length++;
    }
    
    // Replace symbol \r with \n because file system add auto symbol \r
    USART2_Buffer[u8Length] = '\n';

    if(0 == memcmp((void const *)USART2_Buffer, RMC_MESSAGE_ID, LENGTH_RMC_ID))
    {
      memcpy((void *)RMC_MESSAGE, (void const *)USART2_Buffer, MAX_NMEA_LENGTH);
      u8RMC_Mess_Ready = 1;
    }
    else
    if(0 == memcmp((void const *)USART2_Buffer, GGA_MESSAGE_ID, LENGTH_GGA_ID))
    {
      memcpy((void *)GGA_MESSAGE, (void const *)USART2_Buffer, MAX_NMEA_LENGTH);
      u8GGA_Mess_Ready = 1;
    }
    
    //memset((void *)USART2_Buffer, 0, MAX_NMEA_LENGTH);

    u8GPSReadyFlag = NOT_READY;
  }
}

uint8_t ParseRMCMessage(void)
{
  uint8_t u8Index;
  uint8_t u8SpeedIndex;
  uint8_t u8SpeedIndexArray;
  uint8_t u8COGIndex;
  uint8_t u8COGIndexArray;
  uint8_t u8CalcCRCValue;
  uint8_t u8ReceivedCRCValue;
  uint8_t u8ChecksumResult = CHECKSUM_OR_VALIDITY_ERR;
  uint8_t u8CountComma = 0;
  char cSpeedArray[MAX_SPEED_PAR_LENGHT];
  char cCOGArray[MAX_COG_PAR_LENGHT];

  u8CalcCRCValue = RMC_MESSAGE[1];
  for(u8Index = 2; u8Index < MAX_NMEA_LENGTH; u8Index++)
  {
    if('*' == RMC_MESSAGE[u8Index])
    {
      //Get received checksum
      if(isHex(RMC_MESSAGE[u8Index+1]) && isHex(RMC_MESSAGE[u8Index+2]))
      {
        u8ReceivedCRCValue = HexToDec(RMC_MESSAGE[u8Index+1]) << 4;
        u8ReceivedCRCValue |= HexToDec(RMC_MESSAGE[u8Index+2]);
        //Compare calculated and received checksum value
        if(u8ReceivedCRCValue == u8CalcCRCValue)
        {
          u8ChecksumResult = CHECKSUM_OK;
        }
      }
      
      break;
    }
    else
    {
      //Calculate checksum
      u8CalcCRCValue ^= RMC_MESSAGE[u8Index];
    }
  }
  
  if(CHECKSUM_OK == u8ChecksumResult)
  {
    for(u8Index = 0; '*' != RMC_MESSAGE[u8Index]; u8Index++)
    {
      if(',' == RMC_MESSAGE[u8Index])
      {
        u8CountComma++;

        //$GPRMC,172454.000,A,4239.9114,N,02322.4062,E,0.17,0.00,190115,,,A*64 - with GPS fix
        //$GPRMC,235950.799,V,,,,,0.00,0.00,050180,,,N*4E                      - without GPS fix
        switch(u8CountComma)
        {
        case RMC_UTC:
          //example: ....,172454.000,.....  ;  ,,
          if(',' != RMC_MESSAGE[u8Index+1])   //next symbol is different from ',' (empty parameter)
          {
            sGPSInfo.sDateTime.hour = (10*(RMC_MESSAGE[u8Index+1]-'0')) + (RMC_MESSAGE[u8Index+2]-'0');
            sGPSInfo.sDateTime.min  = (10*(RMC_MESSAGE[u8Index+3]-'0')) + (RMC_MESSAGE[u8Index+4]-'0');
            sGPSInfo.sDateTime.sec  = (10*(RMC_MESSAGE[u8Index+5]-'0')) + (RMC_MESSAGE[u8Index+6]-'0');
          }
          else
          {
            sGPSInfo.sDateTime.hour = 0;
            sGPSInfo.sDateTime.min  = 0;
            sGPSInfo.sDateTime.sec  = 0;
          }
          break;
        case RMC_DATA_VALID:
          //example: ...,A,...  ;  ...,V,...  ;  ,,
          if(',' != RMC_MESSAGE[u8Index+1])
          {
            if('A' ==RMC_MESSAGE[u8Index+1])
            {
              sGPSInfo.u8ValidityFlag = 1;
            }
            else
            {
              sGPSInfo.u8ValidityFlag = 0;
            }
          }
          else
          {
            sGPSInfo.u8ValidityFlag = 0;
          }
          break;
        case RMC_LATITUDE:
          //Parase North-South parameter together with latitude (RMC_N_S)
          //example: ...,4239.9114,N,... ; ...,,,...  ‘ddmm.mmmm’ (degree and minutes)
          if(',' != RMC_MESSAGE[u8Index+1])
          {
            sGPSInfo.d32Latitude = 60*((10*(RMC_MESSAGE[u8Index+1]-'0')) + (RMC_MESSAGE[u8Index+2]-'0')) + 
                                      ((10*(RMC_MESSAGE[u8Index+3]-'0')) + (RMC_MESSAGE[u8Index+4]-'0')) +
                                      (((1000*(RMC_MESSAGE[u8Index+6]-'0')) + (100*(RMC_MESSAGE[u8Index+7]-'0')) +
                                       (10*(RMC_MESSAGE[u8Index+8]-'0')) + (RMC_MESSAGE[u8Index+9]-'0'))/10000.0);
            if('S' == RMC_MESSAGE[u8Index+11])
            {
              sGPSInfo.d32Latitude *= -1;
            }
          }
          else
          {
            sGPSInfo.d32Latitude = 0;
          }
          break;
        case RMC_LONGITUDE:
          //Parse East-West parameter together with Longitude (RMC_E_W)
          //example: ...,02322.4062,E,... ; ...,,,... ‘dddmm.mmmm’ (degree and minutes)
          if(',' != RMC_MESSAGE[u8Index+1])
          {
            sGPSInfo.d32Longitude = 60*((100*(RMC_MESSAGE[u8Index+1]-'0')) + (10*(RMC_MESSAGE[u8Index+2]-'0')) + (RMC_MESSAGE[u8Index+3]-'0')) + 
                                      ((10*(RMC_MESSAGE[u8Index+4]-'0')) + (RMC_MESSAGE[u8Index+5]-'0')) +
                                      (((1000*(RMC_MESSAGE[u8Index+7]-'0')) + (100*(RMC_MESSAGE[u8Index+8]-'0')) +
                                       (10*(RMC_MESSAGE[u8Index+9]-'0')) + (RMC_MESSAGE[u8Index+10]-'0'))/10000.0);
            if('W' == RMC_MESSAGE[u8Index+12])
            {
              sGPSInfo.d32Longitude *= -1;
            }
          }
          else
          {
            sGPSInfo.d32Longitude = 0;
          }
          break;
        case RMC_SPEED:
          //example: ...,0.17,...  ;  ...,0.00,...
          if(',' != RMC_MESSAGE[u8Index+1])
          {
            u8SpeedIndex = u8Index+1;
            u8SpeedIndexArray = 0;
            
            while((',' != RMC_MESSAGE[u8SpeedIndex]) && (MAX_SPEED_PAR_LENGHT > u8SpeedIndexArray))
            {
              cSpeedArray[u8SpeedIndexArray] = RMC_MESSAGE[u8SpeedIndex];
              u8SpeedIndexArray++;
              u8SpeedIndex++;
            }
            
            sGPSInfo.dSpeed = KNOTS_TO_KM_H * atof(cSpeedArray);  //convert knots to km/h
          }
          else
          {
            sGPSInfo.dSpeed = 0;
          }
          break;
        case RMC_COG:
          //example: ...,0.00,...  ;  ...,357,64,...   ;  ...,,...          
          if(',' != RMC_MESSAGE[u8Index+1])
          {
            u8COGIndex = u8Index+1;
            u8COGIndexArray = 0;
            
            while((',' != RMC_MESSAGE[u8COGIndex]) && (MAX_COG_PAR_LENGHT > u8COGIndexArray))
            {
              cCOGArray[u8COGIndexArray] = RMC_MESSAGE[u8COGIndex];
              u8COGIndexArray++;
              u8COGIndex++;
            }
            
            sGPSInfo.dDirection = atof(cCOGArray);
          }
          else
          {
            sGPSInfo.dDirection = 0;
          }          
          break;
        case RMC_DATE:
          //example: ...,190115,... ; ...,,...
          if(',' != RMC_MESSAGE[u8Index+1])   //next symbol is different from ',' (empty parameter)
          {
            sGPSInfo.sDateTime.day  = (10*(RMC_MESSAGE[u8Index+1]-'0')) + (RMC_MESSAGE[u8Index+2]-'0');
            sGPSInfo.sDateTime.mon  = (10*(RMC_MESSAGE[u8Index+3]-'0')) + (RMC_MESSAGE[u8Index+4]-'0');
            sGPSInfo.sDateTime.year = 2000 + (10*(RMC_MESSAGE[u8Index+5]-'0')) + (RMC_MESSAGE[u8Index+6]-'0');
          }
          else
          {
            sGPSInfo.sDateTime.day  = 1;
            sGPSInfo.sDateTime.mon  = 1;
            sGPSInfo.sDateTime.year = 2000;
          }
          break;
        case RMC_MAG_VAR:
        case RMC_MAG_VAR_EW:
        case RMC_POS_MODE:
          break;
        }
      }
    }
  }

  return u8ChecksumResult;
}

void ParseCommand(void)
{
  uint8_t u8Length = 0;
  
  if(READY == u8ReadyFlag)
  {
    while(('\r' != USART3_Buffer[u8Length]) && ((MAX_COMMAND_LENGTH - 1) > u8Length))
    {
      u8Length++;
    }

    if(LENGTH_COMMAND_9 == u8Length)
    {
      if(0 == memcmp((void const *)USART3_Buffer, G_LED_ON, LENGTH_COMMAND_9))
      {
        SetGreenLed(Bit_SET);
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, O_LED_ON, LENGTH_COMMAND_9))
      {
        SetOrangeLed(Bit_SET);
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, R_LED_ON, LENGTH_COMMAND_9))
      {
        SetRedLed(Bit_SET);        
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, B_LED_ON, LENGTH_COMMAND_9))
      {
        SetBlueLed(Bit_SET);
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, MOUNT_SD_CARD, LENGTH_COMMAND_9))
      {
        //TODO: Call mount function for SD card
      }
    }
    else
    if(LENGTH_COMMAND_10 == u8Length)
    {
      if(0 == memcmp((void const *)USART3_Buffer, G_LED_OFF, LENGTH_COMMAND_10))
      {
        SetGreenLed(Bit_RESET);
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, O_LED_OFF, LENGTH_COMMAND_10))
      {
        SetOrangeLed(Bit_RESET);        
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, R_LED_OFF, LENGTH_COMMAND_10))
      {
        SetRedLed(Bit_RESET);        
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, B_LED_OFF, LENGTH_COMMAND_10))
      {
        SetBlueLed(Bit_RESET);
      }
    }
    else
    if(LENGTH_COMMAND_11 == u8Length)
    {
      if(0 == memcmp((void const *)USART3_Buffer, UNMOUNT_SD_CARD, LENGTH_COMMAND_11))
      {
        //TODO: Call unmount function for SD card
      }
    }
    else
    if(LENGTH_COMMAND_12 == u8Length)
    {
      if(0 == memcmp((void const *)USART3_Buffer, SAVE_GPS_DATA_ON, LENGTH_COMMAND_12))
      {
        //TODO: Activate save of GPS data in the SD card
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, ROTATION_DEGREES, LENGTH_COMMAND_9)) //Compare only first nine chars because others are degrees value
      {
        //TODO: Set degrees of servo for wheel rotation
        // 9, 10, 11
        if(isDigit(USART3_Buffer[9]) && isDigit(USART3_Buffer[10]) && isDigit(USART3_Buffer[11]))
        {
          u8CurrentServo1Degrees = (100*(USART3_Buffer[9] - '0')) + (10*(USART3_Buffer[10] - '0')) + (USART3_Buffer[11] - '0');
        }
      }
    }
    else
    if(LENGTH_COMMAND_13 == u8Length)
    {
      if(0 == memcmp((void const *)USART3_Buffer, SAVE_GPS_DATA_OFF, LENGTH_COMMAND_13))
      {
        //TODO: Stop logging of GPS data in the SD card
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, START_AUTO_NAVIGATION, LENGTH_COMMAND_13))
      {
        //TODO: Start auto navigation process
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, MANUAL_CONTROL_ON, LENGTH_COMMAND_13))
      {
        //TODO: Activate manual control
      }
    }
    else
    if(LENGTH_COMMAND_14 == u8Length)
    {
      if(0 == memcmp((void const *)USART3_Buffer, STOP_AUTO_NAVIGATION, LENGTH_COMMAND_13))
      {
        //TODO: Stop auto navigation process
      }
      else
      if(0 == memcmp((void const *)USART3_Buffer, MANUAL_CONTROL_OFF, LENGTH_COMMAND_13))
      {
        //TODO: Stop manual control
      }
    }

    u8ReadyFlag = NOT_READY;
  }
}
