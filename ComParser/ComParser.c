#include <string.h>
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

static uint8_t isDigit(char cChar);
uint8_t  u8CurrentServo1Degrees = 50;
volatile uint8_t  u8Index = 0;

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

uint8_t GetRMCStatus(void)
{
  return u8RMC_Mess_Ready;
}

void GetRMCMessage(char string[])
{
  uint8_t u8Index = 0;
  
  memcpy((void *)string, (void const *)RMC_MESSAGE, MAX_NMEA_LENGTH);
  memset((void *)RMC_MESSAGE, 0, MAX_NMEA_LENGTH);
  
  /*
  for(u8Index = 0; u8Index < MAX_NMEA_LENGTH; ++u8Index)
  {
    string[u8Index] = RMC_MESSAGE[u8Index];
  }
  
  for(u8Index = 0; u8Index < MAX_NMEA_LENGTH; ++u8Index)
  {
    RMC_MESSAGE[u8Index] = 0;
  }
  */
  
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
  
  /*
  for(u8Index = 0; u8Index < MAX_NMEA_LENGTH; ++u8Index)
  {
    string[u8Index] = GGA_MESSAGE[u8Index];
  }
  
  for(u8Index = 0; u8Index < MAX_NMEA_LENGTH; ++u8Index)
  {
    GGA_MESSAGE[u8Index] = 0;
  }
  */
  
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
