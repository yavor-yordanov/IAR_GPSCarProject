#include "defines.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "tm_stm32f4_fatfs.h"
#include "diskio.h"
#include "tm_stm32f4_servo.h"
#include "pwm_motor_control.h"
#include "tm_stm32f4_usart.h"
#include "nmea.h"
#include "stm32f407_LedDrv.h"
#include "ComParser.h"
#include "navigator.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

#define  MAX_STRLEN ((uint8_t) 12) // this is the maximum string length of our string in characters
#define   FLAG_10MS ((uint8_t)  1)
#define   FLAG_25MS ((uint8_t)  2)
#define   FLAG_50MS ((uint8_t)  4)
#define  FLAG_100MS ((uint8_t)  8)
#define  FLAG_200MS ((uint8_t) 16)
#define  FLAG_250MS ((uint8_t) 32)
#define  FLAG_500MS ((uint8_t) 64)
#define FLAG_1000MS ((uint8_t)128)

volatile uint32_t u32SystemTimer     = 0;   // milliseconds counter from start up (overflow after 49 days)
volatile uint8_t u8FlagForprocesses  = 0;
volatile uint32_t u32UserButtonValue = 0;

volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string

static void EnableSysTickTimer(void);
//static void InitUART4(uint32_t baudrate);
//static void InitUART5(uint32_t baudrate);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
/*volatile*/ char sendBuffer[] = "$GPRMC,201633.000,A,4239.9272,N,02321.8573,E,0.54,0.00,200114,,,A*65\r\n$GPVTG,0.00,T,,M,0.54,N,0.99,K,A*3C\r\n$GPGGA,201633.000,4239.9272,N,02321.8573,E,1,5,1.28,574.1,M,36.7,M,,*5B\r\n$GPGSA,A,3,12,24,28,26,15,,,,,,,,1.58,1.28,0.93*00\r\n$GPGSV,2,1,06,15,59,228,45,24,52,307,44,39,40,177,38,26,37,169,37*76\r\n$GPGSV,2,2,06,12,25,247,43,28,20,056,30*75\r\n$GPGLL,4239.9272,N,02321.8573,E,201633.000,A,A*55\r\n$GPTXT,01,01,02,ANTSTATUS=OPEN*2B\r\n";
volatile char *pointerSendBuffer;
char tempString[MAX_NMEA_LENGTH];
char tempInfoStatus[40];
tGPSInfo sParsedGPSData;

int main()
{
  uint8_t i;
  float temp;
  uint16_t u16Counter = 0;
  uint8_t  u8LedControlMask = 1;
  uint8_t  u8OldServoDegrees = 50;
  uint8_t  u8DegreesTempValue = 50;
  uint8_t  u8ServoSign = 1;            // 1 - plus, 0 - minus
  uint8_t  u8StateOfSDCard = 0;        // State of SD card 1 - Mount and Open file; 0 - Unmount and close file
  uint32_t u32TempUserButtonValue = 0;
  uint32_t u32TempTimer = 0;
  pointerSendBuffer = sendBuffer;
  nmeaINFO info;
  nmeaPARSER parser;
  
  //Fatfs object
  FATFS FatFs;
  //File objects
  FIL fil;
  FIL fileParsedValues;
  FIL fileTrajectory;
  //Free and total space
  uint32_t total, free;
  //Navi information
  tNaviInfo naviInfo;

  //Initialize system
  SystemInit();
  InitPinsForLeds();
  EnableSysTickTimer();
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
  
  TM_USART_Init(USART2, TM_USART_PinsPack_2, 9600);
  TM_USART_Init(USART3, TM_USART_PinsPack_3, 9600);
  //Put string to terminal
  TM_USART_Puts(USART3, "Hello world\n\r");

  //disk_initialize(0);
  NaviInit();

  while(u32SystemTimer < 5000)
  {
    //5 seconds delay before startup of the system
    ;
  }
  
  // Command to GPS receiver to enable only RMC and GGA messages
  TM_USART_Puts(USART2, "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");

  while(1)
  {
    ParseCommand();
    ParseGPSData();

    // 10ms process
    if(0 != (u8FlagForprocesses & FLAG_10MS))
    {
      //Get servo degrees from manual control
      GetCurrentServoDegrees(&u8DegreesTempValue);
      if(u8DegreesTempValue != u8OldServoDegrees)
      {
        u8OldServoDegrees = u8DegreesTempValue;
        if((50 <= u8OldServoDegrees) && (140 >= u8OldServoDegrees))
        {
          //TM_SERVO_SetDegrees(&Servo, u8OldServoDegrees);
        }
      }
      
      u32TempUserButtonValue = STM_EVAL_PBGetState(BUTTON_USER);
      if(u32TempUserButtonValue != u32UserButtonValue)
      {
        u32UserButtonValue = u32TempUserButtonValue;
        if(0 == u32UserButtonValue)
        {          
          if(0 != u8StateOfSDCard)
          {
            f_close(&fil);
            f_close(&fileParsedValues);
            f_mount(0, "", 1);
            u8StateOfSDCard = 0;
            SetBlueLed(Bit_RESET);
          }
          else
          {
            if(f_mount(&FatFs, "", 1) == FR_OK)
            {
              //Try to open files
              if((f_open(&fil, "log_file.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK) && 
                 (f_open(&fileParsedValues, "par_file.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK) &&
                 (f_open(&fileTrajectory, "TRJ_FILE.txt", FA_READ) == FR_OK))
              {
                //Take all the information from file and fills the array data.
                GetTrajectoryFromSDCard(&fileTrajectory);
                f_close(&fileTrajectory);

                u8StateOfSDCard = 1;
                SetBlueLed(Bit_SET);
              }
            }
          }
        }
      }

      u8FlagForprocesses &= (uint8_t)(~FLAG_10MS);
    }

    // 50ms process
    if(0 != (u8FlagForprocesses & FLAG_50MS))
    {
      //Flashing orange led indicates normal operation of the system
      ToggleOrangeLed();

      //TM_USART_Puts(USART2, "BEG\r\n");

      if(1 == u8StateOfSDCard)  // && (0 != GetRMCStatus()) && (0 != GetGGAStatus()))
      {
        if(0 != GetRMCStatus())
        {
          GetRMCMessage(tempString);
          f_puts(tempString, &fil);

          if(CHECKSUM_OK == ParseRMCMessage())
          {
            GetGPSInfoData(&sParsedGPSData);
            sprintf(tempInfoStatus, "Validity: %i\n", sParsedGPSData.u8ValidityFlag);
            f_puts(tempInfoStatus, &fileParsedValues);  //TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Latitude: %i\n", sParsedGPSData.i32Latitude);
            f_puts(tempInfoStatus, &fileParsedValues);  //TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Longitude: %i\n", sParsedGPSData.i32Longitude);
            f_puts(tempInfoStatus, &fileParsedValues);  //TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Speed: %f\n", sParsedGPSData.dSpeed);
            f_puts(tempInfoStatus, &fileParsedValues);  //TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Direction: %f\n", sParsedGPSData.dDirection);
            f_puts(tempInfoStatus, &fileParsedValues);  //TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Date: %i.%i.%i\n", sParsedGPSData.sDateTime.year, sParsedGPSData.sDateTime.mon, sParsedGPSData.sDateTime.day);
            f_puts(tempInfoStatus, &fileParsedValues);  //TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Hour: %i:%i:%i\n", sParsedGPSData.sDateTime.hour, sParsedGPSData.sDateTime.min, sParsedGPSData.sDateTime.sec);
            f_puts(tempInfoStatus, &fileParsedValues);  //TM_USART_Puts(USART2, tempInfoStatus);
          }
          else
          {
            TM_USART_Puts(USART2, "Error! Checksum check for RMC message failed!\n");
          }

          /*
          if(CHECKSUM_OK == ParseRMCMessage())
          {
            GetGPSInfoData(&sParsedGPSData);
            sprintf(tempInfoStatus, "Validity: %i\n", sParsedGPSData.u8ValidityFlag);
            TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Latitude: %i\n", sParsedGPSData.i32Latitude);
            TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Longitude: %i\n", sParsedGPSData.i32Longitude);
            TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Speed: %f\n", sParsedGPSData.dSpeed);
            TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Direction: %f\n", sParsedGPSData.dDirection);
            TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Date: %i.%i.%i\n", sParsedGPSData.sDateTime.year, sParsedGPSData.sDateTime.mon, sParsedGPSData.sDateTime.day);
            TM_USART_Puts(USART2, tempInfoStatus);
            sprintf(tempInfoStatus, "Hour: %i:%i:%i\n", sParsedGPSData.sDateTime.hour, sParsedGPSData.sDateTime.min, sParsedGPSData.sDateTime.sec);
            TM_USART_Puts(USART2, tempInfoStatus);
          }
          else
          {
            TM_USART_Puts(USART2, "Error! Checksum check for RMC message failed!\n");
          }
          */
        }

        if(0 != GetGGAStatus())
        {
          GetGGAMessage(tempString);
          f_puts(tempString, &fil);
          
          //u32TempTimer = u32SystemTimer;
          //TM_USART_Puts(USART2, "1234567890123456789012345678\r\n");          
          //if(0 < (u32SystemTimer - u32TempTimer))
          //{
          //  sprintf(tempInfoStatus, "%i\r\n", (u32SystemTimer-u32TempTimer));
          //  TM_USART_Puts(USART2, tempInfoStatus);
          //}

          if(CHECKSUM_OK == ParseGGAMessage())
          {
            GetGPSInfoData(&sParsedGPSData);
            sprintf(tempInfoStatus, "HDOP: %f\n", sParsedGPSData.fHDOP);
            f_puts(tempInfoStatus, &fileParsedValues);  //TM_USART_Puts(USART2, tempInfoStatus);
          }
          else
          {
            TM_USART_Puts(USART2, "Error! Checksum check for GGA message failed!\n");
          }

/*
          if(CHECKSUM_OK == ParseGGAMessage())
          {
            GetGPSInfoData(&sParsedGPSData);
            sprintf(tempInfoStatus, "HDOP: %f\n", sParsedGPSData.fHDOP);
            TM_USART_Puts(USART2, tempInfoStatus);
          }
          else
          {
            TM_USART_Puts(USART2, "Error! Checksum check for GGA message failed!\n");
          }
*/
        }
      }

      //TM_USART_Puts(USART2, "END\r\n");
      //TM_USART_Puts(USART2, "Hello world\n\r");
      u8FlagForprocesses &= (uint8_t)(~FLAG_50MS);
    }
    
    // 100ms process
    if(0 != (u8FlagForprocesses & FLAG_100MS))
    {
      
    }

    // 1s process
    if(0 != (u8FlagForprocesses & FLAG_1000MS))
    {
      GetGPSInfoData(&sParsedGPSData);
      /*
      sprintf(tempInfoStatus, "Validity: %i\n", sParsedGPSData.u8ValidityFlag);
      TM_USART_Puts(USART2, tempInfoStatus);
      sprintf(tempInfoStatus, "Latitude: %i\n", sParsedGPSData.i32Latitude);
      TM_USART_Puts(USART2, tempInfoStatus);
      sprintf(tempInfoStatus, "Longitude: %i\n", sParsedGPSData.i32Longitude);
      TM_USART_Puts(USART2, tempInfoStatus);
      sprintf(tempInfoStatus, "Speed: %f\n", sParsedGPSData.dSpeed);
      TM_USART_Puts(USART2, tempInfoStatus);
      sprintf(tempInfoStatus, "Direction: %f\n", sParsedGPSData.dDirection);
      TM_USART_Puts(USART2, tempInfoStatus);
      sprintf(tempInfoStatus, "Date: %i.%i.%i\n", sParsedGPSData.sDateTime.year, sParsedGPSData.sDateTime.mon, sParsedGPSData.sDateTime.day);
      TM_USART_Puts(USART2, tempInfoStatus);
      sprintf(tempInfoStatus, "Hour: %i:%i:%i\n", sParsedGPSData.sDateTime.hour, sParsedGPSData.sDateTime.min, sParsedGPSData.sDateTime.sec);
      TM_USART_Puts(USART2, tempInfoStatus);
      GetGPSInfoData(&sParsedGPSData);
      sprintf(tempInfoStatus, "HDOP: %f\n", sParsedGPSData.fHDOP);
      TM_USART_Puts(USART2, tempInfoStatus);
      */

      NaviProcess(&naviInfo);

      /*
      if(28 >= u16Counter)
      {
        switch(u16Counter)
        {
        case 1:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 10);
          TM_SERVO_SetDegrees(&Servo, 50);
          break;
        case 2:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 20);
          TM_SERVO_SetDegrees(&Servo, 60);
          break;
        case 3:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 30);
          TM_SERVO_SetDegrees(&Servo, 70);
          break;
        case 4:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 40);
          TM_SERVO_SetDegrees(&Servo, 80);
          break;
        case 5:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 50);
          TM_SERVO_SetDegrees(&Servo, 90);
          break;
        case 6:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 60);
          TM_SERVO_SetDegrees(&Servo, 100);
          break;  
        case 7:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 70);
          TM_SERVO_SetDegrees(&Servo, 110);
          break;
        case 8:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 60);
          TM_SERVO_SetDegrees(&Servo, 120);
          break;
        case 9:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 50);
          TM_SERVO_SetDegrees(&Servo, 130);
          break;
        case 10:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 40);
          TM_SERVO_SetDegrees(&Servo, 140);
          break;
        case 11:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 30);
          TM_SERVO_SetDegrees(&Servo, 130);
          break;
        case 12:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 20);
          TM_SERVO_SetDegrees(&Servo, 120);
          break;
        case 13:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 10);
          TM_SERVO_SetDegrees(&Servo, 110);
          break;
        case 14:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 0);
          TM_SERVO_SetDegrees(&Servo, 100);
          break;
        case 15:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 10);
          TM_SERVO_SetDegrees(&Servo, 90);
          break;
        case 16:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 20);
          TM_SERVO_SetDegrees(&Servo, 80);
          break;
        case 17:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 30);
          TM_SERVO_SetDegrees(&Servo, 70);
          break;
        case 18:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 40);
          TM_SERVO_SetDegrees(&Servo, 60);
          break;
        case 19:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 50);
          TM_SERVO_SetDegrees(&Servo, 50);
          break;
        case 20:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 60);
          TM_SERVO_SetDegrees(&Servo, 60);
          break;  
        case 21:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 70);
          TM_SERVO_SetDegrees(&Servo, 70);
          break;
        case 22:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 60);
          TM_SERVO_SetDegrees(&Servo, 80);
          break;
        case 23:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 50);
          TM_SERVO_SetDegrees(&Servo, 90);
          break;
        case 24:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 40);
          TM_SERVO_SetDegrees(&Servo, 100);
          break;
        case 25:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 30);
          TM_SERVO_SetDegrees(&Servo, 90);
          break;
        case 26:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 20);
          TM_SERVO_SetDegrees(&Servo, 80);
          break;
        case 27:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 10);
          TM_SERVO_SetDegrees(&Servo, 70);
          break;
        case 28:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 0);
          TM_SERVO_SetDegrees(&Servo, 60);
          break;
        default:
          break;
        }

        u16Counter++;
      }
      else
      {
        u16Counter = 0;
      }
      */
      
      u8FlagForprocesses &= (uint8_t)(~FLAG_1000MS);
    }
  }
}

void SysTick_Handler(void)
{
  u32SystemTimer++; // increment every milisecond - overflow after ~49 days
  
  if(0 == (u32SystemTimer % 1000)) //set bit which indicated elapsed of 1000ms
  {
    u8FlagForprocesses |= FLAG_1000MS;
  }

  if(0 == (u32SystemTimer % 250)) //set bit which indicated elapsed of 250ms
  {
    u8FlagForprocesses |= FLAG_250MS;
  }

  if(0 == (u32SystemTimer % 100)) // set bit which indicated elapsed of 100ms
  {
    u8FlagForprocesses |= FLAG_100MS;
  }
  
  if(0 == (u32SystemTimer % 50))
  {
    u8FlagForprocesses |= FLAG_50MS;
  }

  if(0 == (u32SystemTimer % 10)) //set bit which indicated elapsed of 10ms
  {
    u8FlagForprocesses |= FLAG_10MS;
  }
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s)
{
  while(*s)
  {
    // wait until data register is empty
    while( !(USARTx->SR & 0x00000040) );
    USART_SendData(USARTx, *s);
    *s++;
  }
}
/*
void UART4_IRQHandler(void)
{
  if( USART_GetITStatus(UART4, USART_IT_RXNE) )
  {
    static uint8_t cnt = 0; // this counter is used to determine the string length
    char t = UART4->DR;     // the character from the UART4 data register is saved in t

      //check if the received character is not the LF character (used to determine end of string)
      //or the if the maximum string length has been been reached
     
    if( (t != 'n') && (cnt < MAX_STRLEN) )
    {
      received_string[cnt] = t;
      cnt++;
    }
    else
    { 
      // otherwise reset the character counter and print the received string
      cnt = 0;
      USART_puts(UART4, received_string);
    }
  }
}

void UART5_IRQHandler(void)
{
  if( USART_GetITStatus(UART5, USART_IT_RXNE) )
  {
    static uint8_t cnt = 0; // this counter is used to determine the string length
    char t = UART5->DR;     // the character from the UART4 data register is saved in t

    //check if the received character is not the LF character (used to determine end of string)
    //or the if the maximum string length has been been reached

    if( (t != 'n') && (cnt < MAX_STRLEN) )
    {
      received_string[cnt] = t;
      cnt++;
    }
    else
    { 
      // otherwise reset the character counter and print the received string
      cnt = 0;
      USART_puts(UART5, received_string);
    }
  }
}*/

static void EnableSysTickTimer(void)
{
  // Enable interrupt for system timer
  if(0 == SysTick_Config((uint32_t) 168000))    //number of ticks per one millisecond
  {
    NVIC_EnableIRQ(SysTick_IRQn);
  }
}

/* This function initializes UART4
 *
 * Arguments: baudrate 
 */
/*
static void InitUART4(uint32_t baudrate)
{
  GPIO_InitTypeDef  GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
  USART_InitTypeDef USART_InitStruct;  // this is for the UART4 initialization
  NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
  
  // enable APB1 peripheral clock for UART4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  // enable the peripheral clock for the pins used by UART4: PA0 for TX and PA1 for RX
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  // Sets up the TX and RX pins so they work correctly with the UART4 peripheral
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // Pins 0 (TX) and 1 (RX) are used
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;           // the pins are configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;      // this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;         // this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;           // this activates the pull up resistors on the IO pins
  GPIO_Init(GPIOA, &GPIO_InitStruct);                 // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

   // The RX and TX pins are now connected to their AF so the 
   // UART4 can take over control of the pins
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

  // Now the USART_InitStruct is used to define the properties of UART4
  USART_InitStruct.USART_BaudRate = baudrate;              // the baudrate is set to the value we passed into this init function
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; // we want the data frame size to be 8 bits (standard)
  USART_InitStruct.USART_StopBits = USART_StopBits_1;      // we want 1 stop bit (standard)
  USART_InitStruct.USART_Parity = USART_Parity_No;         // we don't want a parity bit (standard)
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
  USART_Init(UART4, &USART_InitStruct);                       // again all the properties are passed to the USART_Init function which takes care of all the bit setting

  // Here the UART receive interrupt is enabled
  // and the interrupt controller is configured
  // to jump to the UART4_IRQHandler() function
  // if the UART4 receive interrupt occurs
  //
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); // enable the UART4 receive interrupt

  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;         // we want to configure the UART4 interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the UART4 interrupts
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // this sets the subpriority inside the group
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // the UART4 interrupts are globally enabled
  NVIC_Init(&NVIC_InitStructure);                             // the properties are passed to the NVIC_Init function which takes care of the low level stuff

  // finally this enables the complete UART4 peripheral
  USART_Cmd(UART4, ENABLE);
}
*/

/*
static void InitUART5(uint32_t baudrate)
{
  GPIO_InitTypeDef  GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
  USART_InitTypeDef USART_InitStruct;  // this is for the UART5 initialization
  NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
  
  // enable APB1 peripheral clock for UART5
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
  // enable the peripheral clock for the pins used by UART5: PC12 for TX and PD2 for RX
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  // Sets up the TX pin to work correctly with the UART5 peripheral
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;             // Pin 12 (TX)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;           // the pins are configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;      // this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;         // this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;           // this activates the pull up resistors on the IO pins
  GPIO_Init(GPIOC, &GPIO_InitStruct);                 // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
  
  // Sets up the RX pin to work correctly with the UART5 peripheral
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;              // Pin 2 (RX)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;           // the pins are configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;      // this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;         // this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;           // this activates the pull up resistors on the IO pins
  GPIO_Init(GPIOD, &GPIO_InitStruct);                 // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
  

   // The RX and TX pins are now connected to their AF so the 
   // UART5 can take over control of the pins
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

  // Now the USART_InitStruct is used to define the properties of UART5
  USART_InitStruct.USART_BaudRate = baudrate;              // the baudrate is set to the value we passed into this init function
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; // we want the data frame size to be 8 bits (standard)
  USART_InitStruct.USART_StopBits = USART_StopBits_1;      // we want 1 stop bit (standard)
  USART_InitStruct.USART_Parity = USART_Parity_No;         // we don't want a parity bit (standard)
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
  USART_Init(UART5, &USART_InitStruct);                       // again all the properties are passed to the USART_Init function which takes care of all the bit setting

  // Here the UART receive interrupt is enabled
  // and the interrupt controller is configured
  // to jump to the UART5_IRQHandler() function
  // if the UART5 receive interrupt occurs
  //
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); // enable the UART5 receive interrupt

  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;         // we want to configure the UART5 interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the UART5 interrupts
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // this sets the subpriority inside the group
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // the UART5 interrupts are globally enabled
  NVIC_Init(&NVIC_InitStructure);                             // the properties are passed to the NVIC_Init function which takes care of the low level stuff

  // finally this enables the complete UART5 peripheral
  USART_Cmd(UART5, ENABLE);
}
*/
  