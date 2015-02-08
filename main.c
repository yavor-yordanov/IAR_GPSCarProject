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
static void InitUART4(uint32_t baudrate);
static void InitUART5(uint32_t baudrate);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
/*volatile*/ char sendBuffer[] = "$GPRMC,201633.000,A,4239.9272,N,02321.8573,E,0.54,0.00,200114,,,A*65\r\n$GPVTG,0.00,T,,M,0.54,N,0.99,K,A*3C\r\n$GPGGA,201633.000,4239.9272,N,02321.8573,E,1,5,1.28,574.1,M,36.7,M,,*5B\r\n$GPGSA,A,3,12,24,28,26,15,,,,,,,,1.58,1.28,0.93*00\r\n$GPGSV,2,1,06,15,59,228,45,24,52,307,44,39,40,177,38,26,37,169,37*76\r\n$GPGSV,2,2,06,12,25,247,43,28,20,056,30*75\r\n$GPGLL,4239.9272,N,02321.8573,E,201633.000,A,A*55\r\n$GPTXT,01,01,02,ANTSTATUS=OPEN*2B\r\n";
volatile char *pointerSendBuffer;
char tempString[MAX_NMEA_LENGTH];
char tempInfoStatus[40];
tGPSInfo sParsedGPSData;

int main()
{
  //TM_GPS_Data_t GPS_Data;
  //TM_GPS_Result_t result, current;
  //TM_GPS_Float_t GPS_Float;
  //TM_GPS_Distance_t GPS_Distance;
  uint8_t i;
  float temp;
  uint16_t u16Counter = 0;
  uint8_t  u8LedControlMask = 1;
  uint8_t  u8OldServo1Degrees = 50;
  uint8_t  u8DegreesTempValue = 50;
  uint8_t  u8Servo1Sign = 1;           // 1 - plus, 0 - minus
  uint8_t  u8StateOfSDCard = 0;        // State of SD card 1 - Mount and Open file; 0 - Unmount and close file
  uint32_t u32TempUserButtonValue = 0;
  pointerSendBuffer = sendBuffer;
  nmeaINFO info;
  nmeaPARSER parser;
  
  //Fatfs object
  FATFS FatFs;
  //File object
  FIL fil;
  //Free and total space
  uint32_t total, free;
  //Servo object
  TM_SERVO_t Servo1; //, Servo2;
  // Motor object
  Motor_t Motor;

  //Initialize system
  SystemInit();
  InitPinsForLeds();
  EnableSysTickTimer();
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
  
  //InitUART4(9600);
  /* Initialize UART4 for debug */
  /* PA0 - TX    PA1 - RX */
  //TM_GPS_Init(&GPS_Data, 9600);
  //TM_USART_Init(UART4, TM_USART_PinsPack_1, 9600);
  //InitUART4(9600);
  
  TM_USART_Init(USART2, TM_USART_PinsPack_2, 9600);
  TM_USART_Init(USART3, TM_USART_PinsPack_3, 9600);
  //Put string to terminal
  TM_USART_Puts(USART3, "Hello world\n\r");

  //InitUART5(9600);
  /* Initialize UART5 for debug */
  /* PC12 - TX    PD2 - RX */
  //TM_USART_Init(UART5, TM_USART_PinsPack_1, 9600);
  
   disk_initialize(0);
  /* Initialize servo 1, TIM2, Channel 1, Pinspack 2 = PA5 */
  //TM_SERVO_Init(&Servo1, TIM2, TM_PWM_Channel_1, TM_PWM_PinsPack_2);
  //TM_SERVO_SetDegrees(&Servo1, 90);
  
  PWM_Motor_Init(&Motor, TIM3, TM_PWM_Channel_2, TM_PWM_PinsPack_1);
  PWM_Motor_SetMode(CCW_Dir);
  PWM_Motor_SetDuty(&Motor, 0);

   //nmea_zero_INFO(&info);
   //nmea_parser_init(&parser);
   
  //Dealy from 2 seconds
  while(u32SystemTimer < 5000)
  {
    ;
  }
  
  TM_USART_Puts(USART2, "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
  
  
  /* Version 1.1 added */
    /* Set two test coordinates */
  /*
    GPS_Distance.Latitude1 = 48.300215;
    GPS_Distance.Longitude1 = -122.285903;
    GPS_Distance.Latitude2 = 45.907813;
    GPS_Distance.Longitude2 = 56.659407;
    
    / Calculate distance and bearing between 2 pointes /
    TM_GPS_DistanceBetween(&GPS_Distance);
    / Convert float number /
    TM_GPS_ConvertFloat(GPS_Distance.Distance, &GPS_Float, 6);
    sprintf(buffer, "Distance is: %d.%06d meters\n", GPS_Float.Integer, GPS_Float.Decimal);
    //TM_USART_Puts(UART4, buffer);
    TM_GPS_ConvertFloat(GPS_Distance.Bearing, &GPS_Float, 6);
    sprintf(buffer, "Bearing is: %d.%06d degrees\n\n", GPS_Float.Integer, GPS_Float.Decimal);
    //TM_USART_Puts(UART4, buffer);
  */
  
  //TM_USART_Puts(UART5, sendBuffer);
    
    //TM_SERVO_SetDegrees(&Servo1, 180);
    
      //Mount drive
      //if(f_mount(&FatFs, "", 1) == FR_OK)
      //{
        //Mounted OK, turn on RED LED
        //TM_DISCO_LedOn(LED_RED);
        
        //Try to open file
        //if (f_open(&fil, "1stfile.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE); // == FR_OK)
        //if(f_open(&fil, "1stfile.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
        //{
     //     u8StateOfSDCard = 1;
        //}
     // }
    
  while(1)
  {
    ParseCommand();
    ParseGPSData();
    
    // 10ms process
    if(0 != (u8FlagForprocesses & FLAG_10MS))
    {
      //Get servo degrees from manual control
      GetCurrentServoDegrees(&u8DegreesTempValue);
      if(u8DegreesTempValue != u8OldServo1Degrees)
      {
        u8OldServo1Degrees = u8DegreesTempValue;
        if((50 <= u8OldServo1Degrees) && (140 >= u8OldServo1Degrees))
        {
          TM_SERVO_SetDegrees(&Servo1, u8OldServo1Degrees);
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
            f_mount(0, "", 1);
            u8StateOfSDCard = 0;
            SetBlueLed(Bit_RESET);
          }
          else
          {
            if(f_mount(&FatFs, "", 1) == FR_OK)
            {
              //Try to open file
              if(f_open(&fil, "1stfile.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK)
              {
                u8StateOfSDCard = 1;
                SetBlueLed(Bit_SET);
              }
            }
          }
        }
      }

      u8FlagForprocesses &= (uint8_t)(~FLAG_10MS);
    }

    // 250ms process
    if(0 != (u8FlagForprocesses & FLAG_250MS))
    {
      //Flashing orange led indicates normal operation of the system
      ToggleOrangeLed();

      if(1 == u8StateOfSDCard)
      {
        if(0 != GetRMCStatus())
        {
          GetRMCMessage(tempString);
          f_puts(tempString, &fil);
          
          if(CHECKSUM_OK == ParseRMCMessage())
          {
            
          }
          else
          {
            
          }

          GetGPSInfoData(&sParsedGPSData);
          sprintf(tempInfoStatus, "Validity: %i\n", sParsedGPSData.u8ValidityFlag);
          TM_USART_Puts(USART2, tempInfoStatus);
          sprintf(tempInfoStatus, "Latitude: %f\n", sParsedGPSData.d32Latitude);
          TM_USART_Puts(USART2, tempInfoStatus);
          sprintf(tempInfoStatus, "Longitude: %f\n", sParsedGPSData.d32Longitude);
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
        
        if(0 != GetGGAStatus())
        {
          //GetGGAMessage(tempString);
          //f_puts(tempString, &fil);
          //nmea_parse(&parser, tempString, (int)strlen(tempString), &info);
          //TODO: Send neccessary information to PC
          //TM_USART_Puts(USART3, "Hello world\n\r");
        }
      }

      //TM_USART_Puts(USART2, "Hello world\n\r");
      u8FlagForprocesses &= (uint8_t)(~FLAG_250MS);
    }
    
    //
    // 1s process
    if(0 != (u8FlagForprocesses & FLAG_1000MS))
    {
      if(28 >= u16Counter)
      {
        switch(u16Counter)
        {
        case 1:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 10);
          break;
        case 2:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 20);
          break;
        case 3:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 30);
          break;
        case 4:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 40);
          break;
        case 5:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 50);
          break;
        case 6:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 60);
          break;  
        case 7:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 70);
          break;
        case 8:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 60);
          break;
        case 9:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 50);
          break;
        case 10:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 40);
          break;
        case 11:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 30);
          break;
        case 12:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 20);
          break;
        case 13:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 10);
          break;
        case 14:
          PWM_Motor_SetMode(CCW_Dir);
          PWM_Motor_SetDuty(&Motor, 0);
          break;
        case 15:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 10);
          break;
        case 16:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 20);
          break;
        case 17:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 30);
          break;
        case 18:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 40);
          break;
        case 19:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 50);
          break;
        case 20:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 60);
          break;  
        case 21:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 70);
          break;
        case 22:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 60);
          break;
        case 23:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 50);
          break;
        case 24:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 40);
          break;
        case 25:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 30);
          break;
        case 26:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 20);
          break;
        case 27:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 10);
          break;
        case 28:
          PWM_Motor_SetMode(CW_Dir);
          PWM_Motor_SetDuty(&Motor, 0);
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
      
      u8FlagForprocesses &= (uint8_t)(~FLAG_1000MS);
    }
  }
  
  //nmea_parser_destroy(&parser);
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
static void InitUART4(uint32_t baudrate)
{
  GPIO_InitTypeDef  GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
  USART_InitTypeDef USART_InitStruct;  // this is for the UART4 initialization
  NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
  
  /* enable APB1 peripheral clock for UART4 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  /* enable the peripheral clock for the pins used by UART4: PA0 for TX and PA1 for RX */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  /* Sets up the TX and RX pins so they work correctly with the UART4 peripheral */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // Pins 0 (TX) and 1 (RX) are used
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;           // the pins are configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;      // this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;         // this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;           // this activates the pull up resistors on the IO pins
  GPIO_Init(GPIOA, &GPIO_InitStruct);                 // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

   /* The RX and TX pins are now connected to their AF so the 
    * UART4 can take over control of the pins */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

  /* Now the USART_InitStruct is used to define the properties of UART4 */
  USART_InitStruct.USART_BaudRate = baudrate;              // the baudrate is set to the value we passed into this init function
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; // we want the data frame size to be 8 bits (standard)
  USART_InitStruct.USART_StopBits = USART_StopBits_1;      // we want 1 stop bit (standard)
  USART_InitStruct.USART_Parity = USART_Parity_No;         // we don't want a parity bit (standard)
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
  USART_Init(UART4, &USART_InitStruct);                       // again all the properties are passed to the USART_Init function which takes care of all the bit setting

  /* Here the UART receive interrupt is enabled
   * and the interrupt controller is configured
   * to jump to the UART4_IRQHandler() function
   * if the UART4 receive interrupt occurs
   */
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); // enable the UART4 receive interrupt

  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;         // we want to configure the UART4 interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the UART4 interrupts
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // this sets the subpriority inside the group
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // the UART4 interrupts are globally enabled
  NVIC_Init(&NVIC_InitStructure);                             // the properties are passed to the NVIC_Init function which takes care of the low level stuff

  // finally this enables the complete UART4 peripheral
  USART_Cmd(UART4, ENABLE);
}

static void InitUART5(uint32_t baudrate)
{
  GPIO_InitTypeDef  GPIO_InitStruct;   // this is for the GPIO pins used as TX and RX
  USART_InitTypeDef USART_InitStruct;  // this is for the UART5 initialization
  NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
  
  /* enable APB1 peripheral clock for UART5 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
  /* enable the peripheral clock for the pins used by UART5: PC12 for TX and PD2 for RX */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* Sets up the TX pin to work correctly with the UART5 peripheral */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;             // Pin 12 (TX)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;           // the pins are configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;      // this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;         // this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;           // this activates the pull up resistors on the IO pins
  GPIO_Init(GPIOC, &GPIO_InitStruct);                 // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
  
  /* Sets up the RX pin to work correctly with the UART5 peripheral */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;              // Pin 2 (RX)
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;           // the pins are configured as alternate function so the USART peripheral has access to them
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;      // this defines the IO speed and has nothing to do with the baudrate!
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;         // this defines the output type as push pull mode (as opposed to open drain)
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;           // this activates the pull up resistors on the IO pins
  GPIO_Init(GPIOD, &GPIO_InitStruct);                 // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
  

   /* The RX and TX pins are now connected to their AF so the 
    * UART5 can take over control of the pins */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

  /* Now the USART_InitStruct is used to define the properties of UART5 */
  USART_InitStruct.USART_BaudRate = baudrate;              // the baudrate is set to the value we passed into this init function
  USART_InitStruct.USART_WordLength = USART_WordLength_8b; // we want the data frame size to be 8 bits (standard)
  USART_InitStruct.USART_StopBits = USART_StopBits_1;      // we want 1 stop bit (standard)
  USART_InitStruct.USART_Parity = USART_Parity_No;         // we don't want a parity bit (standard)
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
  USART_Init(UART5, &USART_InitStruct);                       // again all the properties are passed to the USART_Init function which takes care of all the bit setting

  /* Here the UART receive interrupt is enabled
   * and the interrupt controller is configured
   * to jump to the UART5_IRQHandler() function
   * if the UART5 receive interrupt occurs
   */
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); // enable the UART5 receive interrupt

  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;         // we want to configure the UART5 interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the UART5 interrupts
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // this sets the subpriority inside the group
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // the UART5 interrupts are globally enabled
  NVIC_Init(&NVIC_InitStructure);                             // the properties are passed to the NVIC_Init function which takes care of the low level stuff

  // finally this enables the complete UART5 peripheral
  USART_Cmd(UART5, ENABLE);
}