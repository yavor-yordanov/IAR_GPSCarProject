#ifndef COM_PARSER
#define COM_PARSER

#include "stm32f4xx.h"
#include "stm32f407_LedDrv.h"

#define MAX_COMMAND_LENGTH 16
#define MAX_NMEA_LENGTH    82 

#ifndef READY
#define READY 1
#endif

#ifndef NOT_READY
#define NOT_READY 0
#endif

#ifndef LENGTH_RMC_ID
#define LENGTH_RMC_ID 6
#endif

#ifndef LENGTH_GGA_ID
#define LENGTH_GGA_ID 6
#endif

#ifndef LENGTH_COMMAND_9
#define LENGTH_COMMAND_9  9
#endif

#ifndef LENGTH_COMMAND_10
#define LENGTH_COMMAND_10 10
#endif

#ifndef LENGTH_COMMAND_11
#define LENGTH_COMMAND_11 11
#endif

#ifndef LENGTH_COMMAND_12
#define LENGTH_COMMAND_12 12
#endif

#ifndef LENGTH_COMMAND_13
#define LENGTH_COMMAND_13 13
#endif

#ifndef LENGTH_COMMAND_14
#define LENGTH_COMMAND_14 14
#endif

enum ParseErrors
{
  OK,
  CRC_OR_VALIDITY_ERR
};

typedef struct GPSDateTime
{
  int     year;  // Years since 1900
  int     mon;   // Months since January - [0,11]
  int     day;   // Day of the month - [1,31]
  int     hour;  // Hours since midnight - [0,23]
  int     min;   // Minutes after the hour - [0,59]
  int     sec;   // Seconds after the minute - [0,59]
  int     hsec;  // Hundredth part of second - [0,99]
}tGPSDateTime;

typedef struct GPSInfo
{
    uint8_t u8ValidityFlag;
    int32_t i32Latitude;    //Latitude
    int32_t i32Longitude;   //Longitude
    double  ddSpeed;        //Speed
    double  ddDirection;    //Direction depending North in degrees
    tGPSDateTime sDateTime; //Date and time information
} tGPSInfo;


/*
 * Get recived char from port USART3
 */
extern void USART3_ReceivedChar(char cChar);

/*
 * Get recived char from port USART2
 */
extern void USART2_ReceivedChar(char cChar);

/*
 * Parse received command
 */
extern void ParseCommand(void);

/*
 * Parse GPS received data
 */
extern void ParseGPSData(void);

/* 
 * Get received servo degrees
 */
extern void GetCurrentServoDegrees(uint8_t *pu8Degrees);

/* 
 * Get RMC message status
 */
extern uint8_t GetRMCStatus(void);

/* 
 * Get GGA message status
 */
extern uint8_t GetGGAStatus(void);

extern void GetRMCMessage(char string[]);

extern void GetGGAMessage(char string[]);

extern uint8_t ParseRMCMessage(void);

extern void GetRMCInfoStructure();

#endif