#ifndef COM_PARSER
#define COM_PARSER

#include "stm32f4xx.h"
#include "stm32f407_LedDrv.h"
#include <stdlib.h>
#include <string.h>

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

#ifndef MAX_SPEED_PAR_LENGHT
#define MAX_SPEED_PAR_LENGHT 6
#endif

#ifndef KNOTS_TO_KM_H
#define KNOTS_TO_KM_H ((double) 1.852)
#endif

#ifndef MAX_COG_PAR_LENGHT
#define MAX_COG_PAR_LENGHT 6
#endif

//Define parameters sentences in RMC message
//Example: $GPRMC,013732.000,A,3150.7238,N,11711.7278,E,0.00,0.00,220413,,,A*68
#define RMC_UTC 1
#define RMC_DATA_VALID 2
#define RMC_LATITUDE 3
#define RMC_N_S 4          //North and South latitutde
#define RMC_LONGITUDE 5
#define RMC_E_W 6          //East and West longitude
#define RMC_SPEED 7
#define RMC_COG 8          //Course over ground
#define RMC_DATE 9
#define RMC_MAG_VAR 10     //Magnetic variation in degree
#define RMC_MAG_VAR_EW 11  //Magnetic variation E/W indicator
#define RMC_POS_MODE 12    //Positioning mode


enum ParseErrors
{
  CHECKSUM_OK,
  CHECKSUM_OR_VALIDITY_ERR
};

typedef struct GPSDateTime
{
  int     year;  // Years since 1900
  int     mon;   // Months since January - [0,11]
  int     day;   // Day of the month - [1,31]
  int     hour;  // Hours since midnight - [0,23]
  int     min;   // Minutes after the hour - [0,59]
  int     sec;   // Seconds after the minute - [0,59]
//  int     hsec;  // Hundredth part of second - [0,99]
}tGPSDateTime;

typedef struct GPSInfo
{
    uint8_t u8ValidityFlag; //0 = V -invalid data; 1 = A - valid data
    double  d32Latitude;    //Latitude
    double  d32Longitude;   //Longitude
    double  dSpeed;         //Speed
    double  dDirection;     //Direction depending North in degrees
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

extern void GetGPSInfoData(tGPSInfo const * pGPSData);

#endif