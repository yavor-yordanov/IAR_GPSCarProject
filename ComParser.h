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

#endif