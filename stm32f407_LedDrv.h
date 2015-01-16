#ifndef STM32F407_LEDDRV
#define STM32F407_LEDDRV

#include "stm32f4xx.h"

/*
 * Init I/O pins used for led indication.
 *
 */
extern void InitPinsForLeds(void);

/* Set status of orange led
 *
 * Parameters: Bit_SET - Turn on, Bit_RESET - Turn off
 */
extern void SetOrangeLed(uint8_t u8LedStatus);

/*
 * Toggle state of orange led
 */
extern void ToggleOrangeLed();

/* Set status of green led
 *
 * Parameters: Bit_SET - Turn on, Bit_RESET - Turn off
 */
extern void SetGreenLed(uint8_t u8LedStatus);

/*
 * Toggle state of green led
 */
extern void ToggleGreenLed();

/* Set status of blue led
 *
 * Parameters: Bit_SET - Turn on, Bit_RESET - Turn off
 */
extern void SetBlueLed(uint8_t u8LedStatus);

/*
 * Toggle state of blue led
 */
extern void ToggleBlueLed();

/* Set status of red led
 *
 * Parameters: Bit_SET - Turn on, Bit_RESET - Turn off
 */
extern void SetRedLed(uint8_t u8LedStatus);

/*
 * Toggle state of red led
 */
extern void ToggleRedLed();

#endif
