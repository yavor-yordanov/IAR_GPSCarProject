#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "defines.h"
#include "tm_stm32f4_pwm.h"
#include "tm_stm32f4_timer_properties.h"

/* Frequency of the PWM in Hz*/
#ifndef PWM_MOTOR_FREQUENCY
#define PWM_MOTOR_FREQUENCY 25000
#endif
	
/**
 * Motor pwm struct
 *
 * Parameters:
 * 	- TM_PWM_TIM_t PWM:
 * 		PWM settings
 * 	- TIM_TypeDef* TIM:
 * 		Pointer to specific timer you will use for motor
 * 	- TM_PWM_Channel_t Channel:
 * 		Output channel on specific timer
 * 	- TM_PWM_PinsPack_t Pinspack:
 * 		Pinspack for specific channel
 */
typedef struct {
	TM_PWM_TIM_t PWM;
	TIM_TypeDef* TIM;
	TM_PWM_Channel_t Channel;
	TM_PWM_PinsPack_t Pinspack;
} Motor_t;

/**
 * Results enumeration
 *
 * Parameters:
 * 	- TM_SERVO_Result_Ok:
 * 		Everything OK
 * 	- TM_SERVO_Result_Error
 * 		An error occured somewhere
 */
typedef enum {
	Motor_Result_Ok = 0,
	Motor_Result_Error
} Motor_Result_t;

/**
 * Initialize servo
 *
 * Parameters:
 * 	- Motor_t* MotorStruct:
 * 		Pointer to an empty motor struct
 * 	- TIM_TypeDef* TIMx:
 * 		Pointer to TIMx you will use for servo
 * 	- TM_PWM_Channel_t PWMChannel:
 * 		Channel you will use for timer
 * 	- TM_PWM_PinsPack_t Pinspack
 * 		Pinspack for channel
 */
Motor_Result_t PWM_Motor_Init(Motor_t* MotorStruct, TIM_TypeDef* TIMx, TM_PWM_Channel_t PWMChannel, TM_PWM_PinsPack_t Pinspack);

/*
 * Set duty cycle in percentage.
 * Percentage can be between 0 and 100.
 *
 * Parameters:
 * 	- Motor_t* MotorStruct:
 * 		Pointer to motor struct
 * 	- uint8_t percentage
 * 		percentage of the duty cycle between 0 and 100
 */
Motor_Result_t PWM_Motor_SetDuty(Motor_t* MotorStruct, uint8_t u8Percentage);

