
#include "pwm_motor_control.h"


Motor_Result_t PWM_Motor_Init(Motor_t* MotorStruct, TIM_TypeDef* TIMx, TM_PWM_Channel_t PWMChannel, TM_PWM_PinsPack_t Pinspack)
{
    /* Initialize timer with 25 000Hz frequency for PWM */
    if (TM_PWM_InitTimer(TIMx, &MotorStruct->PWM, 25000) != TM_PWM_Result_Ok) {
            /* Return Error */
            return Motor_Result_Error;
    }

    /* Initialize channel */
    if (TM_PWM_InitChannel(TIMx, PWMChannel, Pinspack) != TM_PWM_Result_Ok) {
            /* Return Error */
            return Motor_Result_Error;
    }
    
    /* Fill settings */
    MotorStruct->TIM = TIMx;
    MotorStruct->Channel = PWMChannel;
    MotorStruct->Pinspack = Pinspack;
    
    /* Return OK */
    return Motor_Result_Ok;
}

Motor_Result_t PWM_Motor_SetDuty(Motor_t* MotorStruct, uint8_t u8Percentage)
{
    /* Check parameter value */
    if (u8Percentage < 0 || u8Percentage > 100) {
        return Motor_Result_Error;
    }

    /* Set percentage*/
    TM_PWM_SetChannelPercent(MotorStruct->TIM, &MotorStruct->PWM, MotorStruct->Channel, (double)u8Percentage);

    return Motor_Result_Ok;
}

