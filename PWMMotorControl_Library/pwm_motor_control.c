
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

    //Enable the GPIOD Clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    // GPIOE Configuration
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = PIN_IN1 | PIN_IN2 | STBY;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Return OK */
    return Motor_Result_Ok;
}

Motor_Result_t PWM_Motor_SetDuty(Motor_t* MotorStruct, uint8_t u8Percentage)
{
    /* Check parameter value */
    if (u8Percentage > 100)
    {
      return Motor_Result_Error;
    }

    /* Set percentage*/
    TM_PWM_SetChannelPercent(MotorStruct->TIM, &MotorStruct->PWM, MotorStruct->Channel, (double)u8Percentage);

    return Motor_Result_Ok;
}

void PWM_Motor_SetMode(MotorMode_t modorMode)
{
    switch(modorMode)
    {
    case CW_Dir:
      GPIO_WriteBit(GPIOE, PIN_IN1, Bit_SET);
      GPIO_WriteBit(GPIOE, PIN_IN2, Bit_RESET);
      GPIO_WriteBit(GPIOE, STBY, Bit_SET);
      break;
    case CCW_Dir:
      GPIO_WriteBit(GPIOE, PIN_IN1, Bit_RESET);
      GPIO_WriteBit(GPIOE, PIN_IN2, Bit_SET);
      GPIO_WriteBit(GPIOE, STBY, Bit_SET);
      break;
    case ShortBrake:
      GPIO_WriteBit(GPIOE, PIN_IN1, Bit_SET);
      GPIO_WriteBit(GPIOE, PIN_IN2, Bit_SET);
      GPIO_WriteBit(GPIOE, STBY, Bit_SET);
      break;
    case Stop:
      GPIO_WriteBit(GPIOE, PIN_IN1, Bit_RESET);
      GPIO_WriteBit(GPIOE, PIN_IN2, Bit_RESET);
      GPIO_WriteBit(GPIOE, STBY, Bit_SET);
      break;
    case Standby:
      GPIO_WriteBit(GPIOE, PIN_IN1, Bit_RESET);
      GPIO_WriteBit(GPIOE, PIN_IN2, Bit_RESET);
      GPIO_WriteBit(GPIOE, STBY, Bit_RESET);
      break;
    default:
      break;
    }
}
