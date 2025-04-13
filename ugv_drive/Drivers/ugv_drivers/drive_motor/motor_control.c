#include "motor_control.h"
#include "string.h"
#include "stdlib.h"
#include "main.h"


void MotorControl_Init(MotorControl* motor, TIM_HandleTypeDef* timer, uint8_t channel_1, uint8_t channel_2) {
    motor->htim = timer;
    motor->channel_1 = channel_1;
    motor->channel_2 = channel_2;
    HAL_TIM_PWM_Start(motor->htim, motor->channel_1);
    HAL_TIM_PWM_Start(motor->htim, motor->channel_2);
}

void MotorControl_SetSpeed(MotorControl* motor, TIM_HandleTypeDef* htim, float duty_cycle) {
	float speed = 32767 * duty_cycle;
	float input_speed = abs(speed);

	if (speed > 0)
	{
		motor->htim->Instance->CCR1 = input_speed;
		motor->htim->Instance->CCR3 = 0;

		//Modified Code to include GPIO outputs corresponding to direction
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	}
	else if (speed < 0)
	{
		motor->htim->Instance->CCR1 = 0;
		motor->htim->Instance->CCR3 = input_speed;
		//Modified code to include GPIO outputs corresponding to direction
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
	}
	else {
		motor->htim->Instance->CCR1 = 0;
		motor->htim->Instance->CCR3 = 0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	}

}
