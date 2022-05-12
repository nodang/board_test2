
#include "tim.h"

#include "user_tim.h"

void timer7_ISR(TIM_HandleTypeDef *htim)
{
	// htim->Instance->ARR 		// counter period(auto-reload register) set
	// htim->Instance->psc 		// prescaler set

	//HAL_GPIO_TogglePin(PD7_LED_GPIO_Port, PD7_LED_Pin);
	//HAL_GPIO_TogglePin(PB7_MOTOR_DIR_GPIO_Port, PB7_MOTOR_DIR_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM9)
	{
		timer9_motor_ISR();
	}

	if(htim->Instance == TIM7)
	{
		timer7_ISR(htim);
	}
}

