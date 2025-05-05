/*
 * MotorController.c
 *
 *  Created on: Nov 20, 2024
 *      Author: alexr
 */

#include <MotorController.h>
#include <stdio.h>
#include "stm32h7xx_hal.h"


// Variables to translate from percentage
#define neutral 0.5f;
#define fullForward 1f;
#define fullReverse 0f;

// Variables to handle signals for servo and motor

/*
 * @brief Function to set Speed. Goes from -100 to 100 in percentage. Used for both Servo and the Motor.
 * @param int speed: Percentage of speed (negative for reverse);
 */
void setSpeed(int speed, TIM_HandleTypeDef htim, uint32_t channel){

	// Clamp speed to avoid unintended values
	if (speed < -100) speed = -100;
	if (speed > 100) speed = 100;

	float setpoint = neutral;

	if(speed > 0){
		// -100 is max Reverse, so when speed is -100 the setpoint should be 0
		setpoint = 0.5 - ((speed/ 100.) * 0.5);
	}
	else{
		// 100 is max Forward so when speed is 100 setpoint should be 1
		setpoint = 0.5 + ((speed/ 100.) * 0.5);
	}

	__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, (uint32_t)((19999 * 0.05) + (19999 * 0.05 * setpoint)));

}

void setAngle(float angle, TIM_HandleTypeDef htim, uint32_t channel){
	if (angle < -90.0f) angle = -90.0f;
	    if (angle > 90.0f) angle = 90.0f;

	    // Map angle to pulse width (1 to 2 ms)
	    float pulseWidth = 1.5f + (angle / 180.0f);

	    // Get the current autoreload register value
	    uint32_t autoreload_register_value = __HAL_TIM_GET_AUTORELOAD(&htim);

	    // Calculate the new compare register value
	    uint32_t value = (pulseWidth / 20.0f) * (autoreload_register_value + 1);

	    // Set the compare register value to adjust the PWM signal
	    __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, value);
	    return;
}
