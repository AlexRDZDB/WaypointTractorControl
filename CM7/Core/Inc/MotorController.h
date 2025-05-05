/*
 * MotorController.h
 *
 *  Created on: Nov 20, 2024
 *      Author: alexr
 */

#ifndef INC_MOTORCONTROLLER_H_
#define INC_MOTORCONTROLLER_H_

#include <stdio.h>
#include "stm32h7xx_hal.h"

void setSpeed(int speed, TIM_HandleTypeDef htim, uint32_t channel);
void setAngle(float angle, TIM_HandleTypeDef htim, uint32_t channel);

#endif /* INC_MOTORCONTROLLER_H_ */
