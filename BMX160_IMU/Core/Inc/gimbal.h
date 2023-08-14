/*
 * gimbal.h
 *
 *  Created on: Jul 27, 2023
 *      Author: OrqaPruzinac
 */

#ifndef INC_GIMBAL_H_
#define INC_GIMBAL_H_

#include "stm32f4xx_hal.h"

//========== CONSTANTS
#define MAX_GIMBAL_NUM 30

#define ABSOLUTE_GIMBAL 0
#define PUSH_GIMBAL 1

#define PUSH_SPEED 1

//========== HELPER FUNCTIONS
int8_t getGimbalIndex(char symbol);
int8_t getEmptyGimbalIndex();
uint32_t normalizeValue(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);

//========== USER FUNCTIONS
void initGimbals();
void registerGimbal(char, uint8_t);
void unregisterGimbal(char);
void setGimbalPosition(char, uint32_t, uint32_t*);

#endif /* INC_GIMBAL_H_ */
