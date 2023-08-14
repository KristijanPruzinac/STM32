/*
 * gimbal.c
 *
 *  Created on: Jul 27, 2023
 *      Author: OrqaPruzinac
 */

#include "gimbal.h"

char gimbal_symbols[MAX_GIMBAL_NUM];
uint32_t gimbal_positions[MAX_GIMBAL_NUM];
uint8_t gimbal_types[MAX_GIMBAL_NUM];

//========== HELPER FUNCTIONS
int8_t getGimbalIndex(char symbol){
	for (uint8_t i = 0; i < MAX_GIMBAL_NUM; i++){
		if (gimbal_symbols[i] == symbol){
			return i;
		}
	}

	return -1;
}

int8_t getEmptyGimbalIndex(){
	for (uint8_t i = 0; i < MAX_GIMBAL_NUM; i++){
		if (gimbal_symbols[i] == 0){
			return i;
		}
	}

	return -1;
}

uint32_t normalizeValue(uint32_t value, uint32_t prev_min, uint32_t prev_max, uint32_t after_min, uint32_t after_max){
	if (value < prev_min || value > prev_max){return 0;}
	if (prev_max <= prev_min || after_max <= after_min){return 0;}

	return  (uint32_t) ( ((float)(value - prev_min) / (float)(prev_max - prev_min)) * (float)(after_max - after_min) + (float) after_min);
}

//========== USER FUNCTIONS
void initGimbals(){
	for (uint8_t i = 0; i < MAX_GIMBAL_NUM; i++){
		gimbal_symbols[i] = 0;
		gimbal_positions[i] = 0;
		gimbal_types[i] = 0;
	}
}

void registerGimbal(char symbol, uint8_t type){
	//Find empty slot
	uint8_t emptyIndex = getEmptyGimbalIndex(); if (emptyIndex == -1) {return;}

	//Assign gimbal
	gimbal_symbols[emptyIndex] = symbol;
	gimbal_positions[emptyIndex] = 600;

	//Assign type
	if (type == ABSOLUTE_GIMBAL || type == PUSH_GIMBAL){
		gimbal_types[emptyIndex] = type;
	}
}

void unregisterGimbal(char symbol){
	//Find gimbal
	uint8_t gimbalIndex = getGimbalIndex(symbol); if (gimbalIndex == -1) {return;}

	//Unregister
	gimbal_symbols[gimbalIndex] = 0;
}

void setGimbalPosition(char symbol, uint32_t position, uint32_t* writeRegister){
	//Find gimbal
	uint8_t gimbalIndex = getGimbalIndex(symbol);

	if (gimbal_types[gimbalIndex] == ABSOLUTE_GIMBAL){
		gimbal_positions[gimbalIndex] = normalizeValue(position, 0, 255, 400, 800);
	}
	else if (gimbal_types[gimbalIndex] == PUSH_GIMBAL){
		uint32_t normalValue = normalizeValue(position, 0, 255, 400, 800);
		int32_t diff = normalValue - 600;

		for (int i = 10; i >= 2; i--){
			if (diff >= i * 20){
				gimbal_positions[gimbalIndex] += i * PUSH_SPEED;
				break;
			}
			if (diff <= (-i) * 20){
				gimbal_positions[gimbalIndex] -= i * PUSH_SPEED;
				break;
			}
		}

		//Avoid overflow
		if (gimbal_positions[gimbalIndex] < 400) {
			gimbal_positions[gimbalIndex] = 400;
		}
		else if (gimbal_positions[gimbalIndex] > 800) {
			gimbal_positions[gimbalIndex] = 800;
		}
	}

	*writeRegister = gimbal_positions[gimbalIndex];
}
