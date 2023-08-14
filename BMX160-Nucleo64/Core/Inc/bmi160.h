/*
 * bmi160.h
 *
 *  Created on: Aug 4, 2023
 *      Author: OrqaPruzinac
 */

#ifndef INC_BMI160_H_
#define INC_BMI160_H_

#include "stm32c0xx_hal.h"

#include "bmi160_defs.h"

#include <string.h>

void i2c_write(I2C_HandleTypeDef* handle, uint8_t address, uint8_t reg_address, uint8_t data);
void i2c_read(I2C_HandleTypeDef* handle, uint8_t address, uint8_t reg_address, uint8_t *data, uint8_t length);


uint8_t BMI160_Check(I2C_HandleTypeDef* handle);

void BMI160_write_uint8(I2C_HandleTypeDef* handle, uint8_t reg_address, uint8_t data);

uint8_t BMI160_read_uint8(I2C_HandleTypeDef* handle, uint8_t reg_address);
int16_t BMI160_read_int16(I2C_HandleTypeDef* handle, uint8_t reg_address);
int16_t BMI160_read_uint64(I2C_HandleTypeDef* handle, uint8_t reg_address);

void BMI160_Init(I2C_HandleTypeDef* handle);

#endif /* INC_BMI160_H_ */
