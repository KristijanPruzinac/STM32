/*
 * bmx160.c
 *
 *  Created on: Aug 4, 2023
 *      Author: OrqaPruzinac
 */


#include "bmi160.h"

uint8_t BMI160_WRITE_ADDRESS = 0x68 << 1;
uint8_t BMI160_READ_ADDRESS = (0x68 << 1) + 1;

//BMI160 low level functions
void i2c_write(I2C_HandleTypeDef* handle, uint8_t address, uint8_t reg_address, uint8_t data) {
    uint8_t buffer[2];
    buffer[0] = reg_address;
    buffer[1] = data;

    HAL_I2C_Master_Transmit(handle, address, buffer, 2, HAL_MAX_DELAY);
}

void i2c_read(I2C_HandleTypeDef* handle, uint8_t address, uint8_t reg_address, uint8_t *data, uint8_t length) {
    HAL_I2C_Mem_Read(handle, address, reg_address, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

//BMI160 user functions

//Check connection
uint8_t BMI160_Check(I2C_HandleTypeDef* handle){
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(handle, BMI160_WRITE_ADDRESS, 1, 10) && HAL_I2C_IsDeviceReady(handle, BMI160_READ_ADDRESS, 1, 10);

	if (status == HAL_OK){
		return HAL_OK;
	}
	else {
		return HAL_ERROR;
	}
}

//Write
void BMI160_write_uint8(I2C_HandleTypeDef* handle, uint8_t reg_address, uint8_t data) {
    i2c_write(handle, BMI160_WRITE_ADDRESS, reg_address, data);
}

//Read
uint8_t BMI160_read_uint8(I2C_HandleTypeDef* handle, uint8_t reg_address) {
    uint8_t data;
    i2c_read(handle, BMI160_READ_ADDRESS, reg_address, &data, 1);
    return data;
}

int16_t BMI160_read_int16(I2C_HandleTypeDef* handle, uint8_t reg_address) {
    uint8_t data[2];
    data[0] = BMI160_read_uint8(handle, reg_address);
    data[1] = BMI160_read_uint8(handle, reg_address + 1);

    // Combine the two 8-bit values into a 16-bit signed integer
    int16_t result = (int16_t)((data[1] << 8) | data[0]);

    return result;
}

int16_t BMI160_read_uint64(I2C_HandleTypeDef* handle, uint8_t reg_address) {
	uint8_t data[3];
	data[0] = BMI160_read_uint8(handle, reg_address);
	data[1] = BMI160_read_uint8(handle, reg_address + 1);
	data[2] = BMI160_read_uint8(handle, reg_address + 2);

	// Combine the three 8-bit values into a 64-bit unsigned integer
	uint64_t result = (uint64_t)((uint64_t)data[2] << 16) | ((uint64_t)data[1] << 8) | data[0];

	return result;
}

void BMI160_Init(I2C_HandleTypeDef* handle){
	//System power up
	HAL_Delay(1);

	//Accelerometer
	BMI160_write_uint8(handle, BMI160_COMMAND_REG_ADDR, BMI160_ACCEL_NORMAL_MODE);
	HAL_Delay(5);

	//Gyro
	BMI160_write_uint8(handle, BMI160_COMMAND_REG_ADDR, BMI160_GYRO_NORMAL_MODE);
	HAL_Delay(100);
}
