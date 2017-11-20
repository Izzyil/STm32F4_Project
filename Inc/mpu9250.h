/*
 * mpu9250.h
 *
 *
 **/

#ifndef _MPU9250_H_
#define _MPU9250_H_

//--includes--------------------------------------
#include "mpu9250_drv.h"

//--defiinitions----------------------------------
typedef enum{
  FAIL = 0,
  OK = 1
}MPU_RetValueTypeDef;


//--decleration----------------------------------

uint8_t MPU9250_Connect(I2C_HandleTypeDef *hi2c3);
void MPU9250_InitSensor(I2C_HandleTypeDef *hi2c3);
void MPU9250_Config(I2C_HandleTypeDef *hi2c3);
void MPU9250_calibrate(I2C_HandleTypeDef *hi2c3, float *acceleration_bias, float *degree_bias);
void MPU9250_Accelerometer(I2C_HandleTypeDef *hi2c3, float *acceleration);
void MPU9250_Gyroscope(I2C_HandleTypeDef *hi2c3, float *degrees);
void MPU9250_Temperature(I2C_HandleTypeDef *hi2c3, float *temperature);
void MPU9250_Motion(float *acceleration, float *degrees);




#endif  //_MPU9250_H_