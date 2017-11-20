/**
  ******************************************************************************
  * File Name          : mpu9250_drv.c
  * Description        : Mpu-9250 sensor dirver
  ******************************************************************************
**/

//--------------------includes---------------------
#include "mpu9250_drv.h"

//------------------definitions------------
static     uint8_t buffer[6];

void MPU_WhoAmI(I2C_HandleTypeDef *hi2c3, uint8_t *data)
{
  buffer[0] = WHO_AM_I;
  HAL_I2C_Master_Transmit(hi2c3, MPU_9250, buffer, 1, 50);
  HAL_I2C_Master_Receive(hi2c3, MPU_9250 | 0X01, data, 1, 50);
}

void MPU_ReadAccelerometer(I2C_HandleTypeDef *hi2c3, uint8_t *data)
{
   buffer[0] = ACCEL_XOUT_H;
   HAL_I2C_Master_Transmit(hi2c3, MPU_9250, buffer, 1, 50);
   HAL_I2C_Master_Receive(hi2c3, MPU_9250 | 0X01, data, 6, 50);
}
void MPU_ReadGyroscope(I2C_HandleTypeDef *hi2c3, uint8_t *data)
{
    buffer[0] = GYRO_XOUT_H;
    HAL_I2C_Master_Transmit(hi2c3, MPU_9250, buffer, 1, 50);
    HAL_I2C_Master_Receive(hi2c3, MPU_9250 | 0X01, data, 6, 50);
}

void MPU_ReadTemperature(I2C_HandleTypeDef *hi2c3, uint8_t *data)
{
    buffer[0] = TEMP_OUT_H;
    HAL_I2C_Master_Transmit(hi2c3, MPU_9250, buffer, 1, 50);
    HAL_I2C_Master_Receive(hi2c3, MPU_9250 | 0X01, data, 2, 50);
}
void MPU_WritePowerManagement1(I2C_HandleTypeDef *hi2c3, uint8_t data)
{
   buffer[0] = PWR_MGMT_1;
   buffer[1] = data;
   HAL_I2C_Master_Transmit(hi2c3, MPU_9250, buffer, 2,100);
}
void MPU_WriteConfiguration(I2C_HandleTypeDef *hi2c3, uint8_t data)
{
   buffer[0] = GYRO_CONFIG;
   buffer[1] = data;
   HAL_I2C_Master_Transmit(hi2c3, MPU_9250, &buffer[2], 2,100);
}
void MPU_WriteSampleRateDivider(I2C_HandleTypeDef *hi2c3, uint8_t data)
{
   buffer[0] = SMPLRT_DIV;
   buffer[1] = data;
   HAL_I2C_Master_Transmit(hi2c3, MPU_9250, &buffer[2], 2,100);
}
void MPU_WriteGyroConfiguration(I2C_HandleTypeDef *hi2c3, uint8_t data)
{
   buffer[0] = GYRO_CONFIG;
   buffer[1] = data;
   HAL_I2C_Master_Transmit(hi2c3, MPU_9250, &buffer[2], 2,100);
}
void MPU_WriteAcceConfiguration(I2C_HandleTypeDef *hi2c3, uint8_t data)
{
  buffer[0] = ACCEL_CONFIG;
  buffer[1] = data;
  HAL_I2C_Master_Transmit(hi2c3, MPU_9250, &buffer[4], 2,100);
}
void MPU_WriteAcceConfiguration2(I2C_HandleTypeDef *hi2c3, uint8_t data)
{
   buffer[0] = ACCEL_CONFIG_2;
   buffer[1] = 0x00;
   HAL_I2C_Master_Transmit(hi2c3, MPU_9250, &buffer[4], 2,100);
}

