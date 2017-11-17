/*
 * mpu9250.c
 *
 *
 **/

//--------------------includes---------------------
#include "mpu9250.h"

//------------------definitions------------

//uint8_t MPU_WhoAmI(I2C_HandleTypeDef* hi2c)
//{
//  uint8_t data
//  HAL_I2C_Master_Transmit();
//  HAL_StatusTypeDef HAL_I2C_Master_Transmit(hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
//  return HAL_I2C_Master_Receive(hi2c, MPU9250, &data, uint16_t Size, uint32_t Timeout);
//}