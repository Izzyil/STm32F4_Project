/**
  ******************************************************************************
  * File Name          : mpu9250.c
  * Description        : Mpu-9250 API
  ******************************************************************************
**/

//--------------------includes---------------------
#include "mpu9250.h"

//------------------definitions------------

void MPU9250_InitSensor(I2C_HandleTypeDef *hi2c3)
{
  uint8_t data;

   data = 0x01;
   MPU_WritePowerManagement1(hi2c3, data);

   data = 0x00;
   MPU_WriteGyroConfiguration(hi2c3, data);

   data = 0x00;
   MPU_WriteAcceConfiguration(hi2c3, data);
}

 uint8_t MPU9250_Connect(I2C_HandleTypeDef *hi2c3)
{
  uint8_t deviceID;
  MPU_WhoAmI(hi2c3, &deviceID);
  if(deviceID == 0x71)
   return OK;
  return FAIL;
}


void MPU9250_Config(I2C_HandleTypeDef *hi2c3)
{
  uint8_t data;
  
  data = 0x01;
  MPU_WritePowerManagement1(hi2c3, data);

  data = 0x03;
  MPU_WriteConfiguration(hi2c3, data);

  data = 0x04;
  MPU_WriteSampleRateDivider(hi2c3, data);

  data = 0x00;
  MPU_WriteGyroConfiguration(hi2c3, data);

  data = 0x00;
  MPU_WriteAcceConfiguration(hi2c3, data);

  data = 0x03;
  MPU_WriteAcceConfiguration2(hi2c3, data);
}

void MPU9250_Accelerometer(I2C_HandleTypeDef *hi2c3, float *acceleration)
{
    uint8_t raw_accel[6];
    int16_t accel[3];

    MPU_ReadAccelerometer(hi2c3, raw_accel);
    
    accel[0] = (raw_accel[0] << 8) | raw_accel[1];
    accel[1] = (raw_accel[2] << 8) | raw_accel[3];
    accel[2] = (raw_accel[4] << 8) | raw_accel[5];

    acceleration[0] = (float)(2 * accel[0])/32768;
    acceleration[1] = (float)(2 * accel[1])/32768;
    acceleration[2] = (float)(2 * accel[2])/32768;
}

void MPU9250_Gyroscope(I2C_HandleTypeDef *hi2c3, float *degrees)
{
    uint8_t raw_gyro[6];
    int16_t deg[3];

    MPU_ReadGyroscope(hi2c3, raw_gyro);

    deg[0] = (raw_gyro[0] << 8) | raw_gyro[1];
    deg[1] = (raw_gyro[2] << 8) | raw_gyro[3];
    deg[2] = (raw_gyro[4] << 8) | raw_gyro[5];

    degrees[0] = (float)(250 * deg[0])/32768.0;
    degrees[1] = (float)(250 * deg[1])/32768.0;
    degrees[2] = (float)(250 * deg[2])/32768.0;
}

void MPU9250_Temperature(I2C_HandleTypeDef *hi2c3, float *temperature)
{
    uint8_t raw_temp[6];
    uint16_t temp;

    MPU_ReadTemperature(hi2c3, raw_temp);
    
    temp = (raw_temp[0] << 8) | raw_temp[1];
    *temperature = (float)(temp / 333.87) + 21.0;
}

void MPU9250_calibrate(I2C_HandleTypeDef *hi2c3, float *acceleration_bias, float *degree_bias){
    uint8_t raw_accel[6];
    uint8_t raw_gyro[6];

    int16_t accel[3];
    int16_t deg[3];
    int16_t accel_bias[3];
    int16_t deg_bias[3];
    int64_t accel_sum[3] = {0, 0, 0};
    int64_t deg_sum[3] = {0, 0, 0};
    uint16_t packets = 512;
    uint16_t packetsPer = 511;

    uint16_t i;

    printf("\n");
    printf("Starting sensor calibration process\n\r");
    
    for(i = 0; i < packets; i++)
    {
  
        MPU_ReadAccelerometer(hi2c3, raw_accel);
        MPU_ReadGyroscope(hi2c3, raw_gyro);

        accel[0] = (raw_accel[0] << 8) | raw_accel[1];
        accel[1] = (raw_accel[2] << 8) | raw_accel[3];
        accel[2] = (raw_accel[4] << 8) | raw_accel[5];
        deg[0] = (raw_gyro[0] << 8) | raw_gyro[1];
        deg[1] = (raw_gyro[2] << 8) | raw_gyro[3];
        deg[2] = (raw_gyro[4] << 8) | raw_gyro[5];

        accel_sum[0] += accel[0];
        accel_sum[1] += accel[1];
        accel_sum[2] += accel[2];

        deg_sum[0] += deg[0];
        deg_sum[1] += deg[1];
        deg_sum[2] += deg[2];
        
        switch(i & 0x03)
        {
        case 0x00:
          printf("\bCalibrating         [%d%]\r",(i *100)/packetsPer);
          fflush(stdout);
          break;
        case 0x1:
          printf("\bCalibrating.        [%d%]\r",(i *100)/packetsPer);
          fflush(stdout);
          break;
        case 0x2:
          printf("\bCalibrating..       [%d%]\r",(i *100)/packetsPer);
          fflush(stdout);
          break;
        case 0x3:
          printf("\bCalibrating...      [%d%]\r",(i *100)/packetsPer);
          fflush(stdout);
          break;
        default:
          break;
        }      
    }

    accel_bias[0] = accel_sum[0] / packets;
    accel_bias[1] = accel_sum[1] / packets;
    accel_bias[2] = accel_sum[2] / packets;

    deg_bias[0] = deg_sum[0] / packets;
    deg_bias[1] = deg_sum[1] / packets;
    deg_bias[2] = deg_sum[2] / packets;

    /*The sensitivity should change accordingly*/
    acceleration_bias[0] = (float)(2 * accel_bias[0])/32768;
    acceleration_bias[1] = (float)(2 * accel_bias[1])/32768;
    acceleration_bias[2] = (float)(2 * accel_bias[2])/32768;

    degree_bias[0] = (float)(250 * deg_bias[0])/32768;
    degree_bias[1] = (float)(250 * deg_bias[1])/32768;
    degree_bias[2] = (float)(250 * deg_bias[2])/32768;
    
    printf("\nCalibrating process Finished!!\n\n\r");
}

