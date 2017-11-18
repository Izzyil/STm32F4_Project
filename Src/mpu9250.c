/*
 * mpu9250.c
 *
 *
 **/

//--------------------includes---------------------
#include "mpu9250.h"

//------------------definitions------------



void MPU9250_calibrate(I2C_HandleTypeDef *hi2c3, float *acceleration_bias, float *degree_bias){
    uint8_t raw_accel[6];
    uint8_t raw_gyro[6];
    uint8_t buffer;


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
    for(i = 0; i < packets; i++)
    {
        buffer = ACCEL_XOUT_H;
        HAL_I2C_Master_Transmit(hi2c3, MPU_9250<<1, &buffer, 1, 50);
        HAL_Delay(20);
        HAL_I2C_Master_Receive(hi2c3, MPU_9250<<1, raw_accel, 6, 50);
        HAL_Delay(20);
        accel[0] = (raw_accel[0] << 8) | raw_accel[1];
        accel[1] = (raw_accel[2] << 8) | raw_accel[3];
        accel[2] = (raw_accel[4] << 8) | raw_accel[5];

        buffer = GYRO_XOUT_H;
        HAL_I2C_Master_Transmit(hi2c3, MPU_9250<<1, &buffer, 1, 50);
        HAL_Delay(20);
        HAL_I2C_Master_Receive(hi2c3, MPU_9250<<1, raw_gyro, 6, 50);
        HAL_Delay(20);
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

void MPU9250_Accelerometer(I2C_HandleTypeDef *hi2c3, float *acceleration)
{
    uint8_t raw_accel[6];
    uint8_t buffer;
    int16_t accel[3];

    buffer = ACCEL_XOUT_H;
    HAL_I2C_Master_Transmit(hi2c3, MPU_9250<<1, &buffer, 1, 50);
    HAL_Delay(20);
    HAL_I2C_Master_Receive(hi2c3, MPU_9250<<1, raw_accel, 6, 50);
    HAL_Delay(20);
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
    uint8_t buffer;
    int16_t deg[3];

    buffer = GYRO_XOUT_H;
    HAL_I2C_Master_Transmit(hi2c3, MPU_9250<<1, &buffer, 1, 50);
    HAL_Delay(20);
    HAL_I2C_Master_Receive(hi2c3, MPU_9250<<1, raw_gyro, 6, 50);
    HAL_Delay(20);

    deg[0] = (raw_gyro[0] << 8) | raw_gyro[1];
    deg[1] = (raw_gyro[2] << 8) | raw_gyro[3];
    deg[2] = (raw_gyro[4] << 8) | raw_gyro[5];

    degrees[0] = (float)(250 * deg[0])/32768.0;
    degrees[1] = (float)(250 * deg[1])/32768.0;
    degrees[2] = (float)(250 * deg[2])/32768.0;
}
