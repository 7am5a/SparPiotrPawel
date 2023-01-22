#ifndef MPU6050_h
#define MPU6050_h

// #include <stdio.h>
// #include <string.h>

void read_raw(int16_t accel[3], int16_t gyro[3]);
void mpu6050_init();
void reset();
float map_mpu(float x, float in_min, float in_max, float out_min, float out_max);


#endif