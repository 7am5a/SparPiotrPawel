#pragma once

/**
 * @brief 
 * 
 */
void task_initI2C(void *ignore);

/**
 * @brief 
 * 
 */
void task_display(void*);

/**
 * @brief 
 * 
 * @param meas 
 */
void mpu6050_pitch_read(float *meas);