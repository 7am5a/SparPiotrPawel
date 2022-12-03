#pragma once


/**
 * @brief 
 * 
 */
esp_err_t mpu6050_read_raw (uint16_t accel[3], uint16_t gyro[3], uint16_t *temp);
