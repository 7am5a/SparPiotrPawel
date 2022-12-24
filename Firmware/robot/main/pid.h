#pragma once


/**
 * @brief pid task
 * 
 * @param pvParameter 
 */
void pid_task(void *pvParameter);

/**
 * @brief 
 * 
 * @param x 
 * @param in_min 
 * @param in_max 
 * @param out_min 
 * @param out_max 
 * @return float 
 */
float map_speed(float x, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief 
 * 
 */
void pid_external_loop(void);

/**
 * @brief 
 * 
 * @param setPoint 
 */
void pid_internal_loop(float setPoint);