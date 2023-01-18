#pragma once

struct __attribute__((__packed__)) Data_state_packed
{
  float robot_angle;
  float robot_angle_speed;
  float wheel_angle;
  float wheel_angle_speed;
  float set_speed;
  float right_speed;
  float left_speed;
  float K1;
  float K2;
  float K3;
  float K4;
};

/**
 * @brief pid task
 * 
 * @param pvParameter 
 */
void state_feedback_task(void *pvParameter);


/**
 * @brief 
 * 
 */
void state_feedback_controller(void);

/**
 * @brief 
 * 
 * @param motor 
 * @return float 
 */
float state_feedback_motor_stop(uint8_t motor);