#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/** 
 * @brief Controls motor
 * @param pvParamater parameter of task (not used)
*/
void motor_control_task(void *pvParameter);

/** 
 * @brief Sets PWM duty cycle on motor output pin
 * @param speed duty cycle in %, should range 0 - 100
 * @return OK - when new speed aplied successfully
 *         Error - when given speed is outside the range
 */
esp_err_t motor_set_speed(float speed);

/**
 * @brief Stops the motor
 */
void motor_pump_stop (void);

/**
 * @brief Returns motor speed
 * @return pump speed
 * 
 */
float motor_get_speed (void);
#endif