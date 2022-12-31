#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#define MOTOR_LEFT 0u
#define MOTOR_RIGHT 1u

/** 
 * @brief Controls motor
 * @param pvParamater parameter of task (not used)
*/
void motor_control_task(void *pvParameter);

/** 
 * @brief Sets PWM duty cycle on motor output pin
 * @param speed duty cycle in %, should range 0 - 100
 * @param motor motor number 0-1
 * 
 * @return OK - when new speed aplied successfully
 *         Error - when given speed is outside the range
 */
esp_err_t motor_set_speed(uint8_t motor, float speed);

/**
 * @brief Stops the motor
 * 
 * @param motor motor number 0-1
 */
void motor_stop (uint8_t motor);

/**
 * @brief Returns motor speed
 * @return pump speed
 * 
 */
float motor_get_speed (void);
#endif