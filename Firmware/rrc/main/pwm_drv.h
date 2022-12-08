#ifndef PWM_DRV
#define PWM_DRV

/**
 * @brief Initialize and set up PWM timer and GPIO
 * 
 */
void init_gpio_pwm();

/**
 * @brief Set the pwm task object
 * 
 */
void set_pwm_task();

/**
 * @brief Set LCD background brightness; 10 bit - range 0-1024
 * 
 */
void pwm_task();

#endif