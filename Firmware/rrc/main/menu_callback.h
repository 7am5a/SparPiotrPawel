#ifndef MENU_CALLBACK
#define MENU_CALLBACK

/**
 * @brief Return to higher menu level without using 4th switch (SWJ) - temporary not used, working on function pointers
 * 
 */
void menu_level_back();

/**
 * @brief Set default filter option - PID with hardcode Kp, Ki and Kd
 * 
 */
void default_callback();

/**
 * @brief Set the pid callback object
 * 
 */
void set_pid_callback();

/**
 * @brief Set every single one part of PID filter in real time
 * 
 * @param Kp Proportional
 * @param Ki Integral
 * @param Kd Derivative
 */
void pid_callback_task();//uint8_t *Kp, uint8_t *Ki, uint8_t *Kd);

/**
 * @brief Set every single one part of Kalman Filter in real time
 * 
 * @param temp 
 */
void kalman_callback();//uint8_t *temp);

/**
 * @brief Refresh and display changed value of brightness on screen
 * 
 */
void brightness_refresh();

/**
 * @brief Increas display brightness by 10 %
 * 
 */
void brightness_next();

/**
 * @brief Decrease display brightness by 10 %, min value 10%
 * 
 */
void brightness_prev();

/**
 * @brief Go back to Settings menu - maby change name to universal back to menu? or not - check it
 * 
 */
void brightness_back();

/**
 * @brief Set display brightness by PWM
 * 
 * @param duty Choose duty in range 0-100 (decimal) in %
 */
void brightness_callback();//uint8_t *duty);

/**
 * @brief Set the battery callback object
 * 
 */
void set_battery_callback();

/**
 * @brief Check battery level
 * 
 */
void battery_callback_task();

#endif