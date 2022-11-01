#ifndef SWITCH_DRV
#define SWITCH_DRV

/**
 * @brief Function managing dynamic ram, Interrupt Service Routine - called when button is pressed. Pass information and event type using queue
 * 
 * @param arg Event type used to pass data
 */
void switch_isr_handler(void *arg);

/**
 * @brief Initialize and set up all used gpio in FreeRTOS
 * 
 */
void init_gpio_sw();

/**
 * @brief Set up interrupt managment and flags 
 * 
 */
void install_gpio_sw();

/**
 * @brief Set the switch task object and create queue
 * 
 */
void set_switch_task();

/**
 * @brief Manage pressed switches
 * 
 */
void switch_task();

#endif