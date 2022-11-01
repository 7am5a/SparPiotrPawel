#ifndef ENC_DRV
#define ENC_DRV

/**
 * @brief Function managing dynamic ram, Interrupt Service Routine - called when encoder change state. Pass information and event type using queue
 * 
 * @param arg Event type used to pass data
 */
void enc_isr_handler(void *arg);

/**
 * @brief Initialize and setup encoders
 * 
 * @param unit PCNT unit to manage each encoder 
 */
void init_gpio_enc();

/**
 * @brief Set the encoder task object and create queue
 * 
 */
void set_enc_task();

/**
 * @brief Manage rotating encoders
 * 
 */
void enc_task();

#endif