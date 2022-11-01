#ifndef JOYSTICK_DRV
#define JOYSTICK_DRV

/**
 * @brief Number of samples used in multisampling 
 *        Value can be change (probably multiple of 2)
 * 
 */
#define SAMPLES 64

/**
 * @brief Set default voltage reference
 * Values required to check:
 * 1100 mV - default
 * 2450 mV - max in recommended by datasheet
 * 3300 mV - reference value vrom ESP32
 * 
 * and maby more and atten in dB in esp_setup.c - possibly depend on above value
 */
#define ADC_VREF_VALUE 3300 

/**
 * @brief Print avaliable calibration type
 * 
 */
void print_cal_val_type();

/**
 * @brief Initialize and set up ADC channels in GPIO
 * 
 */
void init_gpio_adc();

/**
 * @brief Set the read adc task object
 * 
 */
void set_read_adc_task();

/**
 * @brief Read raw value from Joystick GPIO and convert it to mV
 * 
 */
void read_adc_task();

/**
 * @brief Read raw value from VBAT GPIO and convert it to mV
 * 
 */
void read_VBAT_adc_task();

#endif