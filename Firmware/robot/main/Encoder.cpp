
// from https://github.com/espressif/esp-idf/blob/93a8603c545fb8e54741d6685146e2f3b874378d/examples/peripherals/pcnt/main/pcnt_example_main.c

#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "driver/timer.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdint.h>
#include "Encoder.h"
#include "esp_err.h"

#include <limits>

#define PULSES_PER_MOTOR_REVOLUTION 28
#define GEAR_RATIO 20
#define PULSES_PER_WHEEL_REVOLUTION (PULSES_PER_MOTOR_REVOLUTION * GEAR_RATIO)

#define TIMER_DIVIDER 100u

static uint64_t last_timer_val;
static int32_t last_encoder_val;
static int32_t curr_encoder_val;
static float last_speed;


portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

Encoder::Encoder(gpio_num_t gpioPinA, gpio_num_t gpioPinB, pcnt_unit_t pcntUnit) {
	this->gpioPinA = gpioPinA;
	this->gpioPinB = gpioPinB;
	this->pcntUnit = pcntUnit;
}

void Encoder::init() {
	gpio_set_pull_mode(this->gpioPinA, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(this->gpioPinB, GPIO_PULLUP_ONLY);

	/* Prepare configuration for the PCNT unit */
	pcnt_config_t pcnt_config;

	pcnt_config.pulse_gpio_num = this->gpioPinA;
	pcnt_config.ctrl_gpio_num = this->gpioPinB;
	pcnt_config.channel = PCNT_CHANNEL_0;
	pcnt_config.unit = pcntUnit;

	// What to do on the positive / negative edge of pulse input?
	pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
	pcnt_config.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge

	// What to do when control input is low or high?
	pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
	pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high

	// Set the maximum and minimum limit values to watch
	pcnt_config.counter_h_lim = std::numeric_limits<int16_t>::max();
	pcnt_config.counter_l_lim = std::numeric_limits<int16_t>::min();  

	
	/* Initialize PCNT unit */
	pcnt_unit_config(&pcnt_config);

	/* Configure and enable the input filter */
	pcnt_set_filter_value(this->pcntUnit, 1023);
	pcnt_filter_enable(this->pcntUnit);

	/* Enable events on zero, maximum and minimum limit values */
	pcnt_event_enable(this->pcntUnit, PCNT_EVT_ZERO);
	pcnt_event_enable(this->pcntUnit, PCNT_EVT_H_LIM);
	pcnt_event_enable(this->pcntUnit, PCNT_EVT_L_LIM);

	/* Initialize PCNT's counter */
	pcnt_counter_pause(this->pcntUnit);
	pcnt_counter_clear(this->pcntUnit);

	/* Register ISR handler and enable interrupts for PCNT unit */
	//pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
	//pcnt_intr_enable(PCNT_TEST_UNIT);

	/* Everything is set up, now go to counting */
	pcnt_counter_resume(this->pcntUnit);

	this->startingOffset = this->getAddition();
}

int32_t Encoder::getValue() const {
	int16_t value;
	pcnt_get_counter_value(this->pcntUnit, &value);
	return (((int32_t) value) << 2)
			+ this->getAddition()
			- (int32_t) this->startingOffset;
}

int32_t Encoder::getAddition() const {
	auto A = gpio_get_level(this->gpioPinA);
	auto B = gpio_get_level(this->gpioPinB);
	int32_t addition = (B << 1) + (A ^ B);
	return addition;
}

void encoder_task(void *pvParameter)
{
    Encoder encoder(GPIO_NUM_34, GPIO_NUM_35);

    encoder.init();
     //Timer----------------------------------------------
    timer_config_t config = { 
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = TIMER_DIVIDER
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
    //End timer------------------------------------------
    vTaskDelay(20/portTICK_PERIOD_MS);
    while(1)
    {
        taskENTER_CRITICAL(&myMutex);
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &last_timer_val);
        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
        curr_encoder_val = encoder.getValue();
        last_speed = (1000*(curr_encoder_val-last_encoder_val))/((float)last_timer_val/800.0f);
        last_speed = (60*last_speed)/PULSES_PER_WHEEL_REVOLUTION;
        taskEXIT_CRITICAL(&myMutex);

        
        last_encoder_val = curr_encoder_val;
        printf("encoder value: %d last timer val: %llu last speed: %0.2f\n", encoder.getValue(), last_timer_val, last_speed);

        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

float encoder_get_speed(void)
{
    return last_speed;
}