#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/mcpwm.h"
#include "motor_control.h"

#define MOTOR_PWM_OUTPUT_PIN 16u
#define MOTOR_PWM_FREQ_HZ 1000u

static float motor_speed;

void motor_control_task(void *pvParameter)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PWM_OUTPUT_PIN);
    mcpwm_config_t pwm_config = {
        .frequency = MOTOR_PWM_FREQ_HZ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
    while (1)
    {
        vTaskDelay(100/ portTICK_RATE_MS);
    }
}

esp_err_t motor_set_speed(float speed)
{
    esp_err_t ret_val = ESP_FAIL;

    if(0.0f <= speed)
    {
        if(100.0f >= speed)
        {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
            motor_speed = speed;
            ret_val = ESP_OK;
        }
    }
    return ret_val;
}

void motor_stop (void)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
    motor_speed = 0.0f;
}

float motor_get_speed (void)
{
    return motor_speed;
}