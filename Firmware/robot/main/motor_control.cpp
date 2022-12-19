#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/mcpwm.h"
#include "motor_control.h"
#include "pid.h"

#define MOTOR_PWM_OUTPUT_LA_PIN 18u
#define MOTOR_PWM_OUTPUT_LB_PIN 13u
#define MOTOR_PWM_OUTPUT_RA_PIN 27u
#define MOTOR_PWM_OUTPUT_RB_PIN 14u
#define MOTOR_PWM_FREQ_HZ 25000u

static float motor_speed;

extern float Kp;
extern float Ki;
extern float Kd;

static mcpwm_config_t pwm_config = {
    .frequency = MOTOR_PWM_FREQ_HZ,
    .cmpr_a = 0,
    .cmpr_b = 0,
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_COUNTER,
};

/**
 * @brief left motor PWM initiation
 * 
 * @return  ESP_OK - success / other - failure
 */
static esp_err_t motor_L_init(void);

/**
 * @brief right motor PWM initiation
 * 
 * @return ESP_OK - success / other - failure 
 */
static esp_err_t motor_R_init(void);

void motor_control_task(void *pvParameter)
{
    motor_L_init();
    motor_R_init();

    while (1)
    {
        vTaskDelay(100/ portTICK_RATE_MS);
    }
}

static esp_err_t motor_L_init(void)
{
    esp_err_t ret = ESP_OK;

    ret = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PWM_OUTPUT_LA_PIN);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_gpio_init 0A error");
        return ret;
    }

    ret = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_init 0A error");
        return ret;
    }

    ret = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_set_duty 0A error");
        return ret;
    }

    ret = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_PWM_OUTPUT_LB_PIN);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_gpio_init 0B error");
        return ret;
    }

    ret = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_init 0B error");
        return ret;
    }

    ret = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_set_duty 0B error");
        return ret;
    }

    return ret;
}

static esp_err_t motor_R_init(void)
{
    esp_err_t ret = ESP_OK;

    ret = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR_PWM_OUTPUT_RA_PIN);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_gpio_init 1A error");
        return ret;
    }

    ret = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_init 1A error");
        return ret;
    }

    ret = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0f);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_set_duty 1A error");
        return ret;
    }

    ret = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR_PWM_OUTPUT_RB_PIN);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_gpio_init 1B error");
        return ret;
    }

    ret = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_init 1B error");
        return ret;
    }

    ret = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0.0f);
    if(ret != ESP_OK)
    {   
        ESP_LOGE("","mcpwm_set_duty 1B error");
        return ret;
    }

    return ret;
}

esp_err_t motor_set_speed(uint8_t motor, float speed)
{
    esp_err_t ret_val = ESP_FAIL;

    if(0.0f <= speed)
    {
        if(100.0f < speed)
        {
            speed = 100.0f;
        }
        //speed = map_speed(speed,0.0f,100.0f,20.0f,100.0f);
        if(0u == motor)
        {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed);
        }
        else
        {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0f);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, speed);
        }
        ESP_LOGI("","speed: %0.2f\n", speed);
        motor_speed = speed;
        ret_val = ESP_OK;
    }
    else
    {
        speed = -speed;
        
        if(100.0f < speed)
        {
            speed = 100.0f;
        }
        //speed = map_speed(speed,0.0f,100.0f,20.0f,100.0f);
        if(MOTOR_LEFT == motor)
        {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
        }
        else
        {
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, speed);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0.0f);
        }

        ESP_LOGI("","speed: %0.2f\n", speed);
        motor_speed = speed;
        ret_val = ESP_OK;
    }
    return ret_val;
}

void motor_stop (uint8_t motor)
{
    if(MOTOR_LEFT == motor)
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
    }
    else
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0f);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0.0f);
    }
    motor_speed = 0.0f;
}

float motor_get_speed (void)
{
    return motor_speed;
}