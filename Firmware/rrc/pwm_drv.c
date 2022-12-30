#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_setup.h"
#include "pwm_drv.h"
#include "esp_log.h"

int pwm_duty_lcd = 256; //10bit - 512 as 50%; 1024 as 100% can be used mapping 1024 to 100% -> nice to have
int pwm_duty_led = 256;

//PWM
void init_gpio_pwm()
{   
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = LCD_PWM,    //Problem with led_info? other gpio don't cause errors
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 128,
        .hpoint = 0
    };

    ledc_channel_config(&channel);    
}

void pwm_task()
{
    ledc_fade_func_install(0);
    while (1)
    {
        //Temporarily most of function is commented - until set up ready PCB + change delay to trigger from menu -> PWM work all time that task only change duty 

        //Task will be used to change background brightness - one of menu options and maby info LED               
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,pwm_duty_lcd,0);
        if(pwm_duty_led < 1024)
        {
            pwm_duty_led += 64;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_duty_led);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        if(pwm_duty_led > 1024)
        {
            pwm_duty_led = 0;
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    
}

void set_pwm_task()
{
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL , 10, NULL);
}

/*
    Maby use second core?
*/