#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "motor_control.h"


void test_task_pwm (void *pvParameter)
{
    uint8_t BDC_MCPWM_DUTY = 0;
    while (1)
    {
            motor_set_speed(BDC_MCPWM_DUTY);
            BDC_MCPWM_DUTY++;
            vTaskDelay(50/ portTICK_RATE_MS);
            BDC_MCPWM_DUTY %= 100;
    }
}

void app_main()
{
    xTaskCreate(&motor_control_task, "motor_control_task", 4096, NULL,5,NULL);
    xTaskCreate(&test_task_pwm, "test_task_pwm", 4096, NULL,5,NULL);
}