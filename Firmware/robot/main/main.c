#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "motor_control.h"


void app_main()
{
    xTaskCreate(&motor_control_task, "motor_control_task", 4096, NULL,5,NULL);
}



