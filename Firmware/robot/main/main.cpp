#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "motor_control.h"
#include "pid.h"
#include "Encoder.h"


extern "C" {
	void app_main(void);
}

extern void task_initI2C(void*);
extern void task_display(void*);


void app_main(void)
{
    xTaskCreate(&task_initI2C, "mpu_task", 8192, NULL, 5, NULL);
    xTaskCreate(&task_display, "disp_task", 8192, NULL, 5, NULL);
    xTaskCreate(&motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);
    //xTaskCreate(&pid_task, "pid_task", 4096, NULL, 5, NULL);
    xTaskCreate(&encoder_task, "encoder_task", 4096, NULL, 5, NULL);
}