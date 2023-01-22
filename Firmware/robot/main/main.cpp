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
#include "state_feedback.h"
#include "esp_now_drv.h"



extern "C" {
	void app_main(void);
}


extern void task_initI2C(void*);
extern void task_display(void*);
// extern void task_display(void*);


void app_main(void)
{
    xTaskCreate(&task_initI2C, "mpu_task", 2048, NULL, 5, NULL);
    vTaskDelay(500/portTICK_PERIOD_MS);
    xTaskCreate(&task_display, "disp_task", 8192, NULL, 5, NULL);
    motor_L_init();
    motor_R_init();
    init_esp_now();
    xTaskCreate(&pid_task, "pid_task", 4096, NULL, 5, NULL);
    xTaskCreate(&encoder_task, "encoder_task", 4096, NULL, 5, NULL);
    xTaskCreate(&state_feedback_task, "state_feedback_task", 8192, NULL, 5, NULL);
}