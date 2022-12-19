#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "mpu6050_read_angle.h"
#include "motor_control.h"
#include "pid.h"

float Kp =6;
float Ki;
float Kd = 2;
float proportional;
float I;
float derivative;
float meas;
float prev_meas;
float e; 
float setPoint = 0;
float tau = 0.030f;
float T = 0.005f;
float output;

extern bool calibrated;


void pid_task(void *pvParameter)
{

    while(1)
    {
        if(calibrated)
        {
        meas = mpu6050_read_angle();
        e = setPoint - meas;

        proportional = Kp * e;

        derivative = -(Kd * (meas - prev_meas) + (tau - T) * derivative)
                        / (tau + T);
        
        // ESP_LOGI("","P: %0.2f\nKp: %0.2f\ne: %0.2f\n\n", P, Kp, e);
        output = proportional + derivative;

        motor_set_speed(1,output);
        motor_set_speed(0,output);

        prev_meas = meas;
        }
       vTaskDelay(5/portTICK_PERIOD_MS);
    }
}

float map_speed(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}