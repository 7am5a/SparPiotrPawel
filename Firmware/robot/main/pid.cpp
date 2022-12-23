#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "mpu6050_read_angle.h"
#include "motor_control.h"
#include "pid.h"
#include <math.h>

float Kp = 20.0f;
float Ki;
float Kd = 0.0f;
float proportional;
float I;
float derivative;
float meas;
float prev_meas;
float e; 
float setPoint = 0.0f;
float tau = 0.04f;
float T = 0.010f;
float output;

extern bool calibrated;


void pid_task(void *pvParameter)
{
  float motor_capture_signal = 0;
  uint8_t time_devide_counter = 0;
    while(1)
    {
        if(calibrated)
        {
          meas = mpu6050_read_angle();
          if(60.0f >= meas && (-60.0f) <= meas) // if angle is more than 60 deg, robot stops
          {
            e = setPoint - meas;

            proportional = Kp * e;
            derivative = -(Kd * (meas - prev_meas))/(2.0f * tau + 2.0f * T);
            
            // ESP_LOGI("","P: %0.2f\nKp: %0.2f\ne: %0.2f\n\n", P, Kp, e);
            output = proportional + derivative;

            output = (1/(1+exp(-output/30))-0.5)*200;

            motor_set_speed(1,output);
            motor_set_speed(0,output);
            prev_meas = meas;

            if(0u == time_devide_counter % 10u)
            {
              motor_capture_signal = motor_get_speed();
              ESP_LOGW("","motor capture signal: %0.2f\n", motor_capture_signal);
            }
            time_devide_counter++;
          }
          else
          {
            motor_stop(1);
            motor_stop(0);
          }
        
        }
       vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

float map_speed(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}