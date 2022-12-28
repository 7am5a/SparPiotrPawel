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
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "Encoder.h"

extern bool calibrated;

void pid_task(void *pvParameter)
{
    while(1)
    {
        if(calibrated)
        {
          pid_external_loop();
        }
       vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

float map_speed(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void pid_external_loop()
{
  // static float Kp = 30.0f;
  // static float Ki = 0.0f;
  // static float Kd = 0.0f;
  // static float proportional;
  // static float integral;
  // static float derivative;
  // static float meas;
  // static float prev_meas;
  // static float e; 
  // static float output;
  // static float setPoint = 0.0f;

  // meas = mpu6050_read_angle();
  // e = setPoint - meas;

  // proportional = Kp * e;
  // integral += Ki * e;
  // if(integral >= 100)
  // {
  //   integral = 100;
  // }

  // output = proportional + integral  + derivative;

  pid_internal_left_loop(250);
  pid_internal_right_loop(250);
}

void pid_internal_left_loop(float setPoint)
{
  static float Kp = 1.0f;
  static float Ki = 0.01f;
  static float Kd = 0.0f;
  static float proportional;
  static float integral;
  static float derivative;
  static float meas;
  static float prev_meas;
  static float e; 
  static float output;

  meas = encoder_get_speed(0u);
  e = setPoint - meas;

  proportional = Kp * e;
  integral += Ki * e;
  if(integral >= 100)
  {
    integral = 100;
  }
  output = proportional + integral  + derivative;

  motor_set_speed(1u,output);
  prev_meas = meas;
}

void pid_internal_right_loop(float setPoint)
{
  static float Kp = 1.0f;
  static float Ki = 0.01f;
  static float Kd = 0.0f;
  static float proportional;
  static float integral;
  static float derivative;
  static float meas;
  static float prev_meas;
  static float e; 
  static float output;

  meas = -encoder_get_speed(1u);
  e = setPoint - meas;

  proportional = Kp * e;
  integral += Ki * e;
  if(integral >= 100)
  {
    integral = 100;
  }
  output = proportional + integral  + derivative;

  motor_set_speed(0u,output);
  prev_meas = meas;
}