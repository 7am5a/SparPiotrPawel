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
#include "SampleFilter.h"
#include "esp_now_drv.h"
#include "esp_wifi.h"
#include "esp_now.h"

extern bool calibrated;
static SampleFilter sample_filter;

Data_packed data_packed;
extern Pid_data pid_data;



void pid_task(void *pvParameter)
{
  SampleFilter_init(&sample_filter);

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
  static float Kp = 23.0f; //pid_data.Kp; //23
  static float Ki = 0.0f; //pid_data.Ki;
  static float Kd = 0.5f; //pid_data.Kd; //0.5
  static float proportional;
  static float integral;
  static float derivative;
  static float meas;
  static float prev_err;
  static float err; 
  static float output;
  static float setPoint = 0.0f;
  static float T = 0.05f;    //low-pass filter constant in s
  static float Ts = 0.01f;  //sampling period in s

  meas = mpu6050_read_angle();

  // if(60 < meas)
  // {
  //   meas = 60;
  // }
  // else if(-60 > meas)
  // {
  //   meas = -60;
  // }
  // SampleFilter_put(&sample_filter, (double)meas);
  // meas = SampleFilter_get(&sample_filter);
  
  if(60 > meas && -60 < meas)
  {
    err = setPoint - meas;

    proportional = Kp * err;
    integral += Ki * err;
    if(integral >= 100)
    {
      integral = 100;
    }

    derivative = (1-Ts/T) * derivative + (Kd*(err-prev_err)/T);
    prev_err = err;
    output = proportional + integral  + derivative;

    if(800 < output)
    {
      output = 800;
    }
    else if(-800 > output)
    {
      output = -800;
    }

    pid_internal_left_loop(output);
    pid_internal_right_loop(output);
    
    // printf(">setSpeed:%f\n", output);
    data_packed.angle = meas;
    data_packed.Kp = Kp;
    data_packed.Ki = Ki;
    data_packed.Kd = Kd;
    data_packed.set_speed = output;
  }
  else
  {
    pid_internal_left_loop(0u);
    pid_internal_right_loop(0u);
    data_packed.set_speed = 0.0f;
  }
  // printf(">angle:%f\n", meas);
  esp_now_send(NULL, (uint8_t *)&data_packed, sizeof(data_packed));  
}

void pid_internal_left_loop(float setPoint)
{
  static float Kp = 1.0f;
  static float Ki = 0.0001f;
  static float Kd = 0.0f;
  static float proportional;
  static float integral;
  static float derivative;
  static float meas;
  static float prev_meas;
  static float err; 
  static float output;

  meas = encoder_get_speed(0u);

  data_packed.left_speed = meas;

  err = setPoint - meas;

  proportional = Kp * err;
  integral += Ki * err;
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
  static float Ki = 0.0001f;
  static float Kd = 0.0f;
  static float proportional;
  static float integral;
  static float derivative;
  static float meas;
  static float prev_meas;
  static float err; 
  static float output;

  meas = -encoder_get_speed(1u);

  data_packed.right_speed = meas;

  err = setPoint - meas;

  proportional = Kp * err;
  integral += Ki * err;
  if(integral >= 100)
  {
    integral = 100;
  }
  output = proportional + integral  + derivative;

  motor_set_speed(0u,output);
  prev_meas = meas;
}