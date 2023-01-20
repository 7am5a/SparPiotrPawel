#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "motor_control.h"
#include "pid.h"
#include <math.h>
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_now_drv.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "Encoder.h"

Data_packed data_packed;
extern PID_vals pid_data;

static float prev_L_meas = 0;
static float prev_R_meas = 0;
static float integral_L = 0;
static float integral_R = 0;


void pid_task(void *pvParameter)
{
  while(1)
  {
      // pid_external_loop();
      vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

float map_speed(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void pid_external_loop()
{
  
  static int counter = 0;
  static float speed = 0.0f;

  if(200 <= counter)//------
  {
    counter = 0u;
    if(0 == speed)
    {
      speed = 500.0f;
    }
    else 
    {
      speed = 0.0f;
    }
  }

  pid_internal_left_loop(speed);//-----
  pid_internal_right_loop(speed);//-----

  data_packed.set_speed = speed;//-------

  data_packed.Kp = pid_data.Kp;
  data_packed.Ki = pid_data.Ki;
  data_packed.Kd = pid_data.Kd;

  esp_now_send(NULL, (uint8_t *)&data_packed, sizeof(data_packed));  
  counter++;//-----
}

void pid_internal_left_loop(float setPoint)
{
  static float Kp = 0.3f;//0.3f;
  static float Ki = 0.006f;//0.006f;
  static float Kd = 0.0f;//0.0f
  static float proportional;
  static float derivative;
  static float meas;
  static float prev_err;
  static float err; 
  static float output;
  static float Ts = 0.01f;  //sampling period in s
  static float T = 0.05f;  


  meas = encoder_get_speed(0u);

  data_packed.left_speed = meas;

  err = setPoint - meas;

  proportional = Kp * err;
  integral_L += Ki * err;
  if(integral_L >= 100)
  {
    integral_L = 100;
  }
  else if(integral_L <= -100)
  {
    integral_L = -100;
  }
  derivative = Kd * (-meas + prev_L_meas)/Ts;
  // derivative = (((T-Ts) * derivative) + (Kd*(err-prev_err)))/T;

  output = proportional + integral_L  + derivative;

  motor_set_speed(1u,output);
  prev_L_meas = meas;
  
}

void pid_internal_right_loop(float setPoint)
{
  static float Kp = 0.3f;//0.3f;
  static float Ki = 0.006f;//0.006f;
  static float Kd = 0.0f;//0.0f
  static float proportional;
  static float derivative;
  static float meas;
  static float prev_err;
  static float err; 
  static float output;
  static float Ts = 0.01f;  //sampling period in s
  static float T = 0.05f;  //sampling period in s

  meas = -encoder_get_speed(1u);

  data_packed.right_speed = meas;

  err = setPoint - meas;

  proportional = Kp * err;
  integral_R += Ki * err;
  if(integral_R >= 100)
  {
    integral_R = 100;
  }
  else if(integral_R <= -100)
  {
    integral_R = -100;
  }
  derivative = Kd * (-meas + prev_R_meas)/Ts;

  output = proportional + integral_R  + derivative;

  motor_set_speed(0u,output);
  prev_R_meas = meas;
}