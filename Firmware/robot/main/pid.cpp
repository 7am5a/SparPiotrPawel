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
  /*
  static int counter = 0;
  static float speed = 0.0f;
  // static float Kp = 20.0f; //23
  // static float Ki = 0.0f;
  // static float Kd = 0.0f; //0.5
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
 
  data_packed.angle = meas;
  
  if(60 > meas && -60 < meas)
  {
    err = setPoint - meas;

    proportional = pid_data.Kp * err;
    integral += pid_data.Ki * err;
    if(integral >= 100)
    {
      integral = 100;
    }

    derivative = ((T-Ts) * derivative) + (pid_data.Kd*(err-prev_err))/T;
    //derivative = (pid_data.Kd*(err-prev_err))/Ts;
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
    data_packed.set_speed = output;
    // printf(">setSpeed:%f\n", output);
  }
  else
  {
    pid_internal_left_loop(0u);
    pid_internal_right_loop(0u);
    data_packed.set_speed = 0.0f;
  }
  // printf(">angle:%f\n", meas);

  // if(100 <= counter)//------
  // {
  //   counter = 0u;
  //   if(0 == speed)
  //   {
  //     speed = 400.0f;
  //   }
  //   else 
  //   {
  //     speed = 0.0f;
  //   }
  // }

  // pid_internal_left_loop(speed);//-----
  // pid_internal_right_loop(speed);//-----

  // data_packed.set_speed = speed;//-------

  data_packed.Kp = pid_data.Kp;
  data_packed.Ki = pid_data.Ki;
  data_packed.Kd = pid_data.Kd;
  
  // printf(">Kp:%f\n",pid_data.Kp);
  // printf(">Ki:%f\n",pid_data.Ki);
  // printf(">Kd:%f\n",pid_data.Kd);

  esp_now_send(NULL, (uint8_t *)&data_packed, sizeof(data_packed));  
  // counter++;//------
  */
  static float meas[2];
  static float prev_err;
  static float err; 
  static float angle; 
  static float gyro_y; 
  static float output;
  static float setPoint = 0.0f;
  static float left_speed;
  static float right_speed;
  static float linear_speed; // cm/s

 

  mpu6050_read_angle(meas);
  angle = meas[0];
  gyro_y = meas[1];

  data_packed.angle = angle;

  left_speed = encoder_get_speed(0u);
  right_speed = -encoder_get_speed(1u);
  linear_speed = ((left_speed + right_speed)/2) * 4.5;

  if(60 > angle && -60 < angle)
  {
  err = setPoint - angle;

 
  //derivative = (pid_data.Kd*(err-prev_err))/Ts;
  prev_err = err;
  output = (pid_data.Ki * gyro_y) - (pid_data.Kp * angle) - (pid_data.Kd * linear_speed);
 
  if(100 < output)
  {
    output = 100;
  }
  else if(-100 > output)
  {
    output = -100;
  }

  motor_set_speed(0u,output);
  motor_set_speed(1u,output);
  // pid_internal_left_loop(output);
  // pid_internal_right_loop(output);
  data_packed.set_speed = output;
  // printf(">setSpeed:%f\n", output);
  }
  else
  {
    pid_internal_left_loop(0u);
    pid_internal_right_loop(0u);
    data_packed.set_speed = 0.0f;
  }
  data_packed.Kp = pid_data.Kp;
  data_packed.Ki = pid_data.Ki;
  data_packed.Kd = pid_data.Kd;

  esp_now_send(NULL, (uint8_t *)&data_packed, sizeof(data_packed));  
}

void pid_internal_left_loop(float setPoint)
{
  static float Kp = 0.6f;//0.6f;
  static float Ki = 0.0071f;//0.0003f;
  static float Kd = 0.001f;
  static float proportional;
  static float integral;
  static float derivative;
  static float meas;
  static float prev_meas;
  static float prev_err;
  static float err; 
  static float output;
  static float Ts = 0.01f;  //sampling period in s


  meas = encoder_get_speed(0u);

  data_packed.left_speed = meas;

  err = setPoint - meas;

  proportional = Kp * err;
  integral += Ki * err;
  if(integral >= 100)
  {
    integral = 100;
  }
  else if(integral <= -100)
  {
    integral = -100;
  }
  derivative = (Kd*(err-prev_err))/Ts;

  output = proportional + integral  + derivative;

  motor_set_speed(1u,output);
  prev_meas = meas;
}

void pid_internal_right_loop(float setPoint)
{
  static float Kp = 0.6f;//0.6f;
  static float Ki = 0.0071f;//0.0003f;
  static float Kd = 0.001f;
  static float proportional;
  static float integral;
  static float derivative;
  static float meas;
  static float prev_meas;
  static float prev_err;
  static float err; 
  static float output;
  static float Ts = 0.01f;  //sampling period in s


  meas = -encoder_get_speed(1u);

  data_packed.right_speed = meas;

  err = setPoint - meas;

  proportional = Kp * err;
  integral += Ki * err;
  if(integral >= 100)
  {
    integral = 100;
  }
  else if(integral <= -100)
  {
    integral = -100;
  }
  derivative = (Kd*(err-prev_err))/Ts;

  output = proportional + integral  + derivative;

  motor_set_speed(0u,output);
  prev_meas = meas;
}