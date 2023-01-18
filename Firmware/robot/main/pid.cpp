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

static float prev_L_meas = 0;
static float prev_R_meas = 0;
static float integral_L = 0;
static float integral_R = 0;


void pid_task(void *pvParameter)
{
  SampleFilter_init(&sample_filter);

  while(1)
  {
      if(calibrated)
      {
        // pid_external_loop();
      }
      vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

float map_speed(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// void pid_external_loop()
// {
//   /*
//   static int counter = 0;
//   static float speed = 0.0f;
//   // static float Kp = 20.0f; //23
//   // static float Ki = 0.0f;
//   // static float Kd = 0.0f; //0.5
//   static float proportional;
//   static float integral;
//   static float derivative;
//   static float meas;
//   static float prev_err;
//   static float err; 
//   static float output;
//   static float setPoint = 0.0f;
//   static float T = 0.05f;    //low-pass filter constant in s
//   static float Ts = 0.01f;  //sampling period in s

//   // meas = mpu6050_read_angle();

//   // if(60 < meas)
//   // {
//   //   meas = 60;
//   // }
//   // else if(-60 > meas)
//   // {
//   //   meas = -60;
//   // }
//   // SampleFilter_put(&sample_filter, (double)meas);
//   // meas = SampleFilter_get(&sample_filter);
 
//   // data_packed.angle = meas;
  
//   // if(60 > meas && -60 < meas)
//   // {
//   //   err = setPoint - meas;

//   //   proportional = pid_data.Kp * err;
//   //   integral += pid_data.Ki * err;
//   //   if(integral >= 100)
//   //   {
//   //     integral = 100;
//   //   }

//   //   derivative = ((T-Ts) * derivative) + (pid_data.Kd*(err-prev_err))/T;
//   //   //derivative = (pid_data.Kd*(err-prev_err))/Ts;
//   //   prev_err = err;
//   //   output = proportional + integral  + derivative;

//   //   if(800 < output)
//   //   {
//   //     output = 800;
//   //   }
//   //   else if(-800 > output)
//   //   {
//   //     output = -800;
//   //   }

//   //   pid_internal_left_loop(output);
//   //   pid_internal_right_loop(output);
//   //   data_packed.set_speed = output;
//   //   // printf(">setSpeed:%f\n", output);
//   // }
//   // else
//   // {
//   //   pid_internal_left_loop(0u);
//   //   pid_internal_right_loop(0u);
//   //   data_packed.set_speed = 0.0f;
//   // }
//   // printf(">angle:%f\n", meas);

//   if(200 <= counter)//------
//   {
//     counter = 0u;
//     if(0 == speed)
//     {
//       speed = 500.0f;
//     }
//     else 
//     {
//       speed = 0.0f;
//     }
//   }

//   pid_internal_left_loop(speed);//-----
//   pid_internal_right_loop(speed);//-----

//   data_packed.set_speed = speed;//-------

//   data_packed.Kp = pid_data.Kp;
//   data_packed.Ki = pid_data.Ki;
//   data_packed.Kd = pid_data.Kd;
  
//   // printf(">Kp:%f\n",pid_data.Kp);
//   // printf(">Ki:%f\n",pid_data.Ki);
//   // printf(">Kd:%f\n",pid_data.Kd);

//   esp_now_send(NULL, (uint8_t *)&data_packed, sizeof(data_packed));  
//   counter++;//------
//   */

 
//   static float meas[2];
//   static float prev_err;
//   static float err; 
//   static float robot_angle; 
//   static float wheel_angle; 
//   static float robot_angle_speed; 
//   static float output;
//   static float setPoint = 0.0f;
//   static float left_speed;
//   static float right_speed;
//   static float wheel_angle_speed; // cm/s
//   static float position; // cm
//   static float last_position; // cm

 

//   mpu6050_read_angle(meas);
//   robot_angle = meas[0];
//   robot_angle_speed = meas[1];

//   data_packed.angle = robot_angle;

//   wheel_angle = (-encoder_get_radian(1u) + encoder_get_radian(0u))/2;

//   left_speed = encoder_get_speed(0u);
//   right_speed = -encoder_get_speed(1u);
//   wheel_angle_speed = ((left_speed + right_speed)/2);

//   printf(">angle:%f\n>gyro_y:%f\n>wheel_angle:%f\n>angle_speed:%f\n", robot_angle, robot_angle_speed, wheel_angle, wheel_angle_speed);

//   if(60 > robot_angle && -60 < robot_angle)
//   {
//   err = setPoint - robot_angle;

 
//   //derivative = (pid_data.Kd*(err-prev_err))/Ts;
//   prev_err = err;
//   output = (-(0.8873* wheel_angle) - (12.6411 * robot_angle) - (5.5322 * wheel_angle_speed) + (3.3670 * robot_angle_speed));
 
//   if(100 < output)
//   {
//     output = 100;
//   }
//   else if(-100 > output)
//   {
//     output = -100;
//   }

//   motor_set_speed(0u,output);
//   motor_set_speed(1u,output);
//   // pid_internal_left_loop(output);
//   // pid_internal_right_loop(output);
//   data_packed.set_speed = output;
//   // printf(">setSpeed:%f\n", output);
//   }
//   else
//   {
//     pid_internal_left_loop(0u);
//     pid_internal_right_loop(0u);
//     data_packed.set_speed = 0.0f;
//   }
//   data_packed.Kp = pid_data.Kp;
//   data_packed.Ki = pid_data.Ki;
//   data_packed.Kd = pid_data.Kd;

//   esp_now_send(NULL, (uint8_t *)&data_packed, sizeof(data_packed));  
 
// }

void pid_internal_left_loop(float setPoint)
{
  static float Kp = 0.6f;//0.6f;
  static float Ki = 0.001f;//0.0003f;
  static float Kd = 0.0f;//0.0001f
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
  static float Kp = 0.6f;//0.6f;
  static float Ki = 0.001f;//0.0003f;
  static float Kd = 0.0f;
  static float proportional;
  static float derivative;
  static float meas;
  static float prev_err;
  static float err; 
  static float output;
  static float Ts = 0.01f;  //sampling period in s
  static float T = 0.05f;  //sampling period in s

  meas = -encoder_get_speed(1u);

  data_packed.right_speed = meas+40;

  err = setPoint - meas;

  proportional = Kp * err;
  integral_R += Ki * err;
  // derivative = (((T-Ts) * derivative) + Kd*(err-prev_err))/T;5
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