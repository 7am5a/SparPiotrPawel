#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "mpu6050_read_angle.h"
#include "motor_control.h"
#include <math.h>
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "Encoder.h"
#include "SampleFilter.h"
#include "esp_now_drv.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "state_feedback.h"
#include "pid.h"

#define FILTER_ELEMENTS_NUMBER 30

extern bool calibrated;

Data_state_packed data_state_packed;
extern State_feedback_vals state_feedback_vals;
float filter_values[FILTER_ELEMENTS_NUMBER];
float sorted_values[FILTER_ELEMENTS_NUMBER];

void state_feedback_task(void *pvParameter)
{
  while(1)
  {
      if(calibrated)
      {
        state_feedback_controller();
      }
      vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}


void state_feedback_controller()
{
    static float meas[2];
    static float output;
    static float left_speed;
    static float right_speed;
    static float robot_angle; //[deg] positive
    static float wheel_angle = 0; //[deg] negative
    static float robot_angle_speed; //[deg/s] negative
    static float wheel_angle_speed; //[RPM] negative

    mpu6050_read_angle(meas);
    robot_angle = meas[0];
    robot_angle_speed = meas[1];

    // for(int i = FILTER_ELEMENTS_NUMBER-1; i > 0; i--)
    // {
    //     filter_values[i] = filter_values[i-1];
    //     sorted_values[i] = filter_values[i-1];
    // }
    // filter_values[0] = robot_angle;
    // sorted_values[0] = robot_angle;
    // qsort(sorted_values, FILTER_ELEMENTS_NUMBER, sizeof(float), compare);
    // robot_angle = sorted_values[FILTER_ELEMENTS_NUMBER/2];

    // wheel_angle = (-encoder_get_radian(1u) + encoder_get_radian(0u))/2; //to repair
    
    left_speed = encoder_get_speed(0u);
    right_speed = -encoder_get_speed(1u);
    wheel_angle_speed = ((left_speed + right_speed)/2);

    printf(">robot_angle:%f\n>robot_angle_speed:%f\n>wheel_angle:%f\n>wheel_angle_speed:%f\n", robot_angle, robot_angle_speed, wheel_angle, wheel_angle_speed);

    data_state_packed.left_speed = left_speed;
    data_state_packed.right_speed = right_speed;
    data_state_packed.robot_angle = robot_angle;
    data_state_packed.robot_angle_speed = robot_angle_speed;
    data_state_packed.wheel_angle = wheel_angle;
    data_state_packed.wheel_angle_speed = wheel_angle_speed;

    if(70 > robot_angle && -70 < robot_angle)
    {
    // output = (-(0* wheel_angle) - (15 * robot_angle) - (0 * wheel_angle_speed) + (0 * robot_angle_speed));
    output = (-(state_feedback_vals.K1 * wheel_angle) - (state_feedback_vals.K2 * robot_angle) 
        - (state_feedback_vals.K3 * wheel_angle_speed) + (state_feedback_vals.K4 * robot_angle_speed));
    
    if(750 < output)
    {
        output = 750;
    }
    else if(-750 > output)
    {
        output = -750;
    }

    // motor_set_speed(0u,0);
    // motor_set_speed(1u,0);
    pid_internal_left_loop(output);
    pid_internal_right_loop(output);
    // printf(">setSpeed:%f\n", output);

    data_state_packed.set_speed = output;
    data_state_packed.K1 = state_feedback_vals.K1;
    data_state_packed.K2 = state_feedback_vals.K2;
    data_state_packed.K3 = state_feedback_vals.K3;
    data_state_packed.K4 = state_feedback_vals.K4;
    }
    else
    {
        state_feedback_motor_stop(0u);
        state_feedback_motor_stop(1u);
        data_state_packed.set_speed = 0.0f;
    }

    // esp_now_send(NULL, (uint8_t *)&data_state_packed, sizeof(data_state_packed));  
}

float state_feedback_motor_stop(uint8_t motor)
{
    static float meas = 0;

    motor_stop(motor);

    if(0u == motor)
    {
        meas = encoder_get_speed(motor);
        data_state_packed.left_speed = meas;
    }
    else
    {
        meas = -encoder_get_speed(motor);
        data_state_packed.right_speed = meas;
    }
    
    return meas;
}