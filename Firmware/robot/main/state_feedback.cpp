#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

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
#include "mpu6050_read_pitch.h"

extern "C" {
	#include "mpu6050_read_angle_acc.h"
    #include "MPU6050_acc.h"
}

#define FILTER_ELEMENTS_NUMBER 10
#define STATE_FEEDBACK_RATE 10

float filter_values[FILTER_ELEMENTS_NUMBER];
float sorted_values[FILTER_ELEMENTS_NUMBER];
static float filtered_robot_angle;
static float wheel_angle = 0; //[deg] negative

extern bool calibrated;

Data_state_packed data_state_packed;
SampleFilter sample_filter;
extern State_feedback_vals state_feedback_vals;


void state_feedback_task(void *pvParameter)
{
    SampleFilter_init(&sample_filter);
    mpu6050_init();
    while(1)
    {
        if(calibrated)
        {
            state_feedback_controller();
        }
        vTaskDelay(STATE_FEEDBACK_RATE/portTICK_PERIOD_MS);
    }
}

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}


void state_feedback_controller()
{
    static float output;
    static float a = 0.99; //lowpass filter constant
    static float meas_acc[2];
    static float meas_pitch[2];
    static float left_speed;
    static float right_speed;
    static float robot_angle; //[deg] positive
    static float robot_angle_speed; //[deg/s] negative
    static float wheel_angle_speed; //[RPM] negative

    mpu6050_acc_read(meas_acc);
    mpu6050_pitch_read(meas_pitch);

    robot_angle = 0.1 * meas_acc[0] - 0.9 * meas_pitch[0];
    robot_angle_speed = meas_acc[1];

    /*FIR lowpass filter*/
    // SampleFilter_put(&sample_filter, (double)robot_angle);
    // robot_angle = SampleFilter_get(&sample_filter);

    /*IIR lowpass filter*/
    // filtered_robot_angle = ((1-a) * filtered_robot_angle) + (robot_angle * a);

    /*Median*/
    // for(int i = FILTER_ELEMENTS_NUMBER-1; i > 0; i--)
    // {
    //     filter_values[i] = filter_values[i-1];
    //     sorted_values[i] = filter_values[i-1];
    // }
    // filter_values[0] = robot_angle;
    // sorted_values[0] = robot_angle;
    // qsort(sorted_values, FILTER_ELEMENTS_NUMBER, sizeof(float), compare);
    // robot_angle = sorted_values[FILTER_ELEMENTS_NUMBER/2];

    wheel_angle = ((-encoder_get_radian(1u) + encoder_get_radian(0u))/2); //repair
    
    left_speed = encoder_get_speed(0u);
    right_speed = -encoder_get_speed(1u);
    wheel_angle_speed = ((left_speed + right_speed)/2);

    //printf(">acc_angle:%f\n>pitch_angle:%f\n>robot_angle:%f\n>robot_angle_speed:%f\n>wheel_angle:%f\n>wheel_angle_speed:%f\n",meas_acc[0], meas_pitch[0], robot_angle, robot_angle_speed, wheel_angle, wheel_angle_speed);

    data_state_packed.left_speed = left_speed;
    data_state_packed.right_speed = right_speed;
    data_state_packed.robot_angle = robot_angle;
    data_state_packed.robot_angle_speed = robot_angle_speed;
    data_state_packed.wheel_angle = wheel_angle;
    data_state_packed.wheel_angle_speed = wheel_angle_speed;

    if(70 > robot_angle && -70 < robot_angle)
    {
    // output = (-(0* wheel_angle) + (20.2 * filtered_robot_angle) - (0 * wheel_angle_speed) + (1.7 * robot_angle_speed));
    output = (-(state_feedback_vals.K1 * wheel_angle) + (state_feedback_vals.K2 * robot_angle) 
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

    esp_now_send(NULL, (uint8_t *)&data_state_packed, sizeof(data_state_packed));  
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