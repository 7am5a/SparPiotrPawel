#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_setup.h"
#include "menu_callback.h"
#include "menu.h"
#include "lcd_buf_drv.h"
#include "switch_drv.h"
#include "enc_drv.h"
#include "lcd_drv.h"
#include "pwm_drv.h"
#include "esp_now_drv.h"
#include "esp_log.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int Kp = 1;
int Ki = 1;
int Kd = 1;

//flag used to manipulate PID parts
int settingsFlag;
extern int Encoder;
extern int encoder_value;

TaskHandle_t encoder_handle;

extern int pin_num;
extern int pwm_duty_lcd;
extern int pwm_duty_led;

extern uint32_t adc_read;
extern uint32_t voltage;

TaskHandle_t battery_handle;

//Below code is not necessary?------------------------------------
//Pointers to menu functions - not sure it work porperly
 void (*key_next_func)(void);
 void (*key_prev_func)(void);
 void (*key_enter_func)(void);
 void (*key_back_func)(void);

extern char lcd_buf[LCD_ROWS][LCD_COLS];

uint8_t percent = 50; //temp value - it should download pwm duty

void menu_level_back()
{
    key_next_func = menu_next;
    key_prev_func = menu_prev;
    key_enter_func = menu_enter;
    key_back_func = menu_back;

    menu_refresh();
}

//-------------------------------------------------------------------

void default_callback()
{
    //Temporary values - hardcode settings required calculations
    Kp = 1;
    Ki = 1;
    Kd = 1;
    printf("Kp, Ki, Kd was successfully reset\n");
    //And trigger sending function to RoboESP
}

void set_pid_callback()
{
    xTaskCreate(pid_callback_task, "pid_callback_task", 2048, NULL , 10, encoder_handle);
}

//add switch callback to turn on/off filter - can be used flag - check out some fancy names
/*
        OR create task to non blocking - flag probably won't be required and it wont be restart esp/trigger watchdog
*/
void pid_callback_task()//uint8_t *Kp, uint8_t *Ki, uint8_t *Kd)
{    
    printf("You choose: \n");
    while (1)
    {
        if(Encoder == 1)
        {
            vTaskDelay(30 / portTICK_RATE_MS);
            if(encoder_value >= 1 && encoder_value != 0)
            {
                Kp += 1;
                printf("Kp: %d\n", Kp);
            }

            if(encoder_value <= -1 && encoder_value != 0)
            {
                Kp -= 1;
                printf("Kp: %d\n", Kp);
            }
            Encoder = 0;
        }

        if(Encoder == 2)
        {
            vTaskDelay(30 / portTICK_RATE_MS);
            if(encoder_value >= 1)
            {
                Ki += 1;
                printf("Ki: %d\n", Ki);
            }

            if(encoder_value <= -1)
            {
                Ki -= 1;
                printf("Ki: %d\n", Ki);
            }
            Encoder = 0;
        }

        if(Encoder == 3)
        {
            vTaskDelay(30 / portTICK_RATE_MS);
            if(encoder_value >= 1)
            {
                Kd += 1;
                printf("Kd: %d\n", Kd);
            }

            if(encoder_value <= -1)
            {
                Kd -= 1;
                printf("Kd: %d\n", Kd);
            }
            Encoder = 0;
        }

        if((gpio_get_level(SWJ) == 0) && pin_num == SWJ)
        {
            vTaskDelete(encoder_handle);
        }

        vTaskDelay(20 / portTICK_RATE_MS);
    }
}

void kalman_callback()//uint8_t *temp)
{
    //To do more lately
}

void brightness_refresh()
{
    //VVV Function generer errors/reset ESP32 VVV

    percent = pwm_duty_lcd/1024 * 10; //10 not 100 to easier display characters 
    // char buffer[3];
    // itoa(percent, buffer, 10);
    buf_str("   BRIGHTNESS   ");
    buf_clear();

    //print brightness bar
    // buf_locate(1,1);
    // buf_str(buffer[0]);
    // buf_locate(2,1);
    // buf_str(buffer[1]);
    // buf_locate(3,1);
    // buf_str(buffer[2]);
    buf_locate(4,1);
    buf_char('%');

    memset(&lcd_buf[1][5], 0xff, percent/10); 
    memset(&lcd_buf[1][5 + percent/10], '-', 10-percent/10);
    
}

//Collecting gpio data could require small corrects -> probably in task in switch_drv.c + maby some latch 
void brightness_next()
{
    if((pin_num == SW3) && (gpio_get_level(SW3) == 0))
    {
        if(percent <= 90)
        {
            percent += 10;    
        }        
    }
//+ change by encoder TO DO
    brightness_refresh();
}

void brightness_prev()
{
    if((pin_num == SW3) && (gpio_get_level(SW3) == 0))
    {
        if(percent >= 20)
        {
            percent -= 10;    
        }        
    }
//+ change by encoder TO DO
    brightness_refresh();
}

void brightness_callback()//uint8_t *duty)
{
    //Do first from all "to do lately" :) default is set 1 by hardware pullup
    key_next_func = brightness_next; //enc or switch in enc
    key_prev_func = brightness_prev; //as above
    key_enter_func = NULL;
    key_back_func = menu_level_back;

    brightness_refresh();
}

void set_battery_callback()
{
    xTaskCreate(battery_callback_task, "battery_callback_task", 2048, NULL , 10, battery_handle);
}

void battery_callback_task()
{
    printf("Battery level is: \n");
    while (1)
    {   
        //3000 - is limit of Mikroe battery from datasheet (does it has a hardware limiter?)     
        float battery_percent = ((float)voltage - 3000)/(3700 - 3000) * 100;
        printf("Voltage: %dmV, %d %%\n", voltage, (int)battery_percent);
        
        if((gpio_get_level(SWJ) == 0) && pin_num == SWJ)
        {
            vTaskDelete(battery_handle);
        }
        
        vTaskDelay(5000 / portTICK_RATE_MS);
        vTaskDelete(battery_handle);
    
    }
}