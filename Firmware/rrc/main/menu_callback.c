#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_setup.h"
#include "menu_callback.h"
#include "menu.h"
#include "switch_drv.h"
#include "enc_drv.h"
#include "pwm_drv.h"
#include "esp_now_drv.h"
#include "esp_log.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_st7032.h"
#include "esp_now_drv.h"
#include "adc_drv.h"

bool manualState = false;
bool manualStateTaskCreate = false;

int Kp = 1;
int Ki = 1;
int Kd = 1;
char KpTab[4];
char KiTab[4];
char KdTab[4];

bool backlight = true;

char batPerc[3];

//flag used to manipulate PID parts
int settingsFlag;
extern int Encoder;
extern int encoder_value;

TaskHandle_t encoder_handle;
TaskHandle_t brightness_handle;
TaskHandle_t battery_handle;
TaskHandle_t manual_handle;
TaskHandle_t joystick_handle;

extern int pin_num;
extern int pwm_duty_lcd;
extern int pwm_duty_led;

extern uint32_t adc_read;
extern uint32_t voltage;

//Below code is not necessary?------------------------------------
//Pointers to menu functions - not sure it work porperly
 void (*key_next_func)(void);
 void (*key_prev_func)(void);
 void (*key_enter_func)(void);
 void (*key_back_func)(void);


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


void set_robot_control_task()
{
    xTaskCreate(read_adc_task, "read_adc_task", 2048, NULL , 9, joystick_handle);
}

void default_callback()
{
    //Temporary values - hardcode settings required calculations
    Kp = 1;
    Ki = 1;
    Kd = 1;    
    printf("Kp, Ki, Kd was successfully reset\n");
    lcd_st7032_set_cursor(0, 0);
	lcd_st7032_print("Kp, Ki, Kd reset");
    lcd_st7032_set_cursor(1, 0);
	lcd_st7032_print("successfully");
    vTaskDelay(2000 / portTICK_RATE_MS);
    //And trigger sending function to RoboESP
}

void set_pid_callback_task()
{
    xTaskCreate(pid_callback_task, "pid_callback_task", 2048, NULL , 10, encoder_handle);
}

void pid_callback_task()//uint8_t *Kp, uint8_t *Ki, uint8_t *Kd)
{    
    vTaskDelay(1000 / portTICK_RATE_MS);
    printf("You choose: \n");
    lcd_st7032_clear();
    lcd_st7032_set_cursor(0, 0);	
    lcd_st7032_print("You choose: ");
    lcd_st7032_set_cursor(1, 0);
    while (1)
    {
        if(Encoder == 1)
        {
            vTaskDelay(30 / portTICK_RATE_MS);
            lcd_st7032_clear();
            lcd_st7032_set_cursor(0, 0);	
            lcd_st7032_print("You choose: ");
            lcd_st7032_set_cursor(1, 0);
            if(encoder_value >= 1 && encoder_value != 0)
            {
                Kp += 1;
                printf("Kp: %d\n", Kp);                
                lcd_st7032_print("Kp: ");
                lcd_st7032_set_cursor(1, 4);
                lcd_st7032_print(itoa(Kp,KpTab,10));
                enc_send_now("Kp",Kp);
            }

            if(encoder_value <= -1 && encoder_value != 0)
            {
                Kp -= 1;
                printf("Kp: %d\n", Kp);                
                lcd_st7032_print("Kp: ");
                lcd_st7032_set_cursor(1, 4);
                lcd_st7032_print(itoa(Kp,KpTab,10));
                enc_send_now("Kp",Kp);
            }
            Encoder = 0;
        }

        if(Encoder == 2)
        {
            vTaskDelay(30 / portTICK_RATE_MS);
            
            lcd_st7032_clear();
            lcd_st7032_set_cursor(0, 0);	
            lcd_st7032_print("You choose: ");
            lcd_st7032_set_cursor(1, 0);

            if(encoder_value >= 1)
            {
                Ki += 1;
                printf("Ki: %d\n", Ki);                
                lcd_st7032_print("Ki: ");
                lcd_st7032_set_cursor(1, 4);
                lcd_st7032_print(itoa(Ki,KiTab,10));
                enc_send_now("Ki",Ki);
            }

            if(encoder_value <= -1)
            {
                Ki -= 1;
                printf("Ki: %d\n", Ki);                
                lcd_st7032_print("Ki: ");
                lcd_st7032_set_cursor(1, 4);
                lcd_st7032_print(itoa(Ki,KiTab,10));
                enc_send_now("Ki",Ki);
            }
            Encoder = 0;
        }

        if(Encoder == 3)
        {
            vTaskDelay(30 / portTICK_RATE_MS);
            lcd_st7032_clear();
            lcd_st7032_set_cursor(0, 0);	
            lcd_st7032_print("You choose: ");
            lcd_st7032_set_cursor(1, 0);

            if(encoder_value >= 1)
            {
                Kd += 1;
                printf("Kd: %d\n", Kd);
                lcd_st7032_print("Kd: ");
                lcd_st7032_set_cursor(1, 4);
                lcd_st7032_print(itoa(Kd,KdTab,10));
                enc_send_now("Kd",Kd);
            }

            if(encoder_value <= -1)
            {
                Kd -= 1;
                printf("Kd: %d\n", Kd);
                lcd_st7032_print("Kd: ");
                lcd_st7032_set_cursor(1, 4);
                lcd_st7032_print(itoa(Kd,KdTab,10));
                enc_send_now("Kd",Kd);
            }
            Encoder = 0;
        }

        if((gpio_get_level(SWJ) == 0) && pin_num == SWJ)
        {
            ESP_LOGW("pid","deleted");
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
    
    //buf_str("   BRIGHTNESS   ");
    //buf_clear();

    //print brightness bar
    // buf_locate(1,1);
    // buf_str(buffer[0]);
    // buf_locate(2,1);
    // buf_str(buffer[1]);
    // buf_locate(3,1);
    // buf_str(buffer[2]);
    //buf_locate(4,1);
    //buf_char('%');

    //memset(&lcd_buf[1][5], 0xff, percent/10); 
    //memset(&lcd_buf[1][5 + percent/10], '-', 10-percent/10);
    
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

void set_brightness_callback()
{
    xTaskCreate(brightness_callback_task, "brihtness_callback_task", 2048, NULL , 10, brightness_handle);
}

void brightness_callback_task()//uint8_t *duty)
{
    //need to be changed
    vTaskDelay(300 / portTICK_RATE_MS);
    lcd_st7032_clear();
    lcd_st7032_set_cursor(0, 0);	
    lcd_st7032_print("Backlight is:");
    while(1)
    {

    
        // //Do first from all "to do lately" :) default is set 1 by hardware pullup
        // key_next_func = brightness_next; //enc or switch in enc
        // key_prev_func = brightness_prev; //as above
        // key_enter_func = NULL;
        // key_back_func = menu_level_back;

        // brightness_refresh();
        
        if(backlight == true)
        {
            lcd_st7032_set_cursor(1, 8);	
            lcd_st7032_print("ON");
        }
        else if (backlight == false)
        {
            lcd_st7032_set_cursor(1, 8);	
            lcd_st7032_print("OFF");
        }
        
        if((pin_num == SW2) && (gpio_get_level(SW2) == 0))
        {
            if (backlight == true)
            {
                backlight = false;
                
            }
            else if (backlight == false)
            {
                backlight = true;
                lcd_st7032_display_on();
            }      
        }

        if((gpio_get_level(SWJ) == 0) && pin_num == SWJ)
            {
                vTaskDelete(brightness_handle);
            }

        vTaskDelay(20 / portTICK_RATE_MS);

    }
}

void set_battery_callback()
{
    xTaskCreate(battery_callback_task, "battery_callback_task", 2048, NULL , 10, battery_handle);
}

void battery_callback_task()
{
    printf("Battery level is: \n");
    lcd_st7032_clear();
    lcd_st7032_set_cursor(0, 0);
    lcd_st7032_print("Battery level:  ");

    while (1)
    {   
        //3000 - is limit of Mikroe battery from datasheet (does it has a hardware limiter?)     
        float battery_percent = ((float)voltage - 3000)/(3700 - 3000) * 100;
        printf("Voltage: %dmV, %d %%\n", voltage, (int)battery_percent);
        
        lcd_st7032_set_cursor(1, 0);
        lcd_st7032_print("      ");
        lcd_st7032_set_cursor(1, 6);
        lcd_st7032_print(itoa((int)battery_percent,batPerc,10));
        lcd_st7032_set_cursor(1, 9);
        lcd_st7032_print("%%");
        lcd_st7032_set_cursor(1, 10);
        lcd_st7032_print(" ");

        if((gpio_get_level(SWJ) == 0) && pin_num == SWJ)
        {
            vTaskDelete(battery_handle);
        }
        
        vTaskDelay(5000 / portTICK_RATE_MS);
        vTaskDelete(battery_handle);
    
    }
}