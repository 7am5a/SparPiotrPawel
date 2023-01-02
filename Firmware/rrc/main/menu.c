#include <stdio.h>
#include "menu.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_setup.h"
#include "menu_callback.h"
#include "esp_log.h"
#include "lcd_st7032.h"

extern TaskHandle_t encoder_handle;
extern TaskHandle_t brightness_handle;
extern TaskHandle_t battery_handle;
extern TaskHandle_t manual_handle;

extern int pin_num;
/*
    Menu tree:
    level   X   index   0       index   1       index...
    level   0   menu1           menu2           NULL
    level   1   sub_menu1_X     sub_menu2_X     NULL
    level   2   NULL            NULL            NULL
*/
//             (*name,                          *next,          *prev,          *child,         *parent,        (*menu_function))
menu_t menu1 = {"Manual control",               &menu2,         &menu4,         &sub_menu1_1,   NULL,           set_robot_control_task};
    menu_t sub_menu1_1 = {"Now you can ride",   NULL,           &sub_menu1_1,   NULL,           &menu1,         NULL};
menu_t menu2 = {"Choose algorithm",             &menu3,         &menu1,         &sub_menu2_1,   NULL,           NULL};
    menu_t sub_menu2_1 = {"Default settings",   &sub_menu2_2,   &sub_menu2_3,   NULL,           &menu2,         default_callback};
    menu_t sub_menu2_2 = {"PID",                &sub_menu2_3,   &sub_menu2_1,   &sub_menu2_2_1, &menu2,         set_pid_callback_task};
        menu_t sub_menu2_2_1 ={"PID settings",  NULL,           &sub_menu2_2_1, NULL,           &sub_menu2_2,   NULL};
    menu_t sub_menu2_3 = {"Kalman Filter",      NULL,           &sub_menu2_2,   NULL,           &menu2,         kalman_callback};
menu_t menu3 = {"Settings",                     &menu4,         &menu2,         &sub_menu3_1,   NULL,           NULL};
    menu_t sub_menu3_1 = {"Brightness",         NULL,           &sub_menu3_1,   NULL,           &menu3,         set_brightness_callback};
menu_t menu4 = {"Battery",                      &menu1,         &menu3,         &sub_menu4_1,   NULL,           set_battery_callback};
    menu_t sub_menu4_1 = {"Battery level",      &sub_menu4_1,   &sub_menu4_1,   NULL,           &menu4,         NULL};

//Current position address in menu
menu_t *currentPointer = &menu1;
menu_t *lastPointer = &menu1;

uint8_t menu_index;                                 //Current menu level; start at 0
uint8_t lcd_row_pos;                                //Current menu row; start at 0
uint8_t lcd_row_pos_level_1, lcd_row_pos_level_2, lcd_row_pos_level_3;   //Row position


char lcd_buf[LCD_ROWS][LCD_COLS];

//probably not necessary
//  extern void (*key_next_func)(void);
//  extern void (*key_prev_func)(void);
//  extern void (*key_enter_func)(void);
//  extern void (*key_back_func)(void);

void menu_refresh()
{
    menu_t *temp;
    uint8_t i;

    if(currentPointer -> parent)
    {
        temp = (currentPointer -> parent) -> child;
    }
    else
    {
        temp = &menu1;
    }

    for ( i = 0; i != menu_index - lcd_row_pos; i++)
    {
        temp = temp -> next;
    } 

    lcd_st7032_clear();
    lcd_st7032_set_cursor(0, 0);	
    lcd_st7032_print(currentPointer -> name); 
    
}

uint8_t menu_get_index(menu_t *index)
{
    menu_t *temp;
    uint8_t i = 0;

     if(index -> parent)
     {
        temp = (index -> parent) -> child;
     }
     else
     {
        temp = &menu1;
     }

     while (temp != index)
     {
        temp = temp -> next;
        i++;
     }
     return i;
}

uint8_t menu_get_level(menu_t *level)
{
    menu_t *temp = level;
    uint8_t i = 0;

    if (!level -> parent)
    {
        return 0;
    }

    while (temp -> parent != NULL)
    {
        temp = temp -> parent;
        i++;
    }
    return i;
}

void menu_next()
{    
    if(currentPointer -> next)
    {
        currentPointer = currentPointer -> next;
        menu_index++;

        if (lcd_row_pos > LCD_ROWS - 1)
        {
            lcd_row_pos = LCD_ROWS - 1;
        }
        
    }
    else
    {
        menu_index = 0;
        lcd_row_pos = 0;

        if (currentPointer -> parent)
        {
            currentPointer = (currentPointer -> parent) -> child;
        }        
    }

    printf("%s \n", currentPointer -> name);
    lcd_st7032_clear();
    lcd_st7032_set_cursor(0, 0);	
    lcd_st7032_print(currentPointer -> name);
    menu_refresh();

}

void menu_prev()
{
    currentPointer = currentPointer -> prev;

    if (menu_index)
    {
        menu_index--;
        if (lcd_row_pos > 0)
        {
            lcd_row_pos--;
        }        
    }
    else
    {
        menu_index = menu_get_index(currentPointer); // = 0 was not be enough?

        if (menu_index >= LCD_ROWS - 1)
        {
            lcd_row_pos = LCD_ROWS - 1;
        }
        else
        {
            lcd_row_pos = menu_index;
        }
    }

    printf("%s \n", currentPointer -> name);
    lcd_st7032_clear();
    lcd_st7032_set_cursor(0, 0);	
    lcd_st7032_print(currentPointer -> name);
    menu_refresh();

}

void menu_enter()
{
    if(currentPointer -> menu_function)
    {        
        currentPointer -> menu_function();
        lastPointer = currentPointer;
        ESP_LOGI("SW2","do function");
        printf("%s \n", lastPointer -> name);
        lcd_st7032_clear();
        lcd_st7032_set_cursor(0, 0);	
        lcd_st7032_print(lastPointer -> name);
    }

    if (currentPointer -> child)
    {
        switch (menu_get_level(currentPointer)) 
        {
            case 0:
            ESP_LOGI("log","enter");
                lcd_row_pos_level_1 = lcd_row_pos;
                break;

            case 1:
                lcd_row_pos_level_2 = lcd_row_pos;
                break;

            case 2:
                lcd_row_pos_level_3 = lcd_row_pos;
                break;            
		}
        menu_index = 0;
        lcd_row_pos = 0;
        currentPointer = currentPointer -> child;
        printf("%s \n", currentPointer -> name);
        lcd_st7032_clear();
        lcd_st7032_set_cursor(0, 0);	
        lcd_st7032_print(currentPointer -> name);
        menu_refresh();
    }    
}

void menu_back()
{    
    vTaskDelay(50 / portTICK_RATE_MS);
    if(currentPointer -> parent)
    {
        switch (menu_get_level(currentPointer)) {
			case 0:
                ESP_LOGI("log","go back0");
				lcd_row_pos = lcd_row_pos_level_1;                
				break;
 
			case 1:
                ESP_LOGI("log","go back1");
				lcd_row_pos = lcd_row_pos_level_2;
				break;

            case 2:
                ESP_LOGI("log","go back2");
				lcd_row_pos = lcd_row_pos_level_3;
				break;
		}

        currentPointer = currentPointer -> parent;
        menu_index = menu_get_index(currentPointer);
    
        printf("%s \n", currentPointer -> name);
        lcd_st7032_clear();
        lcd_st7032_set_cursor(0, 0);	
        lcd_st7032_print(currentPointer -> name);
        menu_refresh();
        
    }
}
//Add flag blocking menu changing - next prev and enter
//Require correction, but first set up
void menu_task()
{
    printf("%s \n", currentPointer -> name);
    lcd_st7032_clear();
    lcd_st7032_set_cursor(0, 0);
    lcd_st7032_clear();
    vTaskDelay(100 / portTICK_RATE_MS);
    lcd_st7032_print(currentPointer -> name);
    while(1)
    {
        if((gpio_get_level(SW1) == 0) && pin_num == SW1)
        {
            menu_prev();
        }

        if((gpio_get_level(SW2) == 0) && pin_num == SW2)
        {
            menu_enter();            
        }

        if((gpio_get_level(SW3) == 0) && pin_num == SW3)
        {
            menu_next();
        }

        if((gpio_get_level(SWJ) == 0) && pin_num == SWJ)
        {
            menu_back();
        }
        vTaskDelay(20 / portTICK_RATE_MS);
    }
}

void set_menu_task()
{
    xTaskCreate(menu_task, "menu_task", 2048, NULL, 10, NULL);
}