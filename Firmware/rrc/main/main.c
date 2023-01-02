#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "esp_hello.h"
#include "esp_setup.h"
#include "switch_drv.h"
#include "adc_drv.h"
#include "pwm_drv.h"
#include "enc_drv.h"
//#include "lcd_drv.h"
//#include "lcd_buf_drv.h"  //maby not insert these libraries causes warnings and esp reset -> commented parts of functions in menu callback and lcd_buf
#include "menu_callback.h"
#include "menu.h"
#include "esp_now_drv.h"
#include "lcd_st7032.h"

void app_main(void)
{
    //initialization
    init_gpio_sw();
    init_gpio_pwm();
    init_gpio_adc();
    init_gpio_enc();

    lcd_st7032_init();

    init_esp_now();

    //------------------------------------------------

    //test/debug task - not necessary in final release
    esp_hello_task(); 
    //------------------------------------------------
       
    //Instalation other functions
    install_gpio_sw();
    //------------------------------------------------

    //Set up all tasks
    set_switch_task(); 
    set_read_adc_task();
    //set_pwm_task();
    set_enc_task();
    set_menu_task();
    //-----------------------------------------------
         
}