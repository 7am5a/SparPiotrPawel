#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_setup.h"
#include "switch_drv.h"
#include "esp_log.h"

int is_pressed = 0;

//Current pin number initial interrupt
int pin_num = 0;

//Queue handler
static QueueHandle_t switch_queue;

//Optionally it can require change name switch_gpio_num->gpio_num if semaphore will require acces to all interrupt if not - no change (optionaly handler line above)
void IRAM_ATTR switch_isr_handler(void *arg)
{
    int switch_gpio_num = (int)arg;
    xQueueSendFromISR(switch_queue, &switch_gpio_num, NULL);
}


void init_gpio_sw()
{
    //Set GPIO to switches
    gpio_pad_select_gpio(SW1);
    gpio_pad_select_gpio(SW2);
    gpio_pad_select_gpio(SW3);
    gpio_pad_select_gpio(SWJ);

    //Set GPIO as input
    gpio_set_direction(SW1, GPIO_MODE_INPUT);
    gpio_set_direction(SW2, GPIO_MODE_INPUT);
    gpio_set_direction(SW3, GPIO_MODE_INPUT);
    gpio_set_direction(SWJ, GPIO_MODE_INPUT);

    //Set interruption trigger
    gpio_set_intr_type(SW1, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(SW2, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(SW3, GPIO_INTR_NEGEDGE); //nededge works on pressing, posedge works on releasing - 0 is default on test pin
    gpio_set_intr_type(SWJ, GPIO_INTR_NEGEDGE);
}

void install_gpio_sw()
{
    //Install ISR driver
    gpio_install_isr_service(0); 
    
    //Attach ISR to each switch
    gpio_isr_handler_add(SW1, switch_isr_handler, (void*)SW1);
    gpio_isr_handler_add(SW2, switch_isr_handler, (void*)SW2);
    gpio_isr_handler_add(SW3, switch_isr_handler, (void*)SW3);
    gpio_isr_handler_add(SWJ, switch_isr_handler, (void*)SWJ);
}


void switch_task()
{    
    int counter = 0;
    
    while(1)
    {    
        //test task - return number of GPIO which was pressed
        if (xQueueReceive(switch_queue, &pin_num, portMAX_DELAY) )
        {
            //work but with i/o pins - switch is on input only - print errors but work
        //     if(!gpio_get_level(pin_num))
        //     {
        //         gpio_hold_en(pin_num);
        //         printf("GPIO[%d] intr, val: %d, pressed: %d\n", pin_num, gpio_get_level(pin_num), counter++);
        //     }
            
        //  }
        //  vTaskDelay(100 / portTICK_RATE_MS);
        //  gpio_hold_dis(pin_num);

        if(!gpio_get_level(pin_num))
            {                
                //printf("GPIO[%d] intr, val: %d, pressed: %d\n", pin_num, gpio_get_level(pin_num), counter++);
                vTaskDelay(40 / portTICK_RATE_MS);
                pin_num = 0;
            }
            
         }
    }
}

void set_switch_task()
{
    switch_queue = xQueueCreate(10, sizeof(int));
    xTaskCreate(switch_task, "switch_task", 2048, NULL , 10, NULL);
}