#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_setup.h"
#include "enc_drv.h"
#include "esp_log.h"

static QueueHandle_t enc_queue;

//Currently triggered encoder
int Encoder = 0;
int encoder_value = 0;

//Set encoders counter limit
int low_limit = -2;
int high_limit = 2;

//Optionaly will be reduce as in switch_drv.c - requires check and changes
/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

void IRAM_ATTR enc_isr_handler(void *arg)
{
    int pcnt_unit = (int)arg;
    pcnt_evt_t evt;
    evt.unit = pcnt_unit;
    /* Save the PCNT event type that caused an interrupt
       to pass it to the main program */
    pcnt_get_event_status(pcnt_unit, &evt.status);
    xQueueSendFromISR(enc_queue, &evt, NULL);
}

//ENC
void init_gpio_enc()
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t enc1_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = ENC1_A,
        .ctrl_gpio_num = ENC1_B,
        .channel = PCNT_CHANNEL_1,
        .unit = PCNT_UNIT_1,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DEC,   // Hold counter state
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_KEEP, // Won't change counter mode
        .hctrl_mode = PCNT_MODE_REVERSE, // Won't change counter mode
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = high_limit,
        .counter_l_lim = low_limit,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&enc1_config);

    pcnt_config_t enc2_config = {
        .pulse_gpio_num = ENC2_A,
        .ctrl_gpio_num = ENC2_B,
        .channel = PCNT_CHANNEL_1,
        .unit = PCNT_UNIT_2,
        .pos_mode = PCNT_COUNT_INC,   
        .neg_mode = PCNT_COUNT_DEC,   
        .lctrl_mode = PCNT_MODE_KEEP, 
        .hctrl_mode = PCNT_MODE_REVERSE, 
        .counter_h_lim = high_limit,
        .counter_l_lim = low_limit,
    };
    pcnt_unit_config(&enc2_config);

    pcnt_config_t enc3_config = {
        .pulse_gpio_num = ENC3_A,
        .ctrl_gpio_num = ENC3_B,
        .channel = PCNT_CHANNEL_1,
        .unit = PCNT_UNIT_3,
        .pos_mode = PCNT_COUNT_INC,   
        .neg_mode = PCNT_COUNT_DEC,   
        .lctrl_mode = PCNT_MODE_KEEP, 
        .hctrl_mode = PCNT_MODE_REVERSE, 
        .counter_h_lim = high_limit,
        .counter_l_lim = low_limit,
    };
    pcnt_unit_config(&enc3_config);

    //unit is equal PCNT_UNIT_1(2,3) it just a int with value 1, 2 or 3 - it make it easier way to initialize in loop
    pcnt_isr_service_install(0); // propably instalation can be there
    for (int unit = 0; unit <= 3; unit++)
    {
        /* Configure and enable the input filter */
        //pcnt_set_filter_value(unit, 100);
        //pcnt_filter_enable(unit);
    
        /* Set threshold -1 and 1 values and enable events to watch */
        pcnt_set_event_value(unit, PCNT_EVT_THRES_1, 1);
        pcnt_event_enable(unit, PCNT_EVT_THRES_1);
        pcnt_set_event_value(unit, PCNT_EVT_THRES_0, -1);
        pcnt_event_enable(unit, PCNT_EVT_THRES_0);
        /* Enable events on zero, maximum and minimum limit values */
        //pcnt_event_enable(unit, PCNT_EVT_ZERO);
        pcnt_event_enable(unit, PCNT_EVT_H_LIM);
        pcnt_event_enable(unit, PCNT_EVT_L_LIM);

        /* Initialize PCNT's counter */
        pcnt_counter_pause(unit);
        pcnt_counter_clear(unit);

        /* Install interrupt service and add isr callback handler */
        //pcnt_isr_service_install(0);
        pcnt_isr_handler_add(unit, enc_isr_handler, (void *)unit);

        /* Everything is set up, now go to counting */
        pcnt_counter_resume(unit);
        }        
    }
    
void enc_task()
{
    int16_t count = 0;
    pcnt_evt_t evt; //change event per pin number if required after test on ready board
    //portBASE_TYPE res; //could be not qeruired after above changes - possibly delete or change declaration
    //Main body of that task will be changed 
    while(1)
    {
        /* Wait for the event information passed from PCNT's interrupt handler.
         * Once received, decode the event type and print it on the serial monitor.
         */
        //Optionaly wait tile could be changed like in switch
        //res = xQueueReceive(enc_queue, &evt, portMAX_DELAY);
        if (xQueueReceive(enc_queue, &evt, portMAX_DELAY)) {
            if(evt.unit == PCNT_UNIT_1)
            {
                pcnt_get_counter_value(PCNT_UNIT_1, &count);
                Encoder = PCNT_UNIT_1;
                if (count != 0)
                {
                    encoder_value = count;
                }
                
                
            }
            else if(evt.unit == PCNT_UNIT_2)
            {
                pcnt_get_counter_value(PCNT_UNIT_2, &count);
                Encoder = PCNT_UNIT_2;
                if (count != 0)
                {
                    encoder_value = count;
                }            
            }
            else if(evt.unit == PCNT_UNIT_3)
            {
                pcnt_get_counter_value(PCNT_UNIT_3, &count);
                Encoder = PCNT_UNIT_3;
                if (count != 0)
                {
                    encoder_value = count;
                }            
            }
            
            /*
            case to not print 0 -> current  look like:
            1
            0
            or 
            -1
             0
             what is ok if it will be used as a parameter to increase or decrease sth
            */
            //Debug function
            // if(count != 0)
            // {
            //    printf("Event PCNT unit[%d]; cnt: %d \n", evt.unit, count);
            // }
            vTaskDelay(20 / portTICK_RATE_MS);            
        }
    }
}

void set_enc_task()
{
    enc_queue = xQueueCreate(20, sizeof(pcnt_evt_t));
    xTaskCreate(enc_task, "enc_task", 2048, NULL , 10, NULL);
}