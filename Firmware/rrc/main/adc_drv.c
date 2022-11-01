#include <stdio.h>
#include "stdlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_setup.h"
#include "esp_adc_cal.h"
#include "adc_drv.h"

int adc_pin_num = 0;

static esp_adc_cal_characteristics_t *adc_characteristics;

//Queue handler
static QueueHandle_t adc_queue;

//Read value from GPIO
uint32_t adc_read = 0;
uint32_t voltage = 0;

void IRAM_ATTR adc_isr_handler(void *arg)
{
    int adc_pin_num = (int)arg;
    xQueueSendFromISR(adc_queue, &adc_pin_num, NULL);
}

void print_cal_val_type(esp_adc_cal_value_t val_type)
{    
    //Check ADC calibration type
    printf("Checking ADC calibration type... \n");
    adc_characteristics = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_12Bit, ADC_VREF_VALUE, adc_characteristics);

    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

//ADC
void init_gpio_adc()
{   
    //To check atten and vref in adc_drv.h
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(VBAT_GPIO35, ADC_ATTEN_11db);
    adc1_config_channel_atten(JOY_X_GPIO34, ADC_ATTEN_11db);
    adc1_config_channel_atten(JOY_Y_GPIO39, ADC_ATTEN_11db);

    //not sure if it is proper set------------------------------
    //Install ISR driver
    gpio_install_isr_service(0);
    
    //Attach ISR to each switch
    gpio_isr_handler_add(JOY_X, adc_isr_handler, (void*)JOY_X);
    gpio_isr_handler_add(JOY_Y, adc_isr_handler, (void*)JOY_Y);
    gpio_isr_handler_add(VBAT, adc_isr_handler, (void*)VBAT);
    //----------------------------------------------------------
}

//Temporary set adc read from only one gpio - use enum or if or read all time from every adc's
void read_VBAT_adc_task()
{
    while (1)
    {
        uint32_t adc_read = 0;
        
        for (int i = 0; i < SAMPLES; i++)
        {
            adc_read = adc_read + adc1_get_raw((adc1_channel_t)VBAT_GPIO35);
        }
        adc_read = adc_read / SAMPLES;
        
        //Convert raw value to mV
        voltage = esp_adc_cal_raw_to_voltage(adc_read, adc_characteristics);
       
        vTaskDelay(5000 / portTICK_RATE_MS);        
    }
}

void read_adc_task()
{
    int x_adc_read = 0;
    int y_adc_read = 0;
    
    while(1)
    {        
        //Multisampling for x and y axis
        for (int i = 0; i < SAMPLES; i++)
        {
            x_adc_read = x_adc_read + adc1_get_raw((adc1_channel_t)JOY_X_GPIO34);
        }
        x_adc_read = x_adc_read / SAMPLES;

        for (int i = 0; i < SAMPLES; i++)
        {
            y_adc_read = y_adc_read + adc1_get_raw((adc1_channel_t)JOY_Y_GPIO39);
        }
        y_adc_read = y_adc_read / SAMPLES;

        //Convert raw value to duty cycle of motors PWM
        //X axis
        int voltage_x = esp_adc_cal_raw_to_voltage(x_adc_read, adc_characteristics);
        //Right
        if(voltage_x < 2200)
        {
            voltage_x = voltage_x * 0.2997; //minus min value
            voltage_x =  (100 - (voltage_x - 113) / 5.65); //no minus to change direction
        }
        //Left
        else if(voltage_x > 2550)
        {
            voltage_x = 3130 - voltage_x;
            voltage_x = -( 100 - voltage_x / 5.65); //minus to change direction
        }
        else
        {
            voltage_x = 0;
        }           
        
        if(voltage_x != 0 && (voltage_x > 10 || voltage_x < -10))
        {
            printf("X axis Value: %d %% \n", voltage_x);
        }
        
        //Y axis    
        int voltage_y = esp_adc_cal_raw_to_voltage(y_adc_read, adc_characteristics);
        //Down
        if(voltage_y < 2230)
        {
            voltage_y = voltage_y * 0.3099;
            voltage_y = - (100 - (voltage_y -165) / 5.3);
        }
        //Up
        else if(voltage_y > 2580)
        {
            voltage_y = 3123 - voltage_y;
            voltage_y = 100 - voltage_y / 5.3;
        }
        else
        {
            voltage_y = 0;
        }
        
        if(voltage_y != 0 && (voltage_y > 10 || voltage_y < -10))
        {
            printf("Y axis Value: %d %% \n", voltage_y);
        }
        //delay can be change to increase smooth of control
        vTaskDelay(pdMS_TO_TICKS(50));               
    }
}

void set_read_adc_task()
{
    adc_queue = xQueueCreate(10, sizeof(int));
    xTaskCreate(read_VBAT_adc_task, "read_VBAT_adc_task", 2048, NULL , 10, NULL);
    xTaskCreate(read_adc_task, "read_adc_task", 2048, NULL , 10, NULL);
}