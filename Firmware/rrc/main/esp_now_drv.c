#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_now_drv.h"
//#include "freertos/semphr.h"

#define TAG "ESP_NOW"

extern uint32_t adc_read;


// static QueueHandle_t joystick_queue;

// void IRAM_ATTR joy_isr_handler(void *arg)
// {
//     int event = (int)arg;
//     xQueueSendFromISR(joystick_queue, &event, NULL);
// }

uint8_t sparkFun[6] = {0x30, 0xae, 0xa4, 0x2c, 0x9f, 0xac}; //Hardcode peer mac address - current is mac of SparkfunThing

char send_buffer[250];

//receive mac addres
char *mac_to_str(char *buffer, uint8_t *mac)
{
    sprintf(buffer, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return buffer;
}

//where sent and succes/failed - not necessery to transmite data between esp
void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    char buffer[13];

    switch (status)
    {
        case ESP_NOW_SEND_SUCCESS:
            ESP_LOGI(TAG, "message sent to %s", mac_to_str(buffer, (uint8_t *) mac_addr));
            break;
        
        case ESP_NOW_SEND_FAIL:
            ESP_LOGE(TAG, "message sent to %s failed", mac_to_str(buffer, (uint8_t *) mac_addr));
            break;

        default:
            break;
    }
}

void on_receive(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    char buffer[13];
    ESP_LOGI(TAG, "message receive from %s", mac_to_str(buffer, (uint8_t *) mac_addr));
    printf("message: %d  %s\n", data_len, data);
}

void init_esp_now()
{
    //receive mac addres
    uint8_t my_mac[6];
    esp_efuse_mac_get_default(my_mac);
    char my_mac_str[13];
    ESP_LOGI(TAG, " My mac %s", mac_to_str(my_mac_str, my_mac));
    //---------------------------------------------------------

    //init wifi - required set up before start esp-now
    nvs_flash_init();
    tcpip_adapter_init();
    //it return error so it can be with errorcheck but it is not necessary
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    //Below is not mandatory?

    // esp_event_loop_delete_default();
    // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // esp_wifi_init(&cfg);
    // esp_wifi_set_storage(WIFI_STORAGE_RAM);
    // esp_wifi_set_mode(WIFI_MODE_STA);
    // esp_wifi_start();

    //Optionaly add below into send_now_task and probably not uncomment Free resources
    esp_now_init();
    //infos
    esp_now_register_send_cb(on_sent);
    esp_now_register_recv_cb(on_receive);
    
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    memset(peer.peer_addr, sparkFun, 6);

    esp_now_add_peer(&peer);
}

void send_now_task(int16_t param1, int16_t value1, int16_t param2, int16_t value2)
{
    int queue_buffer = 0;
    
    while(1)
    {      
        //clear buffer after every sent?
        // use defined interrupt in existing function to trigger and send data
        for (int i = 0; i < sizeof(param1); i++)
            {
                send_buffer[i] = (char*)param1;
            }
        
        esp_now_send(sparkFun, (uint8_t*) send_buffer, strlen(send_buffer));

        for (int i = 0; i < sizeof(value1); i++)
            {
                send_buffer[i] = (char*)value1;
            }

        esp_now_send(sparkFun, (uint8_t*) send_buffer, strlen(send_buffer));

        for (int i = 0; i < sizeof(param2); i++)
            {
                send_buffer[i] = (char*)param2;
            }
        
        esp_now_send(sparkFun, (uint8_t*) send_buffer, strlen(send_buffer));

        for (int i = 0; i < sizeof(value2); i++)
            {
                send_buffer[i] = (char*)value2;
            }

        esp_now_send(sparkFun, (uint8_t*) send_buffer, strlen(send_buffer));

        vTaskDelay(1000 / portTICK_RATE_MS);  
        // if (adc_read != adc_read_last)
        // {
        //     for (int i = 0; i < sizeof(adc_read); i++)
        //     {
        //         send_buffer[i] = (char*)adc_read;
        //     }

        //     esp_now_send(sparkFun, (uint8_t*) send_buffer, strlen(send_buffer));
        //     printf("send: %s", send_buffer);
        //     adc_read_last = adc_read;
        //     vTaskDelay(3000 / portTICK_RATE_MS);
            
        // }
        // vTaskDelay(1000 / portTICK_RATE_MS);
        //below was comment---------------------- probably handler from joistick queue require corrections
        // if (xQueueReceive(joystick_queue, &queue_buffer, portMAX_DELAY))
        // {
        //     for (int i = 0; i < sizeof(adc_read); i++)
        //     {
        //         send_buffer[i] = (char*)adc_read;
        //     }

        //     esp_now_send(sparkFun, (uint8_t*) send_buffer, strlen(send_buffer));
        //     printf("send: %s", send_buffer);
        //     vTaskDelay(pdMS_TO_TICKS(1000)); //or add interrupt after change paremeters of joystick or filter parts

        //Free resources
        // esp_now_deinit();
        // esp_wifi_stop();
        //}
        
    }
}

//Does send_now_task is necessary? - it can be called by triggered interrupt from enc/switch/joystick
// void set_send_now_task()
// {
//     xTaskCreate(send_now_task, "send_now_task", 2048, NULL, 10, NULL);
// }