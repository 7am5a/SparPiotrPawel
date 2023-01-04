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

#define TAG "ESP_NOW"

#define MAX_DATA_LENGTH 250

extern uint32_t adc_read;

TaskHandle_t esp_now_handle;

struct State_Based_data state_based_data;
struct PID_data pid_data;

//30 c6 f7 18 a0 d8 current chip
//58:bf:25:91:d1:e4 test
uint8_t sparkFun[6] = {0x30, 0xc6, 0xf7, 0x18, 0xa0, 0xd8}; //Hardcode peer mac address - current is mac of SparkfunThing
uint8_t wroom_robot[6] = {0x44, 0x17, 0x93, 0x7c, 0x3e, 0x7c};
uint8_t wroom_test[6] = {0x58, 0xbf, 0x25, 0x91, 0xd1, 0xe4};
char mac_buffer[13];

uint8_t *peer_mac = wroom_robot;

//max package of data
char send_buffer[MAX_DATA_LENGTH];

//find mac addres
char *mac_to_str(char *buffer, uint8_t *mac)
{
    sprintf(buffer, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return buffer;
}

//function to debug
void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    //buffer to mac addres
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

//function to debug
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

    //Optionaly add below into send_now_task and probably not uncomment Free resources
    esp_now_init();
    //infos
    //temporary comment - hard to debug with
    //esp_now_register_send_cb(on_sent);
    esp_now_register_recv_cb(on_receive);
    
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    memcpy(peer.peer_addr, peer_mac, 6);

    esp_now_add_peer(&peer);

}

void enc_state_based_send_now(float k1, float k2, float k3)
{
    state_based_data.K1 = k1;
    state_based_data.K2 = k2;
    state_based_data.K3 = k3;
    esp_now_send(NULL, (uint8_t *)&state_based_data, sizeof(state_based_data));
}

void enc_pid_send_now(float kp, float ki, float kd)//char param[3], int paramVal)
{        
    // //sprintf(send_buffer, "Hello from %s", mac_to_str(mac_buffer, (uint8_t *) sparkFun));
    // sprintf(send_buffer, "%s %d ", param, paramVal);
    // esp_now_send(NULL, (uint8_t *)send_buffer, strlen(send_buffer));   
    pid_data.Kp = kp;
    pid_data.Ki = ki;
    pid_data.Kd = kd;
    //sprintf(send_buffer, "Kp %d Ki %d Kd %d ", kp, ki, kd);
    esp_now_send(NULL, (uint8_t *)&pid_data, sizeof(pid_data)); 
}

void joy_send_now(int xVal, int yVal)
{
    sprintf(send_buffer, "X %d Y %d ", xVal, yVal);
    esp_now_send(NULL, (uint8_t *)send_buffer, strlen(send_buffer));
}

void reset_send_now(float k1, float k2, float k3, float kp, float ki, float kd)
{
    state_based_data.K1 = k1;    
    state_based_data.K2 = k2;    
    state_based_data.K3 = k1;    
    pid_data.Kp = kp;
    pid_data.Ki = ki;
    pid_data.Kd = kd;
    //sprintf(send_buffer, "Kp %d Ki %d Kd %d ", kp, ki, kd);
    esp_now_send(NULL, (uint8_t *)&state_based_data, sizeof(state_based_data));
    esp_now_send(NULL, (uint8_t *)&pid_data, sizeof(pid_data));
}