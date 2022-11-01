#include <stdio.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_hello.h"
#include "adc_drv.h"

void esp_hello_task()
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %d chip with %d CPU core(s), WiFi%s%s,\n",
            CONFIG_IDF_TARGET_ESP32,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    uint8_t chipID[6];
    esp_efuse_mac_get_default(chipID);
    printf("mac / chip ID: ");
    for (int i = 0; i <= 5; i++)
    {
        printf("%02x ", chipID[i]);
    }
    printf("\n\n");    

    print_cal_val_type();
}