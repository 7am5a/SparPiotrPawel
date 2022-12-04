/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2c_simple_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <string.h>
#include "mpu6050.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void app_main(void)
{
    mpu6050_acceleration_t accel = {0};
    mpu6050_rotation_t gyro = {0};
    float temp;
    esp_err_t ret = ESP_OK;

    uint8_t gyro_scale = 0;
    uint8_t accel_scale = 0;
    float gyro_res = 0;
    float accel_res = 0;

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    mpu6050_init();

    ret = mpu6050_test_connection();

     if(ret == true)
    {
        ESP_LOGI(TAG,"Connection test OK");
    }
    else
    {
        ESP_LOGE(TAG,"MPU6050 odpierdala kosiarę");
    }

    gyro_scale = mpu6050_get_full_scale_gyro_range();
    accel_scale = mpu6050_get_full_scale_accel_range();
    printf("gyro range: %i\naccel range: %i\n", gyro_scale, accel_scale);

    gyro_res = mpu6050_get_gyro_res(gyro_scale);
    accel_res = mpu6050_get_accel_res(accel_scale);
    printf("gyro resolution: %f\naccel resolution: %f\n", gyro_res, accel_res);

    while(1)
    {

        temp = (((float)mpu6050_get_temperature())/340.0f) + 36.53f;

        mpu6050_get_motion(&accel, &gyro);

        printf("temp: %f\n", temp);
        printf("x: %i\n", accel.accel_x);
        printf("y: %i\n", accel.accel_y);
        printf("z: %i\n", accel.accel_z);

        printf("x: %i\n", gyro.gyro_x);
        printf("y: %i\n", gyro.gyro_y);
        printf("z: %i\n", gyro.gyro_z);
        // ret = mpu6050_read_raw(accel, gyro, &temp);
        // if(ret != ESP_OK)
        // {
        //     ESP_LOGE(TAG,"Read raw error");
        // }
        // else
        // {
        //     printf("accel1: %d, accel2: %d, accel3: %d\n\r"
        //         "gyro1: %d, gyro2: %d, gyro3: %d\n\r"
        //         "temp: %d\n\n",
        //         accel[0], accel[1], accel[2],
        //         gyro[0], gyro[1], gyro[2],
        //         temp);
        // }
        vTaskDelay(1000/ portTICK_RATE_MS);
    }
}