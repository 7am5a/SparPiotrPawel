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

uint16_t accel[3] = {0, 0, 0};
uint16_t gyro[3] = {0, 0, 0};
uint16_t temp;

/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

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
    uint8_t data[2];
    uint16_t accel[3] = {0, 0, 0};
    uint16_t gyro[3] = {0, 0, 0};
    uint16_t temp;
    esp_err_t ret = ESP_OK;

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU6050 WHO_AM_I register, on power up the register should have the value 0x68 */
    ESP_ERROR_CHECK(mpu6050_register_read(MPU6050_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    while(1)
    {
        ret = mpu6050_read_raw(accel, gyro, &temp);
        if(ret != ESP_OK)
        {
            ESP_LOGE(TAG,"Read raw error");
        }
        else
        {
            printf("accel1: %d, accel2: %d, accel3: %d\n\r"
                "gyro1: %d, gyro2: %d, gyro3: %d\n\r"
                "temp: %d\n\n",
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2],
                temp);
        }
        vTaskDelay(1000/ portTICK_RATE_MS);
    }
    
    /* Demonstrate writing by reseting the MPU6050 */
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG_ADDR, 1 << MPU6050_RESET_BIT));

    // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    // ESP_LOGI(TAG, "I2C unitialized successfully");
}


// esp_err_t mpu6050_reset()
// {
//     esp_err_t ret = ESP_OK;
//     ret = mpu6050_register_write_byte(0x6B, 0x00);
//     if(ret != ESP_OK)
//     {

//     }
// }

esp_err_t mpu6050_read_raw (uint16_t accel[3], uint16_t gyro[3], uint16_t *temp)
{
    esp_err_t ret = ESP_OK;
    uint8_t buffer[6];
    uint8_t val = 0x3B;

    ret = mpu6050_register_write_byte(MPU6050_SENSOR_ADDR, val);
    if(ret != ESP_OK)
    {   
        ESP_LOGE(TAG,"MPU6050 write byte error %x", val);
        return ret;
    }
    ret = mpu6050_register_read(MPU6050_SENSOR_ADDR, buffer, 6);
    if(ret != ESP_OK)
    {   
        ESP_LOGE(TAG,"MPU6050 read byte error %x", val);
        return ret;
    }

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x43;

    ret = mpu6050_register_write_byte(MPU6050_SENSOR_ADDR, val);
    if(ret != ESP_OK)
    {   
        ESP_LOGE(TAG,"MPU6050 write byte error %x", val);
        return ret;
    }
    ret = mpu6050_register_read(MPU6050_SENSOR_ADDR, buffer, 6);
    if(ret != ESP_OK)
    {   
        ESP_LOGE(TAG,"MPU6050 read byte error %x", val);
        return ret;
    }

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    val = 0x41;

    ret = mpu6050_register_write_byte(MPU6050_SENSOR_ADDR, val);
    if(ret != ESP_OK)
    {   
        ESP_LOGE(TAG,"MPU6050 write byte error %x", val);
        return ret;
    }
    ret = mpu6050_register_read(MPU6050_SENSOR_ADDR, buffer, 6);
    if(ret != ESP_OK)
    {   
        ESP_LOGE(TAG,"MPU6050 read byte error %x", val);
        return ret;
    }

    *temp = buffer[0] << 8 | buffer[1];

    return ret;
}