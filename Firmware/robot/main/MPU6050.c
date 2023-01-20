#include <stdio.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "MPU6050.h"
#include "esp_log.h"

#define IMU_ADDR 0x68
#define I2C_SCL_GPIO 22
#define I2C_SDA_GPIO 21

void reset()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    // i2c_write_blocking(_i2c, _i2c_address, buf, 2, false);
    i2c_master_write_to_device(I2C_NUM_0, IMU_ADDR, buf, 2, 1000/ portTICK_RATE_MS);
    return;
}

void mpu6050_init()
{
    int i2c_master_port = I2C_NUM_0;

        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_SDA_GPIO,
            .scl_io_num = I2C_SCL_GPIO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000,
        };

        i2c_param_config(i2c_master_port, &conf);
        i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    reset();

    return;
}


// These are the raw numbers from the chip, so will need tweaking to be really useful.
// Temperature is simple so use the datasheet calculation to get deg C.
// Note this is chip temperature.
// printf("Temp. = %f\n", (temp / 340.0) + 36.53);
void read_raw(int16_t accel[3], int16_t gyro[3])
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    // i2c_write_blocking(_i2c, _i2c_address, &val, 1, true); // true to keep master control of bus
    // i2c_read_blocking(_i2c, _i2c_address, buffer, 6, false);

    i2c_master_write_to_device(I2C_NUM_0, IMU_ADDR, &val, 1, 1000/ portTICK_RATE_MS);
    i2c_master_read_from_device(I2C_NUM_0, IMU_ADDR, buffer, 6, 1000/portTICK_RATE_MS);
    
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    // i2c_write_blocking(_i2c, _i2c_address, &val, 1, true);
    // i2c_read_blocking(_i2c, _i2c_address, buffer, 6, false);  // False - finished with bus

    i2c_master_write_to_device(I2C_NUM_0, IMU_ADDR, &val, 1, 1000/ portTICK_RATE_MS);
    i2c_master_read_from_device(I2C_NUM_0, IMU_ADDR, buffer, 6, 1000/portTICK_RATE_MS);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    return;
}

float map_mpu(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}