/*
 * Display.c
 *
 *  Created on: 14.08.2017
 *      Author: darek
 */
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"
#include "motor_control.h"
#include "mpu6050_read_angle.h"
#include "esp_now_drv.h"
#include "esp_wifi.h"
#include "esp_now.h"

#define PIN_SDA 21
#define PIN_CLK 22

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
float pitch = 0u;
bool calibrated = 0u;
float send_pitch;


MPU6050 mpu = MPU6050();

void task_initI2C(void *ignore) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	init_esp_now();
	vTaskDelete(NULL);
}

float mpu6050_read_angle()
{
	mpuIntStatus = mpu.getIntStatus();
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	    // reset so we can continue cleanly
	    mpu.resetFIFO();
	// otherwise, check for DMP data ready interrupt frequently)
	} else if (mpuIntStatus & 0x02) {
	    // wait for correct available data length, should be a VERY short wait
	    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	    // read a packet from FIFO
	    mpu.getFIFOBytes(fifoBuffer, packetSize);
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		//printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
		//printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
		//printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
		pitch = ypr[1] * 180/M_PI;
		
		/*uncomment to send current angle via ESP_NOW*/
		// send_pitch = pitch;
		// printf("send_pitch: %0.2f\n",send_pitch);
		// esp_now_send(NULL, (uint8_t *)&send_pitch, sizeof(send_pitch));  
	}

	return pitch;
}

void task_display(void*){
	mpu.initialize();
	mpu.dmpInitialize();

	// This need to be setup individually
	// mpu.setXGyroOffset(220);
	// mpu.setYGyroOffset(76);
	// mpu.setZGyroOffset(-85);
	// mpu.setZAccelOffset(1788);
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

	mpu.setDMPEnabled(true);
	calibrated = 1;

	while(1){
	
		vTaskDelay(50/portTICK_PERIOD_MS);
	    //Best result is to match with DMP refresh rate
	    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
	    // Now its 0x13, which means DMP is refreshed with 10Hz rat]e
		// vTaskDelay(5/portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
	}