#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "mpu6050_acc.h"
#include "math.h"
#include "mpu6050_read_angle_acc.h"

// extern "C" {
// 	void mpu6050_init(void);
// }

// #define SAMPLES 256
static float RAD_TO_DEG = 57.2957795; 
static int16_t accel[3];
static int16_t Acx = 0;
static int16_t Acy = 0;
static int16_t Acz = 0; 
static int xAng = 0;
static int yAng = 0;
static int zAng = 0;
static double y = 0;
static int16_t gyro[3];
static float gyro_y = 0;
static float minVal = -16384;//-16384;//-16384;
static float maxVal = 16383;//32767;//65535;//16383;
static float minOut = -90;
static float maxOut = 90;

void mpu6050_acc_read(float *meas)
{
    read_raw(accel, gyro);

    if(Acx < 0)
    {
        Acx = (~(Acx<<1)>>1) + 0b1;
    }
    if(Acy < 0)
    {
        Acy = (~(Acy<<1)>>1) + 0b1;
    }
    if(Acz < 0)
    {
        Acz = (~(Acz<<1)>>1) + 0b1;
    }
    
    yAng = map_mpu(accel[1], minVal, maxVal, minOut, maxOut);
    zAng = map_mpu(accel[2], minVal, maxVal, minOut, maxOut);// - 90;
    xAng = map_mpu(accel[0], minVal, maxVal, minOut, maxOut);

    //z = RAD_TO_DEG * (atan2(-y, -x) + M_PI);
    y = RAD_TO_DEG * (atan2(-zAng, -xAng) + M_PI) - 90;
    gyro_y = gyro[1]/131.0f;

    if(-90 >= y || (280 >= y && 180 <= y ))
    {
        y = -90;
    }
    else if(90 <= y)
    {
        y = 90;
    }

    meas[0] = (float)y;
    meas[1] = gyro_y;
}
