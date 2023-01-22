idf_component_register(SRCS 
                        "mpu6050_read_pitch.cpp" 
                        "I2Cdev.cpp" 
                        "MPU6050.cpp" 
                        "MPU6050_acc.c" 
                        "mpu6050_read_angle_acc.c" 
                        "state_feedback.cpp" 
                        "SampleFilter.cpp" 
                        "Encoder.cpp" 
                        "esp_now_drv.cpp"
                        "pid.cpp" 
                        "main.cpp" 
                        "motor_control.cpp"
                    INCLUDE_DIRS ".")

