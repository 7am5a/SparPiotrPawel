#ifndef ESP_SETUP
#define ESP_SETUP

//GPIO definition

//Info LED
#define INFO_LED 1

//Switches
#define SW1 16
#define SW2 17
#define SW3 5 
#define SWJ 36 

//ENC channels
#define ENC1_A 13
#define ENC1_B 12
#define ENC2_A 27
#define ENC2_B 14
#define ENC3_A 26
#define ENC3_B 25

//LCD control
#define LCD_SDA 19
#define LCD_SCL 22
#define LCD_RST 21
#define LCD_PWM 23

//Battery voltage measurement - ADC
#define VBAT 35
#define VBAT_GPIO35 ADC_CHANNEL_7

//Joystick ADC
#define JOY_X 34
#define JOY_X_GPIO34 ADC_CHANNEL_6
#define JOY_Y 39
#define JOY_Y_GPIO39 ADC_CHANNEL_3

#endif