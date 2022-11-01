#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "unistd.h"
#include "lcd_st7032.h"
#include "esp_err.h"

#define LCD_ADDR			0x3e

#define CONTROL_BYTE			0x00 // followed by command bytes
#define CONTROL_BYTE_CO			0x80 // followed by 1 command byte
#define CONTROL_BYTE_RS			0x40 // after last control byte, followed by DDRAM data byte(s)

#define CLEAR_DISPLAY			0x01 // Clear display

#define RETURN_HOME			0x02 // Cursor home to 00H

#define FUNCTION_SET			0x20 // DL: interface data is 8/4 bits, N: number of line is 2/1 DH: double height font, IS: instr
#define FUNCTION_SET_IS			0x01 // IS: instruction table select
#define FUNCTION_SET_DH			0x04 // DH: double height font
#define FUNCTION_SET_N			0x08 // N: number of line is 2/1
#define FUNCTION_SET_DL			0x20 // DL: interface data is 8/4 bits //change from 0x10

#define INTERNAL_OSC_FREQ		0x10 // BS=1:1/4 bias, BS=0:1/5 bias, F2~0: adjust internal OSC frequency for FR frequency.
#define INTERNAL_OSC_FREQ_F0		0x01 // F2~0: adjust internal OSC frequency for FR frequency.
#define INTERNAL_OSC_FREQ_F1		0x02 // F2~0: adjust internal OSC frequency for FR frequency.
#define INTERNAL_OSC_FREQ_F2		0x04 // F2~0: adjust internal OSC frequency for FR frequency.
#define INTERNAL_OSC_FREQ_BS		0x08 // BS=1:1/4 bias (BS=0:1/5 bias)

#define POWER_ICON_BOST_CONTR		0x50 // Ion: ICON display on/off, Bon: set booster circuit on/off, C5,C4: Contrast set
#define POWER_ICON_BOST_CONTR_Bon	0x04 // Ion: ICON display on/off
#define POWER_ICON_BOST_CONTR_Ion	0x08 // Bon: set booster circuit on/off

#define FOLLOWER_CONTROL		0x60 // Fon: set follower circuit on/off, Rab2~0: select follower amplified ratio.
#define FOLLOWER_CONTROL_Rab0		0x01 // Rab2~0: select follower amplified ratio
#define FOLLOWER_CONTROL_Rab1		0x02 // Rab2~0: select follower amplified ratio
#define FOLLOWER_CONTROL_Rab2		0x04 // Rab2~0: select follower amplified ratio
#define FOLLOWER_CONTROL_Fon		0x08 // Fon: set follower circuit on/off

#define CONTRAST_SET			0x70 // C0-C3: Contrast set 

#define DISPLAY_ON_OFF			0x08 // display on, cursor on, cursor position on
#define DISPLAY_ON_OFF_B		0x01 // cursor position on
#define DISPLAY_ON_OFF_C		0x02 // cursor on
#define DISPLAY_ON_OFF_D		0x04 // display on

#define SET_DDRAM_ADDRESS		0x80 // Set DDRAM address in address counter
#define SET_CGRAM_ADDRESS		0x40 //Set CGRAM address in address counter

#define LINE_1_ADDR			0x00
#define LINE_2_ADDR			0x40

#define WRITE_DELAY_US			30 // see data sheet
#define HOME_CLEAR_DELAY_US		1200 // see data sheet

static uint8_t contrast = 0x04;
static uint8_t display;

static void writeInstruction(uint8_t cmd)
{    
	uint8_t data_t[2];
	
	data_t[0] = CONTROL_BYTE;
	data_t[1] = cmd;
	
	ESP_LOGI("cmd: ","%x ", cmd);

	esp_err_t esp_err_test = i2c_master_write_to_device(I2C_NUM_0, LCD_ADDR, data_t, 2, 1000/ portTICK_RATE_MS);

	if(esp_err_test != 0)
	{
		ESP_LOGI("i2c command","Error %d", esp_err_test);
	}

}

static void display_on_off()
{
	writeInstruction(DISPLAY_ON_OFF | display);
}

void lcd_st7032_init()
{
    //reset LCD
    gpio_pad_select_gpio(21);
	gpio_set_direction(21, GPIO_MODE_OUTPUT);
	gpio_set_level(21, 1);
    usleep(2000);
    gpio_set_level(21, 0);
    usleep(2000);
    gpio_set_level(21, 1);

	int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 19,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    usleep(50);

	writeInstruction(FUNCTION_SET
		// | FUNCTION_SET_DL
		| FUNCTION_SET_N);
		// | FUNCTION_SET_IS);

	writeInstruction(FUNCTION_SET
		// | FUNCTION_SET_DL
		| FUNCTION_SET_N
		| FUNCTION_SET_IS);

	// writeInstruction(INTERNAL_OSC_FREQ
	// 	| INTERNAL_OSC_FREQ_BS
	// 	| INTERNAL_OSC_FREQ_F2);

	writeInstruction(0x14);

	// writeInstruction(POWER_ICON_BOST_CONTR
	// 	| POWER_ICON_BOST_CONTR_Ion
	// 	| POWER_ICON_BOST_CONTR_Bon);
	
	lcd_st7032_set_contrast(contrast);

	// writeInstruction(FOLLOWER_CONTROL
	// 	| FOLLOWER_CONTROL_Fon
	// 	| FOLLOWER_CONTROL_Rab2);

	writeInstruction(0x6f);
	
	display = 0x7;
	display_on_off();
    usleep(300);
}

void lcd_st7032_set_contrast(uint8_t value)
{
	if (value > ST7032_CONTRAST_MAX)
		value = ST7032_CONTRAST_MIN;
	else if (value < ST7032_CONTRAST_MIN)
		value = ST7032_CONTRAST_MAX;
	writeInstruction(CONTRAST_SET | (value & 0x0f));
	writeInstruction((value >> 4) | POWER_ICON_BOST_CONTR | POWER_ICON_BOST_CONTR_Bon);
	contrast = value;
}

void lcd_st7032_display_on()
{
	display |= DISPLAY_ON_OFF_D;
	display_on_off();
}

void lcd_st7032_display_off()
{
	display &= ~DISPLAY_ON_OFF_D;
	display_on_off();
}

void lcd_st7032_cursor_on()
{
	display |= DISPLAY_ON_OFF_C;
	display_on_off();
}

void lcd_st7032_cursor_off()
{
	display &= ~DISPLAY_ON_OFF_C;
	display_on_off();
}

void lcd_st7032_blink_on()
{
	display |= DISPLAY_ON_OFF_B;
	display_on_off();
}

void lcd_st7032_blink_off()
{
	display &= ~DISPLAY_ON_OFF_B;
	display_on_off();
}

void lcd_st7032_clear()
{
	writeInstruction(CLEAR_DISPLAY);
}

void lcd_st7032_home()
{
	writeInstruction(RETURN_HOME);
}

void lcd_st7032_write(uint8_t data)
{
	uint8_t data_t[2];
	
	data_t[0] = CONTROL_BYTE_RS;
	data_t[1] = data;

	ESP_LOGI("data: ","%x ", data);

    esp_err_t err2 = i2c_master_write_to_device(I2C_NUM_0, LCD_ADDR, data_t, 2, 1000 / portTICK_RATE_MS);

	if(err2 != 0)
	{
		ESP_LOGI("i2c data","Error %d", err2);
	}

}

void lcd_st7032_print(char* data)
{
	while (*data)
	{
		lcd_st7032_write(*data);
		data++;
	}
}

void lcd_st7032_set_cursor(uint8_t line, uint8_t pos)
{
	uint8_t p;

	if (pos > 15)
		pos = 15;
	if (line == 0)
		p = LINE_1_ADDR + pos;
	else
		p = LINE_2_ADDR + pos;

	writeInstruction(SET_DDRAM_ADDRESS | p);
}

void lcd_st7032_create_char(uint8_t location, uint8_t* charmap)
{
	// change instruction set
	writeInstruction(FUNCTION_SET
		| FUNCTION_SET_DL
		| FUNCTION_SET_N);

	writeInstruction(SET_CGRAM_ADDRESS | ((location & 0x7) << 3));
	for (int i = 0; i < 8; i++)
	{
		lcd_st7032_write(charmap[i]);
	}

	// change back to previous instruction set
	writeInstruction(FUNCTION_SET
		| FUNCTION_SET_DL
		| FUNCTION_SET_N
		| FUNCTION_SET_IS);
}

/*
In main function - requires:

    lcd_st7032_init();

	// lcd_st7032_create_char(0x01, (uint8_t*)playChar);
	// lcd_st7032_create_char(0x02, (uint8_t*)stopChar);

	lcd_st7032_set_cursor(0, 0);
	lcd_st7032_print("Hi!");
	// lcd_st7032_set_cursor(1, 0);
	// lcd_st7032_print("Play Stop: ");

*/

void lcd_task()
{
    while (1)
    {
        lcd_st7032_set_cursor(0, 0);
        lcd_st7032_print("Hi!");
        vTaskDelay(1000);
    }
    
}

void set_lcd_task()
{
    xTaskCreate(lcd_task, "lcd_task", 2048, NULL, 10, NULL);
}

//all library need to be rewritten (current file or last one - just to make code clear) and required to add menu display functions
// https://github.com/bchodorowski/lcd_st7032 - inspiration