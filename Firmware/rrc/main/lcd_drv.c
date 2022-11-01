/*
    Drivers inspired DavidAntliff project and libraries 
    https://github.com/DavidAntliff/esp32-i2c-lcd1602-example
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_setup.h"
#include "lcd_drv.h"
#include "ascii_table.h"
//optional
#include "unistd.h"

static int i2c_master_port = I2C_NUM_0;

void init_lcd_i2c()
{
    gpio_pad_select_gpio(LCD_RST);
	gpio_set_direction(LCD_RST, GPIO_MODE_OUTPUT);
	gpio_set_level(LCD_RST, 1);

    //vTaskDelay(4000 / portTICK_RATE_MS);
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LCD_SDA,
        .scl_io_num = LCD_SCL,
        .scl_pullup_en = 0,             //disable internal SCL pullup - as above 
        .sda_pullup_en = 0,             //disable internal SDA pullup - setup hardware one
        .master.clk_speed = MASTER_CLK
    };

    i2c_param_config(i2c_master_port, &i2c_config);
    i2c_driver_install(i2c_master_port, i2c_config.mode, MASTER_RX_BUF, MASTER_TX_BUF, true);

    // i2c_port_t i2c_master_num = I2C_NUM_0; - set up global in this file
     //uint8_t lcd_addres = LCD_ADDRES; //- not required

    //Optionaly there can be required Set up LCD -> more info in lcd example in code folder
    //vTaskDelay(40 / portTICK_RATE_MS);
    usleep(50000);
    // May it will be essential to proper initialize LCD
    lcd_write_data("00000011");
    usleep(5000);
    //vTaskDelay(4 / portTICK_RATE_MS);
    lcd_write_data("00000011");
    usleep(200);
    //vTaskDelay(100 /(1000* portTICK_RATE_MS)); // or vTaskDelay(100/(1000*portTICK_RATE_MS));
    lcd_write_data("00000011");
    //vTaskDelay(100 / (1000 * portTICK_RATE_MS)); //optionaly 200us or TICK_TO_MS or sth similar function to manage time
    usleep(1000);

    lcd_write_command(DATA_LENGTH_8_BIT);
    lcd_write_command(DISPLAY_ON);
    lcd_write_command(CLEAR_DISPLAY);
    lcd_write_command(CURSOR_ON);
    lcd_write_command(SHIFT_CURSOR_LEFT);
}

void write_byte_to_lcd(uint8_t menu_data)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, ((LCD_ADDRES << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd_handle, menu_data, true);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(i2c_master_port, cmd_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd_handle);
}

void read_byte_from_lcd(uint8_t *lcd_data)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, ((LCD_ADDRES << 1) | I2C_MASTER_READ), true);
    i2c_master_read_byte(cmd_handle, lcd_data, true);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(i2c_master_port, cmd_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd_handle);
}


void lcd_define_symbols(uint8_t char_address, const char *char_name)
{
    write_byte_to_lcd((CGRAM_ADDRES | (char_address << 3)) | RS_COMMAND);
    
    for(int i = 0; i < 8; i++)
    {
        write_byte_to_lcd((char_name[i] & 0xf0) | RS_DATA);//does 0xf0/f0 is required to 8 bit data, it look like 4 bit or require more corrections from example
    }

    write_byte_to_lcd(RETURN_HOME);    
}
//Optionaly there can be 1ms (60us) delay to proper set character on LCD
void lcd_write_command(uint8_t type_command)
{    
    write_byte_to_lcd((type_command & 0xf0) | RS_COMMAND);
}

//
//Require corrections!! - causes errors/reset of esp - or init function trying write to lcd
//

void lcd_write_data(const char *type_data)
{
    //Does condition is necessary?
     if(type_data)
     {
        //or strlen();         or while(*type_data)
        for (int i = 0; i < sizeof(type_data); i++)
        { 
            write_byte_to_lcd((type_data[i] & 0xf0) | RS_DATA);
        }
     }    
 }

void lcd_set_cursor(uint8_t x, uint8_t y)
{
    switch (y)
    {
        case 0:
            lcd_write_command(LCD_LINE1 + x);
            break;

        case 1:
            lcd_write_command(LCD_LINE2 + x);
            break;    
    }
}