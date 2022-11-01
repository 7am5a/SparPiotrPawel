#include <stdio.h>
#include "lcd_buf_drv.h"
#include "lcd_drv.h"

//LCD buffer -> should improve sending data by one block by i2c instead every single character one by one
char lcd_buf[LCD_ROWS][LCD_COLS];

uint8_t lcd_buf_x;
uint8_t lcd_buf_y;

void buf_locate(uint8_t x, uint8_t y)
{
    lcd_buf_x = x;
    lcd_buf_y = y;
}

void buf_char(char c)
{
    if(lcd_buf_x < LCD_COLS && lcd_buf_y < LCD_ROWS)
    {
        lcd_buf[lcd_buf_y][lcd_buf_x] = c;
        lcd_buf_x++;

        if (lcd_buf_x == LCD_COLS)
        {
            lcd_buf_x = 0;
            lcd_buf_y++;
            
            if(lcd_buf_y == LCD_ROWS)
            {
                lcd_buf_y = 0;
            }
        }        
    }
}

void buf_str(const char *text)
{
    while (*text)
    {
        buf_char(*text++);
    }
    
}

void buf_clear(void)
{
    for (uint8_t y = 0; y < LCD_ROWS; y++)
    {
        for (uint8_t x = 0; x < LCD_COLS; x++)
        {
            lcd_buf[y][x] = ' ';
        }        
    }
    lcd_buf_x = 0;
    lcd_buf_y = 0;
}

void lcd_refresh(void)
{
    for ( uint8_t y = 0; y < LCD_ROWS; y++)
    {
        lcd_set_cursor(0, y);

        for (uint8_t x = 0; x < LCD_COLS; x++)
        {            
            lcd_set_cursor(x, y);
            write_byte_to_lcd(lcd_buf[y][x]);
        }        
    }    
}