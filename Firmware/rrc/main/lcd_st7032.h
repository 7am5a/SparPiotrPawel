#pragma once

/*
 * LCD ST7032 library for avr-gcc.
 *
 * (c) 2019 Bartosz Chodorowski http://chodorowski.pro
 *
 * 1) Purpose:
 *
 * This is avr-gcc library for LCD displays based on Sitronix ST7032
 * controler.
 *
 * It was tested with RAYSTAR OPTRONICS RX1602A4-BIW-TS and Atmel ATMEGA 328.
 *
 * 2) Usage:
 *
 * This library uses Peter Fleury's I2C master library. Get it from:
 * http://www.peterfleury.epizy.com/avr-software.html
 *
 * To use it, put those includes in your .c file:
 * #include "i2cmaster.h"
 * #include "lcd_st7032.h"
 *
 * See lcd_st7032_example.c for usage example.
 *
 * 3) Resources:
 * 
 * - https://github.com/olkal/LCD_ST7032 -- an Arduino library by Olav Kallhovd
 *   which I based this library on.
 */

/* Minimum contrast value */
#define ST7032_CONTRAST_MIN		0x00

/* Maximum contrast value */
#define ST7032_CONTRAST_MAX		0x3F

// /*
//  * Initialize I2C interface. Use it before anything else.
//  */
// static esp_err_t lcd_st7032_init_i2c();

/*
 * Initialize the driver. Contents of this function may need to be modified for
 * your use case depending on how your display is set up. Refer your display
 * and ST7032 datasheet.
 */
void lcd_st7032_init();

/*
 * Set contrast value where value is between ST7032_CONTRAST_MIN and
 * ST7032_CONTRAST_MAX.
 */
void lcd_st7032_set_contrast(uint8_t value);

/*
 * Turn the display on.
 */
void lcd_st7032_display_on();

/*
 * Turn the display off.
 */
void lcd_st7032_display_off();

/*
 * Turn the cursor on.
 */
void lcd_st7032_cursor_on();

/*
 * Turn the cursor off.
 */
void lcd_st7032_cursor_off();

/*
 * Turn blinking of the cursor on.
 */
void lcd_st7032_blink_on();

/*
 * Turn blinking of the cursor off.
 */
void lcd_st7032_blink_off();

/*
 * Clear the display.
 */
void lcd_st7032_clear();

/*
 * Move the cursor to the top left corner.
 */
void lcd_st7032_home();

/*
 * Write a single character to the display.
 */
void lcd_st7032_write(uint8_t data);

/*
 * Write a string to the display.
 */
void lcd_st7032_print(char* data);

/*
 * Move the cursor to the specified position.
 * line: [0, 1]
 * pos: [0, 15]
 */
void lcd_st7032_set_cursor(uint8_t line, uint8_t pos);

/*
 * Define a custom character. Call lcd_st7032_set_cursor() afterwards if you
 * need to write to the screen.
 */
void lcd_st7032_create_char(uint8_t location, uint8_t* charmap);

/**
 * @brief Print data on LCD
 * 
 */
void lcd_task();

/**
 * @brief Set the lcd task object
 * 
 */
void set_lcd_task();