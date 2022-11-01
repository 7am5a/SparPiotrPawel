#ifndef LCD_DRV
#define LCD_DRV

#define MASTER_CLK 10000        //Master clock frequency - required 10k
#define MASTER_TX_BUF 0         //Size of Master TX buffer
#define MASTER_RX_BUF 0         //Size of Master RC buffer
#define LCD_ADDRES 0b0111110    //Slave addres from RX1602 datasheet

/*
    Macros of display commands
*/
#define CLEAR_DISPLAY       0b00000001 //Clear all display and set cursor in origin
#define RETURN_HOME         0b00000011 //Set cursor in origin
#define MOVE_RIGHT          0b00000101 //Set moving direction on right
#define MOVE_LEFT           0b00000111 //Set moving direction on left
#define SHIFT_CURSOR_LEFT   0b00010011 //Shift cursor to the left
#define SHIFT_CURSOR_RIGHT  0b00010111 //Shift cursor to the right
#define SHIFT_DISPLAY_LEFT  0b00011011 //Shift display to the left, cursor follow shift
#define SHIFT_DISPLAY_RIGHT 0b00011111 //Shift display to the right, cursor follow shift

//Setup display
/*Turning order: Display_X > CURSOR_BLINK_X (if need both) > CURSOR_X_X && BLINK_X */

#define DISPLAY_OFF         0b00001000 //Turn OFF display, remember data in DDRAM 
#define DISPLAY_ON          0b00001100 //Turn entire display ON
#define CURSOR_OFF          0b00001100 //Turn OFF cursor, I/D register remain its data
#define CURSOR_ON           0b00001110 //Turn cursor ON
#define CURSOR_BLINK_OFF    0b00001110 //Turn cursor blink OFF
#define CURSOR_BLINK_ON     0b00001111 //Turn cursor blink ON, alternate between data and display at curosr position 
#define BLINK_OFF           0b00001100 //Turn blink OFF
#define BLINK_ON            0b00001101 //Turn blink ON, alternate between data and display at curosr position

#define DATA_LENGTH_4_BIT   0b00101001 //Set 4 bit bus mode, 2 rows and extension instruction table
#define DATA_LENGTH_8_BIT   0b00111001 //Set 8 bit bus mode, 2 rows and extension instruction table
#define CGRAM_ADDRES        0b01000000 //Set CGRAM addres to AC
#define DDRAM_ADDRES        0b10000000 //Set DDRAM addres to AC

#define ENABLE              0b00000100 //Enable flag
#define READ                0b00000010 //Read flag
#define WRITE               0b00000000 //Write flag
#define RS_DATA             0b00000001 //Register Select data
#define RS_COMMAND          0b00000000 //Register Select command

#define LCD_LINE1           0b00000000 //First line of display
#define LCD_LINE2           0b00001000 //Second line of display

#define LCD_ROWS            2          //Display rows
#define LCD_COLS            16         //Display columns


/**
 * @brief Initialize and set up I2C to transmite data into LCD
 * 
 */
void init_lcd_i2c(); 

/**
 * @brief Send one byte of data to LCD
 * 
 * @param menu_data Data sended to LCD like one char 
 */
void write_byte_to_lcd(uint8_t menu_data);

/**
 * @brief Receive one byte of data from LCD
 * 
 * @param lcd_data Data received fom LCD like one char
 */
void read_byte_from_lcd(uint8_t *lcd_data);

/**
 * @brief Define new character in LCD CGRAM
 * 
 * @param char_address Addres of defined in ascii_table.h character
 * @param char_name Name of defined in ascii_table.h character
 */
void lcd_define_symbol(uint8_t char_address, const char *char_name);

/**
 * @brief Write a command to LCD
 * 
 * @param type_command Command defined in lcd_drv.h library or in binary or in hexadecimal
 */
void lcd_write_command(uint8_t type_command);

/**
 * @brief Write character or longer text in a LCD
 * 
 * @param type_data Type character or string 
 */
void lcd_write_data(const char *type_data);

/**
 * @brief Set cursor in required position
 * 
 * @param x Number of column, start from 0 - can be written in binary, decimal or hex
 * @param y Number of row, start from 0 - can be written in binary, decimal or hex
 */
void lcd_set_cursor(uint8_t x, uint8_t y);

#endif