#ifndef LCD_BUF_DRV
#define LCD_BUF_DRV

/**
 * @brief Set buffer addres 
 * 
 * @param x Column
 * @param y Row
 */
void buf_locate(uint8_t x, uint8_t y);

/**
 * @brief Add character to buffer
 * 
 * @param c Character
 */
void buf_char(char c);

/**
 * @brief Add string to a buffer
 * 
 * @param text String
 */
void buf_str(const char *text);

/**
 * @brief Clear buffer - set ' ' in every cell
 * 
 */
void buf_clear(void);

/**
 * @brief Send buffer to LCD
 * 
 */
void lcd_refresh(void);

#endif