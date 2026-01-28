/**
  ******************************************************************************
  * @file		lcd.h
  * @brief		LCD Header file API
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef LCD_H_
#define LCD_H_

#define LCD_RS_SET		(1<<8)
#define LCD_RS_CLEAR	(1<<(8+16))
#define LCD_RW_READ		(1<<9)
#define LCD_RW_WRITE	(1<<(9+16))
#define LCD_E_SET		(1<<10)
#define LCD_E_CLEAR		(1<<(10+16))

#define DATA_CLEAR		(0xFF<<(4+16))
#define MAX_CHAR		80


void lcd_init(void);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_position(int row, int col);
void lcd_print_char(char);
int lcd_print_string(char*);
int lcd_print_num(int);

int lcd_print_hex(int, int, int);

#endif /* LCD_H_ */
