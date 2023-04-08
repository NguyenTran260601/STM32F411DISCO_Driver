/*
 * lcd.h
 *
 *  Created on: Apr 5, 2023
 *      Author: Nguyen Tran
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "driver.h"

/* bsp exposed apis */
void lcd_init(void);
void lcd_send_command(uint8_t cmd);
void lcd_print_char(uint8_t data);
void display_clear(void);
void lcd_display_return_home(void);
void lcd_print_string(char*msg);
void lcd_set_cursor(uint8_t row, uint8_t column);
void display_msg(char*msg, uint8_t colum, uint8_t row);

/*Application configurable items */

#define LCD_GPIO_PORT  GPIOE
#define LCD_GPIO_RS	   GPIO_PIN_9
#define LCD_GPIO_RW	   GPIO_PIN_10
#define LCD_GPIO_EN	   GPIO_PIN_11
#define LCD_GPIO_D4	   GPIO_PIN_12
#define LCD_GPIO_D5	   GPIO_PIN_13
#define LCD_GPIO_D6	   GPIO_PIN_14
#define LCD_GPIO_D7	   GPIO_PIN_15


/*LCD commands */
#define LCD_CMD_4DL_2N_5X8F  		0x28
#define LCD_CMD_DON_CURON    		0x0E
#define LCD_CMD_INCADD       		0x06
#define LCD_CMD_DIS_CLEAR    		0X01
#define LCD_CMD_DIS_RETURN_HOME  	0x02

#endif /* INC_LCD_H_ */
