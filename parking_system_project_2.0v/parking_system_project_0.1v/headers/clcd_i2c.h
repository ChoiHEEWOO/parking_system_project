/*
 * clcd_i2c.h
 *
 * Created: 2020-09-07 오후 2:40:49
 *  Author: 최희우
 */ 
#ifndef CLCD_I2C_H
#define CLCD_I2C_H

#define F_CPU 16000000UL
#define LCD_BACKLIGHT_ON 1
#define LCD_BACKLIGHT_OFF 0

#include "i2c_lib.h"
#include <util/delay.h>

#include <avr/io.h>
#include <stdint.h>

void twi_init(void);
void i2c_lcd_init(void);
void i2c_lcd_command(uint8_t command); //4bit 커맨드
void i2c_lcd_command_8(uint8_t command); //8bit 커맨드
void i2c_lcd_noBacklight(void); //화면 클리어시키고 화면끌 때 사용
void i2c_lcd_setBacklight(uint8_t light); //글자들은 유지시킨 채 백라이트 온오프

void i2c_lcd_data(uint8_t data);
void i2c_lcd_goto_XY(uint8_t row, uint8_t col);
void i2c_lcd_string(uint8_t row, uint8_t col, char* string);
void i2c_lcd_clear();

#endif /* CLCD_H_ */