/*
 * myLCD.h
 *
 *  Created on: Sep 18, 2019
 *      Author: Andi
 */

#ifndef INC_MYLCD_H_
#define INC_MYLCD_H_

#include "stm32f1xx_hal.h"
#include "main.h"

#define myLCD_RS(state)	{ HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, (state)? GPIO_PIN_SET:GPIO_PIN_RESET); }
#define myLCD_E()		{ HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);\
						  for(uint16_t i=0; i<1500; i++);\
						  HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);\
						  for(uint16_t i=0; i<1500; i++);}
#define myLCD_Bkl(state) { HAL_GPIO_WritePin(LCD_Bkl_GPIO_Port, LCD_Bkl_Pin, (state)? GPIO_PIN_SET : GPIO_PIN_RESET); }

void myLCD_init(void);
void myLCD_write4bit(uint8_t data);
void myLCD_send(_Bool mode, uint8_t data);
void myLCD_command(uint8_t data);
void myLCD_data(uint8_t data);
void myLCD_setCursor(uint8_t x, uint8_t y);
void myLCD_print(char *pData);
void myLCD_printNum(int16_t number);
void myLCD_clear(void);





#endif /* INC_MYLCD_H_ */
