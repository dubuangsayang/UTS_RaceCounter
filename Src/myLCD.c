/*
 * myLCD.c
 *
 *  Created on: Sep 18, 2019
 *      Author: Andi
 */

#include "stm32f1xx_hal.h"
#include "main.h"
#include "myLCD.h"
#include <stdio.h>

void myLCD_init(void){
	HAL_Delay(2);
	myLCD_command(0x33);
	myLCD_command(0x32);
	myLCD_command(0x28);
	myLCD_command(0x0C);
	myLCD_command(0x01);
	HAL_Delay(2);
}
void myLCD_write4bit(uint8_t data){
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, ((data>>0)&1)? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, ((data>>1)&1)? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, ((data>>2)&1)? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, ((data>>3)&1)? GPIO_PIN_SET:GPIO_PIN_RESET);
}

void myLCD_send(_Bool mode, uint8_t data){
	if(mode){ myLCD_RS(1); }
	else	{ myLCD_RS(0); }

	myLCD_write4bit(data>>4);
	myLCD_E();
	myLCD_write4bit(data);
	myLCD_E();

}
void myLCD_command(uint8_t data){
	myLCD_send(0, data);
}
void myLCD_data(uint8_t data){
	myLCD_send(1, data);
}
void myLCD_setCursor(uint8_t x, uint8_t y){
	if(y==0) {myLCD_command(0x80 + x); }
	if(y==1) {myLCD_command(0xC0 + x); }
	if(y==2) {myLCD_command(0x94 + x); }
	if(y==3) {myLCD_command(0xD4 + x); }
}
void myLCD_print(char *pData){
	while(*pData){
		myLCD_data(*pData);
		pData++;
	}
}
void myLCD_printNum(int16_t number){
	char buff[20];
	sprintf(buff,"%d",number);
	myLCD_print(buff);
}
void myLCD_clear(void){
	myLCD_command(0x01);
	for(uint16_t i=0; i<2000; i++);
}
