/*
 * myMax7219.h
 *
 *  Created on: Oct 30, 2019
 *      Author: Andi
 */

#ifndef INC_MYMAX7219_H_
#define INC_MYMAX7219_H_

#include "stdint.h"
#include "main.h"

//define ports
#define Ser_Clk()			{ HAL_GPIO_WritePin(Ser_Clk_GPIO_Port, Ser_Clk_Pin,GPIO_PIN_SET);\
							  for(uint8_t i=0; i<255; i++);\
							  HAL_GPIO_WritePin(Ser_Clk_GPIO_Port, Ser_Clk_Pin, GPIO_PIN_RESET);\
							  for(uint8_t i=0; i<255; i++);}
#define Ser_Load()			{ HAL_GPIO_WritePin(Ser_Load_GPIO_Port, Ser_Load_Pin, GPIO_PIN_SET);\
							  for(uint8_t i=0; i<255; i++);\
							  HAL_GPIO_WritePin(Ser_Load_GPIO_Port, Ser_Load_Pin, GPIO_PIN_RESET);\
							 for(uint8_t i=0; i<255; i++); }
#define Ser_Data(state)		{ HAL_GPIO_WritePin(Ser_Data_GPIO_Port, Ser_Data_Pin, (state)? GPIO_PIN_SET: GPIO_PIN_RESET); }

//setup number of chip attached to the board
#define MAX7219_ICNUMBER 4

//define registers
#define MAX7218_REGNOOP 0x00
#define MAX7218_REGDIGIT0 0x01
#define MAX7218_REGDIGIT1 0x02
#define MAX7218_REGDIGIT2 0x03
#define MAX7218_REGDIGIT3 0x04
#define MAX7218_REGDIGIT4 0x05
#define MAX7218_REGDIGIT5 0x06
#define MAX7218_REGDIGIT6 0x07
#define MAX7218_REGDIGIT7 0x08
#define MAX7218_REGDECODE 0x09
#define MAX7218_REGINTENSITY 0x0A
#define MAX7218_REGSCANLIMIT 0x0B
#define MAX7218_REGSHUTDOWN 0x0C
#define MAX7218_REGTEST 0x0F


//functions
void max7219_send(uint8_t icnum, uint8_t reg, uint8_t data);
void max7219_shutdown(uint8_t icnum, uint8_t value);
void max7219_intensity(uint8_t icnum, uint8_t value);
void max7219_test(uint8_t icnum, uint8_t value);
void max7219_scanlimit(uint8_t icnum, uint8_t value);
void max7219_decode(uint8_t icnum, uint8_t value);
void max7219_digit0(uint8_t icnum, uint8_t value);
void max7219_digit1(uint8_t icnum, uint8_t value);
void max7219_digit2(uint8_t icnum, uint8_t value);
void max7219_digit3(uint8_t icnum, uint8_t value);
void max7219_digit4(uint8_t icnum, uint8_t value);
void max7219_digit5(uint8_t icnum, uint8_t value);
void max7219_digit6(uint8_t icnum, uint8_t value);
void max7219_digit7(uint8_t icnum, uint8_t value);
void max7219_digit(uint8_t icnum, uint8_t digit, uint8_t value);
void max7219_init(void);





#endif /* INC_MYMAX7219_H_ */
