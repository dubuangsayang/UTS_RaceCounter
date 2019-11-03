/*
 * myUART.c
 *
 *  Created on: Nov 3, 2019
 *      Author: Andi
 */

#include "myUART.h"

void myUART_Print(char *pData){
	HAL_UART_Transmit(&huart1, (uint8_t *)pData, strlen(pData), 10);
}

void myUART_Println(char *pData){
	HAL_UART_Transmit(&huart1, (uint8_t *)pData, strlen(pData), 10);
	char newLine[2] = "\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t *)newLine, 2, 10);
}

void myUART_PrintNum(uint16_t number){
	char buff[50];
	sprintf(buff,"%d", number);
	HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 10);
}

