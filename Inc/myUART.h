/*
 * myUART.h
 *
 *  Created on: Nov 3, 2019
 *      Author: Andi
 */

#ifndef INC_MYUART_H_
#define INC_MYUART_H_

#include "myHardware.h"
#include "string.h"
#include "stdio.h"

/*	my Function Prototype	*/
void myUART_Print(char *pData);
void myUART_Println(char *pData);
void myUART_PrintNum(uint16_t number);




#endif /* INC_MYUART_H_ */
