/*
 * myADC.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */

#ifndef INC_MYADC_H_
#define INC_MYADC_H_

#include "stm32f1xx.h"
#include "stdint.h"

ADC_HandleTypeDef hadc1;

uint32_t adcVal[3];
void myADC_start(void);




#endif /* INC_MYADC_H_ */
