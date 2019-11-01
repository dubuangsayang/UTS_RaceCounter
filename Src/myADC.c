/*
 * myADC.c
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */

#include "myADC.h"

void myADC_start(void){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcVal, 4);
}
