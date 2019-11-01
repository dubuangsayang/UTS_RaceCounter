/*
 * myTask.c
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "myTask.h"
#include "myLCD.h"
#include "myHardware.h"
#include "myADC.h"

//my declaratoin variable
unsigned char bouncing1=0xFF;
unsigned char bouncing2=0xFF;
unsigned char bouncing3=0xFF;
unsigned char bouncing4=0xFF;

uint16_t timeOutLoop, timeOutVal = 1000;
uint16_t refreshDisplay;
uint8_t count;

_Bool stopwatchEnable;

void myTask_init(void){
miliSecond=0;
second=0;
minute=0;
myLCD_init();
}

void myTask_Stopwatch(void){
	if(stopwatchEnable){
		miliSecond++;
		if(miliSecond>999){
			miliSecond=0;
			second++;
			if(second>59){
				second=0;
				minute++;
			}
		}
	}
}

void myTask_StopwatchReset(void){
	miliSecond=0; second=0; minute=0;
}

void myTask_Button(void){
	if(pushStart)
		bouncing1 = (bouncing1<<1);
	else
		bouncing1 = (bouncing1<<1)|1;
	if(bouncing1==3){
		stopwatchEnable = !(stopwatchEnable);
	}

	if(pushReset)
		bouncing2 = (bouncing2<<1)|1;
	else
		bouncing2 = (bouncing2<<1);
	if(bouncing2==3){
		//myTask_StopwatchReset();
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
				HAL_Delay(1);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
	}
}

void myTask_Sensor(void){
	if(adcVal[0] < 1000){
//		timeOutLoop++;
//		if(timeOutLoop > timeOutVal){
//			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
//		}
//		else
			bouncing3 = (bouncing3<<1)|1;
	}

	else{
		timeOutLoop=0;
		bouncing3 = bouncing3<<1;
	}

	if(bouncing3==3){
		count++;
//		HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
		//HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
		//for(uint16_t i=0; i<20; i++) for(uint16_t j=0; j<65535; j++);
		//HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
	}
	myLCD_setCursor(0, 0);	myLCD_printNum(adcVal[0]);
	myLCD_setCursor(0, 1);	myLCD_printNum(adcVal[1]);
	myLCD_setCursor(0, 2);	myLCD_printNum(adcVal[2]);
	myLCD_setCursor(10, 2);myLCD_printNum(adcVal[3]);
	myLCD_setCursor(10, 0);	myLCD_printNum(count);
//	myLCD_clear();
	refreshDisplay++;
	if(refreshDisplay > 10){
		refreshDisplay = 0;
		myLCD_clear();
	}
}


