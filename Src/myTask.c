/*
 * myTask.c
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */

/*	my Variable */
#include "myTask.h"


uint8_t lap_A, lap_B, lap_C;
uint16_t miliSecond;
uint8_t second,minute;
uint8_t timeOut1, timeOut2, timeOut3, timeOut4, timeOut5, timeOutVal = 100;
uint16_t refreshDisplay;
unsigned char bouncing1=0xFF;
unsigned char bouncing2=0xFF;
unsigned char bouncing3=0xFF;
unsigned char bouncing4=0xFF;
unsigned char bouncing5=0xFF;
_Bool error;
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
	stopwatchEnable=0;
	miliSecond=0; second=0; minute=0;
	error=0;
}

void myTask_Run(void){
	/* Start Button */
	if(pushStart){
		if(timeOut1++ > timeOutVal){
			myTask_ErrorMassage(1, "Button Start Error !");
		}
		else
			bouncing1 = (bouncing1<<1)|1;
	}
	else{
		timeOut1=0;
		bouncing1 = bouncing1<<1;
	}
	if(bouncing1==3)
		stopwatchEnable = !(stopwatchEnable);

	/*	Reset Button */
	if(pushReset){
		if(timeOut2++ > timeOutVal)
			myTask_ErrorMassage(1, "Button Reset Error !");
		else
			bouncing2 = (bouncing2<<1)|1;
	}
	else{
		timeOut2=0;
		bouncing2 = bouncing2<<1;
	}
	if(bouncing2==3)
		myTask_StopwatchReset();

	/* Sensor channel 0 */
	if(adcVal[0] < 1000){
		if(timeOut3++ > timeOutVal)
			myTask_ErrorMassage(1, "Sensor CH0 Error !");
		else
			bouncing3 = (bouncing3<<1)|1;
	}
	else{
		timeOut3=0;
		bouncing3 = bouncing3<<1;
	}

	if(bouncing3==3){
		lap_A++;
	}

	/* Sensor channel 1 */
	if(adcVal[1] < 1000){
		if(timeOut4++ > timeOutVal)
			myTask_ErrorMassage(1, "Sensor CH1 Error !");
		else
			bouncing4 = (bouncing4<<1)|1;
	}
	else{
		timeOut4=0;
		bouncing4 = bouncing4<<1;
	}

	if(bouncing4==3){
		lap_B++;
	}

	/* Sensor channel 2 */
	if(adcVal[3] < 1000){
		if(timeOut5++ > timeOutVal)
			myTask_ErrorMassage(1, "Sensor CH2 Error !");
		else
			bouncing5 = (bouncing5<<1)|1;
	}
	else{
		timeOut5=0;
		bouncing5 = bouncing5<<1;
	}

	if(bouncing5==3){
		lap_C++;
	}

	/* Print in the LCD Display */
	myTask_DisplayOut(!error);
}

void myTask_ErrorMassage(_Bool state, char *msg){
	error=1;
	//HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, (error)? GPIO_PIN_SET:GPIO_PIN_RESET);
	myLCD_setCursor(0, 0);	myLCD_print(msg);
	myTask_RefreshDisplay();
}

void myTask_RefreshDisplay(void){
	if(refreshDisplay++ > 10){
		refreshDisplay = 0;
		myLCD_clear();
	}
}

void myTask_DisplayOut(_Bool state){
	if(state){
		myLCD_setCursor(0, 0);	myLCD_printNum(adcVal[0]);
		myLCD_setCursor(0, 1);	myLCD_printNum(adcVal[1]);
		myLCD_setCursor(0, 2);	myLCD_printNum(adcVal[2]);
		myLCD_setCursor(0, 3);	myLCD_printNum(adcVal[3]);
		myLCD_setCursor(10, 0);	myLCD_printNum(lap_A);
		myLCD_setCursor(10, 1);	myLCD_printNum(lap_B);
		myLCD_setCursor(10, 2);	myLCD_printNum(lap_C);
		myTask_RefreshDisplay();
	}
}

