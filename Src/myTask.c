/*
 * myTask.c
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */


#include "myTask.h"

/*	my Variable Led Matrix*/
uint8_t numberLap[10][8] = {
	0x00, 0x3e, 0x51, 0x49, 0x45, 0x3e, 0x00, 0x00,
	0x00, 0x00, 0x42, 0x7f, 0x40, 0x00, 0x00, 0x00,
	0x00, 0x62, 0x51, 0x51, 0x49, 0x46, 0x00, 0x00,
	0x00, 0x22, 0x41, 0x49, 0x49, 0x36, 0x00, 0x00,
	0x00, 0x18, 0x14, 0x12, 0x7f, 0x10, 0x00, 0x00,
	0x00, 0x7f, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00
};

/*
 * my Variable Lap
 */
#define n 5
uint8_t lap_A, lap_B, lap_C, lap_D;
uint16_t miliSecond, miliSecond_A[6], miliSecond_B[6], miliSecond_C[6];
uint8_t second, second_A[6], second_B[6], second_C[6];
uint8_t minute, minute_A[6], minute_B[6], minute_C[6];
uint16_t timeOut1, timeOut2, timeOut3, timeOut4, timeOut5, timeOutVal = 700;
uint16_t refreshDisplay;
unsigned char bouncing1=0xFF;
unsigned char bouncing2=0xFF;
unsigned char bouncing3=0xFF;
unsigned char bouncing4=0xFF;
unsigned char bouncing5=0xFF;
_Bool error;
_Bool stopwatchEnable;

/*
 * my Functions
 */
void myTask_init(void){
	miliSecond=0;
	second=0;
	minute=0;
	myLCD_init();
	myLedMatrix_init();
	myLedMatrix_setmatrix(1, numberLap[0]);
}

void myTask_Stopwatch(void){
	if(stopwatchEnable){
		miliSecond++;
		if(miliSecond>99){
			miliSecond=0;
			second++;
			if(second>59){
				second=0;
				minute++;
			}
		}
		if(!error){
			myLCD_setCursor(9, 0); myLCD_printNum(minute); myLCD_print(":");	myLCD_printNum(second); myLCD_print(".");	myLCD_printNum(miliSecond);
		}

	}

}

void myTask_StopwatchReset(void){

	miliSecond=0; second=0; minute=0;
	stopwatchEnable=0;
	error=0;

	/* Reset value in all track*/
	lap_A=0;	lap_B=0;	lap_C=0;
	for(uint8_t i=0; i<5; i++){
		miliSecond_A[i]=0;	miliSecond_B[i]=0;	miliSecond_C[i]=0;
		second_A[i]=0;	second_B[i]=0;	second_C[i]=0;
		minute_A[i]=0;	minute_B[i]=0;	minute_C[i]=0;
	}
	/*	Reset Diplay	*/
	myLCD_clear();
	myLedMatrix_setmatrix(0, numberLap[lap_A]);
	myLedMatrix_setmatrix(1, numberLap[lap_B]);
	myLedMatrix_setmatrix(2, numberLap[lap_C]);

}

void myTask_Run(void){
	/* Start Button */
	if(pushStart){
		if(timeOut1++ > timeOutVal){
			stopwatchEnable=0;
			myTask_ErrorMassage(1, "Button Start Error !");
		}
		else
			bouncing1 = (bouncing1<<1)|1;
	}
	else{
		timeOut1=0;
		bouncing1 = bouncing1<<1;
	}
	if(bouncing1==3){
		stopwatchEnable = !(stopwatchEnable);
		myTask_DisplayOut(stopwatchEnable);
	}

	/*	Reset Button */
	if(pushReset){
		if(timeOut2++ > timeOutVal){
			stopwatchEnable=0;
			myTask_ErrorMassage(1, "Button Reset Error !");
		}
		else
			bouncing2 = (bouncing2<<1)|1;
	}
	else{
		timeOut2=0;
		bouncing2 = bouncing2<<1;
	}
	if(bouncing2==3)
		myTask_StopwatchReset();

	/* Sensor active when stopwatch enable */
	if(stopwatchEnable){
		/* Sensor channel 0 */
		if(adcVal[0] < 500){
			if(timeOut3++ > timeOutVal){
				stopwatchEnable=0;
				myLCD_clear();
				myTask_ErrorMassage(1, "Sensor CH0 Error !");
			}
			else
				bouncing3 = (bouncing3<<1)|1;
		}
		else{
			timeOut3=0;
			bouncing3 = bouncing3<<1;
		}

		if(bouncing3==3){
			lap_A++;
			if(lap_A > n) lap_A = n;
			miliSecond_A[lap_A] = miliSecond;
			second_A[lap_A] = second;
			minute_A[lap_A] = minute;
			myTask_DisplayOut(stopwatchEnable);

		}

		/* Sensor channel 1 */
		if(adcVal[1] < 500){
			if(timeOut4++ > timeOutVal){
				stopwatchEnable=0;
				myLCD_clear();
				myTask_ErrorMassage(1, "Sensor CH1 Error !");
			}

			else
				bouncing4 = (bouncing4<<1)|1;
		}
		else{
			timeOut4=0;
			bouncing4 = bouncing4<<1;
		}

		if(bouncing4==3){
			lap_B++;
			if(lap_B > n) lap_B=n;
			miliSecond_B[lap_B] = miliSecond;
			second_B[lap_B] = second;
			minute_B[lap_B] = minute;
			myTask_DisplayOut(stopwatchEnable);
		}

		/* Sensor channel 2 */
		if(adcVal[2] < 500){
			if(timeOut5++ > timeOutVal){
				stopwatchEnable=0;
				myLCD_clear();
				myTask_ErrorMassage(1, "Sensor CH2 Error !");
			}

			else
				bouncing5 = (bouncing5<<1)|1;
		}
		else{
			timeOut5=0;
			bouncing5 = bouncing5<<1;
		}

		if(bouncing5==3){
			lap_C++;
			if(lap_C > n) lap_C=n;
			miliSecond_C[lap_C] = miliSecond;
			second_C[lap_C] = second;
			minute_C[lap_C] = minute;
			myTask_DisplayOut(stopwatchEnable);
		}

	}
}

void myTask_ErrorMassage(_Bool state, char *msg){
	error=1;
	myLCD_setCursor(0, 0);	myLCD_print(msg);
	myTask_RefreshDisplay();
}

void myTask_RefreshDisplay(void){
	if(refreshDisplay++ > 20){
		refreshDisplay = 0;
		myLCD_clear();
	}
}

void myTask_DisplayOut(_Bool state){
	if(state){
		/*	LCD Display	*/
//		myLCD_setCursor(18, 1);	myLCD_printNum(adcVal[0]);
//		myLCD_setCursor(18, 2);	myLCD_printNum(adcVal[1]);
//		myLCD_setCursor(18, 3);	myLCD_printNum(adcVal[2]);
//		myLCD_setCursor(0, 3);	myLCD_printNum(adcVal[3]);
		myLCD_setCursor(0, 0);	myLCD_print("Timer: ");
		myLCD_setCursor(0, 1);	myLCD_print("Lap A:"); 	myLCD_printNum(lap_A);	myLCD_setCursor(8, 1);	myLCD_data(0x7E);
		myLCD_setCursor(0, 2);	myLCD_print("Lap B:");	myLCD_printNum(lap_B);	myLCD_setCursor(8, 2);	myLCD_data(0x7E);
		myLCD_setCursor(0, 3);	myLCD_print("Lap C:");	myLCD_printNum(lap_C);	myLCD_setCursor(8, 3);	myLCD_data(0x7E);
		myLCD_setCursor(9, 1); 	myLCD_printNum(minute_A[lap_A]); myLCD_print(":");	myLCD_printNum(second_A[lap_A]); myLCD_print(".");	myLCD_printNum(miliSecond_A[lap_A]);
		myLCD_setCursor(9, 2); 	myLCD_printNum(minute_B[lap_B]); myLCD_print(":");	myLCD_printNum(second_B[lap_B]); myLCD_print(".");	myLCD_printNum(miliSecond_B[lap_B]);
		myLCD_setCursor(9, 3); 	myLCD_printNum(minute_C[lap_C]); myLCD_print(":");	myLCD_printNum(second_C[lap_C]); myLCD_print(".");	myLCD_printNum(miliSecond_C[lap_C]);

		/*	DotMatrix Dispay	*/
		myLedMatrix_setmatrix(0, numberLap[lap_A]);
		myLedMatrix_setmatrix(1, numberLap[lap_B]);
		myLedMatrix_setmatrix(2, numberLap[lap_C]);
	}
}

