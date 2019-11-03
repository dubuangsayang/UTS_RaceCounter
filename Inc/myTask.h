/*
 * myTask.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */

#ifndef INC_MYTASK_H_
#define INC_MYTASK_H_

/* my Include */
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "main.h"
#include "myTask.h"
#include "myLCD.h"
#include "myHardware.h"
#include "myLedMatrix.h"
#include "myUART.h"
//#include "myADC.h"


/* my Function Prototype */
void myTask_init(void);
void myTask_Run(void);
void myTask_Stopwatch(void);
void myTask_StopwatchReset(void);
void myTask_ErrorMassage(_Bool state, char *msg);
void myTask_RefreshDisplay(void);
void myTask_DisplayOut(_Bool state);
void myTask_Calibrate(_Bool state);

#endif /* INC_MYTASK_H_ */
