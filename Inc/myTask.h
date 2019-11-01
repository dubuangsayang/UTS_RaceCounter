/*
 * myTask.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */

#ifndef INC_MYTASK_H_
#define INC_MYTASK_H_
#include <stdint.h>

uint16_t miliSecond;
uint8_t second,minute;
void myTask_init(void);
void myTask_Button(void);
void myTask_Stopwatch(void);
void myTask_StopwatchReset(void);
void myTask_Sensor(void);




#endif /* INC_MYTASK_H_ */
