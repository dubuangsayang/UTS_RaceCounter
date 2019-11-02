/*
 * myTask.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */

#ifndef INC_MYTASK_H_
#define INC_MYTASK_H_
#include <stdint.h>

/* my Function Prototype */
void myTask_init(void);
void myTask_Run(void);
void myTask_Stopwatch(void);
void myTask_StopwatchReset(void);
void myTask_ErrorMassage(_Bool state, char *msg);
void myTask_RefreshDisplay(void);
void myTask_DisplayOut(_Bool state);

#endif /* INC_MYTASK_H_ */
