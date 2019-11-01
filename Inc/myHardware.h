/*
 * myHardware.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */

#ifndef INC_MYHARDWARE_H_
#define INC_MYHARDWARE_H_

#define pushStart			 HAL_GPIO_ReadPin(PB_START_GPIO_Port, PB_START_Pin)==GPIO_PIN_RESET
#define pushReset			 HAL_GPIO_ReadPin(PB_RESET_GPIO_Port, PB_RESET_Pin)==GPIO_PIN_RESET



#endif /* INC_MYHARDWARE_H_ */
