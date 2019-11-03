/*
 * myHardware.h
 *
 *  Created on: Oct 29, 2019
 *      Author: Andi
 */

#ifndef INC_MYHARDWARE_H_
#define INC_MYHARDWARE_H_

#include "stm32f1xx_hal.h"
#include "string.h"

/*	my PORT Configurations	*/
#define Ser_Data_Pin GPIO_PIN_4
#define Ser_Data_GPIO_Port GPIOA
#define Ser_Load_Pin GPIO_PIN_5
#define Ser_Load_GPIO_Port GPIOA
#define Ser_Clk_Pin GPIO_PIN_6
#define Ser_Clk_GPIO_Port GPIOA
#define PB_RESET_Pin GPIO_PIN_7
#define PB_RESET_GPIO_Port GPIOA
#define PB_START_Pin GPIO_PIN_0
#define PB_START_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_15
#define Buzzer_GPIO_Port GPIOA
#define LCD_Bkl_Pin GPIO_PIN_3
#define LCD_Bkl_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_4
#define LCD_D7_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_5
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_6
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_7
#define LCD_D4_GPIO_Port GPIOB
#define LCD_E_Pin GPIO_PIN_8
#define LCD_E_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_9
#define LCD_RS_GPIO_Port GPIOB

#define pushStart			 HAL_GPIO_ReadPin(PB_START_GPIO_Port, PB_START_Pin)==GPIO_PIN_RESET
#define pushReset			 HAL_GPIO_ReadPin(PB_RESET_GPIO_Port, PB_RESET_Pin)==GPIO_PIN_RESET

/* my Variable */
uint32_t adcVal[3];

/*	Handler Typedef	*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/*	STM32 functions	*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_DMA_Init(void);
void Error_Handler(void);

#endif /* INC_MYHARDWARE_H_ */
