/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f4xx_hal.h"


/* Private includes ----------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
/* PWM Peripheral */
#define PWM_Out_Pin 													GPIO_PIN_7
#define PWM_Out_GPIO_Port 										GPIOC

/* ADC Peripheral */
#define ADC_Input_Pin 												GPIO_PIN_0
#define ADC_Input_GPIO_Port 									GPIOA

/* SPI GPIO Pins */
#define SPI1_CS_Pin 													GPIO_PIN_3
#define SPI1_CS_GPIO_Port 										GPIOA

/* NUCLEO-64 Boards User I/O */
#define NUCLEO_GLED_Pin 											GPIO_PIN_5
#define NUCLEO_GLED_GPIO_Port 								GPIOA
#define NUCLEO_PushButton_Pin 								GPIO_PIN_13
#define NUCLEO_PushButton_GPIO_Port 					GPIOC
#define NUCLEO_PushButton_EXTI_IRQn 					EXTI15_10_IRQn



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
