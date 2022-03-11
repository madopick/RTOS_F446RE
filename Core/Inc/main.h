/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define PUSH_BUTTON_PIN 					GPIO_PIN_13
#define PUSH_BUTTON_PORT 					GPIOC
#define USART_TX_Pin 						GPIO_PIN_2
#define USART_TX_GPIO_Port 					GPIOA
#define USART_RX_Pin 						GPIO_PIN_3
#define USART_RX_GPIO_Port 					GPIOA
#define LD2_Pin 							GPIO_PIN_5
#define LD2_GPIO_Port 						GPIOA
#define TMS_Pin 							GPIO_PIN_13
#define TMS_GPIO_Port 						GPIOA
#define TCK_Pin 							GPIO_PIN_14
#define TCK_GPIO_Port 						GPIOA
#define SWO_Pin 							GPIO_PIN_3
#define SWO_GPIO_Port 						GPIOB


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
