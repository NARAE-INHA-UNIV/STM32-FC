/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BLUE_Pin LL_GPIO_PIN_2
#define LED_BLUE_GPIO_Port GPIOE
#define LED_DEBUG_Pin LL_GPIO_PIN_1
#define LED_DEBUG_GPIO_Port GPIOC
#define LED_DEBUG3_Pin LL_GPIO_PIN_11
#define LED_DEBUG3_GPIO_Port GPIOE
#define LED_DEBUG2_Pin LL_GPIO_PIN_13
#define LED_DEBUG2_GPIO_Port GPIOE
#define Telm2_TX_Pin LL_GPIO_PIN_8
#define Telm2_TX_GPIO_Port GPIOD
#define Telm2_RX_Pin LL_GPIO_PIN_9
#define Telm2_RX_GPIO_Port GPIOD
#define Buzzer_Pin LL_GPIO_PIN_14
#define Buzzer_GPIO_Port GPIOD
#define RC_SRXL2_Pin LL_GPIO_PIN_9
#define RC_SRXL2_GPIO_Port GPIOA
#define GPS1_TX_Pin LL_GPIO_PIN_10
#define GPS1_TX_GPIO_Port GPIOC
#define GPS2_RX_Pin LL_GPIO_PIN_11
#define GPS2_RX_GPIO_Port GPIOC
#define GPS2_TX_Pin LL_GPIO_PIN_12
#define GPS2_TX_GPIO_Port GPIOC
#define GPS2_RXD2_Pin LL_GPIO_PIN_2
#define GPS2_RXD2_GPIO_Port GPIOD
#define Telm1_TX_Pin LL_GPIO_PIN_5
#define Telm1_TX_GPIO_Port GPIOD
#define Telm1_RX_Pin LL_GPIO_PIN_6
#define Telm1_RX_GPIO_Port GPIOD
#define LED_RED_Pin LL_GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOE
#define LED_YELLOW_Pin LL_GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
