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

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_i2c.h"
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
#define GYRO2_NSS_Pin LL_GPIO_PIN_3
#define GYRO2_NSS_GPIO_Port GPIOE
#define MAG_NSS_Pin LL_GPIO_PIN_4
#define MAG_NSS_GPIO_Port GPIOE
#define GYRO1_NSS_Pin LL_GPIO_PIN_5
#define GYRO1_NSS_GPIO_Port GPIOE
#define BARO_NSS_Pin LL_GPIO_PIN_6
#define BARO_NSS_GPIO_Port GPIOE
#define FLASH_NSS_Pin LL_GPIO_PIN_13
#define FLASH_NSS_GPIO_Port GPIOC
#define MSD_DETECT_Pin LL_GPIO_PIN_15
#define MSD_DETECT_GPIO_Port GPIOC
#define V_BATT_Pin LL_GPIO_PIN_0
#define V_BATT_GPIO_Port GPIOC
#define RSSI_Pin LL_GPIO_PIN_1
#define RSSI_GPIO_Port GPIOC
#define PM_CURRENT_Pin LL_GPIO_PIN_2
#define PM_CURRENT_GPIO_Port GPIOC
#define PM_VOLTAGE_Pin LL_GPIO_PIN_3
#define PM_VOLTAGE_GPIO_Port GPIOC
#define MAIN_CH5_Pin LL_GPIO_PIN_0
#define MAIN_CH5_GPIO_Port GPIOA
#define MAIN_CH6_Pin LL_GPIO_PIN_1
#define MAIN_CH6_GPIO_Port GPIOA
#define MAIN_CH7_Pin LL_GPIO_PIN_2
#define MAIN_CH7_GPIO_Port GPIOA
#define MAIN_CH8_Pin LL_GPIO_PIN_3
#define MAIN_CH8_GPIO_Port GPIOA
#define CAN1_STBY_Pin LL_GPIO_PIN_4
#define CAN1_STBY_GPIO_Port GPIOC
#define CAN2_STBY_Pin LL_GPIO_PIN_5
#define CAN2_STBY_GPIO_Port GPIOC
#define MAIN_CH9_Pin LL_GPIO_PIN_0
#define MAIN_CH9_GPIO_Port GPIOB
#define MAIN_CH10_Pin LL_GPIO_PIN_1
#define MAIN_CH10_GPIO_Port GPIOB
#define BARO_INT_Pin LL_GPIO_PIN_7
#define BARO_INT_GPIO_Port GPIOE
#define MAIN_CH11_Pin LL_GPIO_PIN_11
#define MAIN_CH11_GPIO_Port GPIOE
#define MAIN_CH12_Pin LL_GPIO_PIN_13
#define MAIN_CH12_GPIO_Port GPIOE
#define PPM_IN_Pin LL_GPIO_PIN_14
#define PPM_IN_GPIO_Port GPIOE
#define PPM_IN_EXTI_IRQn EXTI15_10_IRQn
#define GPS2_SCL_Pin LL_GPIO_PIN_10
#define GPS2_SCL_GPIO_Port GPIOB
#define GPS2_SCLB11_Pin LL_GPIO_PIN_11
#define GPS2_SCLB11_GPIO_Port GPIOB
#define Telm2_TX_Pin LL_GPIO_PIN_8
#define Telm2_TX_GPIO_Port GPIOD
#define Telm2_RX_Pin LL_GPIO_PIN_9
#define Telm2_RX_GPIO_Port GPIOD
#define GPS1_SW_LED_Pin LL_GPIO_PIN_10
#define GPS1_SW_LED_GPIO_Port GPIOD
#define GPS1_SW_Pin LL_GPIO_PIN_13
#define GPS1_SW_GPIO_Port GPIOD
#define Buzzer_Pin LL_GPIO_PIN_14
#define Buzzer_GPIO_Port GPIOD
#define MAIN_CH4_Pin LL_GPIO_PIN_15
#define MAIN_CH4_GPIO_Port GPIOD
#define MAIN_CH2_Pin LL_GPIO_PIN_6
#define MAIN_CH2_GPIO_Port GPIOC
#define MAIN_CH3_Pin LL_GPIO_PIN_7
#define MAIN_CH3_GPIO_Port GPIOC
#define GPS1_SDA_Pin LL_GPIO_PIN_9
#define GPS1_SDA_GPIO_Port GPIOC
#define GPS1_SCL_Pin LL_GPIO_PIN_8
#define GPS1_SCL_GPIO_Port GPIOA
#define RC_SRXL2_Pin LL_GPIO_PIN_9
#define RC_SRXL2_GPIO_Port GPIOA
#define GPS1_TX_Pin LL_GPIO_PIN_10
#define GPS1_TX_GPIO_Port GPIOC
#define GPS1_RX_Pin LL_GPIO_PIN_11
#define GPS1_RX_GPIO_Port GPIOC
#define GPS2_TX_Pin LL_GPIO_PIN_12
#define GPS2_TX_GPIO_Port GPIOC
#define GPS2_RX_Pin LL_GPIO_PIN_2
#define GPS2_RX_GPIO_Port GPIOD
#define Telm1_TX_Pin LL_GPIO_PIN_5
#define Telm1_TX_GPIO_Port GPIOD
#define Telm1_RX_Pin LL_GPIO_PIN_6
#define Telm1_RX_GPIO_Port GPIOD
#define MAIN_CH1_Pin LL_GPIO_PIN_7
#define MAIN_CH1_GPIO_Port GPIOB
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
