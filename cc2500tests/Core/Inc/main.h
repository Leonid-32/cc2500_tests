/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define REG_3V3_AO_PG_Pin GPIO_PIN_13
#define REG_3V3_AO_PG_GPIO_Port GPIOC
#define TPS_STATUS_Pin GPIO_PIN_0
#define TPS_STATUS_GPIO_Port GPIOA
#define TEMP_ALARM_Pin GPIO_PIN_15
#define TEMP_ALARM_GPIO_Port GPIOA
#define CAN_LOOPBACK_EN_Pin GPIO_PIN_2
#define CAN_LOOPBACK_EN_GPIO_Port GPIOA
#define CAN_STANDBY_Pin GPIO_PIN_3
#define CAN_STANDBY_GPIO_Port GPIOA
#define CC2500_SS_Pin GPIO_PIN_11
#define CC2500_SS_GPIO_Port GPIOB
#define ADF4371_SS_Pin GPIO_PIN_5
#define ADF4371_SS_GPIO_Port GPIOA
#define ADF4371_EN_Pin GPIO_PIN_0
#define ADF4371_EN_GPIO_Port GPIOB
#define CC2500_STATUS1_Pin GPIO_PIN_1
#define CC2500_STATUS1_GPIO_Port GPIOB
#define CC2500_STATUS2_Pin GPIO_PIN_2
#define CC2500_STATUS2_GPIO_Port GPIOB
#define REG_3V3_EN_Pin GPIO_PIN_15
#define REG_3V3_EN_GPIO_Port GPIOC
#define REG_2V0_EN_Pin GPIO_PIN_10
#define REG_2V0_EN_GPIO_Port GPIOB
#define REG_2V0_PG_Pin GPIO_PIN_14
#define REG_2V0_PG_GPIO_Port GPIOB
#define ADF4371_OSC_EN_Pin GPIO_PIN_15
#define ADF4371_OSC_EN_GPIO_Port GPIOB
#define POWER_ALARM_Pin GPIO_PIN_14
#define POWER_ALARM_GPIO_Port GPIOC
#define REG_6V5_PG_Pin GPIO_PIN_5
#define REG_6V5_PG_GPIO_Port GPIOB
#define REG_5V0_PG_Pin GPIO_PIN_4
#define REG_5V0_PG_GPIO_Port GPIOB
#define REG_6V5_EN_Pin GPIO_PIN_9
#define REG_6V5_EN_GPIO_Port GPIOB
#define REG_5V0_EN_Pin GPIO_PIN_8
#define REG_5V0_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
