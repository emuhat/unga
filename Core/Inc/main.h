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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RADAR_INT_Pin GPIO_PIN_1
#define RADAR_INT_GPIO_Port GPIOA
#define RADAR_TX_Pin GPIO_PIN_2
#define RADAR_TX_GPIO_Port GPIOA
#define RADAR_RX_Pin GPIO_PIN_3
#define RADAR_RX_GPIO_Port GPIOA
#define SENC_A_Pin GPIO_PIN_0
#define SENC_A_GPIO_Port GPIOB
#define SENC_A_EXTI_IRQn EXTI0_IRQn
#define SENC_B_Pin GPIO_PIN_1
#define SENC_B_GPIO_Port GPIOB
#define SENC_B_EXTI_IRQn EXTI1_IRQn
#define SENC_SW_Pin GPIO_PIN_2
#define SENC_SW_GPIO_Port GPIOB
#define PI_TX_Pin GPIO_PIN_9
#define PI_TX_GPIO_Port GPIOA
#define PI_RX_Pin GPIO_PIN_10
#define PI_RX_GPIO_Port GPIOA
#define PI_PWR_Pin GPIO_PIN_11
#define PI_PWR_GPIO_Port GPIOA
#define LED_BATT_TIMER_Pin GPIO_PIN_15
#define LED_BATT_TIMER_GPIO_Port GPIOA
#define LED_MAIN_TIMER_Pin GPIO_PIN_3
#define LED_MAIN_TIMER_GPIO_Port GPIOB
#define VCNL_SCL_Pin GPIO_PIN_6
#define VCNL_SCL_GPIO_Port GPIOB
#define VCNL_SDA_Pin GPIO_PIN_7
#define VCNL_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
