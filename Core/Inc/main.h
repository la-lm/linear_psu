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
#include "stm32g4xx_hal.h"

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
#define ENC_1_A_Pin GPIO_PIN_0
#define ENC_1_A_GPIO_Port GPIOC
#define ENC_1_B_Pin GPIO_PIN_1
#define ENC_1_B_GPIO_Port GPIOC
#define ADC_TEMP_Pin GPIO_PIN_1
#define ADC_TEMP_GPIO_Port GPIOA
#define ADC_Vfb_Pin GPIO_PIN_3
#define ADC_Vfb_GPIO_Port GPIOA
#define DAC_set_V_Pin GPIO_PIN_4
#define DAC_set_V_GPIO_Port GPIOA
#define ADC_Isens_Pin GPIO_PIN_5
#define ADC_Isens_GPIO_Port GPIOA
#define DAC_set_I_Pin GPIO_PIN_6
#define DAC_set_I_GPIO_Port GPIOA
#define TO_RELAY_DRV_Pin GPIO_PIN_11
#define TO_RELAY_DRV_GPIO_Port GPIOB
#define RELAY_SW_Pin GPIO_PIN_13
#define RELAY_SW_GPIO_Port GPIOB
#define RELAY_LED_Pin GPIO_PIN_15
#define RELAY_LED_GPIO_Port GPIOB
#define ENC_2_B_Pin GPIO_PIN_11
#define ENC_2_B_GPIO_Port GPIOA
#define ENC_2_A_Pin GPIO_PIN_12
#define ENC_2_A_GPIO_Port GPIOA
#define FAN_PWM_Pin GPIO_PIN_5
#define FAN_PWM_GPIO_Port GPIOB
#define FAN_TACH_Pin GPIO_PIN_7
#define FAN_TACH_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
