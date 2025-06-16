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
#define ADS1256_RESET_Pin GPIO_PIN_0
#define ADS1256_RESET_GPIO_Port GPIOC
#define AMP_G1_Pin GPIO_PIN_0
#define AMP_G1_GPIO_Port GPIOA
#define AMP_G10_Pin GPIO_PIN_2
#define AMP_G10_GPIO_Port GPIOA
#define AMP_G100_Pin GPIO_PIN_4
#define AMP_G100_GPIO_Port GPIOA
#define AMP_A0_Pin GPIO_PIN_6
#define AMP_A0_GPIO_Port GPIOA
#define AMP_A1_Pin GPIO_PIN_4
#define AMP_A1_GPIO_Port GPIOC
#define AMP_A2_Pin GPIO_PIN_0
#define AMP_A2_GPIO_Port GPIOB
#define ALPF_S2_Pin GPIO_PIN_1
#define ALPF_S2_GPIO_Port GPIOB
#define ALPF_S1_Pin GPIO_PIN_2
#define ALPF_S1_GPIO_Port GPIOB
#define ALPF_S3_Pin GPIO_PIN_11
#define ALPF_S3_GPIO_Port GPIOF
#define LCD_RES_Pin GPIO_PIN_1
#define LCD_RES_GPIO_Port GPIOG
#define RE4_Pin GPIO_PIN_10
#define RE4_GPIO_Port GPIOB
#define RE3_Pin GPIO_PIN_12
#define RE3_GPIO_Port GPIOB
#define RE2_Pin GPIO_PIN_13
#define RE2_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOB
#define LCD_BLK_Pin GPIO_PIN_11
#define LCD_BLK_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_5
#define KEY1_GPIO_Port GPIOG
#define KEY2_Pin GPIO_PIN_6
#define KEY2_GPIO_Port GPIOG
#define KEY3_Pin GPIO_PIN_7
#define KEY3_GPIO_Port GPIOG
#define KEY4_Pin GPIO_PIN_8
#define KEY4_GPIO_Port GPIOG
#define KEY5_Pin GPIO_PIN_6
#define KEY5_GPIO_Port GPIOC
#define KEY6_Pin GPIO_PIN_7
#define KEY6_GPIO_Port GPIOC
#define KEY7_Pin GPIO_PIN_8
#define KEY7_GPIO_Port GPIOC
#define KEY8_Pin GPIO_PIN_9
#define KEY8_GPIO_Port GPIOC
#define BOARD_LED_Pin GPIO_PIN_11
#define BOARD_LED_GPIO_Port GPIOA
#define ADS1256_CS_Pin GPIO_PIN_15
#define ADS1256_CS_GPIO_Port GPIOA
#define ADS1256_DRDY_Pin GPIO_PIN_6
#define ADS1256_DRDY_GPIO_Port GPIOB
#define ADS1256_DRDY_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
