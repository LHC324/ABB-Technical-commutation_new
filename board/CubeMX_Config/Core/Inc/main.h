/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#ifdef Error_Handler
#undef Error_Handler
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Q1A_Pin GPIO_PIN_2
#define Q1A_GPIO_Port GPIOE
#define WDI_Pin GPIO_PIN_6
#define WDI_GPIO_Port GPIOE
#define Q10_Pin GPIO_PIN_7
#define Q10_GPIO_Port GPIOE
#define Q11_Pin GPIO_PIN_8
#define Q11_GPIO_Port GPIOE
#define Q12_Pin GPIO_PIN_9
#define Q12_GPIO_Port GPIOE
#define Q13_Pin GPIO_PIN_10
#define Q13_GPIO_Port GPIOE
#define Q14_Pin GPIO_PIN_11
#define Q14_GPIO_Port GPIOE
#define Q15_Pin GPIO_PIN_12
#define Q15_GPIO_Port GPIOE
#define Q16_Pin GPIO_PIN_13
#define Q16_GPIO_Port GPIOE
#define Q17_Pin GPIO_PIN_14
#define Q17_GPIO_Port GPIOE
#define Q18_Pin GPIO_PIN_15
#define Q18_GPIO_Port GPIOE
#define Q19_Pin GPIO_PIN_10
#define Q19_GPIO_Port GPIOB
#define Q1B_Pin GPIO_PIN_12
#define Q1B_GPIO_Port GPIOB
#define Q1C_Pin GPIO_PIN_13
#define Q1C_GPIO_Port GPIOB
#define Q1D_Pin GPIO_PIN_14
#define Q1D_GPIO_Port GPIOB
#define Q26_Pin GPIO_PIN_8
#define Q26_GPIO_Port GPIOD
#define Q25_Pin GPIO_PIN_9
#define Q25_GPIO_Port GPIOD
#define Q24_Pin GPIO_PIN_10
#define Q24_GPIO_Port GPIOD
#define Q23_Pin GPIO_PIN_11
#define Q23_GPIO_Port GPIOD
#define Q22_Pin GPIO_PIN_12
#define Q22_GPIO_Port GPIOD
#define Q21_Pin GPIO_PIN_13
#define Q21_GPIO_Port GPIOD
#define Q20_Pin GPIO_PIN_14
#define Q20_GPIO_Port GPIOD
#define Q29_Pin GPIO_PIN_15
#define Q29_GPIO_Port GPIOD
#define Q28_Pin GPIO_PIN_6
#define Q28_GPIO_Port GPIOC
#define Q27_Pin GPIO_PIN_7
#define Q27_GPIO_Port GPIOC
#define Q2A_Pin GPIO_PIN_8
#define Q2A_GPIO_Port GPIOC
#define Q2B_Pin GPIO_PIN_9
#define Q2B_GPIO_Port GPIOC
#define Q2C_Pin GPIO_PIN_8
#define Q2C_GPIO_Port GPIOA
#define Q2D_Pin GPIO_PIN_11
#define Q2D_GPIO_Port GPIOA
#define C_S_Pin GPIO_PIN_15
#define C_S_GPIO_Port GPIOA
#define FSYCN_Pin GPIO_PIN_4
#define FSYCN_GPIO_Port GPIOB
#define WIFI_RELOAD_Pin GPIO_PIN_8
#define WIFI_RELOAD_GPIO_Port GPIOB
#define WIFI_RESET_Pin GPIO_PIN_9
#define WIFI_RESET_GPIO_Port GPIOB
#define WIFI_READY_Pin GPIO_PIN_0
#define WIFI_READY_GPIO_Port GPIOE
#define WIFI_LINK_Pin GPIO_PIN_1
#define WIFI_LINK_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
