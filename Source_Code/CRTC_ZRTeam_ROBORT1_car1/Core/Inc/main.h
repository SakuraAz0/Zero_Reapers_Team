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
#include "stm32f1xx_hal.h"

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
void Bringup_Halt(uint8_t level);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PS2_DI_Pin GPIO_PIN_12
#define PS2_DI_GPIO_Port GPIOB
#define PS2_DO_Pin GPIO_PIN_13
#define PS2_DO_GPIO_Port GPIOB
#define PS2_CS_Pin GPIO_PIN_14
#define PS2_CS_GPIO_Port GPIOB
#define PS2_CLK_Pin GPIO_PIN_15
#define PS2_CLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// ===== 启动分步停机（调试用，定位"哪一步开始把电源拉垮"）=====
// 0 = 关闭（正常运行，默认）
// 1 = HAL_Init 之后
// 2 = SystemClock_Config 之后（已 72MHz，引脚尚未被驱动）
// 3 = MX_GPIO_Init 之后（PS2 四根线已配成输出/输入）
// 4 = MX_TIM2/TIM4_Init 之后（8 路 PWM 引脚已配成复用推挽输出）
// 5 = PS2_Init 之后
// 6 = Motor_Init 之后
// 用法：改这个数字 -> 重新 build + 烧录 -> 观察指示灯亮度 / 量电流
#ifndef BRINGUP_HALT_LEVEL
#define BRINGUP_HALT_LEVEL 0
#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
