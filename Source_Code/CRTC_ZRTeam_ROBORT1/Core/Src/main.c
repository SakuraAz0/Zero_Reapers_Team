/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ps2.h"
#include "motor.h"
#include "rc.h"
#include "control.h"
#include "led.h"
#include "servo.h"
#include "watchdog.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 诊断开关：置 1 则上电时用慢速 LED(PA8) 闪码显示"上次复位原因"（不接调试器也能看）。
 *   闪 1 次 = 只有 PINRSTF          -> 复位脚(NRST)被干扰拉低（EMI，电源没问题）
 *   闪 2 次 = PORRSTF 且无 PIN      -> 上电/欠压类（供电被切断/塌陷）
 *   闪 3 次 = PIN + SFTRSTF         -> 调试器/烧录器发起的复位（正常，不是故障）
 *   闪 4 次 = POR + PIN             -> 断电式复位（既掉电又过复位脚）
 *   闪 5 次 = 其它（未知组合）
 *   闪 6 次 = IWDGRSTF 独立看门狗   -> **程序卡死超过 1 秒**（软件问题，不是供电！）
 *   闪 7 次 = WWDG 窗口看门狗
 * 每次开机闪 BOOT_RESET_CAUSE_BLINK_ROUNDS 遍，每遍之间停 BOOT_RESET_CAUSE_BLINK_GAP_MS 毫秒；
 * ⭐ 目前为 0（已关闭）：正常使用时不闪灯、上电不额外等待。
 *    以后要排查复位原因时改成 1 即可（g_reset_cause 的锁存逻辑一直保留，调试器仍可 Watch）。 */
#define BOOT_RESET_CAUSE_BLINK  0
/* 开机闪码重复遍数（3 轮，方便数清楚） */
#define BOOT_RESET_CAUSE_BLINK_ROUNDS  3
/* 每轮之间的间隔（ms）：1.5 秒，足够分清"一组闪完"和"下一组开始" */
#define BOOT_RESET_CAUSE_BLINK_GAP_MS  1500

/* ===== 主循环节拍（ms）=====
 * 每隔这么久读一次 PS2，并用同一个节拍驱动：底盘控制、舵机插值 (Servo_Task)、软启动斜坡。
 * ⚠️ 现场问题：5ms（200Hz）轮询时手柄**总是掉模式**（红灯->绿灯）。
 *    廉价 2.4G 接收器/手柄跟不上 200Hz 的包流，会自己退回数字模式。
 *    因此改回 **10ms（100Hz）**——这也是手柄原生的帧周期，最稳；代价是输入延迟多约 5ms。
 * ⚠️ 改这里必须同步改 USER/servo.h 的 SERVO_TICK_MS（舵机速度按它换算）
 *    和 USER/motor.h 的 MOTOR_SLEW_STEP（软启动斜坡时间按它换算）。 */
#define PS2_READ_PERIOD_MS  10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 上次复位原因（RCC->CSR 的复位标志位），调试器里 Watch 这个变量即可。
 * 0x08000000=PORRSTF 上电/欠压类  0x04000000=PINRSTF 复位脚(NRST)被拉低(干扰)
 * 0x10000000=SFTRSTF 软件复位    0x20000000=IWDGRSTF 独立看门狗
 * 上电首次通常为 0x0C000000（POR + PIN 同时置位） */
volatile uint32_t g_reset_cause;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 锁存"上次复位原因"再清除标志：RCC->CSR 的复位标志只能软件清，
   * 本工程其它地方都不清，所以必须在这里"先读后清"，否则上电那次 POR 标志会一直留着、
   * 后面任何复位都读不出真实原因。清完后，下一次复位的标志就会被下次启动准确捕获。 */
  g_reset_cause = (uint32_t)(RCC->CSR & 0xFE000000u);
  __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Bringup_Halt(1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  Bringup_Halt(2);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  PS2_Init();
  Bringup_Halt(5);
  Motor_Init();
  Bringup_Halt(6);

  LED_Init();          // 档位指示 LED：PA8=慢 PA11=中 PA12=快
#if BOOT_RESET_CAUSE_BLINK
  {
      /* 用闪码显示上次复位原因（见 USER CODE PD 的说明） */
      uint8_t code;

      if (g_reset_cause & 0x08000000u)                                   /* POR 置位（掉电/欠压） */
          code = (g_reset_cause & 0x04000000u) ? 4u : 2u;
      else if (g_reset_cause & 0x04000000u)                              /* PIN：NRST 被拉低 */
          code = (g_reset_cause & 0x10000000u) ? 3u : 1u;
      else if (g_reset_cause & 0x10000000u)                              /* SFT：软件复位 */
          code = 3u;
      else if (g_reset_cause & 0x20000000u)                              /* IWDG：看门狗（程序卡死！） */
          code = 6u;
      else if (g_reset_cause & 0x40000000u)                              /* WWDG */
          code = 7u;
      else
          code = 5u;                                                     /* 未知/其它 */

      for (uint8_t r = 0; r < BOOT_RESET_CAUSE_BLINK_ROUNDS; r++)
      {
          LED_BlinkCode(code);
          if ((uint8_t)(r + 1u) < BOOT_RESET_CAUSE_BLINK_ROUNDS)
              HAL_Delay(BOOT_RESET_CAUSE_BLINK_GAP_MS);   // 轮间隔（最后一轮不再等）
      }
  }
#endif
  Servo_Init();        // 舵机 PWM：TIM3 (PA6/PA7/PB0/PB1)，50Hz
  Ctrl_Init();
  RC_Init();

  /* 看门狗放在所有初始化之后启动：避免 PS2_Init/自检等长流程被误判为死机。
   * 启动后主循环每轮喂一次，1 秒内没喂 -> 自动复位（重新进入安全启动）。 */
  Watchdog_Init();

  PS2_Data_t ps2;
  uint32_t last_read = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { // 每 PS2_READ_PERIOD_MS 毫秒读一次手柄（同时作为控制/舵机/软启动的 tick）
    Watchdog_Feed();     // 喂狗：本循环远快于 1s，不会误复位

    if (HAL_GetTick() - last_read >= PS2_READ_PERIOD_MS)
    {
      last_read = HAL_GetTick();

      if (PS2_ReadData(&ps2))
      {
        Ctrl_Output_t cmd;

        if (ps2.mode == 1)
        {
          /* 红灯(模拟)模式：舵机 + 底盘 */
          uint8_t enabled = RC_Feed(&ps2);          // 失联保护/安全启动

          Servo_Task(&ps2);                         // 四路机械臂：MG995 看右摇杆Y，其余看按键

          if (Ctrl_Update(&ps2, enabled, &cmd))
          {
            if (cmd.brake)
              Motor_BrakeAll();                     // START：真刹车
            else
#if MOTOR_SLEW_ENABLE
              Mecanum_MoveLimitSlew(cmd.vx, cmd.vy, cmd.vw, cmd.limit);  // 软启动 + 档位限幅
#else
              Mecanum_MoveLimit(cmd.vx, cmd.vy, cmd.vw, cmd.limit);      // 按档位限幅
#endif
          }
          else
          {
            Motor_StopAll();
          }

          LED_SetGear(cmd.gear);                    // 档位指示
        }
        else
        {
          /* 掉出模拟模式（绿灯）：驱动会自动重发配置把它拉回红灯模式；
             这里停车并强制重新安全启动，避免拉回瞬间窜车 */
          Servo_Task(&ps2);
          Motor_StopAll();
          RC_ForceReArm();

          (void)Ctrl_Update(&ps2, 0, &cmd);         // 仅为刷新档位显示
          LED_SetGear(cmd.gear);
        }
      }
      else
      {
        /* 通信失败：停车并强制重新安全启动 */
        Motor_StopAll();
        RC_ForceReArm();
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 启动分步停机：当 level == BRINGUP_HALT_LEVEL 时停机（关中断死循环），
// 用于观察"走到哪一步开始把电源拉垮"。正常使用请把 BRINGUP_HALT_LEVEL 保持为 0。
void Bringup_Halt(uint8_t level)
{
  if (level != BRINGUP_HALT_LEVEL)
  {
    return;
  }
  __disable_irq();
  while (1)
  {
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
