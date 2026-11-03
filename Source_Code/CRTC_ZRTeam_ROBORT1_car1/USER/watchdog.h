//
// watchdog.h —— 独立看门狗（IWDG）
//
// 作用：软件卡死 / 程序跑飞 / 强干扰导致控制流异常时，自动复位芯片，
//       避免小车"原地僵住或带着最后的 PWM 冲出去"、只能靠人去拔电池。
//
// 参数（STM32F103 的 IWDG 由独立 LSI 时钟驱动，不依赖主时钟/PLL）：
//   超时 = 4 × 2^PRESCALER × RELOAD / F_LSI
//   默认 PR=4(/64)、RELOAD=625 -> 625/625 = **标称 1.0 秒**
//   注意 LSI 实际 30~60kHz（典型 40kHz），所以真实超时约 0.67 ~ 1.33 秒。
//   想改 2 秒：RELOAD 改 1250；想改 1.5 秒：RELOAD 改 937。
//
// 重要特性：
//   1) 一旦启动，**软件无法关闭**（只能靠复位关闭）——这正是它的安全价值；
//   2) 主循环必须每轮喂狗（Watchdog_Feed），否则到点就复位；
//   3) 初始化里会置 DBGMCU_CR.DBG_IWDG_STOP：调试器暂停内核时冻结 IWDG，
//      这样你在线调试单步/暂停时不会被看门狗踢复位。
//
#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include "main.h"

/* 置 0 可整体关闭（台架自检、或需要用调试器长时间暂停时临时关掉） */
#ifndef WATCHDOG_ENABLE
#define WATCHDOG_ENABLE     1
#endif

/* IWDG_PR 编码：0=/4 1=/8 2=/16 3=/32 4=/64 5=/128 6=/256 */
#define WATCHDOG_PRESCALER  4       /* /64 -> 计数频率 625Hz */
#define WATCHDOG_RELOAD     625     /* 625 / 625Hz = 1.0s */

/* 在初始化最末尾、进入主循环之前调用 */
void Watchdog_Init(void);
/* 主循环每轮调用一次（本工程主循环 10ms 一轮，1s 超时余量充足） */
void Watchdog_Feed(void);

#endif
