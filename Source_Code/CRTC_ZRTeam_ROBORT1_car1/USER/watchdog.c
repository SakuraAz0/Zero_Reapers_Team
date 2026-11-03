//
// Created by SukuraFalling on 2026/10/15.
// 独立看门狗（IWDG）：直接用寄存器配置，不依赖 CubeMX 生成代码，重新生成也不会丢。
//
#include "watchdog.h"

#if WATCHDOG_ENABLE

void Watchdog_Init(void)
{
    /* 调试器暂停内核时冻结 IWDG：否则一暂停(halt) 1 秒后就复位，调试断连 */
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

    IWDG->KR  = 0x5555U;                    /* 解锁 PR/RLR 寄存器 */
    IWDG->PR  = WATCHDOG_PRESCALER;         /* /64 -> 625Hz */
    IWDG->RLR = WATCHDOG_RELOAD;            /* 625 计数 -> 1.0s */
    IWDG->KR  = 0xAAAAU;                    /* 装入计数值 */
    IWDG->KR  = 0xCCCCU;                    /* 启动（启动后软件不能关闭） */
}

void Watchdog_Feed(void)
{
    IWDG->KR = 0xAAAAU;                     /* 重装载（喂狗） */
}

#else

void Watchdog_Init(void) { }                /* 关闭时不启动看门狗 */
void Watchdog_Feed(void) { }

#endif
