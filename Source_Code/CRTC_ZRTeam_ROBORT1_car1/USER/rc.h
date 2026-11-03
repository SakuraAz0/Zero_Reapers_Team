//
// rc.h —— 遥控失联保护（数据变化超时检测）
//
#ifndef __RC_H
#define __RC_H

#include "main.h"
#include "ps2.h"

// ===== 原理 =====
// 接收器与发射端失联后，通常会把"最后一帧"数据原样重复发出（字节完全不变）；
// 而真实手柄即使操作手不动，摇杆 LSB 也会有 ±1 抖动。
// 因此：连续 RC_STALE_TIMEOUT_MS 收到的数据（4 个摇杆 + 2 个按键字节）逐字节完全相同时，
//       判定为失联/无有效控制，停车等待；一旦数据发生变化立即恢复。
//
// 注意：若操作手把摇杆按住完全不动超过该时间，也会被判定为失联而停车（动一下即恢复）。
// 权衡：真手柄摇杆 LSB 会抖动，但有些 2.4G 接收器回的数据非常稳，保持摇杆不动就会被误判，
//       表现为"必须不停动摇杆才能维持匀速"。故默认放宽到 10s，只作弱兜底。
#define RC_STALE_TIMEOUT_MS   10000U

// ===== 上电安全启动 =====
// 上电（或解除使能）后，必须连续 RC_STARTUP_ARM_MS 读到"摇杆都在中心附近且数据合理"，
// 才允许驱动电机。避免接收器欠压/失联时回垃圾数据（四轴全 0x00 或全 0xFF）导致上电满速窜车。
#define RC_STARTUP_ARM_MS     200U
// 摇杆"中心附近"的原始码值容差（中心 127/128 都算）
#define RC_ARM_DEADZONE       20

typedef struct {
    uint8_t  has_last;          // 是否已建立参考帧
    uint8_t  stale;             // 1 = 当前处于失联保护（应停车）
    uint8_t  armed;             // 1 = 已通过安全启动，允许驱动
    uint16_t unchanged_ms;      // 数据已保持不变的时长
    uint32_t protect_count;     // 触发失联保护的累计次数
    uint32_t reject_count;      // 被判为垃圾数据/未回中的累计次数
    uint8_t  last[6];           // LX, LY, RX, RY, 按键高字节, 按键低字节
} RC_Debug_t;

extern volatile RC_Debug_t g_rc_dbg;

void    RC_Init(void);
// 强制退回"安全启动"：手柄掉出模拟模式、通信中断、或重新连上后调用，
// 这样恢复瞬间必须重新满足"摇杆回中 200ms"才允许驱动，避免突然窜车。
void    RC_ForceReArm(void);
// 每轮读到有效手柄数据后调用：返回 1 = 允许驱动，0 = 应立即停车
uint8_t RC_Feed(const PS2_Data_t *data);

#endif
