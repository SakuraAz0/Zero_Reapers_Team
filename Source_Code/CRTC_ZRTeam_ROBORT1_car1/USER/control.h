//
// control.h —— 遥控操作映射（手柄数据 -> 底盘速度指令）v3
//
// 方案（2026-10-15 v3，按实测反馈修订）：
//   △ / ✕          : 档位 +1 / -1（慢 -> 中 -> 快，到顶/到底不再变），LED 指示见 led.c
//   START          : 制动（真刹车）
//   方向键 上/下/左/右 : 70% 速度平移（上下=前后，左右=左右平移；可组合成斜向）
//   左摇杆 Y        : 前后直行/后退；推杆幅度在当前档位区间内线性映射到速度
//   左摇杆 X        : 原地自转（绕底盘中心，纯旋转）
//   右摇杆 X        : 绕"对侧"后轮水平旋转（推左=绕右后轮，推右=绕左后轮）
//   右摇杆 Y        : 已清空（不使用）
//   L1/L2 R1/R2 □/○ : 舵机（见 servo.c）
//
// 档位（动力区间，千分比刻度 1000 = 满速）：
//   0 慢速 10%~30%   1 中速 30%~60%   2 快速 60%~92%（上电默认 1 中速）
//   全局上限 CTRL_DUTY_CAP = 92%：实测更高占空比会导致系统板复位甚至打火，故留余量。
//   **除制动外所有正常底盘动作（左摇杆 Y/X、方向键、右摇杆 X）都受档位控制**：
//   - 平移和自转（左摇杆 Y / 左摇杆 X / 方向键）：直接用档位区间 [lo, hi]
//   - 绕后轮（右摇杆 X）：档位映射到物理可用区间 CTRL_PIVOT_LIMIT_LO~HI（见下），
//     因为它天生要求 2:1 差速，低于 ~600 峰值时次快轮会低于马达起转阈值而转不动。
//
// 方向标定：实测哪个方向相反，就改对应的 SIGN 宏（见下），不用改逻辑。
//
#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "ps2.h"

// ===== 档位 =====
#define CTRL_GEAR_COUNT        3
#define CTRL_GEAR_DEFAULT      1        // 上电默认档位：1 = 中速

// 全局占空比上限：实测高档占空比会导致系统板复位甚至打火（电机电流把逻辑轨拉垮），
// 这里留 8% 余量，所有输出（平移/自转/绕后轮）都不会超过它。
#define CTRL_DUTY_CAP          920      // 92%

#define CTRL_GEAR0_LO          100      // 慢速档下限 10%
#define CTRL_GEAR0_HI          300      // 慢速档上限 30%
#define CTRL_GEAR1_LO          300      // 中速档下限 30%
#define CTRL_GEAR1_HI          600      // 中速档上限 60%（现场要求：中速档最大降到 60%）
#define CTRL_GEAR2_LO          600      // 快速档下限 60%（与中速档首尾相接，无重叠）
#define CTRL_GEAR2_HI          850      // 快速档上限 85%（现场要求：快速档最大降到 85%；全局 92% 上限仍保留）
// 换档去抖：两次换档至少间隔这么久（按住不会连续跳档）
#define CTRL_GEAR_LOCKOUT_MS   250U

// 旋转类动作（右摇杆 X 绕后轮）的输出上限：不用档位原始值，而是映射到"物理可用区间"。
// 原因（重要）：绕后轮时四轮速度比值固定为 2 : 1.08 : 0.92 : 0（被绕的那个轮子恒为 0），
//   "次快轮"永远只有峰值轮的 ~54%。TT 马达齿轮箱静摩擦约需 ≥30% 占空才起转，
//   所以峰值轮至少要 ~600 才能让三只轮子都动起来（慢速档的 300 做不到，会退化成"只有一个轮子转"）。
//   同时峰值轮 = 2 × ω，不得超过全局占空比上限 92% → ω 上限对应 920。
//   于是绕转档位映射为：慢速 -> 600，中速 -> ~754，快速 -> 920（档位依然控制绕转快慢，1.5 倍范围）。
// 想让慢速档绕转更慢：调小 CTRL_PIVOT_LIMIT_LO（但可能又转不动）。
#define CTRL_PIVOT_LIMIT_LO    600
#define CTRL_PIVOT_LIMIT_HI    CTRL_DUTY_CAP

// ===== 摇杆/方向键 =====
#define CTRL_DEADZONE          10       // 死区（原始码值；PS2 惯例：推上/推左 = 小值）
#define CTRL_SCALE             1000     // 摇杆满杆对应的刻度
#define CTRL_DPAD_SPEED        700      // 方向键平移幅度（70%，仍受档位上限约束）

// ===== 方向标定（反了就改符号）=====
#define CTRL_LY_SIGN           (+1)     // 左摇杆 前后：+1 = 推前为正
#define CTRL_LX_SIGN           (+1)     // 左摇杆 左右（原地自转）
#define CTRL_RX_SIGN           (+1)     // 右摇杆 左右（绕后轮）
#define CTRL_PIVOT_SIGN        (+1)     // 绕后轮旋转的方向（推左/推右的转向反了就翻）

// 1 = 绕"对侧"后轮（推左 -> 绕右后轮）；0 = 绕同侧后轮
#ifndef CTRL_PIVOT_OPPOSITE_SIDE
#define CTRL_PIVOT_OPPOSITE_SIDE 1
#endif

// ===== 底盘几何（毫米，按实车量一下填进来）=====
#define CTRL_HALF_LEN_MM       70       // 中心到前/后轴中心的距离（半个轴距）
#define CTRL_HALF_WID_MM       60       // 中心到左/右轮中心的距离（半个轮距）

typedef enum {
    CTRL_MODE_IDLE = 0,     // 未使能/无输入
    CTRL_MODE_STICK,        // 摇杆
    CTRL_MODE_DPAD,         // 方向键平移
    CTRL_MODE_BRAKE         // START 制动
} Ctrl_Mode_t;

typedef struct {
    int16_t     vx;         // 前后（+前进）
    int16_t     vy;         // 左右平移
    int16_t     vw;         // 旋转
    uint8_t     brake;      // 1 = 调用 Motor_BrakeAll()
    uint8_t     gear;       // 当前档位 0..CTRL_GEAR_COUNT-1
    int16_t     limit;      // 本次输出的上限（交给 Mecanum_MoveLimit）
    Ctrl_Mode_t mode;       // 当前生效的操作源（调试可观测）
} Ctrl_Output_t;

void    Ctrl_Init(void);
// ps2: 手柄数据；enabled: 1=允许驱动（已通过失联/安全启动检查）
// 返回：1 = 可以驱动（out 有效）；0 = 应停车
uint8_t Ctrl_Update(const PS2_Data_t *ps2, uint8_t enabled, Ctrl_Output_t *out);

uint8_t Ctrl_GetGear(void);
int16_t Ctrl_GetGearLimit(void);
const char *Ctrl_ModeName(Ctrl_Mode_t mode);

#endif
