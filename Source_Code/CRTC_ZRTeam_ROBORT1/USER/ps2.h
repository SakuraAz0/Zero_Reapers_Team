//
// Created by SukuraFalling on 2026/10/14.
// 修订：整包 CS 时序 + 正确的模式/ID 判定 + 掉线自恢复（详见 docs/PS2-诊断与修复方案.md）
//
#ifndef __PS2_H
#define __PS2_H

#include "main.h"

// ===== 引脚定义（与你的接线一致）=====
// 命名按模块丝印：DI = 手柄数据输出给 MCU（DAT），DO = MCU 发给手柄（CMD）
#define PS2_DI_PORT   GPIOB
#define PS2_DI_PIN    GPIO_PIN_12   // 手柄 -> MCU，上拉输入

#define PS2_DO_PORT   GPIOB
#define PS2_DO_PIN    GPIO_PIN_13   // MCU -> 手柄，推挽输出

#define PS2_CS_PORT   GPIOB
#define PS2_CS_PIN    GPIO_PIN_14   // ATT/CS，推挽输出

#define PS2_CLK_PORT  GPIOB
#define PS2_CLK_PIN   GPIO_PIN_15   // CLK，推挽输出

// ===== 时序参数（单位 us，参考 PS2X：CTRL_CLK=4~5）=====
// 若线较长或电平裕量不足，可把 PS2_CLK_US 放大到 10
#define PS2_CLK_US             5
#define PS2_BYTE_DELAY_US      10
// 包与包之间必须留间隔：参考实现用 read_delay(1~10ms)，廉价无线接收器在 10us 间隔下会丢包
#define PS2_PKT_DELAY_US       1000
// 连续读取失败多少次后自动重发配置序列（10ms 轮询下 50 次 ≈ 0.5s）
#define PS2_RECONFIG_FAILS     50
// 两次"自动拉回模拟模式"的最小间隔（连续失败/反复掉模式时用；掉模式第一次会无视这个间隔立刻拉回）
#ifndef PS2_RECONFIG_INTERVAL_MS
#define PS2_RECONFIG_INTERVAL_MS  200U
#endif
// 掉出模拟模式后连续几次都拉不回来 -> 升级为"整包重新初始化 PS2_Init()"（应对接收器/手柄卡死）
#ifndef PS2_MODE_ESCALATE_AFTER
#define PS2_MODE_ESCALATE_AFTER   3
#endif
// 上电后等待手柄/接收器稳定
#define PS2_BOOT_DELAY_MS      50
// 是否在配置序列里开启振动功能（用不到振动或无线接收器异常时改为 0；也可用 -D 覆盖）
// ⚠️ 现场出现过"手柄总是掉模式"，已按稳定性优先关闭振动（与车1 一致）：
//    不再发送振动配置/振动字节，减少对接收器的额外写操作。以后要用振动再改回 1。
#ifndef PS2_ENABLE_RUMBLE
#define PS2_ENABLE_RUMBLE      0
#endif

// ===== 手柄数据结构 =====
typedef struct {
    uint8_t  left_x;    // 左摇杆X轴 0~255，中心约 127/128
    uint8_t  left_y;    // 左摇杆Y轴
    uint8_t  right_x;   // 右摇杆X轴
    uint8_t  right_y;   // 右摇杆Y轴
    uint16_t buttons;   // 按键位：buf[3] 是低字节、buf[4] 是高字节
    uint8_t  mode;      // 0=数字模式(绿灯)，1=模拟模式(红灯)
} PS2_Data_t;

// ===== 按键位（原始数据为"低有效"：对应位 = 0 表示按下）=====
#define PS2_BTN_SELECT    0x0001u
#define PS2_BTN_L3        0x0002u
#define PS2_BTN_R3        0x0004u
#define PS2_BTN_START     0x0008u
#define PS2_BTN_UP        0x0010u
#define PS2_BTN_RIGHT     0x0020u
#define PS2_BTN_DOWN      0x0040u
#define PS2_BTN_LEFT      0x0080u
#define PS2_BTN_L2        0x0100u
#define PS2_BTN_R2        0x0200u
#define PS2_BTN_L1        0x0400u
#define PS2_BTN_R1        0x0800u
#define PS2_BTN_TRIANGLE  0x1000u
#define PS2_BTN_CIRCLE    0x2000u
#define PS2_BTN_CROSS     0x4000u
#define PS2_BTN_SQUARE    0x8000u

// 低有效：该位为 0 即按下
#define PS2_BtnDown(raw, btn)   ((((uint16_t)(raw)) & (uint16_t)(btn)) == 0u)

// ===== 调试可观测：CLion 调试时直接在 Watch 里加 g_ps2_dbg =====
typedef struct {
    uint8_t  raw[9];        // 最近一包的原始返回字节
    uint8_t  id;            // raw[1]：0x41/0x42/0x73/0x79
    uint8_t  ctrl_type;     // 手柄类型：0x03=有线DualShock，0x0C=2.4G无线
    uint8_t  valid;         // 最近一次读取是否有效
    uint8_t  init_ok;       // PS2_Init 是否成功进入模拟模式
    uint32_t ok_count;      // 有效包计数
    uint32_t fail_count;    // 无效包计数
    uint32_t reconfig_count;// 自动重配置次数（连续失败触发）
    uint32_t mode_recover_count; // 掉出模拟模式后被自动拉回的次数
} PS2_Debug_t;

extern volatile PS2_Debug_t g_ps2_dbg;

// ===== 函数声明 =====
void    PS2_Init(void);
uint8_t PS2_ReadData(PS2_Data_t *data);
// 设置振动：motor1 小马达(0/1)，motor2 大马达(0~255，非0即≥0x40)。
// 值会由之后每一次 PS2_ReadData 的轮询包持续刷新；PS2_Vibrate(0,0) 关闭。
void    PS2_Vibrate(uint8_t motor1, uint8_t motor2);

#endif
