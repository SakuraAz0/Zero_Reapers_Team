/*
 * 主机端回归测试：不接硬件、不依赖 ARM 工具链，直接在 PC 上跑 PS2 驱动 + 电机映射逻辑。
 *
 * 编译运行（在工程根目录执行，使用任意本机 gcc）：
 *   gcc -std=c11 -Wall -Wextra -DSTM32F103xB -DUSE_HAL_DRIVER ^
 *       -ICore/Inc -IDrivers/STM32F1xx_HAL_Driver/Inc -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy ^
 *       -IDrivers/CMSIS/Device/ST/STM32F1xx/Include -IDrivers/CMSIS/Include -IUSER ^
 *       tests/host_test.c -o tests/host_test.exe && tests/host_test.exe
 *
 * 原理：用假的 DWT/CoreDebug（避免访问 0xE0001xxx），把 SystemCoreClock 设为 0 让
 *       delay_us() 立即返回；用 HAL_GPIO_* 桩函数实现一个"PS2 手柄模型"，
 *       手柄按参考协议（CS 下降沿开始一包、CLK 下降沿采样 DO 并送出 DI、LSB first）
 *       与驱动对打，从而验证整包时序、位序、ID/mode 判定与电机通道映射。
 *
 * 该目录不参与固件构建（根 CMakeLists.txt 未引用 tests/）。
 */
#include <stdio.h>
#include <string.h>
#include "ps2.h"
#include "motor.h"
#include "rc.h"
#include "control.h"
#include "led.h"
#include "servo.h"
#include "watchdog.h"

/* ================= 伪造内核调试单元 ================= */
static DWT_Type       g_fake_dwt;
static CoreDebug_Type g_fake_coredebug;
#undef DWT
#define DWT (&g_fake_dwt)
#undef CoreDebug
#define CoreDebug (&g_fake_coredebug)

/* ================= 伪造 RCC / TIM3（servo.c、led.c 会直接写寄存器）================= */
static RCC_TypeDef g_fake_rcc;
#undef RCC
#define RCC (&g_fake_rcc)

static TIM_TypeDef g_fake_tim3;
#undef TIM3
#define TIM3 (&g_fake_tim3)

/* ================= 伪造 IWDG / DBGMCU（watchdog.c 直接写寄存器）================= */
static IWDG_TypeDef   g_fake_iwdg;
static DBGMCU_TypeDef g_fake_dbgmcu;
#undef IWDG
#define IWDG (&g_fake_iwdg)
#undef DBGMCU
#define DBGMCU (&g_fake_dbgmcu)

uint32_t SystemCoreClock = 0U;   /* =0 使 delay_us() 的 ticks 为 0，立即返回 */

/* ================= 假的定时器实例 ================= */
static TIM_TypeDef g_fake_tim2;
static TIM_TypeDef g_fake_tim4;
TIM_HandleTypeDef htim2;         /* 对应 Core/Src/tim.c 中的定义 */
TIM_HandleTypeDef htim4;

/* ================= PS2 手柄模型 ================= */
#define PIN_DI   GPIO_PIN_12
#define PIN_DO   GPIO_PIN_13
#define PIN_CS   GPIO_PIN_14
#define PIN_CLK  GPIO_PIN_15

static int s_cs = 1, s_clk = 1, s_do = 1, s_di = 1;

/* 手柄可配置状态 */
static uint8_t pad_id        = 0x73;                        /* 0x73=模拟红灯 0x41=数字绿灯 0xFF=掉线 */
static uint8_t pad_stick[4]  = {0x7F, 0x80, 0x00, 0xFF};    /* RX, RY, LX, LY */
static uint8_t pad_btn[2]    = {0xF3, 0x0F};                /* buf[3]=低字节, buf[4]=高字节 */
static uint8_t pad_type      = 0x03;                        /* 0x03=有线 0x0C=2.4G无线 */

/* 手柄侧观测 */
static uint8_t s_rx[9];             /* 手柄收到的命令字节 */
static uint8_t s_di_bytes[9];       /* 手柄发出的响应字节 */
static int     s_bit;
static uint8_t s_cur_tx;
static int     s_window_bytes;      /* 当前 CS 低电平窗口内已收到多少字节 */
static int     s_last_window_len;   /* 上一包在 CS 低电平内收到多少字节 */
static int     s_packets;

static uint8_t pad_resp_byte(int idx)
{
    if (s_rx[0] != 0x01) return 0xFF;          /* 包头不对 -> 不响应 */

    switch (idx) {
    case 0: return 0xFF;
    case 1: return pad_id;                     /* 读取模式：模式/型号 ID */
    case 2: return 0x5A;
    case 3: return (s_rx[1] == 0x45) ? pad_type : pad_btn[0];
    case 4: return (s_rx[1] == 0x45) ? 0x01     : pad_btn[1];
    case 5: return pad_stick[0];
    case 6: return pad_stick[1];
    case 7: return pad_stick[2];
    case 8: return pad_stick[3];
    default: return 0x00;
    }
}

static void pad_on_cs(int level)
{
    if (level == 0) {                          /* CS 下降沿：一包开始 */
        memset(s_rx, 0, sizeof s_rx);
        memset(s_di_bytes, 0, sizeof s_di_bytes);
        s_window_bytes = 0;
        s_bit = 0;
        s_cur_tx = 0;
    } else {                                   /* CS 上升沿：一包结束 */
        s_last_window_len = s_window_bytes;
        if (s_window_bytes > 0) s_packets++;
    }
}

static void pad_on_clk_falling(void)
{
    if (s_cs != 0) return;                     /* CS 为高时手柄忽略时钟 */

    if (s_do) s_cur_tx |= (uint8_t)(1u << s_bit);

    s_di_bytes[s_window_bytes < 9 ? s_window_bytes : 8] = pad_resp_byte(s_window_bytes);
    s_di = (s_di_bytes[s_window_bytes < 9 ? s_window_bytes : 8] >> s_bit) & 1u;

    if (++s_bit == 8) {
        if (s_window_bytes < 9) s_rx[s_window_bytes] = s_cur_tx;
        s_cur_tx = 0;
        s_bit = 0;
        if (s_window_bytes < 9) s_window_bytes++;
    }
}

/* ================= HAL 桩 ================= */
static uint16_t s_gpio_a_odr = 0;    /* 记录 PA 口输出（档位 LED 用） */

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    int v = (PinState == GPIO_PIN_RESET) ? 0 : 1;

    if (GPIOx == GPIOA) {            /* 档位 LED：PA8/PA11/PA12 */
        if (v) s_gpio_a_odr |= GPIO_Pin;
        else   s_gpio_a_odr &= (uint16_t)~GPIO_Pin;
        return;
    }

    switch (GPIO_Pin) {
    case PIN_CS:
        if (s_cs != v) { s_cs = v; pad_on_cs(v); }
        break;
    case PIN_CLK:
        if (s_clk != v) { s_clk = v; if (v == 0) pad_on_clk_falling(); }
        break;
    case PIN_DO:
        s_do = v;
        break;
    default:
        break;
    }
}

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    (void)GPIOx; (void)GPIO_Init;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    (void)GPIOx; (void)GPIO_Pin;
    return s_di ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    (void)htim; (void)Channel;
    return HAL_OK;
}
void HAL_Delay(uint32_t Delay) { (void)Delay; }

/* 可控 tick，用于失联保护的时间测试 */
static uint32_t s_tick = 0;
uint32_t HAL_GetTick(void) { return s_tick; }

/* ================= 被测代码 ================= */
#include "../USER/ps2.c"
#include "../USER/motor.c"
#include "../USER/rc.c"
#include "../USER/control.c"
#include "../USER/led.c"
#include "../USER/servo.c"
#include "../USER/watchdog.c"

/* 麦轮逆解参考实现（仅供测试校验用） */
static int16_t wheel_cmd(int16_t vx, int16_t vy, int16_t vw, int which)
{
    switch (which) {
    case 0:  return (int16_t)(vx + vy + vw);   /* LF */
    case 1:  return (int16_t)(vx - vy + vw);   /* LR */
    case 2:  return (int16_t)(vx - vy - vw);   /* RF */
    default: return (int16_t)(vx + vy - vw);   /* RR */
    }
}

/* ================= 舵机测试辅助 ================= */
/* Servo_Task 现在接收整包手柄数据（MG995 要看右摇杆 Y），这里包一层方便测试 */
static PS2_Data_t s_servo_pad;

static void servo_pad_reset(void)
{
    memset(&s_servo_pad, 0, sizeof s_servo_pad);
    s_servo_pad.mode    = 1;
    s_servo_pad.left_x  = 0x80;
    s_servo_pad.left_y  = 0x80;
    s_servo_pad.right_x = 0x80;
    s_servo_pad.right_y = 0x80;
    s_servo_pad.buttons = 0xFFFF;      /* 无按键（低有效） */
}

/* 只按某些键、摇杆回中 */
static void servo_task_buttons(uint16_t buttons)
{
    servo_pad_reset();
    s_servo_pad.buttons = buttons;
    Servo_Task(&s_servo_pad);
}

/* 右摇杆 Y 给指定原始值（0x00 = 推到最前，0xFF = 拉到最底，0x80 = 中位），无按键 */
static void servo_task_stick(uint8_t ry_raw)
{
    servo_pad_reset();
    s_servo_pad.right_y = ry_raw;
    Servo_Task(&s_servo_pad);
}

/* 摇杆 + 按键同时给（验证"错峰：同时输入时只执行优先级最高的一路"） */
static void servo_task_stick_and_buttons(uint8_t ry_raw, uint16_t buttons)
{
    servo_pad_reset();
    s_servo_pad.right_y = ry_raw;
    s_servo_pad.buttons = buttons;
    Servo_Task(&s_servo_pad);
}

/* 把"配置值"按通道限幅到 [lo,hi]，用于断言上电姿态（配置可能故意超出安全行程） */
static uint16_t expect_clamped(int32_t us, uint16_t lo, uint16_t hi)
{
    if (us < (int32_t)lo) return lo;
    if (us > (int32_t)hi) return hi;
    return (uint16_t)us;
}

/* ================= 测试框架 ================= */
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        if (cond) { g_pass++; printf("  [PASS] %s\n", msg); }               \
        else      { g_fail++; printf("  [FAIL] %s  (%s:%d)\n", msg, __FILE__, __LINE__); } \
    } while (0)

static void reset_pad(void)
{
    s_cs = 1; s_clk = 1; s_do = 1; s_di = 1;
    s_window_bytes = 0; s_last_window_len = -1; s_bit = 0; s_cur_tx = 0;
    s_packets = 0;
    memset(s_rx, 0, sizeof s_rx);
    memset(s_di_bytes, 0, sizeof s_di_bytes);
    memset((void *)&g_ps2_dbg, 0, sizeof g_ps2_dbg);
    s_fail_streak = 0;
    s_rumble_small = 0;
    s_rumble_large = 0;
}

int main(void)
{
    PS2_Data_t d;

    /* ---------- 1. 红灯(模拟)模式：读取成功 + 整包时序 + 位序 ---------- */
    printf("[1] 红灯(模拟)模式\n");
    reset_pad();
    pad_id = 0x73;
    pad_stick[0] = 0x7F; pad_stick[1] = 0x80; pad_stick[2] = 0x00; pad_stick[3] = 0xFF;
    pad_btn[0] = 0xF3;   pad_btn[1] = 0x0F;

    PS2_Init();
    CHECK(g_ps2_dbg.ctrl_type == pad_type, "type_read 解析出手柄类型 0x03");
    CHECK(g_ps2_dbg.init_ok == 1, "PS2_Init 成功进入模拟模式");

    memset(s_rx, 0, sizeof s_rx);
    memset(s_di_bytes, 0, sizeof s_di_bytes);
    CHECK(PS2_ReadData(&d) == 1, "PS2_ReadData 返回成功");
    CHECK(d.mode == 1, "识别为模拟(红灯)模式");
    CHECK(s_last_window_len == 9, "一包 9 字节在同一个 CS 低电平窗口内完成");
    CHECK(s_rx[0] == 0x01 && s_rx[1] == 0x42, "驱动发出的命令字节正确(0x01 0x42)");
    CHECK(s_rx[2] == 0x00 && s_rx[8] == 0x00, "尾部填充字节为 0（位序 LSB first 正确）");
    CHECK(d.right_x == 0x7F && d.right_y == 0x80 && d.left_x == 0x00 && d.left_y == 0xFF,
          "摇杆字节映射正确(buf[5..8])");
    CHECK(d.buttons == 0x0FF3, "按键解码正确(buf[4]<<8)|buf[3]，低字节在前)");
    CHECK(memcmp((const void *)g_ps2_dbg.raw, s_di_bytes, 9) == 0, "g_ps2_dbg.raw 与手柄返回一致");

    /* ---------- 2. 数字(绿灯)模式：必须回中，不能窜车 ---------- */
    printf("[2] 数字(绿灯)模式安全性\n");
    reset_pad();
    pad_id = 0x41;
    for (int i = 0; i < 4; i++) pad_stick[i] = 0xFF;   /* 数字模式下摇杆字节通常为 0xFF */
    CHECK(PS2_ReadData(&d) == 1, "数字模式仍算通信成功");
    CHECK(d.mode == 0, "判定为数字模式(mode=0)");
    CHECK(d.left_x == 0x80 && d.left_y == 0x80 && d.right_x == 0x80 && d.right_y == 0x80,
          "摇杆强制回中(0x80)，不会输出满速");

    /* ---------- 3. 掉线：回中 + 自动重配置 ---------- */
    printf("[3] 掉线自恢复\n");
    reset_pad();
    pad_id = 0xFF;                                     /* 手柄不响应 */
    for (int i = 0; i < PS2_RECONFIG_FAILS; i++)
        (void)PS2_ReadData(&d);
    CHECK(d.mode == 0 && d.left_y == 0x80, "掉线时回中");
    CHECK(g_ps2_dbg.reconfig_count >= 1, "连续失败达到阈值后自动重配置");

    /* ---------- 4. PWM 频率与通道映射 ---------- */
    printf("[4] PWM 刻度与电机通道映射\n");
    htim2.Instance = &g_fake_tim2;
    htim4.Instance = &g_fake_tim4;
    memset(&g_fake_tim2, 0, sizeof g_fake_tim2);
    memset(&g_fake_tim4, 0, sizeof g_fake_tim4);

    Motor_Init();
    CHECK(g_fake_tim2.PSC == 71 && g_fake_tim2.ARR == 999, "TIM2 PSC=71/ARR=999 -> 1kHz");
    CHECK(g_fake_tim4.PSC == 71 && g_fake_tim4.ARR == 999, "TIM4 PSC=71/ARR=999 -> 1kHz");
    CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim2.CCR2 == 0 && g_fake_tim2.CCR3 == 0 && g_fake_tim2.CCR4 == 0 &&
          g_fake_tim4.CCR1 == 0 && g_fake_tim4.CCR2 == 0 && g_fake_tim4.CCR3 == 0 && g_fake_tim4.CCR4 == 0,
          "上电初始占空比为 0");

    Motor_SetPWM(MOTOR_LF, 500);
    CHECK(g_fake_tim2.CCR1 == 500 && g_fake_tim2.CCR2 == 0, "左前正转: PA0=PWM, PA1=0");
    Motor_SetPWM(MOTOR_LF, -500);
    CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim2.CCR2 == 500, "左前反转: PA0=0, PA1=PWM");
    Motor_SetPWM(MOTOR_LF, 5000);
    CHECK(g_fake_tim2.CCR1 == 1000, "超程限幅到 1000");

    Motor_StopAll();
    Mecanum_Move(100, 0, 0);                            /* 前进：四轮同向 */
    CHECK(g_fake_tim2.CCR1 == 100 && g_fake_tim2.CCR3 == 100 &&
          g_fake_tim4.CCR1 == 100 && g_fake_tim4.CCR3 == 100 &&
          g_fake_tim2.CCR2 == 0 && g_fake_tim2.CCR4 == 0 &&
          g_fake_tim4.CCR2 == 0 && g_fake_tim4.CCR4 == 0,
          "前进时四个轮子同向 +100");

    Motor_StopAll();
    Mecanum_Move(0, 100, 0);                            /* 右移：LF/RR 正、LR/RF 反 */
    CHECK(g_fake_tim2.CCR1 == 100 && g_fake_tim2.CCR4 == 100 &&
          g_fake_tim4.CCR2 == 100 && g_fake_tim4.CCR3 == 100 &&
          g_fake_tim2.CCR3 == 0 && g_fake_tim4.CCR1 == 0,
          "平移时 LF/RR 正转、LR/RF 反转（对角配对）");

    Motor_StopAll();
    Mecanum_Move(0, 0, 300);                            /* 旋转：LF/LR 与 RF/RR 反向 */
    CHECK(g_fake_tim2.CCR1 == 300 && g_fake_tim2.CCR3 == 300 &&
          g_fake_tim4.CCR2 == 300 && g_fake_tim4.CCR4 == 300 &&
          g_fake_tim2.CCR2 == 0 && g_fake_tim2.CCR4 == 0 &&
          g_fake_tim4.CCR1 == 0 && g_fake_tim4.CCR3 == 0,
          "旋转时左侧与右侧反向");

    Motor_StopAll();
    CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim2.CCR2 == 0 && g_fake_tim2.CCR3 == 0 && g_fake_tim2.CCR4 == 0 &&
          g_fake_tim4.CCR1 == 0 && g_fake_tim4.CCR2 == 0 && g_fake_tim4.CCR3 == 0 && g_fake_tim4.CCR4 == 0,
          "Motor_StopAll 全部归零");

    /* ---------- 5. 振动刷新与饱和等比缩放 ---------- */
    printf("[5] 振动刷新与饱和处理\n");
#if PS2_ENABLE_RUMBLE
    reset_pad();
    pad_id = 0x73;
    PS2_Vibrate(1, 0x80);
    (void)PS2_ReadData(&d);
    CHECK(s_rx[3] == 0x01 && s_rx[4] == 0x80, "振动值随每次轮询包刷新(小=0x01,大=0x80)");
    PS2_Vibrate(0, 0x10);                              /* 0x10 < 0x40 应抬到 0x40 */
    (void)PS2_ReadData(&d);
    CHECK(s_rx[3] == 0x00 && s_rx[4] == 0x40, "PS2_Vibrate(0,0x10) -> 大马达抬到 0x40");
#endif

    Motor_StopAll();
    Mecanum_Move(1000, 0, 1000);                       /* LF/LR 需求 2000 -> 等比缩到 0.5 */
    CHECK(g_fake_tim2.CCR1 == 1000 && g_fake_tim2.CCR3 == 1000 &&
          g_fake_tim4.CCR1 == 0 && g_fake_tim4.CCR2 == 0 &&
          g_fake_tim4.CCR3 == 0 && g_fake_tim4.CCR4 == 0,
          "饱和时等比缩放: 2000/2000/0/0 -> 1000/1000/0/0");

    Motor_StopAll();
    Mecanum_Move(1000, 500, 0);                        /* 需求 1500/500/500/1500 -> 比例保持 3:1:1:3 */
    CHECK(g_fake_tim2.CCR1 == 1000 && g_fake_tim2.CCR3 == 333 &&
          g_fake_tim4.CCR1 == 333 && g_fake_tim4.CCR3 == 1000,
          "饱和时四轮比例不变(LF:LR:RF:RR = 3:1:1:3)");

    /* ---------- 6. 上电安全启动 + 失联保护 ---------- */
    printf("[6] 上电安全启动 与 失联保护\n");
    reset_pad();
    pad_id = 0x73;
    for (int i = 0; i < 4; i++) pad_stick[i] = 0x80;   /* 摇杆回中 */
    pad_btn[0] = 0x00; pad_btn[1] = 0x00;

    s_tick = 0;
    RC_Init();
    (void)PS2_ReadData(&d);

    CHECK(RC_Feed(&d) == 0, "上电后未完成安全启动前不允许驱动");
    s_tick = 100;
    CHECK(RC_Feed(&d) == 0, "回中持续 <200ms 仍不允许");
    s_tick = RC_STARTUP_ARM_MS + 50;
    CHECK(RC_Feed(&d) == 1, "回中持续 >=200ms 后允许驱动");
    CHECK(g_rc_dbg.armed == 1, "armed 标志置位");

    /* 垃圾数据：四轴全 0x00 -> 立即解除使能并停车 */
    d.left_x = 0x00; d.left_y = 0x00; d.right_x = 0x00; d.right_y = 0x00;
    CHECK(RC_Feed(&d) == 0, "四轴全 0x00 判为垃圾数据 -> 停车");
    CHECK(g_rc_dbg.armed == 0 && g_rc_dbg.reject_count == 1, "解除使能并计数");

    /* 摇杆不在中位也不能恢复 */
    d.left_x = 0xFF; d.left_y = 0xFF; d.right_x = 0xFF; d.right_y = 0xFF;
    CHECK(RC_Feed(&d) == 0, "四轴全 0xFF 同样判为垃圾数据");

    /* 摇杆不在中位（数据本身合理）也不能恢复 */
    d.left_x = 0xA0; d.left_y = 0x80; d.right_x = 0x80; d.right_y = 0x80;
    CHECK(RC_Feed(&d) == 0, "数据合理但摇杆未回中 -> 仍不允许驱动");

    /* 重新回中 -> 需再次满足 200ms（需先有一帧起算计时） */
    d.left_x = 0x80; d.left_y = 0x80; d.right_x = 0x80; d.right_y = 0x80;
    CHECK(RC_Feed(&d) == 0, "刚回中的第一帧只起算计时，仍不允许驱动");
    s_tick += RC_STARTUP_ARM_MS + 50;
    CHECK(RC_Feed(&d) == 1, "重新回中并持续 200ms 后恢复驱动");
    CHECK(g_rc_dbg.armed == 1, "armed 重新置位");
    CHECK(g_rc_dbg.reject_count == 3, "reject_count = 垃圾0x00 + 垃圾0xFF + 未回中");

    /* 失联保护：数据逐字节不变超过超时 */
    CHECK(RC_Feed(&d) == 1, "数据未变但未到超时时间仍允许");
    s_tick += RC_STALE_TIMEOUT_MS + 100;
    CHECK(RC_Feed(&d) == 0, "数据逐字节不变超过超时 -> 保护停车");
    CHECK(g_rc_dbg.stale == 1 && g_rc_dbg.protect_count == 1, "失联保护标志与计数正确");

    d.left_x = 0x81;                                   /* 操作手动一下摇杆 */
    CHECK(RC_Feed(&d) == 1, "数据一变化立即恢复驱动");
    CHECK(g_rc_dbg.stale == 0 && g_rc_dbg.unchanged_ms == 0, "恢复后失联标志清零");
    d.left_x = 0x80;

    /* 时间回绕：tick 从 0xFFFFFF00 走到 0x00000100 不应误判 */
    s_tick = 0xFFFFFF00u;
    RC_Init();
    (void)RC_Feed(&d);
    s_tick = 0xFFFFFF00u + RC_STARTUP_ARM_MS + 10u;
    CHECK(RC_Feed(&d) == 1, "回绕前完成安全启动");
    s_tick = 0x00000100u;                              /* 相对上次 +266ms，未到 1000ms */
    CHECK(RC_Feed(&d) == 1, "tick 回绕后按无符号差值计算，不误触发保护");

    /* ---------- 7. 遥控映射 v2：档位 / 绕对侧后轮 / 差速 ---------- */
    printf("[7] 遥控映射 v2\n");
    {
        Ctrl_Output_t cmd;
        PS2_Data_t p;

        memset(&p, 0, sizeof p);
        p.mode = 1;
        p.left_x = p.left_y = p.right_x = p.right_y = 0x80;   /* 摇杆回中 */
        p.buttons = 0xFFFF;                                   /* 无按键（低有效） */

        Ctrl_Init();                     /* 档位回默认（中速） */
        s_tick = 5000;                   /* 远离 0，脱离换档锁定期 */

        CHECK(Ctrl_Update(&p, 0, &cmd) == 0 && cmd.vx == 0, "未使能 -> 不驱动");
        CHECK(cmd.gear == CTRL_GEAR_DEFAULT && cmd.gear == 1, "上电默认档位 = 中速（1）");

        p.mode = 0;
        CHECK(Ctrl_Update(&p, 1, &cmd) == 0, "绿灯(数字)模式 -> 不驱动");
        p.mode = 1;

        /* --- 档位：△ 加档 / ✕ 减档（边沿触发 + 去抖） --- */
        p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 2, "中速档按 △ -> 档位 2（快速）");
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 2, "按住 △ 不重复加档（边沿触发）");
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);   /* 松手 */
        p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 2, "到最高档后不再增加");

        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);
        p.buttons = (uint16_t)(~PS2_BTN_CROSS);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 1 && cmd.brake == 0, "✕ 是减档（不制动）");

        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);
        p.buttons = (uint16_t)(~PS2_BTN_START);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.mode == CTRL_MODE_BRAKE && cmd.brake == 1, "START -> 制动");
        p.buttons = (uint16_t)(~PS2_BTN_SELECT);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.brake == 0, "SELECT 已不再用作制动");
        p.buttons = 0xFFFF;

        /* --- 档位内的速度映射 --- */
        Ctrl_Init();
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.left_y = 0x00;                                   /* 左摇杆满杆推前 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 1 && cmd.vx == CTRL_GEAR1_HI, "默认中速档满杆 -> 600（档位上限）");
        p.left_y = 59;                                     /* 约半杆 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx > CTRL_GEAR1_LO && cmd.vx < CTRL_GEAR1_HI,
              "中速档半杆 -> 落在 30%~60% 区间内");
        p.left_y = 0x80;

        /* 降一档到慢速 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = (uint16_t)(~PS2_BTN_CROSS); (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 0, "按 ✕ 降到慢速档");
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF;
        p.left_y = 0x00;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == CTRL_GEAR0_HI, "慢速档满杆 -> 300（档位上限）");
        p.left_y = 0x80;

        /* 升到快速档：满杆 = 1000 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE); (void)Ctrl_Update(&p, 1, &cmd);   /* 0 -> 1 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);                          /* 松手（边沿需要） */
        p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE); (void)Ctrl_Update(&p, 1, &cmd);   /* 1 -> 2 */
        CHECK(cmd.gear == 2, "再按 △ 回到快速档");
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF;
        p.left_y = 0x00;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == CTRL_GEAR2_HI && CTRL_GEAR2_HI <= CTRL_DUTY_CAP,
              "快速档满杆 -> 档位上限 85%（不超过全局 92% 上限）");
        p.left_y = 0x80;

        /* --- 左摇杆 X：原地自转（纯旋转，绕底盘中心） --- */
        Ctrl_Init();                                       /* 默认中速档 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF;

        p.left_x = 0x00;                                   /* 推左 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == 0 && cmd.vy == 0 && cmd.vw != 0, "左摇杆推左 -> 纯原地自转（vx=vy=0）");
        CHECK(cmd.vw == -CTRL_GEAR1_HI, "自转受档位限制（中速档 = -600）");
        p.left_x = 0xFF;                                   /* 推右 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vw == CTRL_GEAR1_HI, "左摇杆推右 -> 反向自转");
        p.left_x = 0x80;

        /* --- 右摇杆 X：绕"对侧"后轮（推左 -> 绕右后轮；推右 -> 绕左后轮） --- */
        p.right_x = 0x00;                                  /* 推左 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx != 0 && cmd.vy != 0 && cmd.vw != 0,
              "右摇杆推左 -> 产生 vx/vy/vw 组合（绕转，不是纯自转）");
#if CTRL_PIVOT_OPPOSITE_SIDE
        CHECK(wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 3) >= -2 &&
              wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 3) <= 2,
              "右摇杆推左 -> 右后轮指令为 0（绕右后轮，对侧）");
#else
        CHECK(wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 1) >= -2 &&
              wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 1) <= 2,
              "右摇杆推左 -> 左后轮指令为 0（绕左后轮，同侧）");
#endif

        p.right_x = 0xFF;                                  /* 推右 */
        (void)Ctrl_Update(&p, 1, &cmd);
#if CTRL_PIVOT_OPPOSITE_SIDE
        CHECK(wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 1) >= -2 &&
              wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 1) <= 2,
              "右摇杆推右 -> 左后轮指令为 0（绕左后轮，对侧）");
#else
        CHECK(wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 3) >= -2 &&
              wheel_cmd(cmd.vx, cmd.vy, cmd.vw, 3) <= 2,
              "右摇杆推右 -> 右后轮指令为 0（绕右后轮，同侧）");
#endif
        p.right_x = 0x80;

        /* --- 右摇杆 Y：已清空（不产生任何指令） --- */
        p.left_y = 0x80; p.left_x = 0x80; p.right_x = 0x80;
        p.right_y = 0x00;                                  /* 满杆推前 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == 0 && cmd.vy == 0 && cmd.vw == 0, "右摇杆 Y 已清空（推前不产生指令）");
        p.right_y = 0xFF;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == 0 && cmd.vw == 0, "右摇杆 Y 拉后也无指令");
        p.right_y = 0x80;

        /* --- 方向键：平移（上下=前后，左右=左右平移；可组合斜向） --- */
        p.buttons = (uint16_t)(~PS2_BTN_UP);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.mode == CTRL_MODE_DPAD && cmd.vx == CTRL_DPAD_SPEED && cmd.vy == 0,
              "方向键上 -> 70% 前进");
        p.buttons = (uint16_t)(~PS2_BTN_DOWN);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == -CTRL_DPAD_SPEED, "方向键下 -> 70% 后退");
        p.buttons = (uint16_t)(~PS2_BTN_LEFT);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == 0 && cmd.vy == -CTRL_DPAD_SPEED, "方向键左 -> 70% 左移");
        p.buttons = (uint16_t)(~PS2_BTN_RIGHT);
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vy == CTRL_DPAD_SPEED, "方向键右 -> 70% 右移");
        p.buttons = (uint16_t)(~(PS2_BTN_UP | PS2_BTN_LEFT));
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vx == CTRL_DPAD_SPEED && cmd.vy == -CTRL_DPAD_SPEED, "方向键上+左 -> 斜向平移");
        p.buttons = 0xFFFF;

        /* --- 平移仍受档位限幅；旋转不受限 --- */
        Ctrl_Init();                                        /* 中速档 */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.left_y = 0x00; p.left_x = 0x80; p.right_y = 0x80; p.right_x = 0x80;
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.limit == CTRL_GEAR1_HI, "纯平移时输出上限 = 档位上限");
        p.left_y = 0x80;

        /* --- 慢速档：平移/自转仍受限，绕后轮不受限 --- */
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = (uint16_t)(~PS2_BTN_CROSS); (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.gear == 0, "降到慢速档");
        s_tick += CTRL_GEAR_LOCKOUT_MS;
        p.buttons = 0xFFFF;

        p.left_x = 0x00; p.right_x = 0x80;                  /* 只推左摇杆 X：自转 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.vw == -CTRL_GEAR0_HI && cmd.limit == CTRL_GEAR0_HI,
              "慢速档自转 -> 300（与平移一致，受档位限制）");
        p.left_x = 0x80;

        p.right_x = 0x00;                                   /* 只推右摇杆 X：绕后轮 */
        (void)Ctrl_Update(&p, 1, &cmd);
        CHECK(cmd.limit == CTRL_PIVOT_LIMIT_LO,
              "慢速档绕转上限 = 物理可用下限 600（不是档位原始值 300）");
        Motor_StopAll();
        Mecanum_MoveLimit(cmd.vx, cmd.vy, cmd.vw, cmd.limit);
        {
            uint32_t w[8] = { g_fake_tim2.CCR1, g_fake_tim2.CCR2, g_fake_tim2.CCR3, g_fake_tim2.CCR4,
                              g_fake_tim4.CCR1, g_fake_tim4.CCR2, g_fake_tim4.CCR3, g_fake_tim4.CCR4 };
            uint32_t peak = 0, second = 0;

            for (int i = 0; i < 8; i++) if (w[i] > peak) peak = w[i];
            for (int i = 0; i < 8; i++) if (w[i] != peak && w[i] > second) second = w[i];

            CHECK(peak == (uint32_t)CTRL_PIVOT_LIMIT_LO &&
                  second >= 300u,
                  "慢速档下绕后轮：最快轮 = 600、第二快轮 >= 300（不会只剩一个轮子转）");
        }
        p.right_x = 0x80;

        /* --- 档位对绕转的控制：三档的绕转上限必须单调递增 --- */
        {
            int16_t pivot_peak[CTRL_GEAR_COUNT];

            Ctrl_Init();                                        /* 回到中速档(1) */
            s_tick += CTRL_GEAR_LOCKOUT_MS;
            p.buttons = (uint16_t)(~PS2_BTN_CROSS); (void)Ctrl_Update(&p, 1, &cmd);  /* 1 -> 0 */
            s_tick += CTRL_GEAR_LOCKOUT_MS;
            p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);

            for (uint8_t g = 0; g < CTRL_GEAR_COUNT; g++)
            {
                if (g > 0)
                {
                    p.buttons = (uint16_t)(~PS2_BTN_TRIANGLE); (void)Ctrl_Update(&p, 1, &cmd);  /* 加一档 */
                    s_tick += CTRL_GEAR_LOCKOUT_MS;
                    p.buttons = 0xFFFF; (void)Ctrl_Update(&p, 1, &cmd);
                }

                p.right_x = 0x00;                               /* 只推右摇杆 X */
                (void)Ctrl_Update(&p, 1, &cmd);
                CHECK(cmd.gear == g, "绕转档位遍历：档位序号正确");
                pivot_peak[g] = cmd.limit;
                p.right_x = 0x80;
            }

            CHECK(pivot_peak[0] == CTRL_PIVOT_LIMIT_LO &&
                  pivot_peak[1] > pivot_peak[0] &&
                  pivot_peak[2] == CTRL_PIVOT_LIMIT_HI,
                  "三档绕转上限单调递增 600 -> 755 -> 920（档位确实控制绕转快慢）");
            CHECK(pivot_peak[0] > CTRL_GEAR0_HI,
                  "慢速档绕转上限(600)高于档位原始上限(300)：2:1 差速的物理需要");
        }

        /* --- 档位 LED --- */
        s_gpio_a_odr = 0;
        LED_SetGear(0);
        CHECK((s_gpio_a_odr & LED_PIN_SLOW) != 0 && (s_gpio_a_odr & LED_PIN_MID) == 0, "档位 0 -> 只亮慢速 LED");
        LED_SetGear(1);
        CHECK((s_gpio_a_odr & LED_PIN_MID) != 0 && (s_gpio_a_odr & LED_PIN_SLOW) == 0, "档位 1 -> 只亮中速 LED");
        LED_SetGear(2);
        CHECK((s_gpio_a_odr & LED_PIN_FAST) != 0 && (s_gpio_a_odr & LED_PIN_MID) == 0, "档位 2 -> 只亮快速 LED");
    }

    /* 刹车输出：DRV8833 双输入 100% */
    memset(&g_fake_tim2, 0, sizeof g_fake_tim2);
    memset(&g_fake_tim4, 0, sizeof g_fake_tim4);
    Motor_BrakeAll();
#if MOTOR_BRAKE_BOTH_HIGH
    CHECK(g_fake_tim2.CCR1 == 1000 && g_fake_tim2.CCR2 == 1000 &&
          g_fake_tim2.CCR3 == 1000 && g_fake_tim2.CCR4 == 1000 &&
          g_fake_tim4.CCR1 == 1000 && g_fake_tim4.CCR2 == 1000 &&
          g_fake_tim4.CCR3 == 1000 && g_fake_tim4.CCR4 == 1000,
          "START 刹车 -> 四路双输入全部 100% 占空比");
#else
    CHECK(g_fake_tim2.CCR1 == 0 && g_fake_tim4.CCR1 == 0, "刹车方式=滑行 -> 全部为 0");
#endif

    /* ---------- 8. 舵机（TIM3，50Hz） ---------- */
    printf("[8] 舵机\n");
    {
        Servo_Init();
        CHECK(g_fake_tim3.PSC == 71 && g_fake_tim3.ARR == 19999,
              "TIM3 时基 = 1MHz / 20000 -> 50Hz");
        /* 上电姿态按"机械角 -> 脉宽"换算，再被 MIN/MAX 限幅，所以用 clamp 后的期望值比较 */
        {
            const uint16_t exp0 = expect_clamped(SERVO0_START_US, SERVO0_MIN_US, SERVO0_MAX_US);
            const uint16_t exp1 = expect_clamped(SERVO1_START_US, SERVO1_MIN_US, SERVO1_MAX_US);

            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == exp0 &&
                  Servo_GetPulse(SERVO_CH_JOINT2) == exp1,
                  "上电舵机停在配置姿态（角度换算 + 限幅）");
            CHECK(g_fake_tim3.CCR1 == exp0 && g_fake_tim3.CCR2 == exp1,
                  "CCR1/CCR2 已写入上电默认脉宽");
        }
        /* 三路臂舵机：限位必须落在舵机物理范围 500~2500us 内、下限<上限；
         * 只有在"角度换算没有碰到物理边界"时，跨度才等于 (上限-下限)° × 11.11us。
         * （现场若把偏移/下限填到超出舵机物理范围，宏会自动限幅到 500/2500，这里不判失败） */
        {
            CHECK(SERVO0_MIN_US < SERVO0_MAX_US && SERVO1_MIN_US < SERVO1_MAX_US &&
                  SERVO3_MIN_US < SERVO3_MAX_US,
                  "三路臂舵机：下限 < 上限");
            CHECK(SERVO0_MIN_US >= 500 && SERVO0_MAX_US <= 2500 &&
                  SERVO1_MIN_US >= 500 && SERVO1_MAX_US <= 2500 &&
                  SERVO3_MIN_US >= 500 && SERVO3_MAX_US <= 2500,
                  "三路臂舵机：限位都在舵机物理范围 500~2500us 内");

            /* MG995 偏移 0 且 0~140° 全在物理范围内 -> 跨度应严格等于角度差 */
            {
                const uint16_t span = (uint16_t)(SERVO0_MAX_US - SERVO0_MIN_US);
                const uint16_t want = (uint16_t)((SERVO0_MAX_ANGLE_DEG - SERVO0_MIN_ANGLE_DEG) * 1000u / 90u);

                CHECK(span == want || span + 1u == want || span == want + 1u,
                      "MG995 行程跨度 = (上限-下限)° × 11.11us");
            }
            CHECK(SERVO0_SPAN_DEG == SERVO0_MAX_ANGLE_DEG - SERVO0_MIN_ANGLE_DEG &&
                  SERVO1_SPAN_DEG == SERVO1_MAX_ANGLE_DEG - SERVO1_MIN_ANGLE_DEG &&
                  SERVO3_SPAN_DEG == SERVO3_MAX_ANGLE_DEG - SERVO3_MIN_ANGLE_DEG,
                  "Servo_SetAngle 的角度跨度 = 上限-下限");
        }
        /* ===== 以下断言全部"按宏自洽计算"，现场改零点偏移/一键行程时不用动测试 ===== */
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
              "机械爪上电停在完全张开（= SERVO2_MAX_US）");

        /* 零点偏移：标定值应落在合理区间；平移后行程跨度必须仍是 1000us（= 90°） */
        CHECK(SERVO_GRIP_ANGLE_SHIFT_DEG10 <= -100 && SERVO_GRIP_ANGLE_SHIFT_DEG10 >= -200,
              "机械爪零点偏移标定值在 -10° ~ -20° 范围内");
        CHECK(SERVO_GRIP_SHIFT_US == (int32_t)SERVO_GRIP_ANGLE_SHIFT_DEG10 * 1000 / 900,
              "偏移换算：0.1° -> us（1° = 1000/90 us）");
        CHECK(SERVO2_MIN_US == (uint16_t)(500 + SERVO_GRIP_SHIFT_US + SERVO_GRIP_TRIM_US) &&
              SERVO2_MAX_US == (uint16_t)(1500 + SERVO_GRIP_SHIFT_US - SERVO_GRIP_TRIM_US),
              "两端各收进 SERVO_GRIP_TRIM_DEG 度（原 500/1500 平移后再收进）");
        CHECK(SERVO2_MAX_US - SERVO2_MIN_US == (uint16_t)(1000 - 2 * SERVO_GRIP_TRIM_US),
              "行程跨度 = 1000us - 2×5°收进量（当前 890us = 80°）");

        /* 角度接口：机械爪 0=全闭、SPAN=全开（角度是"机构角度"，已含零点偏移与收进量） */
        {
            const uint16_t span = (uint16_t)(SERVO2_MAX_US - SERVO2_MIN_US);

            Servo_SetAngle(SERVO_CH_GRIPPER, 0);
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO2_MIN_US, "爪 0° -> MIN（完全闭合）");
            Servo_SetAngle(SERVO_CH_GRIPPER, SERVO2_MAX_DEG);
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO2_MAX_US, "爪 SPAN -> MAX（完全张开）");
            Servo_SetAngle(SERVO_CH_GRIPPER, (uint16_t)(SERVO2_MAX_DEG / 2));
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == (uint16_t)(SERVO2_MIN_US + span / 2),
                  "爪 半程 -> 行程中点");
            Servo_SetAngle(SERVO_CH_GRIPPER, 200);
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO2_MAX_US, "角度超范围 -> 限幅到 MAX");

            /* 一键式合上端点：从全开端回收 SERVO_GRIP_ONESHOT_DEG 度（1° = 1000/90 us） */
            CHECK(SERVO_GRIP_CLOSE_DEG == SERVO_GRIP_OPEN_DEG - SERVO_GRIP_ONESHOT_DEG,
                  "一键合上角 = 完全张开角 - 一键行程");
            CHECK(SERVO_GRIP_CLOSE_US ==
                      (uint16_t)(SERVO2_MAX_US - (uint32_t)SERVO_GRIP_ONESHOT_DEG * 1000 / 90),
                  "一键合上端点 = 全开端 - 一键行程×11.11us");
        }
        Servo_SetAngle(SERVO_CH_JOINT1, SERVO0_SPAN_DEG + 20);          /* 超上限 */
        CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == SERVO0_MAX_US, "角度超上限 -> 限幅到 MAX");
        Servo_SetAngle(SERVO_CH_JOINT1, 90);                           /* 按 deg_to_us 的映射算期望值 */
        CHECK(Servo_GetPulse(SERVO_CH_JOINT1) ==
                  (uint16_t)(SERVO0_MIN_US + (uint32_t)(SERVO0_MAX_US - SERVO0_MIN_US) * 90u / SERVO0_SPAN_DEG),
              "MG995 90° -> 按 0~跨度 线性映射");
        Servo_Init();

        /* 限幅 */
        Servo_SetPulse(SERVO_CH_JOINT1, 300);
        CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == SERVO0_MIN_US, "低于下限 -> 限幅到 MIN");
        Servo_SetPulse(SERVO_CH_JOINT1, 3000);
        CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == SERVO0_MAX_US, "高于上限 -> 限幅到 MAX");

        /* ===== MG995（CH1，最下面的臂）：右摇杆前后控制（带加减速曲线，位移给容差） ===== */
        Servo_Init();
        {
            const uint16_t p0  = Servo_GetPulse(SERVO_CH_JOINT1);
            const uint16_t nom = (uint16_t)(SERVO_MG995_STEP_US10 * 50u / 10u);   /* 50 tick 名义位移 */

            for (int i = 0; i < 50; i++) servo_task_stick(0x00);     /* 摇杆推前 */
            {
                const uint16_t moved = (uint16_t)(Servo_GetPulse(SERVO_CH_JOINT1) - p0);

                CHECK(moved > 1u && moved <= nom,
                      "右摇杆推前 -> MG995 前进（加减速曲线影响起步，略慢于名义值）");
                CHECK((uint32_t)moved * 10u >= (uint32_t)nom * 6u,
                      "右摇杆推前 -> 位移不小于名义值的 60%");
            }

            for (int i = 0; i < 200; i++) servo_task_stick(0x80);    /* 摇杆回中，等插值到位 */
            {
                const uint16_t before = Servo_GetPulse(SERVO_CH_JOINT1);

                for (int i = 0; i < 20; i++) servo_task_stick(0x80);
                CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == before,
                      "右摇杆回中 -> MG995 停止（不阻塞、松开即停）");
            }
            /* 一直推到顶/拉到底 -> 被 MIN/MAX 限幅；tick 数给足 */
            for (int i = 0; i < 3000; i++) servo_task_stick(0x00);
            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == SERVO0_MAX_US, "推到底 -> 限幅到安全上限");
            for (int i = 0; i < 3000; i++) servo_task_stick(0xFF);
            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == SERVO0_MIN_US, "拉到底 -> 限幅到安全下限");
            CHECK(g_fake_tim3.CCR1 == Servo_GetPulse(SERVO_CH_JOINT1),
                  "TIM3->CCR1 与实际脉宽一致");
        }

        /* ===== SG90-下（CH2）：L1 / L2 点动（带加减速曲线） ===== */
        Servo_Init();
        Servo_SetGripMode(SERVO_GRIP_MODE_JOG);
        {
            const uint16_t p0  = Servo_GetPulse(SERVO_CH_JOINT2);
            const uint16_t nom = (uint16_t)(SERVO_SG90_LOW_STEP_US10 * 50u / 10u);     /* 50 tick 名义位移 */

            CHECK(p0 == expect_clamped(SERVO1_START_US, SERVO1_MIN_US, SERVO1_MAX_US),
                  "SG90(下) 上电停在配置姿态（限幅后）");
            /* 上电在端点附近，先按有余量的方向走 */
            for (int i = 0; i < 50; i++) servo_task_buttons((uint16_t)(~PS2_BTN_L2));
            {
                const uint16_t moved = (uint16_t)(p0 - Servo_GetPulse(SERVO_CH_JOINT2));

                CHECK(moved > 1u && moved <= nom, "按住 L2 -> SG90(下) 按宏速度移动");
            }
        }

        /* ===== 一键放下（模式1 按 L1：MG995 -> SG90下 -> SG90上 -> 等0.5s -> 机械爪松开） ===== */
        {
            const uint16_t tgt_ch1 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_DROP_MG995_DEG, SERVO0_OFFSET_DEG);
            const uint16_t tgt_ch2 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_DROP_SG90_LOW_DEG, SERVO1_OFFSET_DEG);
            const uint16_t tgt_ch4 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_DROP_SG90_UP_DEG, SERVO3_OFFSET_DEG);

            Servo_Init();
            Servo_SetGripMode(SERVO_GRIP_MODE_ONESHOT);      /* 模式 1：一键张合 + 一键放下 */
            Servo_SetPulse(SERVO_CH_GRIPPER, SERVO_GRIP_CLOSE_US);   /* 先把爪子合上，便于验证后面松开 */
            servo_task_buttons(0xFFFF);                      /* 放开所有键，建立边沿状态 */

            {
                const uint16_t ch2_before = Servo_GetTarget(SERVO_CH_JOINT2);
                const uint16_t ch4_before = Servo_GetTarget(SERVO_CH_ARM_UP);

                servo_task_buttons((uint16_t)(~PS2_BTN_L1)); /* 按 L1 -> 触发 */
                CHECK(Servo_GetTarget(SERVO_CH_JOINT1) == tgt_ch1,
                      "一键放下：第1步 MG995 目标 = SERVO_DROP_MG995_DEG");
                CHECK(Servo_GetTarget(SERVO_CH_JOINT2) == ch2_before &&
                      Servo_GetTarget(SERVO_CH_ARM_UP) == ch4_before,
                      "一键放下：第1步时上面两个 SG90 还不动（严格从下往上）");
            }

            /* 空跑 4 秒（400 tick）让三个臂到位；注意不推进 tick，所以会卡在"等延时"那一步 */
            for (int i = 0; i < 3000; i++) servo_task_buttons(0xFFFF);

            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == expect_clamped(tgt_ch1, SERVO0_MIN_US, SERVO0_MAX_US),
                  "一键放下：MG995 到达放下目标角");
            CHECK(Servo_GetPulse(SERVO_CH_JOINT2) == expect_clamped(tgt_ch2, SERVO1_MIN_US, SERVO1_MAX_US),
                  "一键放下：SG90(下) 到达放下目标角");
            CHECK(Servo_GetPulse(SERVO_CH_ARM_UP) == expect_clamped(tgt_ch4, SERVO3_MIN_US, SERVO3_MAX_US),
                  "一键放下：SG90(上) 到达放下目标角（超限位会自动限幅）");
            CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US &&
                  Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US,
                  "一键放下：0.5s 延时期间机械爪保持不动（还合着）");

            /* 推进 0.5s 延时 -> 机械爪才开始松开 */
            s_tick += SERVO_DROP_GRIP_DELAY_MS + 20;
            servo_task_buttons(0xFFFF);
            CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
                  "一键放下：延时到 -> 机械爪目标切到完全张开");
            for (int i = 0; i < 100; i++) servo_task_buttons(0xFFFF);
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
                  "一键放下：机械爪松开到位");

            /* 模式 1 下 L1 是触发键：再按一次 L1 是重新触发序列（不会点动 CH2） */
            {
                const uint16_t ch2_after = Servo_GetTarget(SERVO_CH_JOINT2);

                servo_task_buttons(0xFFFF);
                servo_task_buttons((uint16_t)(~PS2_BTN_L1));
                CHECK(Servo_GetTarget(SERVO_CH_JOINT2) == ch2_after,
                      "模式1：L1 是一键放下触发键，不再点动 CH2");
            }
        }

        /* ===== 模式 1 下 R1 = 一键摆臂（角度由 SERVO_POSE_* 宏控制，延时后张开爪子） ===== */
        {
            const uint16_t pose_ch1 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_POSE_MG995_DEG, SERVO0_OFFSET_DEG);
            const uint16_t pose_ch2 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_POSE_SG90_LOW_DEG, SERVO1_OFFSET_DEG);
            const uint16_t pose_ch4 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_POSE_SG90_UP_DEG, SERVO3_OFFSET_DEG);

            Servo_Init();
            Servo_SetGripMode(SERVO_GRIP_MODE_ONESHOT);
            Servo_SetPulse(SERVO_CH_GRIPPER, SERVO_GRIP_CLOSE_US);
            servo_task_buttons(0xFFFF);

            {
                const uint16_t ch1_before = Servo_GetTarget(SERVO_CH_JOINT1);
                const uint16_t ch2_before = Servo_GetTarget(SERVO_CH_JOINT2);

                servo_task_buttons((uint16_t)(~PS2_BTN_R1));   /* 按 R1 -> 触发一键摆臂 */
                CHECK(Servo_GetTarget(SERVO_CH_ARM_UP) == expect_clamped(pose_ch4, SERVO3_MIN_US, SERVO3_MAX_US),
                      "一键摆臂：第1步先动 SG90(上)（避免卡住）");
                CHECK(Servo_GetTarget(SERVO_CH_JOINT1) == ch1_before &&
                      Servo_GetTarget(SERVO_CH_JOINT2) == ch2_before,
                      "一键摆臂：第1步时 MG995/SG90(下) 还不动");
            }

            for (int i = 0; i < 3000; i++) servo_task_buttons(0xFFFF);
            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == expect_clamped(pose_ch1, SERVO0_MIN_US, SERVO0_MAX_US),
                  "一键摆臂：MG995 到位");
            CHECK(Servo_GetPulse(SERVO_CH_JOINT2) == expect_clamped(pose_ch2, SERVO1_MIN_US, SERVO1_MAX_US),
                  "一键摆臂：SG90(下) 到位");
            CHECK(Servo_GetPulse(SERVO_CH_ARM_UP) == expect_clamped(pose_ch4, SERVO3_MIN_US, SERVO3_MAX_US),
                  "一键摆臂：SG90(上) 到位");
            CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US,
                  "一键摆臂：延时期间机械爪还没张开");

            s_tick += SERVO_POSE_GRIP_DELAY_MS + 20;           /* 推进延时 */
            servo_task_buttons(0xFFFF);
            CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
                  "一键摆臂：延时到 -> 机械爪张开");
            for (int i = 0; i < 100; i++) servo_task_buttons(0xFFFF);
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
                  "一键摆臂：机械爪张开到位");
        }

        /* ===== 模式 1 下 R2 = 自定义一键序列（角度由 SERVO_R2_* 宏控制） ===== */
        {
            const uint16_t r2_ch1 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_R2_MG995_DEG, SERVO0_OFFSET_DEG);
            const uint16_t r2_ch2 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_R2_SG90_LOW_DEG, SERVO1_OFFSET_DEG);
            const uint16_t r2_ch4 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_R2_SG90_UP_DEG, SERVO3_OFFSET_DEG);

            Servo_Init();
            Servo_SetGripMode(SERVO_GRIP_MODE_ONESHOT);
            Servo_SetPulse(SERVO_CH_GRIPPER, SERVO_GRIP_CLOSE_US);
            servo_task_buttons(0xFFFF);

            {
                const uint16_t ch1_before = Servo_GetTarget(SERVO_CH_JOINT1);
                const uint16_t ch2_before = Servo_GetTarget(SERVO_CH_JOINT2);

                servo_task_buttons((uint16_t)(~PS2_BTN_R2));   /* 按 R2 -> 触发自定义序列1 */
                CHECK(Servo_GetTarget(SERVO_CH_ARM_UP) == expect_clamped(r2_ch4, SERVO3_MIN_US, SERVO3_MAX_US),
                      "自定义序列：第1步先动 SG90(上)（避免卡住）");
                CHECK(Servo_GetTarget(SERVO_CH_JOINT1) == ch1_before &&
                      Servo_GetTarget(SERVO_CH_JOINT2) == ch2_before,
                      "自定义序列：第1步时 MG995/SG90(下) 还不动");
            }

            for (int i = 0; i < 3000; i++) servo_task_buttons(0xFFFF);
            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == expect_clamped(r2_ch1, SERVO0_MIN_US, SERVO0_MAX_US),
                  "自定义序列：MG995 到位");
            CHECK(Servo_GetPulse(SERVO_CH_JOINT2) == expect_clamped(r2_ch2, SERVO1_MIN_US, SERVO1_MAX_US),
                  "自定义序列：SG90(下) 到位");
            CHECK(Servo_GetPulse(SERVO_CH_ARM_UP) == expect_clamped(r2_ch4, SERVO3_MIN_US, SERVO3_MAX_US),
                  "自定义序列：SG90(上) 到位");
            CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US,
                  "自定义序列：延时期间机械爪还没张开");

            s_tick += SERVO_R2_GRIP_DELAY_MS + 20;
            servo_task_buttons(0xFFFF);
            CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
                  "自定义序列：延时到 -> 机械爪张开");
            for (int i = 0; i < 100; i++) servo_task_buttons(0xFFFF);
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
                  "自定义序列：机械爪张开到位");
        }

        /* ===== 模式 1 下 L2 = 自定义一键序列 2（角度由 SERVO_L2_* 宏控制） ===== */
        {
            const uint16_t l2_ch1 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_L2_MG995_DEG, SERVO0_OFFSET_DEG);
            const uint16_t l2_ch2 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_L2_SG90_LOW_DEG, SERVO1_OFFSET_DEG);
            const uint16_t l2_ch4 = (uint16_t)SERVO_ANGLE_TO_US(SERVO_L2_SG90_UP_DEG, SERVO3_OFFSET_DEG);

            Servo_Init();
            Servo_SetGripMode(SERVO_GRIP_MODE_ONESHOT);
            Servo_SetPulse(SERVO_CH_GRIPPER, SERVO_GRIP_CLOSE_US);
            servo_task_buttons(0xFFFF);

            {
                const uint16_t ch1_before = Servo_GetTarget(SERVO_CH_JOINT1);
                const uint16_t ch4_before = Servo_GetTarget(SERVO_CH_ARM_UP);

                servo_task_buttons((uint16_t)(~PS2_BTN_L2));   /* 按 L2 -> 触发自定义序列2 */
                CHECK(Servo_GetTarget(SERVO_CH_JOINT1) == expect_clamped(l2_ch1, SERVO0_MIN_US, SERVO0_MAX_US),
                      "自定义序列2：第1步是 MG995（标准顺序）");
                CHECK(Servo_GetTarget(SERVO_CH_ARM_UP) == ch4_before,
                      "自定义序列2：第1步时 SG90(上) 还不动");
                (void)ch1_before;
            }

            for (int i = 0; i < 3000; i++) servo_task_buttons(0xFFFF);
            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) == expect_clamped(l2_ch1, SERVO0_MIN_US, SERVO0_MAX_US),
                  "自定义序列2：MG995 到位");
            CHECK(Servo_GetPulse(SERVO_CH_JOINT2) == expect_clamped(l2_ch2, SERVO1_MIN_US, SERVO1_MAX_US),
                  "自定义序列2：SG90(下) 到位");
            CHECK(Servo_GetPulse(SERVO_CH_ARM_UP) == expect_clamped(l2_ch4, SERVO3_MIN_US, SERVO3_MAX_US),
                  "自定义序列2：SG90(上) 到位");
            CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US,
                  "自定义序列2：延时期间机械爪还没张开");

            s_tick += SERVO_L2_GRIP_DELAY_MS + 20;
            servo_task_buttons(0xFFFF);
            CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
                  "自定义序列2：延时到 -> 机械爪张开");
            for (int i = 0; i < 100; i++) servo_task_buttons(0xFFFF);
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
                  "自定义序列2：机械爪张开到位");
        }

        /* ===== 模式 0 下 R1/R2 点动 SG90(上)（带加减速曲线，位移给容差） ===== */
        Servo_Init();
        Servo_SetGripMode(SERVO_GRIP_MODE_JOG);
        {
            const uint16_t p0  = Servo_GetPulse(SERVO_CH_ARM_UP);
            const uint16_t nom = (uint16_t)(SERVO_SG90_UP_STEP_US10 * 20u / 10u);

            for (int i = 0; i < 20; i++) servo_task_buttons((uint16_t)(~PS2_BTN_R2));
            {
                const uint16_t moved = (uint16_t)(p0 - Servo_GetPulse(SERVO_CH_ARM_UP));

                CHECK(moved > 0u && moved <= nom, "模式0：R2 点动 SG90(上) 向下");
            }
        }

        /* ===== 模式 0 下 L1/L2 点动 SG90(下) ===== */
        Servo_Init();
        Servo_SetGripMode(SERVO_GRIP_MODE_JOG);
        {
            const uint16_t p0  = Servo_GetPulse(SERVO_CH_JOINT2);
            const uint16_t nom = (uint16_t)(SERVO_SG90_LOW_STEP_US10 * 20u / 10u);

            for (int i = 0; i < 20; i++) servo_task_buttons((uint16_t)(~PS2_BTN_L2));
            {
                const uint16_t moved = (uint16_t)(p0 - Servo_GetPulse(SERVO_CH_JOINT2));

                CHECK(moved > 0u && moved <= nom, "模式0：L2 点动 SG90(下) 向下");
            }
        }



        /* ===== SG90-上（CH4）：R1 / R2 点动（按当前上电位置自动选有余量的方向） ===== */
        Servo_Init();
        Servo_SetGripMode(SERVO_GRIP_MODE_JOG);   /* R1/R2 点动只在模式0有效（模式1 下 R1=一键摆臂） */
        {
            const uint16_t p0  = Servo_GetPulse(SERVO_CH_ARM_UP);
            const uint16_t nom = (uint16_t)(SERVO_SG90_UP_STEP_US10 * 50u / 10u);   /* 50 tick 名义位移 */

            /* 上电位置靠上限就先用 R2 往下走，否则先用 R1 往上走 */
            const uint8_t  down_first = ((uint16_t)(p0 - SERVO3_MIN_US) < nom) ? 0u : 1u;
            const uint16_t btn_first  = down_first ? (uint16_t)(~PS2_BTN_R2) : (uint16_t)(~PS2_BTN_R1);

            CHECK(p0 == expect_clamped(SERVO3_START_US, SERVO3_MIN_US, SERVO3_MAX_US),
                  "SG90(上) 上电停在配置姿态（限幅后）");
            for (int i = 0; i < 50; i++) servo_task_buttons(btn_first);
            {
                const uint16_t p1 = Servo_GetPulse(SERVO_CH_ARM_UP);
                const uint16_t mv = down_first ? (uint16_t)(p0 - p1) : (uint16_t)(p1 - p0);

                CHECK(mv > 1u && mv <= nom,
                      "按住方向键 -> SG90(上) 按宏速度移动（曲线影响起步，略小于名义值）");
            }
            CHECK(g_fake_tim3.CCR4 == Servo_GetPulse(SERVO_CH_ARM_UP),
                  "SG90(上) 在 TIM3_CH4(PB1) 上输出");

            /* 一直按住到端头 -> 被 SERVO3_MIN/MAX_US 限住；tick 数给足 */
            for (int i = 0; i < 3000; i++) servo_task_buttons((uint16_t)(~PS2_BTN_R2));
            CHECK(Servo_GetPulse(SERVO_CH_ARM_UP) == SERVO3_MIN_US, "R2 按到底 -> 限幅到安全下限");
            for (int i = 0; i < 3000; i++) servo_task_buttons((uint16_t)(~PS2_BTN_R1));
            CHECK(Servo_GetPulse(SERVO_CH_ARM_UP) == SERVO3_MAX_US, "R1 按到底 -> 限幅到安全上限");
        }

        /* ===== 机械爪点动（带加减速曲线 + 近闭合端减速） ===== */
        Servo_Init();
        {
            const uint16_t p0  = Servo_GetPulse(SERVO_CH_GRIPPER);
            const uint16_t nom = (uint16_t)(SERVO_GRIP_STEP_US10 * 30u / 10u);

            for (int i = 0; i < 30; i++) servo_task_buttons((uint16_t)(~PS2_BTN_CIRCLE));
            {
                const uint16_t moved = (uint16_t)(p0 - Servo_GetPulse(SERVO_CH_GRIPPER));

                CHECK(moved > 1u && moved <= nom, "机械爪点动 30 tick 位移不超过名义值（曲线）");
            }
        }
        CHECK(SERVO_MG995_DEG_PER_SEC == 114, "MG995 目标速度 = 114°/s（与车2 一致）");
        /* 步长由 SERVO_STEP_FROM_DEG 自动换算（随 SERVO_TICK_MS 变化），这里只校验"物理速度对得上" */
        {
            /* 每 tick 步长(0.1us) × (1000/SERVO_TICK_MS) tick/s ÷ 10 ÷ 11.111 = °/s */
            const uint32_t deg_s = (uint32_t)SERVO_MG995_STEP_US10 * (1000u / SERVO_TICK_MS) / 111u;

            CHECK(deg_s + 2u >= SERVO_MG995_DEG_PER_SEC && deg_s <= SERVO_MG995_DEG_PER_SEC + 2u,
                  "MG995 实际速度 ≈ 目标速度（步长与节拍自洽）");
        }
        /* 两个 SG90 各自：步长(0.1us/tick) 与 目标速度 自洽 */
        {
            const uint16_t want_low = (uint16_t)SERVO_STEP_FROM_DEG(SERVO_SG90_LOW_DEG_PER_SEC);
            const uint16_t want_up  = (uint16_t)SERVO_STEP_FROM_DEG(SERVO_SG90_UP_DEG_PER_SEC);

            CHECK(SERVO_SG90_LOW_STEP_US10 == want_low, "SG90(下) 步长按节拍自动换算");
            CHECK(SERVO_SG90_UP_STEP_US10 == want_up, "SG90(上) 步长按节拍自动换算");
            CHECK(SERVO_SG90_UP_STEP_US10 > SERVO_SG90_LOW_STEP_US10,
                  "SG90(上) 比 SG90(下) 快（1.1 倍）");
        }
        /* 机械爪：步长同样自动换算 */
        {
            const uint16_t want = (uint16_t)SERVO_STEP_FROM_DEG(SERVO_GRIP_DEG_PER_SEC);

            CHECK(SERVO_GRIP_STEP_US10 == want,
                  "机械爪步长按节拍自动换算（改 SERVO_GRIP_DEG_PER_SEC 自动跟随）");
        }

        /* 机械爪：□ 张开 / ○ 合拢（一键式；默认模式0是点动，这里先切模式1） */
        Servo_Init();
        Servo_SetGripMode(SERVO_GRIP_MODE_ONESHOT);
        for (int i = 0; i < 100; i++) servo_task_buttons((uint16_t)(~PS2_BTN_SQUARE));
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US, "按住 □ -> 机械爪张开到位");
        CHECK(g_fake_tim3.CCR3 == SERVO_GRIP_OPEN_US, "机械爪 CCR3 同步更新");
        for (int i = 0; i < 100; i++) servo_task_buttons((uint16_t)(~PS2_BTN_CIRCLE));
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US, "按住 ○ -> 机械爪合拢到位");

        /* ---------- 机械臂总控模式（SELECT 切换；上电默认 = 模式 0 全点动） ---------- */
        Servo_Init();
        CHECK(Servo_GetGripMode() == SERVO_GRIP_MODE_JOG, "上电默认 = 点动模式(0)");
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US, "上电停在完全张开 90°");
        CHECK(SERVO_GRIP_OPEN_DEG == 90 &&
              SERVO_GRIP_CLOSE_DEG == SERVO_GRIP_OPEN_DEG - SERVO_GRIP_ONESHOT_DEG &&
              SERVO_GRIP_ONESHOT_DEG > 0 && SERVO_GRIP_ONESHOT_DEG < SERVO_GRIP_OPEN_DEG,
              "一键张合：合上角 = 全开角 - 一键行程（行程在 0~90° 之间）");
        CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US, "上电目标 = 完全张开");

        /* 按一次 SELECT -> 模式 1（一键张合） */
        s_tick += SERVO_MODE_TOGGLE_MS;
        servo_task_buttons((uint16_t)(~PS2_BTN_SELECT));
        CHECK(Servo_GetGripMode() == SERVO_GRIP_MODE_ONESHOT, "SELECT -> 切到一键张合模式(1)");
        servo_task_buttons(0xFFFF);

        /* 一键式：按一次自动走到位，松开不影响 */
        for (int i = 0; i < 100; i++) servo_task_buttons((uint16_t)(~PS2_BTN_CIRCLE));
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US,
              "一键式：按 ○ 自动合上（SERVO_GRIP_CLOSE_US）");
        servo_task_buttons(0xFFFF);                                     /* 松开 */
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US,
              "一键式：松开后保持在合上位置");
        for (int i = 0; i < 100; i++) servo_task_buttons((uint16_t)(~PS2_BTN_SQUARE));
        CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_OPEN_US,
              "一键式：按 □ 自动张开回 90°");

        /* SELECT -> 切到点动保持模式 */
        s_tick += SERVO_MODE_TOGGLE_MS;
        servo_task_buttons((uint16_t)(~PS2_BTN_SELECT));
        CHECK(Servo_GetGripMode() == SERVO_GRIP_MODE_JOG, "SELECT -> 切到点动保持模式(0)");
        servo_task_buttons((uint16_t)(~PS2_BTN_SELECT));                /* 仍按住 */
        CHECK(Servo_GetGripMode() == SERVO_GRIP_MODE_JOG, "按住 SELECT 不会连续切换");

        /* 点动模式：按住才动，松开就停在当前位置（速度见 SERVO_GRIP_DEG_PER_SEC，曲线影响起步） */
        s_tick += SERVO_MODE_TOGGLE_MS;
        servo_task_buttons(0xFFFF);                                     /* 松开 SELECT */
        {
            uint16_t p0 = Servo_GetPulse(SERVO_CH_GRIPPER);
            uint16_t p1;

            for (int i = 0; i < 40; i++) servo_task_buttons((uint16_t)(~PS2_BTN_CIRCLE));
            p1 = Servo_GetPulse(SERVO_CH_GRIPPER);
            {
                const uint16_t moved = (uint16_t)(p0 - p1);
                const uint16_t nom   = (uint16_t)(40u * SERVO_GRIP_STEP_US10 / 10u);

                CHECK(moved > 1u && moved <= nom,
                      "点动：按住 ○ 向闭合方向逐步走（不超过名义速度）");
            }

            /* 松开后：目标不再变化，舵机插值到该目标就停住 */
            {
                const uint16_t tgt_after = Servo_GetTarget(SERVO_CH_GRIPPER);

                for (int i = 0; i < 300; i++) servo_task_buttons(0xFFFF);
                CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == tgt_after,
                      "点动：松开后目标不再变化（没有继续走）");
                CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == tgt_after,
                      "点动：松开后停在该目标（不像一键式那样继续走到端点）");
                p1 = tgt_after;
            }

            for (int i = 0; i < 40; i++) servo_task_buttons((uint16_t)(~PS2_BTN_SQUARE));
            {
                const uint16_t moved = (uint16_t)(Servo_GetPulse(SERVO_CH_GRIPPER) - p1);
                const uint16_t nom   = (uint16_t)(40u * SERVO_GRIP_STEP_US10 / 10u);

                CHECK(moved > 1u && moved <= nom, "点动：按住 □ 向张开方向逐步走");
            }
        }

        /* 再切回一键模式：不应立刻动作，按键后才走到端点 */
        {
            uint16_t tgt_before = Servo_GetTarget(SERVO_CH_GRIPPER);

            s_tick += SERVO_MODE_TOGGLE_MS;
            servo_task_buttons((uint16_t)(~PS2_BTN_SELECT));
            CHECK(Servo_GetGripMode() == SERVO_GRIP_MODE_ONESHOT, "再按 SELECT -> 切回一键张合");
            CHECK(Servo_GetTarget(SERVO_CH_GRIPPER) == tgt_before,
                  "切模式不会立刻改变目标（保持当前位置，不会自己跑回张开位）");

            for (int i = 0; i < 300; i++) servo_task_buttons(0xFFFF);   /* 松开 SELECT，等插值走完 */
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == tgt_before,
                  "切换后松开按键，机械爪停在原位不动作");

            for (int i = 0; i < 200; i++) servo_task_buttons((uint16_t)(~PS2_BTN_CIRCLE));
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == SERVO_GRIP_CLOSE_US,
                  "切换后按 ○ 才一次走到闭合端点（683us）");
        }

        /* ===== 加减速曲线：起步比巡航慢，巡航阶段按名义速度 ===== */
        Servo_Init();
        {
            const uint16_t p0 = Servo_GetPulse(SERVO_CH_JOINT1);

            for (int i = 0; i < 3; i++) servo_task_stick(0x00);     /* 起步前 3 tick */
            {
                const uint16_t early = (uint16_t)(Servo_GetPulse(SERVO_CH_JOINT1) - p0);
                const uint16_t nom3  = (uint16_t)(3u * SERVO_MG995_STEP_US10 / 10u);

                CHECK(early < nom3, "加减速曲线：起步阶段位移小于巡航名义值（更柔和）");
            }

            for (int i = 0; i < 100; i++) servo_task_stick(0x00);    /* 进入巡航 */
            {
                const uint16_t a = Servo_GetPulse(SERVO_CH_JOINT1);

                for (int i = 0; i < 20; i++) servo_task_stick(0x00);
                {
                    const uint16_t moved = (uint16_t)(Servo_GetPulse(SERVO_CH_JOINT1) - a);
                    const uint16_t nom   = (uint16_t)(20u * SERVO_MG995_STEP_US10 / 10u);

                    CHECK(moved + 2u >= nom && moved <= nom + 2u,
                          "加减速曲线：巡航阶段按名义速度走");
                }
            }
        }

        /* ===== 错峰动作：多个动作同时请求时，只执行优先级最高的一路 ===== */
        Servo_Init();
        Servo_SetGripMode(SERVO_GRIP_MODE_JOG);
        {
            const uint16_t ch2_0 = Servo_GetPulse(SERVO_CH_JOINT2);
            const uint16_t gr_0  = Servo_GetPulse(SERVO_CH_GRIPPER);

            /* L1(SG90下，优先级高于机械爪) + □(机械爪) 同时按 -> 只有 SG90(下) 动。
             * 注意：CH2 上电可能停在行程端，所以按"有余量的方向"选 L1/L2 */
            {
                const uint16_t dir_btn = (ch2_0 + 100u <= SERVO1_MAX_US) ? (uint16_t)PS2_BTN_L1
                                                                         : (uint16_t)PS2_BTN_L2;

                for (int i = 0; i < 40; i++)
                    servo_task_buttons((uint16_t)(~(dir_btn | PS2_BTN_SQUARE)));
            }
            CHECK(Servo_GetPulse(SERVO_CH_JOINT2) != ch2_0,
                  "错峰：高优先级 SG90(下) 正常动作");
            CHECK(Servo_GetPulse(SERVO_CH_GRIPPER) == gr_0,
                  "错峰：同一时刻机械爪不动（让给更高优先级的 SG90）");
        }
        Servo_Init();
        {
            const uint16_t ch1_0 = Servo_GetPulse(SERVO_CH_JOINT1);
            const uint16_t ch4_0 = Servo_GetPulse(SERVO_CH_ARM_UP);

            /* 摇杆(最高) + R1 同时给 -> 只有 MG995 动 */
            for (int i = 0; i < 40; i++)
                servo_task_stick_and_buttons(0x00, (uint16_t)(~PS2_BTN_R1));
            CHECK(Servo_GetPulse(SERVO_CH_JOINT1) != ch1_0,
                  "错峰：最高优先级 MG995(摇杆) 正常动作");
            CHECK(Servo_GetPulse(SERVO_CH_ARM_UP) == ch4_0,
                  "错峰：同一时刻 SG90(上) 不动（让给 MG995）");
        }

        /* ===== 机械爪：靠近闭合端自动减速（越近越慢） ===== */
        {
            uint16_t dist_far_us, dist_near_us, fast, slow;

            /* 远处（离闭合端 40°）按住 ○ 20 tick 的位移 */
            Servo_Init();
            Servo_SetGripMode(SERVO_GRIP_MODE_JOG);
            dist_far_us = (uint16_t)(40u * 1000u / 90u);
            Servo_SetPulse(SERVO_CH_GRIPPER, (uint16_t)(SERVO2_MIN_US + dist_far_us));
            {
                const uint16_t p0 = Servo_GetPulse(SERVO_CH_GRIPPER);

                for (int i = 0; i < 20; i++) servo_task_buttons((uint16_t)(~PS2_BTN_CIRCLE));
                fast = (uint16_t)(p0 - Servo_GetPulse(SERVO_CH_GRIPPER));
            }

            /* 近处（离闭合端 15°，已进入慢速区）同样 20 tick 的位移 */
            Servo_Init();
            Servo_SetGripMode(SERVO_GRIP_MODE_JOG);
            dist_near_us = (uint16_t)(15u * 1000u / 90u);
            Servo_SetPulse(SERVO_CH_GRIPPER, (uint16_t)(SERVO2_MIN_US + dist_near_us));
            {
                const uint16_t p0 = Servo_GetPulse(SERVO_CH_GRIPPER);

                for (int i = 0; i < 20; i++) servo_task_buttons((uint16_t)(~PS2_BTN_CIRCLE));
                slow = (uint16_t)(p0 - Servo_GetPulse(SERVO_CH_GRIPPER));
            }

            CHECK(SERVO_GRIP_SLOW_ENABLE == 1, "机械爪靠近闭合端减速功能已开启");
            CHECK(fast > 0u && slow > 0u && slow < fast,
                  "机械爪：靠近闭合端时速度明显变慢（越近越慢）");
        }
    }

    /* ---------- 9. 掉出模拟模式自动拉回 ---------- */
    printf("[9] 掉出模拟模式自动拉回\n");
    {
        PS2_Data_t d;
        uint32_t c1;

        reset_pad();
        pad_id = 0x73;
        s_tick = 0x7FFFFFFFu;
        (void)PS2_ReadData(&d);
        c1 = g_ps2_dbg.mode_recover_count;

        pad_id = 0x41;                                  /* 手柄掉到数字(绿灯)模式 */
        (void)PS2_ReadData(&d);
        CHECK(d.mode == 0, "掉出模拟模式 -> mode=0（停车）");
        CHECK(g_ps2_dbg.mode_recover_count == c1 + 1, "自动重发配置序列把它拉回红灯模式");
        CHECK(s_packets >= 4, "确实发出了配置包（enter/type/set_mode/exit）");

        (void)PS2_ReadData(&d);
        CHECK(g_ps2_dbg.mode_recover_count == c1 + 1, "限速：同一时刻不会每次读取都重发");

        s_tick += PS2_RECONFIG_INTERVAL_MS + 10u;
        (void)PS2_ReadData(&d);
        CHECK(g_ps2_dbg.mode_recover_count == c1 + 2, "超过间隔后再次重发");

        /* 模拟模式下不应触发 */
        pad_id = 0x73;
        c1 = g_ps2_dbg.mode_recover_count;
        s_tick += PS2_RECONFIG_INTERVAL_MS + 10u;
        (void)PS2_ReadData(&d);
        CHECK(g_ps2_dbg.mode_recover_count == c1, "模拟模式正常时不重发配置");
    }

    /* ---------- 10b. 电机软启动 + 加减速曲线（Mecanum_MoveLimitSlew） ---------- */
    printf("[10b] 电机软启动\n");
#if MOTOR_SLEW_ENABLE
    {
        memset(&g_fake_tim2, 0, sizeof g_fake_tim2);
        memset(&g_fake_tim4, 0, sizeof g_fake_tim4);
        Motor_StopAll();                        /* 顺带清零软启动状态 */

        /* 第一步：曲线起步，增量比巡航小（不会一步跳到位） */
        Mecanum_MoveLimitSlew(1000, 0, 0, 1000);
        {
            const uint32_t c1 = g_fake_tim2.CCR1;
            const uint32_t c3 = g_fake_tim2.CCR3;
            const uint32_t c4 = g_fake_tim4.CCR1;
            const uint32_t c6 = g_fake_tim4.CCR3;

            CHECK(c1 > 0 && c1 <= (uint32_t)MOTOR_SLEW_STEP && c1 == c3 && c3 == c4 && c4 == c6,
                  "软启动：第一步增量 ≤ MOTOR_SLEW_STEP（不会一步跳到位）");
            CHECK(c1 < (uint32_t)MOTOR_SLEW_STEP,
                  "加减速曲线：马达起步增量小于巡航增量（起步更柔和）");
        }

        /* 连续调用足够多次后应到达目标（曲线会多花几个 tick） */
        for (int i = 0; i < (2000 / MOTOR_SLEW_STEP) + 100; i++) Mecanum_MoveLimitSlew(1000, 0, 0, 1000);
        CHECK(g_fake_tim2.CCR1 == 1000 && g_fake_tim2.CCR3 == 1000 &&
              g_fake_tim4.CCR1 == 1000 && g_fake_tim4.CCR3 == 1000,
              "软启动：持续调用后爬到目标值");

        /* 反向：从 +1000 到 -1000 也要逐步过渡（不会瞬间反接） */
        memset(&g_fake_tim2, 0, sizeof g_fake_tim2);
        memset(&g_fake_tim4, 0, sizeof g_fake_tim4);
        Mecanum_MoveLimitSlew(-1000, 0, 0, 1000);
        CHECK(g_fake_tim2.CCR1 < 1000u &&
              g_fake_tim2.CCR1 >= (uint32_t)(1000 - MOTOR_SLEW_STEP),
              "软启动：正转->反转是逐步过渡（第一步只降一点，不会瞬间反接）");
        for (int i = 0; i < (4000 / MOTOR_SLEW_STEP) + 100; i++) Mecanum_MoveLimitSlew(-1000, 0, 0, 1000);
        CHECK(g_fake_tim2.CCR2 == 1000 && g_fake_tim2.CCR1 == 0,
              "软启动：逐步过渡后完成反转（IN2 占空、IN1 归零）");

        /* 停车会清零软启动状态：再起步又从小增量开始 */
        Motor_StopAll();
        memset(&g_fake_tim2, 0, sizeof g_fake_tim2);
        Mecanum_MoveLimitSlew(1000, 0, 0, 1000);
        CHECK(g_fake_tim2.CCR1 > 0 && g_fake_tim2.CCR1 <= (uint32_t)MOTOR_SLEW_STEP,
              "停车后软启动状态清零（下次起步从零开始爬）");

        Motor_SlewReset();
    }
#else
    printf("  [SKIP] MOTOR_SLEW_ENABLE=0\n");
#endif

    /* ---------- 10. 独立看门狗（IWDG，1 秒超时） ---------- */
    printf("[10] 独立看门狗\n");
    {
        memset(&g_fake_iwdg, 0, sizeof g_fake_iwdg);
        memset(&g_fake_dbgmcu, 0, sizeof g_fake_dbgmcu);

        Watchdog_Init();
        CHECK(g_fake_iwdg.PR == WATCHDOG_PRESCALER, "预分频 = /64 -> 625Hz 计数");
        CHECK(g_fake_iwdg.RLR == WATCHDOG_RELOAD, "重装值 = 625");
        CHECK(g_fake_iwdg.KR == 0x0000CCCCU, "以 0xCCCC 启动（启动后软件不能关）");
        CHECK((g_fake_dbgmcu.CR & DBGMCU_CR_DBG_IWDG_STOP) != 0,
              "已置 DBG_IWDG_STOP：调试器暂停时冻结看门狗，不会踢复位");
        CHECK(WATCHDOG_RELOAD <= 4095u, "重装值在 12 位范围内");

        /* 标称超时 = 4 × 2^PRESCALER × RELOAD / 40000 (LSI 典型 40kHz) */
        {
            uint32_t nominal_ms = (uint32_t)(4UL << WATCHDOG_PRESCALER) * WATCHDOG_RELOAD / 40UL;
            CHECK(nominal_ms == 1000u, "标称超时 = 1000ms（1 秒）");
        }

        /* 喂狗 */
        g_fake_iwdg.KR = 0;
        Watchdog_Feed();
        CHECK(g_fake_iwdg.KR == 0x0000AAAAU, "喂狗写 0xAAAA（重装载）");
    }

    printf("\n结果: %d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
