#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"

// ===== 电机编号 =====
#define MOTOR_LF  0   // 左前
#define MOTOR_LR  1   // 左后
#define MOTOR_RF  2   // 右前
#define MOTOR_RR  3   // 右后

// ===== 四轮接线自检开关 =====
// 置 1 后，Motor_Init() 会依次让四个轮子各正转/反转一次，用于校验接线、转向与索引对应关系。
// 注意：自检会让轮子真的转起来，务必先把车体架空再上电！测完请置回 0。
#define MOTOR_SELFTEST_ENABLE  0
#define MOTOR_SELFTEST_DUTY    300   // 30% 占空比（刻度 1000）

// ===== 刹车方式 =====
// DRV8833：IN1=IN2=1（两路同时给满占空比）为"刹车"，IN1=IN2=0 为"滑行"。
// 若你的驱动真值表不同（少数驱动 0/0 才是刹车），把下面改成 0（也可用 -D 覆盖）。
#ifndef MOTOR_BRAKE_BOTH_HIGH
#define MOTOR_BRAKE_BOTH_HIGH  1
#endif

// ===== 软启动 + 加减速曲线（防启动浪涌 / 降低电机线 dV/dt 干扰）=====
// 主循环每次 tick 调一次 Mecanum_MoveLimitSlew()，每轮"巡航增量" = MOTOR_SLEW_STEP。
// 注意：节拍是 **10ms**（见 main.c 的 PS2_READ_PERIOD_MS），所以这里是"每 10ms 的步长"。
// 26 = 每 tick 2.6% -> 满量程(1000)约 385ms 爬到位（与 5ms×13 相同）。
// 曲线：起步/收尾阶段增量更小，中间才是 MOTOR_SLEW_STEP（见 motor.c 的 slew_step）。
#ifndef MOTOR_SLEW_ENABLE
#define MOTOR_SLEW_ENABLE 1
#endif
#ifndef MOTOR_SLEW_STEP
#define MOTOR_SLEW_STEP   26
#endif
#ifndef MOTOR_RAMP_TICKS
#define MOTOR_RAMP_TICKS      5    /* 收尾减速段（tick，10ms -> 50ms） */
#endif
#ifndef MOTOR_RAMP_ACCEL_DIV
#define MOTOR_RAMP_ACCEL_DIV  8    /* 起步：约 8 tick(80ms) 加速到巡航增量 */
#endif
#ifndef MOTOR_RAMP_TAIL_DIV
#define MOTOR_RAMP_TAIL_DIV   4    /* 末端最低速 = 巡航增量/本值（防最后几格爬太久） */
#endif

// ===== 函数声明 =====
void Motor_Init(void);
void Motor_SetPWM(uint8_t motor, int16_t speed);
void Motor_StopAll(void);
void Motor_BrakeAll(void);
void Mecanum_Move(int16_t vx, int16_t vy, int16_t vw);
void Mecanum_MoveLimit(int16_t vx, int16_t vy, int16_t vw, int16_t limit);

// ===== 软启动（主循环用这个）=====
// 与 Mecanum_MoveLimit 相同，但每个轮子每次调用最多变化 MOTOR_SLEW_STEP。
#if MOTOR_SLEW_ENABLE
void Mecanum_MoveLimitSlew(int16_t vx, int16_t vy, int16_t vw, int16_t limit);
void Motor_SlewReset(void);      // 清零软启动状态（Motor_StopAll/BrakeAll 内部已自动调用）
#endif

#if MOTOR_SELFTEST_ENABLE
void Motor_SelfTest(void);
extern volatile uint8_t g_motor_testing;   // 调试器里看：0=空闲，1~4=当前测试的轮子(MOTOR_xx+1)
#endif

#endif
