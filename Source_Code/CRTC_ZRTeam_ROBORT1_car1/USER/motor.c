//
// Created by SukuraFalling on 2026/10/14.
//
#include "motor.h"

// ===== 电机PWM通道映射（DRV8833：每路 IN1/IN2 两个输入，用 "PWM+0" / "0+PWM" 控制正反转）=====
// 左前：PA0(TIM2_CH1=IN1) / PA1(TIM2_CH2=IN2)
// 左后：PA2(TIM2_CH3) / PA3(TIM2_CH4)
// 右前：PB6(TIM4_CH1) / PB7(TIM4_CH2)
// 右后：PB8(TIM4_CH3) / PB9(TIM4_CH4)

// ===== 标定宏 =====
// 单轮方向：TT 马达左右镜像安装时，同一接线往往一侧正转另一侧反转。
// 哪个轮子方向反了，就把对应宏改成 (-1)（也可以在接线/驱动端对调该马达两根线）
// 注：麦轮整体的"前后/旋转/绕转"方向标定已统一挪到 USER/control.h 的 CTRL_*_SIGN，
//     这里不再放 MECANUM_*_SIGN（控制层的绕后轮推导依赖本文件的 vy/vw 系数约定）。
#define MOTOR_SIGN_LF  (+1)
#define MOTOR_SIGN_LR  (+1)
#define MOTOR_SIGN_RF  (+1)
#define MOTOR_SIGN_RR  (+1)

typedef struct {
    TIM_HandleTypeDef *htim_pwm;   // PWM引脚所在定时器
    uint32_t pwm_channel;          // PWM通道
    TIM_HandleTypeDef *htim_dir;   // 方向引脚所在定时器
    uint32_t dir_channel;          // 方向通道
} Motor_t;

static Motor_t motors[4] = {
    // 左前：PA0(PWM) / PA1(方向)
    { &htim2, TIM_CHANNEL_1, &htim2, TIM_CHANNEL_2 },
    // 左后：PA2(PWM) / PA3(方向)
    { &htim2, TIM_CHANNEL_3, &htim2, TIM_CHANNEL_4 },
    // 右前：PB6(PWM) / PB7(方向)
    { &htim4, TIM_CHANNEL_1, &htim4, TIM_CHANNEL_2 },
    // 右后：PB8(PWM) / PB9(方向)
    { &htim4, TIM_CHANNEL_3, &htim4, TIM_CHANNEL_4 },
};

// ===== 初始化：统一 PWM 频率 + 启动所有PWM通道 =====
void Motor_Init(void)
{
    // 频率标定：计数时钟 = 72MHz/(72) = 1MHz，ARR = 1000-1 -> 1kHz
    // 与下面 Motor_SetPWM 的 1000 刻度一致。
    // 原 CubeMX 配置为 Prescaler=71、Period=65535：只有 15.26Hz、最大占空比 1000/65536≈1.5%，
    // 电机因此几乎不转。这里在 USER 层覆盖，CubeMX 重新生成 Core/Src/tim.c 也不会丢。
    __HAL_TIM_SET_PRESCALER(&htim2, 72U - 1U);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 1000U - 1U);
    __HAL_TIM_SET_PRESCALER(&htim4, 72U - 1U);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 1000U - 1U);
    htim2.Instance->EGR = TIM_EGR_UG;   // 立即把新的 PSC/ARR 装入影子寄存器
    htim4.Instance->EGR = TIM_EGR_UG;

    for (uint8_t i = 0; i < 4; i++)
    {
        HAL_TIM_PWM_Start(motors[i].htim_pwm, motors[i].pwm_channel);
        HAL_TIM_PWM_Start(motors[i].htim_dir, motors[i].dir_channel);
        // 初始占空比设为0，电机停止
        __HAL_TIM_SET_COMPARE(motors[i].htim_pwm, motors[i].pwm_channel, 0);
        __HAL_TIM_SET_COMPARE(motors[i].htim_dir, motors[i].dir_channel, 0);
    }

#if MOTOR_SELFTEST_ENABLE
    Motor_SelfTest();
#endif
}

// ===== 设置单个电机速度 =====
// speed范围：-1000 ~ +1000，正数正转，负数反转，0停止
void Motor_SetPWM(uint8_t motor, int16_t speed)
{
    static const int8_t wheel_sign[4] = {MOTOR_SIGN_LF, MOTOR_SIGN_LR, MOTOR_SIGN_RF, MOTOR_SIGN_RR};

    if (motor > 3) return;

    speed = (int16_t)(speed * wheel_sign[motor]);   // 单轮方向标定

    // 限幅
    if (speed > 1000)  speed = 1000;
    if (speed < -1000) speed = -1000;

    uint16_t pwm_val = (speed >= 0) ? (uint16_t)speed : (uint16_t)(-speed);
    // PWM周期由 Motor_Init 统一设为 1000，占空比 = pwm_val/1000

    if (speed > 0) {
        // 正转：IN1 输出占空比，IN2 输出0
        __HAL_TIM_SET_COMPARE(motors[motor].htim_pwm, motors[motor].pwm_channel, pwm_val);
        __HAL_TIM_SET_COMPARE(motors[motor].htim_dir, motors[motor].dir_channel, 0);
    } else if (speed < 0) {
        // 反转：IN1 输出0，IN2 输出占空比
        __HAL_TIM_SET_COMPARE(motors[motor].htim_pwm, motors[motor].pwm_channel, 0);
        __HAL_TIM_SET_COMPARE(motors[motor].htim_dir, motors[motor].dir_channel, pwm_val);
    } else {
        // 停止：两路都输出0
        __HAL_TIM_SET_COMPARE(motors[motor].htim_pwm, motors[motor].pwm_channel, 0);
        __HAL_TIM_SET_COMPARE(motors[motor].htim_dir, motors[motor].dir_channel, 0);
    }
}

// ===== 全部停止（滑行：两路输出都 0）=====
void Motor_StopAll(void)
{
    for (uint8_t i = 0; i < 4; i++)
        Motor_SetPWM(i, 0);
#if MOTOR_SLEW_ENABLE
    Motor_SlewReset();       // 停车后清软启动状态，下次起步从 0 开始爬
#endif
}

// ===== 全部刹车（✕ 键用）=====
// DRV8833：IN1=IN2=1 -> 两路输出都拉低，电机绕组短路 -> 立即制动（而不是滑行）。
// 用 100% 占空比实现；若驱动真值表不同，见 motor.h 的 MOTOR_BRAKE_BOTH_HIGH。
void Motor_BrakeAll(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
#if MOTOR_BRAKE_BOTH_HIGH
        __HAL_TIM_SET_COMPARE(motors[i].htim_pwm, motors[i].pwm_channel, 1000);
        __HAL_TIM_SET_COMPARE(motors[i].htim_dir, motors[i].dir_channel, 1000);
#else
        Motor_SetPWM(i, 0);
#endif
    }
#if MOTOR_SLEW_ENABLE
    Motor_SlewReset();       // 刹车后清软启动状态，松开刹车时从 0 开始爬
#endif
}

// ===== 麦轮运动学逆解（标准 X 型布局）=====
// vx: 前后平移（正=前进），vy: 左右平移，vw: 旋转
// 判据：前进时四个轮子必须同向；左右平移为对角配对；旋转为左右两侧反向
//
// 注意：这里**不再放方向标定宏**。控制层（control.c）的"绕后轮旋转"是按本函数的
//       vy/vw 系数约定推导出来的，任何在这里翻转 vy/vw 符号的做法都会破坏那个推导。
//       方向标定统一在 USER/control.h 的 CTRL_*_SIGN 宏里做。
//
// limit: 四轮输出上限（等比缩放，保持四轮比例 = 不改变运动方向）
// out[4]: 依次为 LF / LR / RF / RR 的占空比（-1000~1000）
static void mecanum_compute(int16_t vx, int16_t vy, int16_t vw, int16_t limit, int16_t out[4])
{
    if (limit <= 0)    limit = 1;
    if (limit > 1000)  limit = 1000;

    // 逆运动学解算
    // 注意：vx 的系数必须四个轮子全为 +1，否则"前进"会变成对角差速（原地打转/斜移）
    out[0] =  vx + vy + vw;   // 左前 LF
    out[1] =  vx - vy + vw;   // 左后 LR
    out[2] =  vx - vy - vw;   // 右前 RF
    out[3] =  vx + vy - vw;   // 右后 RR

    // 饱和处理：任一轴超程时按最大绝对值等比缩小，保证运动方向不变
    // （逐轮单独限幅会改变四轮比例，导致斜移/旋转时方向畸变）
    int16_t peak = 0;
    for (uint8_t i = 0; i < 4; i++) {
        int16_t a = (out[i] < 0) ? (int16_t)(-out[i]) : out[i];
        if (a > peak) peak = a;
    }
    if (peak > limit) {
        for (uint8_t i = 0; i < 4; i++)
            out[i] = (int16_t)((int32_t)out[i] * limit / peak);
    }
    // 兜底限幅
    for (uint8_t i = 0; i < 4; i++) {
        if (out[i] > limit)  out[i] = limit;
        if (out[i] < -limit) out[i] = -limit;
    }
}

static void mecanum_apply(int16_t vx, int16_t vy, int16_t vw, int16_t limit)
{
    int16_t vals[4];

    mecanum_compute(vx, vy, vw, limit, vals);

    Motor_SetPWM(MOTOR_LF, vals[0]);
    Motor_SetPWM(MOTOR_LR, vals[1]);
    Motor_SetPWM(MOTOR_RF, vals[2]);
    Motor_SetPWM(MOTOR_RR, vals[3]);
}

void Mecanum_Move(int16_t vx, int16_t vy, int16_t vw)
{
    mecanum_apply(vx, vy, vw, 1000);
}

/* ===== 软启动 + 加减速曲线 =====
 * 目的：指令跳变（推杆 0 -> 60%）时不让四个电机同时瞬变，
 *       削掉启动浪涌电流和电机线上的 dV/dt（电磁干扰源）。
 * 曲线（和车2 一致）：
 *   起步：每 tick 的"增量"从 0 逐渐加到 MOTOR_SLEW_STEP（约 8 tick = 40ms）；
 *   收尾：离目标近时增量按剩余距离线性减小，但不低于 巡航增量/MOTOR_RAMP_TAIL_DIV；
 *   中间：恒定 MOTOR_SLEW_STEP（= 1.3%/5ms，满量程约 385ms）。
 * 主循环每 tick（5ms）调一次，无阻塞。 */
#if MOTOR_SLEW_ENABLE
static int16_t s_slew_state[4] = {0, 0, 0, 0};   /* 每轮当前输出 */
static int16_t s_slew_rate[4]  = {0, 0, 0, 0};   /* 每轮当前"每 tick 增量"（加减速用） */

static int16_t slew_step(int16_t cur, int16_t tgt, uint8_t idx)
{
    const int16_t diff = (int16_t)(tgt - cur);
    int16_t dir;
    int16_t remain;
    int16_t v;
    int16_t vmin;
    int16_t accel;

    if (diff == 0)
    {
        s_slew_rate[idx] = 0;                    /* 到位：下次重新加速 */
        return cur;
    }

    dir    = (diff > 0) ? 1 : -1;
    remain = (int16_t)((diff > 0) ? diff : -diff);

    /* 收尾减速：剩余距离 / RAMP_TICKS */
    v = (int16_t)(remain / MOTOR_RAMP_TICKS);
    if (v > (int16_t)MOTOR_SLEW_STEP) v = (int16_t)MOTOR_SLEW_STEP;

    /* 末端最低速，避免最后几格爬太久 */
    vmin = (int16_t)(MOTOR_SLEW_STEP / MOTOR_RAMP_TAIL_DIV);
    if (vmin < 1) vmin = 1;
    if (v < vmin) v = vmin;

    /* 起步加速：每 tick 增量最多增加 巡航/MOTOR_RAMP_ACCEL_DIV */
    accel = (int16_t)(MOTOR_SLEW_STEP / MOTOR_RAMP_ACCEL_DIV);
    if (accel < 1) accel = 1;
    if (v > (int16_t)(s_slew_rate[idx] + accel)) v = (int16_t)(s_slew_rate[idx] + accel);
    if (v < 1) v = 1;

    s_slew_rate[idx] = v;

    if (v > remain) v = remain;
    return (int16_t)(cur + dir * v);
}

/* 清零软启动状态：停车/刹车后调用，保证下次起步从 0 开始爬 */
void Motor_SlewReset(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        s_slew_state[i] = 0;
        s_slew_rate[i]  = 0;
    }
}

// 带软启动 + 加减速曲线的版本（主循环用这个）
void Mecanum_MoveLimitSlew(int16_t vx, int16_t vy, int16_t vw, int16_t limit)
{
    int16_t vals[4];

    mecanum_compute(vx, vy, vw, limit, vals);

    for (uint8_t i = 0; i < 4; i++)
    {
        s_slew_state[i] = slew_step(s_slew_state[i], vals[i], i);
        vals[i] = s_slew_state[i];
    }

    Motor_SetPWM(MOTOR_LF, vals[0]);
    Motor_SetPWM(MOTOR_LR, vals[1]);
    Motor_SetPWM(MOTOR_RF, vals[2]);
    Motor_SetPWM(MOTOR_RR, vals[3]);
}
#endif  /* MOTOR_SLEW_ENABLE */

// 带输出上限的版本：用于"档位"限幅（四轮等比缩放到 limit 以内）
void Mecanum_MoveLimit(int16_t vx, int16_t vy, int16_t vw, int16_t limit)
{
    mecanum_apply(vx, vy, vw, limit);
}

// ===== 四轮接线自检（由 MOTOR_SELFTEST_ENABLE 控制）=====
#if MOTOR_SELFTEST_ENABLE
volatile uint8_t g_motor_testing = 0;

// 依次：左前正转1s -> 左前反转1s -> 左后 ... -> 右后，最后全部停止
// 观察点：每段时间内应当只有对应的一个轮子转动，且"正转"方向与预期一致
void Motor_SelfTest(void)
{
    static const uint8_t order[4] = {MOTOR_LF, MOTOR_LR, MOTOR_RF, MOTOR_RR};

    Motor_StopAll();
    HAL_Delay(500);

    for (uint8_t i = 0; i < 4; i++)
    {
        g_motor_testing = (uint8_t)(order[i] + 1);

        Motor_SetPWM(order[i], MOTOR_SELFTEST_DUTY);    // 正转 1s
        HAL_Delay(1000);
        Motor_SetPWM(order[i], -MOTOR_SELFTEST_DUTY);   // 反转 1s
        HAL_Delay(1000);
        Motor_SetPWM(order[i], 0);                      // 停 0.3s
        HAL_Delay(300);
    }

    g_motor_testing = 0;
    Motor_StopAll();
}
#endif
