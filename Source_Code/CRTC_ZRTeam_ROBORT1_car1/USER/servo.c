//
// Created by SukuraFalling on 2026/10/14.
// 舵机 PWM：TIM3 四通道，50Hz（1MHz 计数 / 20000 = 50Hz）
//
// 直接用寄存器初始化（不依赖 CubeMX 生成代码），CubeMX 重新生成也不会丢。
//
#include "servo.h"

#define SERVO_TIM          TIM3
#define SERVO_PSC          (72U - 1U)      /* 72MHz / 72 = 1MHz -> 1us 一个计数 */
#define SERVO_ARR          (20000U - 1U)   /* 1MHz / 20000 = 50Hz (20ms) */

/* 每路行程（由 servo.h 的角度宏自动换算） */
static const uint16_t s_min_us[SERVO_CH_NUM] = { SERVO0_MIN_US, SERVO1_MIN_US, SERVO2_MIN_US, SERVO3_MIN_US };
static const uint16_t s_max_us[SERVO_CH_NUM] = { SERVO0_MAX_US, SERVO1_MAX_US, SERVO2_MAX_US, SERVO3_MAX_US };
/* 上电默认位置（int32：允许按角度换算出的值暂时越界，由 clamp_us 限幅到 MIN/MAX） */
static const int32_t s_poweron_us[SERVO_CH_NUM] = { SERVO0_START_US, SERVO1_START_US,
                                                    SERVO2_POWERON_US, SERVO3_START_US };
/* 角度跨度（度）：Servo_SetAngle 的 0..SPAN 映射到 MIN_US..MAX_US */
static const uint16_t s_max_deg[SERVO_CH_NUM] = { SERVO0_SPAN_DEG, SERVO1_SPAN_DEG,
                                                  SERVO2_MAX_DEG, SERVO3_SPAN_DEG };

static uint16_t s_pulse_us[SERVO_CH_NUM];    /* 当前实际脉宽 */
static uint16_t s_target_us[SERVO_CH_NUM];   /* 目标脉宽 */

/* 每通道"目标移动速度"（单位 0.1us / 每 tick；tick = SERVO_TICK_MS = 主循环的 PS2_READ_PERIOD_MS）：
 *   步长由 servo.h 的 SERVO_STEP_FROM_DEG(°/s) 自动换算，这里只做通道分配 */
static const uint16_t s_step_us10[SERVO_CH_NUM] = {
    (uint16_t)SERVO_MG995_STEP_US10,      /* CH1 MG995 */
    (uint16_t)SERVO_SG90_LOW_STEP_US10,   /* CH2 SG90 下面 */
    (uint16_t)SERVO_GRIP_STEP_US10,       /* CH3 机械爪 */
    (uint16_t)SERVO_SG90_UP_STEP_US10     /* CH4 SG90 上面（速度可单独配） */
};
static uint16_t s_step_acc_x10[SERVO_CH_NUM];   /* 目标步长累加器 */

/* 每通道当前速度（0.1us / 每 tick），用于加减速曲线：0 -> 巡航速度 -> 收尾减速 */
static uint16_t s_ramp_us10[SERVO_CH_NUM];
static uint16_t s_slew_acc_x10[SERVO_CH_NUM];

/* 一键序列状态（L1=一键放下 / R1=一键摆臂），Servo_Init 会清零。
 * g_servo_seq_id  : 0=空闲, SERVO_SEQ_DROP=放下, SERVO_SEQ_POSE=摆臂
 * g_servo_seq_step: 0=空闲, 1..5=当前步骤
 * 两个都做成全局 volatile，方便调试器 Watch：step 非 0 表示"序列正在跑"，
 * 此时所有手动控制被跳过（表现就是"舵机都不听话"）。 */
volatile uint8_t g_servo_seq_id   = 0;
volatile uint8_t g_servo_seq_step = 0;
static uint8_t  s_l1_prev   = 0;
static uint8_t  s_l2_prev   = 0;
static uint8_t  s_r1_prev   = 0;
static uint8_t  s_r2_prev   = 0;
static uint32_t s_seq_ms    = 0;      /* 第4步"延时"的起始时刻 */

static uint16_t clamp_us(uint8_t ch, int32_t us)
{
    if (us < (int32_t)s_min_us[ch]) return s_min_us[ch];
    if (us > (int32_t)s_max_us[ch]) return s_max_us[ch];
    return (uint16_t)us;
}

#if SERVO_GRIP_RELEASE_MS > 0
/* 机械爪堵转保护：断开/恢复该通道（CH3）的 PWM 输出 */
static uint8_t  s_grip_attached = 1;
static uint32_t s_grip_hold_ms  = 0;

static void gripper_attach(uint8_t on)
{
    if (on) SERVO_TIM->CCER |=  TIM_CCER_CC3E;
    else    SERVO_TIM->CCER &= (uint16_t)~TIM_CCER_CC3E;
    s_grip_attached = on;
}
#endif

static void write_ccr(uint8_t ch, uint16_t us)
{
    switch (ch)
    {
    case 0: SERVO_TIM->CCR1 = us; break;
    case 1: SERVO_TIM->CCR2 = us; break;
    case 2: SERVO_TIM->CCR3 = us; break;
    case 3: SERVO_TIM->CCR4 = us; break;
    default: break;
    }
}

void Servo_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* PA6/PA7/PB0/PB1 -> 复用推挽输出 */
    gi.Mode  = GPIO_MODE_AF_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    gi.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gi);

    gi.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gi);

    /* 时基 */
    SERVO_TIM->PSC = SERVO_PSC;
    SERVO_TIM->ARR = SERVO_ARR;
    SERVO_TIM->CR1 = 0;

    /* CH1~CH4：PWM 模式 1（OCxM=110），使能预装载（OCxPE） */
    SERVO_TIM->CCMR1 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    SERVO_TIM->CCMR2 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);

    /* 使能四个通道输出 */
    SERVO_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    for (uint8_t ch = 0; ch < SERVO_CH_NUM; ch++)
    {
        s_pulse_us[ch]  = clamp_us(ch, s_poweron_us[ch]);   /* 上电默认位置 */
        s_target_us[ch] = s_pulse_us[ch];
        write_ccr(ch, s_pulse_us[ch]);
    }

    SERVO_TIM->EGR = TIM_EGR_UG;                    /* 立即装载 PSC/ARR/CCR */
    SERVO_TIM->SR  = (uint16_t)~TIM_SR_UIF;         /* 清更新标志 */
    SERVO_TIM->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;    /* 自动重装载 + 启动 */

    for (uint8_t i = 0; i < SERVO_CH_NUM; i++)
    {
        s_step_acc_x10[i] = 0;                      /* 目标步长累加器清零 */
        s_slew_acc_x10[i] = 0;                      /* 插值累加器清零 */
    }
    g_servo_seq_id   = SERVO_SEQ_NONE;              /* 一键序列复位 */
    g_servo_seq_step = 0;
    s_l1_prev   = 0;
    s_l2_prev   = 0;
    s_r1_prev   = 0;
    s_r2_prev   = 0;
    s_seq_ms    = 0;
    g_servo_grip_mode = SERVO_GRIP_MODE_DEFAULT;    /* 模式恢复上电默认（0=全点动） */

#if SERVO_GRIP_RELEASE_MS > 0
    s_grip_attached = 1;
    s_grip_hold_ms  = 0;
#endif
}

void Servo_SetPulse(uint8_t ch, uint16_t pulse_us)
{
    if (ch >= SERVO_CH_NUM) return;

    s_pulse_us[ch]  = clamp_us(ch, (int32_t)pulse_us);
    s_target_us[ch] = s_pulse_us[ch];
    write_ccr(ch, s_pulse_us[ch]);
}

// 按机械角度设置：pulse = MIN + (MAX-MIN) × 角度 / 该通道最大角度
void Servo_SetAngle(uint8_t ch, uint16_t deg)
{
    uint32_t span;

    if (ch >= SERVO_CH_NUM) return;
    if (deg > s_max_deg[ch]) deg = s_max_deg[ch];

    span = (uint32_t)(s_max_us[ch] - s_min_us[ch]);
    Servo_SetPulse(ch, (uint16_t)(s_min_us[ch] + span * deg / s_max_deg[ch]));
}

uint16_t Servo_GetPulse(uint8_t ch)
{
    return (ch < SERVO_CH_NUM) ? s_pulse_us[ch] : 0u;
}

uint16_t Servo_GetTarget(uint8_t ch)
{
    return (ch < SERVO_CH_NUM) ? s_target_us[ch] : 0u;
}

// 把某通道目标设为"当前位置 + delta"，并限幅
static void set_rel_target(uint8_t ch, int16_t delta)
{
    if (ch >= SERVO_CH_NUM) return;
    s_target_us[ch] = clamp_us(ch, (int32_t)s_target_us[ch] + delta);
}

// 把某通道目标直接设为指定脉宽
static void set_abs_target(uint8_t ch, uint16_t us)
{
    if (ch >= SERVO_CH_NUM) return;
    s_target_us[ch] = clamp_us(ch, (int32_t)us);
}

// 说明：原来内部有个 deg_to_us()（把 0..跨度 的角度线性映射到 MIN..MAX 脉宽），
// 只被"机械爪一键张合"用到；现在一键张合直接用 servo.h 换算好的绝对脉宽，故删除以免 unused 告警。
// 需要"角度 -> 脉宽"时用公共接口 Servo_SetAngle()。

/* ===== 机械爪控制模式 ===== */
volatile uint8_t g_servo_grip_mode = SERVO_GRIP_MODE_DEFAULT;

uint8_t Servo_GetGripMode(void) { return g_servo_grip_mode; }

void Servo_SetGripMode(uint8_t mode)
{
    g_servo_grip_mode = (mode == SERVO_GRIP_MODE_JOG) ? SERVO_GRIP_MODE_JOG : SERVO_GRIP_MODE_ONESHOT;

    /* 注意：切换模式时**不改变目标位置**——机械爪保持当前位置不动，
     * 只有操作手按下 □ / ○ 之后才会一次走到对应端点。
     * （否则从点动模式按 SELECT 切到一键模式时，爪子会突然自己跑回张开位，很吓人） */
}

/* 本 tick 该通道的"巡航速度上限"（0.1us/tick）。机械爪额外做"靠近闭合端减速"。 */
static int32_t ch_cruise_us10(uint8_t ch)
{
    int32_t vmax = (int32_t)s_step_us10[ch];

#if SERVO_GRIP_SLOW_ENABLE
    if (ch == SERVO_CH_GRIPPER)
    {
        /* 离闭合端（SERVO2_MIN_US）越近，速度越小：线性从 100% 降到 SERVO_GRIP_SLOW_MIN_PCT% */
        const int32_t dist     = (int32_t)s_pulse_us[ch] - (int32_t)SERVO2_MIN_US;
        const int32_t range_us = (int32_t)SERVO_GRIP_SLOW_RANGE_DEG * 1000 / 90;
        const int32_t span     = (int32_t)(SERVO2_MAX_US - SERVO2_MIN_US);

        if (range_us > 0 && dist < range_us && range_us <= span)
        {
            int32_t pct = (int32_t)SERVO_GRIP_SLOW_MIN_PCT +
                          (100 - (int32_t)SERVO_GRIP_SLOW_MIN_PCT) * (dist > 0 ? dist : 0) / range_us;

            vmax = vmax * pct / 100;
            if (vmax < 1) vmax = 1;
        }
    }
#endif

    return vmax;
}

/* 向目标插值（每 tick 更新一次，**不阻塞**）：用 0.1us 累加避免整数截断。
 * 加减速曲线：速度从 0 开始，每 tick 最多 +巡航/ACCEL_DIV，巡航速度 = 该通道步长；
 *             剩余距离不足 SERVO_RAMP_TICKS 个 tick 时按剩余距离线性降速 -> 起步/停止柔和。 */
static void slew_toward_target(uint8_t ch)
{
    const int32_t diff = (int32_t)s_target_us[ch] - (int32_t)s_pulse_us[ch];
    const int32_t vmax = ch_cruise_us10(ch);
    int32_t   v;
    uint16_t  step = 0;
    int32_t   remain10;

    if (diff == 0)
    {
        s_slew_acc_x10[ch] = 0;
        s_ramp_us10[ch]    = 0;                 /* 已到位：速度归零，下次重新加速 */
        return;
    }

    remain10 = (diff > 0 ? diff : -diff) * 10;  /* 剩余距离（0.1us） */

#if SERVO_RAMP_ENABLE
    /* 收尾减速：剩余距离 / RAMP_TICKS 就是允许的速度 */
    v = remain10 / (int32_t)SERVO_RAMP_TICKS;
    if (v > vmax) v = vmax;

    /* 末端最低速（巡航的 1/4）：避免最后几十 us 用 0.1us/tick 爬行太久 */
    {
        int32_t vmin = vmax / 4;

        if (vmin < 1) vmin = 1;
        if (v < vmin) v = vmin;
    }

    /* 起步加速：每 tick 最多增加 巡航速度/RAMP_ACCEL_DIV（至少 1） */
    {
        int32_t accel = vmax / (int32_t)SERVO_RAMP_ACCEL_DIV;

        if (accel < 1) accel = 1;
        if (v > (int32_t)s_ramp_us10[ch] + accel) v = (int32_t)s_ramp_us10[ch] + accel;
    }
    if (v < 1) v = 1;
    s_ramp_us10[ch] = (uint16_t)v;
#else
    v = vmax;
    s_ramp_us10[ch] = (uint16_t)v;
#endif

    /* v(0.1us/tick) -> 本 tick 实际走多少 us（累加器保证平均速度准确） */
    s_slew_acc_x10[ch] += (uint16_t)v;
    while (s_slew_acc_x10[ch] >= 10u)
    {
        s_slew_acc_x10[ch] -= 10u;
        step++;
    }
    if (step == 0u) return;                     // 本 tick 不足 1us，下次再走

    if ((int32_t)step * 10 > remain10) step = (uint16_t)((remain10 + 9) / 10);
    if (step == 0u) step = 1u;

    s_pulse_us[ch] = (uint16_t)((diff > 0) ? ((int32_t)s_pulse_us[ch] + step)
                                           : ((int32_t)s_pulse_us[ch] - step));
    write_ccr(ch, s_pulse_us[ch]);
}

/* 某通道本 tick 的目标步长（us）：按该通道速度累加 0.1us，避免整数截断导致速度偏小 */
static uint16_t ch_step_us(uint8_t ch)
{
    uint16_t step = 0;

    if (ch >= SERVO_CH_NUM) return 0u;

    s_step_acc_x10[ch] += s_step_us10[ch];
    while (s_step_acc_x10[ch] >= 10u)
    {
        s_step_acc_x10[ch] -= 10u;
        step++;
    }
    return step;
}

/* ===== 一键序列：每个序列自己的"舵机动作顺序" =====
 * 前三个元素依次是第 1/2/3 步要动的舵机通道，第 4 步统一是延时、第 5 步是机械爪张开。
 * 有两套顺序，因为不同的目标姿态下"从下往上"会互相卡住：
 *   A 标准顺序（一键放下 L1 / 自定义2 L2）：MG995 -> SG90(下) -> SG90(上)
 *   B 先上后下（一键摆臂 R1 / 自定义1 R2）：SG90(上) -> MG995 -> SG90(下)   <- 不会卡
 */
static const uint8_t s_seq_order_down_first[3] = {
    SERVO_CH_JOINT1, SERVO_CH_JOINT2, SERVO_CH_ARM_UP
};
static const uint8_t s_seq_order_up_first[3] = {
    SERVO_CH_ARM_UP, SERVO_CH_JOINT1, SERVO_CH_JOINT2
};

static const uint8_t *seq_order(uint8_t seq)
{
    return (seq == SERVO_SEQ_POSE || seq == SERVO_SEQ_R2) ? s_seq_order_up_first
                                                          : s_seq_order_down_first;
}

/* 一键序列：第 step 步对应哪个舵机通道（1..3 按顺序表，其余是机械爪） */
static uint8_t seq_step_channel(uint8_t seq, uint8_t step)
{
    if (step >= 1u && step <= 3u) return seq_order(seq)[step - 1u];
    return SERVO_CH_GRIPPER;
}

/* 一键序列：取某个舵机的目标角度（机械角）。四个序列各自有一套角度宏（servo.h）：
 *   SERVO_SEQ_DROP (L1 一键放下) -> SERVO_DROP_*
 *   SERVO_SEQ_POSE (R1 一键摆臂) -> SERVO_POSE_*
 *   SERVO_SEQ_R2   (R2 自定义1)  -> SERVO_R2_*
 *   SERVO_SEQ_L2   (L2 自定义2)  -> SERVO_L2_*   <- 改这几个宏即可改姿态 */
static int16_t seq_angle_of(uint8_t seq, uint8_t ch)
{
    if (seq == SERVO_SEQ_DROP)
    {
        if (ch == SERVO_CH_JOINT1) return (int16_t)SERVO_DROP_MG995_DEG;
        if (ch == SERVO_CH_JOINT2) return (int16_t)SERVO_DROP_SG90_LOW_DEG;
        return (int16_t)SERVO_DROP_SG90_UP_DEG;
    }
    if (seq == SERVO_SEQ_POSE)
    {
        if (ch == SERVO_CH_JOINT1) return (int16_t)SERVO_POSE_MG995_DEG;
        if (ch == SERVO_CH_JOINT2) return (int16_t)SERVO_POSE_SG90_LOW_DEG;
        return (int16_t)SERVO_POSE_SG90_UP_DEG;
    }
    if (seq == SERVO_SEQ_R2)
    {
        if (ch == SERVO_CH_JOINT1) return (int16_t)SERVO_R2_MG995_DEG;
        if (ch == SERVO_CH_JOINT2) return (int16_t)SERVO_R2_SG90_LOW_DEG;
        return (int16_t)SERVO_R2_SG90_UP_DEG;
    }
    if (ch == SERVO_CH_JOINT1) return (int16_t)SERVO_L2_MG995_DEG;
    if (ch == SERVO_CH_JOINT2) return (int16_t)SERVO_L2_SG90_LOW_DEG;
    return (int16_t)SERVO_L2_SG90_UP_DEG;
}

/* 一键序列：机械爪张开前的延时（四个序列可分别配置） */
static uint16_t seq_grip_delay_ms(uint8_t seq)
{
    if (seq == SERVO_SEQ_DROP) return (uint16_t)SERVO_DROP_GRIP_DELAY_MS;
    if (seq == SERVO_SEQ_POSE) return (uint16_t)SERVO_POSE_GRIP_DELAY_MS;
    if (seq == SERVO_SEQ_R2)   return (uint16_t)SERVO_R2_GRIP_DELAY_MS;
    return (uint16_t)SERVO_L2_GRIP_DELAY_MS;
}

/* 一键序列：设置第 step 步的目标位置（角度是"偏移后的机械角"，与 servo.h 里的宏同一坐标系）
 * 步骤：1..3 = 按 seq_order() 的顺序依次动三个臂舵机；4 = 延时；5 = 机械爪张开 */
static void seq_set_target(uint8_t seq, uint8_t step)
{
    if (step >= 1u && step <= 3u)
    {
        const uint8_t ch  = seq_step_channel(seq, step);
        const int16_t deg = seq_angle_of(seq, ch);

        if (ch == SERVO_CH_JOINT1)
            set_abs_target(ch, SERVO_ANGLE_TO_US(deg, SERVO0_OFFSET_DEG));
        else if (ch == SERVO_CH_JOINT2)
            set_abs_target(ch, SERVO_ANGLE_TO_US(deg, SERVO1_OFFSET_DEG));
        else
            set_abs_target(ch, SERVO_ANGLE_TO_US(deg, SERVO3_OFFSET_DEG));
    }
    else if (step == 4u)
    {
        s_seq_ms = HAL_GetTick();          /* 开始计时：延时到第 5 步才张开爪子 */
    }
    else if (step == 5u)
    {
        set_abs_target(SERVO_CH_GRIPPER, SERVO_GRIP_OPEN_US);   /* 机械爪一键张开 */
    }
}

void Servo_Task(const PS2_Data_t *ps2)
{
    if (ps2 == NULL) return;

#if SERVO_CALIBRATE_CH > 0
    /* ===== 行程校准模式：每 SERVO_CALIBRATE_STEP_MS 毫秒跳一步、往返扫，忽略按键 =====
     * 用来实测机械极限：盯住舵机从哪个脉宽开始卡住/发出堵转声，
     * 再把安全值填回 SERVOx_MIN/MAX_US，并把 SERVO_CALIBRATE_CH 改回 0 */
    {
        const uint8_t ch = (uint8_t)(SERVO_CALIBRATE_CH - 1);

        if (ch < SERVO_CH_NUM)
        {
            static uint32_t next_ms = 0;
            static int16_t  offset  = 0;      /* 相对 MIN 的偏移（us） */
            static int8_t   dir     = 1;
            uint32_t now  = HAL_GetTick();
            int16_t  span = (int16_t)(s_max_us[ch] - s_min_us[ch]);

            if ((int32_t)now >= (int32_t)next_ms)
            {
                next_ms = now + SERVO_CALIBRATE_STEP_MS;

                offset = (int16_t)(offset + dir * SERVO_CALIBRATE_STEP_US);
                if (offset >= span) { offset = span; dir = -1; }
                if (offset <= 0)    { offset = 0;    dir = 1;  }

                Servo_SetPulse(ch, (uint16_t)(s_min_us[ch] + offset));
            }
        }
        (void)ps2;
        return;
    }
#else
    {
        const uint16_t buttons = ps2->buttons;

        /* ===== 一键序列（只在模式 1 = 一键张合 下有效）=====
         *   L1 -> 一键放下（角度用 SERVO_DROP_*）
         *   R1 -> 一键摆臂（角度用 SERVO_POSE_*）
         *   R2 -> 自定义序列 1（角度用 SERVO_R2_*）
         *   L2 -> 自定义序列 2（角度用 SERVO_L2_*）  <- 想加/改姿态只改这几个宏
         * 动作顺序都是 **从下往上**：MG995 -> SG90下 -> SG90上 -> 延时 -> 机械爪张开；
         * 前一个舵机到位后下一个才开始动，角度超出行程会自动限幅。 */
        {
            const uint8_t l1 = PS2_BtnDown(buttons, PS2_BTN_L1);
            const uint8_t l2 = PS2_BtnDown(buttons, PS2_BTN_L2);
            const uint8_t r1 = PS2_BtnDown(buttons, PS2_BTN_R1);
            const uint8_t r2 = PS2_BtnDown(buttons, PS2_BTN_R2);

            if (g_servo_grip_mode == SERVO_GRIP_MODE_ONESHOT && g_servo_seq_step == 0)
            {
                if (l1 && !s_l1_prev)
                {
                    g_servo_seq_id   = SERVO_SEQ_DROP;
                    g_servo_seq_step = 1u;
                    seq_set_target(g_servo_seq_id, 1u);
                }
                else if (r1 && !s_r1_prev)
                {
                    g_servo_seq_id   = SERVO_SEQ_POSE;
                    g_servo_seq_step = 1u;
                    seq_set_target(g_servo_seq_id, 1u);
                }
                else if (r2 && !s_r2_prev)
                {
                    g_servo_seq_id   = SERVO_SEQ_R2;
                    g_servo_seq_step = 1u;
                    seq_set_target(g_servo_seq_id, 1u);
                }
                else if (l2 && !s_l2_prev)
                {
                    g_servo_seq_id   = SERVO_SEQ_L2;
                    g_servo_seq_step = 1u;
                    seq_set_target(g_servo_seq_id, 1u);
                }
            }
            s_l1_prev = l1;
            s_l2_prev = l2;
            s_r1_prev = r1;
            s_r2_prev = r2;
        }

        if (g_servo_seq_step != 0u)
        {
            /* 安全保护：序列进行中按任意手动输入 -> 立即中止序列、交还控制权。
             * （万一序列异常/卡住，不用断电也能救回来，避免"整车失控"）
             * 注意：L1/L2/R1/R2 是触发键，不列入中止键。 */
            {
                const int16_t st = (int16_t)(127 - (int16_t)ps2->right_y);

                if ((st > SERVO_STICK_DEADZONE) || (st < -SERVO_STICK_DEADZONE) ||
                    PS2_BtnDown(buttons, PS2_BTN_SQUARE) || PS2_BtnDown(buttons, PS2_BTN_CIRCLE))
                {
                    g_servo_seq_step = 0u;
                    g_servo_seq_id   = SERVO_SEQ_NONE;
                }
            }

            /* 序列进行中：等当前步骤完成，忽略手动控制。
             * 第 4 步是"延时"，其余步骤等舵机实际脉宽追上目标脉宽。 */
            uint8_t done = 0;

            if (g_servo_seq_step == 4u)
            {
                done = ((uint32_t)(HAL_GetTick() - s_seq_ms) >= (uint32_t)seq_grip_delay_ms(g_servo_seq_id)) ? 1u : 0u;
            }
            else if (g_servo_seq_step != 0u)
            {
                const uint8_t ch = seq_step_channel(g_servo_seq_id, g_servo_seq_step);
                done = (s_pulse_us[ch] == s_target_us[ch]) ? 1u : 0u;
            }

            if (done)
            {
                if (g_servo_seq_step < 5u)
                {
                    g_servo_seq_step++;
                    seq_set_target(g_servo_seq_id, g_servo_seq_step);
                }
                else
                {
                    g_servo_seq_step = 0u;       /* 全部完成，恢复正常手动控制 */
                    g_servo_seq_id   = SERVO_SEQ_NONE;
                }
            }
        }
        else
        {
        /* ===== 手动控制（错峰：同一 tick 只允许一个机构动作）=====
         * 优先级：MG995(右摇杆) > SG90(下)(L1/L2) > SG90(上)(R1/R2) > 机械爪(□/○)
         * —— 多键同时按时只执行优先级最高的那一路，避免几个舵机抢电流/瞬间大负载。
         * SELECT 只切模式、不动作舵机，所以单独处理，不受错峰影响。 */
        {
            static uint8_t  s_prev_select = 0;
            static uint32_t s_sel_lock_ms = 0;
            const uint32_t  now   = HAL_GetTick();
            const uint8_t   sel   = PS2_BtnDown(buttons, PS2_BTN_SELECT);
            const int16_t   stick = (int16_t)(127 - (int16_t)ps2->right_y);   /* 推前 = 正 */
            const int16_t   mag   = (stick < 0) ? (int16_t)(-stick) : stick;
            uint8_t         need_mg995 = (mag > SERVO_STICK_DEADZONE) ? 1u : 0u;
            uint8_t         need_sg_low = 0u;
            uint8_t         need_sg_up  = 0u;
            uint8_t         need_grip   = 0u;
            uint8_t         pressed     = 0u;

            /* SELECT：边沿触发 + 250ms 去抖，在两种控制模式间切换 */
            if (sel && !s_prev_select && (uint32_t)(now - s_sel_lock_ms) >= SERVO_MODE_TOGGLE_MS)
            {
                s_sel_lock_ms = now;
                Servo_SetGripMode(g_servo_grip_mode == SERVO_GRIP_MODE_ONESHOT
                                      ? SERVO_GRIP_MODE_JOG
                                      : SERVO_GRIP_MODE_ONESHOT);
            }
            s_prev_select = sel;

            if (g_servo_grip_mode == SERVO_GRIP_MODE_JOG &&
                (PS2_BtnDown(buttons, PS2_BTN_L1) || PS2_BtnDown(buttons, PS2_BTN_L2)))
                need_sg_low = 1u;
            if (g_servo_grip_mode == SERVO_GRIP_MODE_JOG &&
                (PS2_BtnDown(buttons, PS2_BTN_R1) || PS2_BtnDown(buttons, PS2_BTN_R2)))
                need_sg_up = 1u;
            if (PS2_BtnDown(buttons, PS2_BTN_SQUARE) || PS2_BtnDown(buttons, PS2_BTN_CIRCLE))
                need_grip = 1u;

            /* ---- 按优先级只执行一路 ---- */
            if (need_mg995)
            {
                const uint16_t step = ch_step_us(SERVO_CH_JOINT1);

                if (step > 0u)
                {
                    const int16_t dir = (stick > 0) ? +1 : -1;

                    set_rel_target(SERVO_CH_JOINT1,
                                   (int16_t)(dir * SERVO_MG995_STICK_SIGN * (int16_t)step));
                }
            }
            else if (need_sg_low)
            {
                const uint16_t step = ch_step_us(SERVO_CH_JOINT2);

                if (step > 0u)
                {
                    if (PS2_BtnDown(buttons, PS2_BTN_L1))
                        set_rel_target(SERVO_CH_JOINT2, +(int16_t)step);
                    else
                        set_rel_target(SERVO_CH_JOINT2, -(int16_t)step);
                }
            }
            else if (need_sg_up)
            {
                const uint16_t step = ch_step_us(SERVO_CH_ARM_UP);

                if (step > 0u)
                {
                    if (PS2_BtnDown(buttons, PS2_BTN_R1))
                        set_rel_target(SERVO_CH_ARM_UP, +(int16_t)step);
                    else
                        set_rel_target(SERVO_CH_ARM_UP, -(int16_t)step);
                }
            }
            else if (need_grip)
            {
                if (g_servo_grip_mode == SERVO_GRIP_MODE_ONESHOT)
                {
                    /* 模式 1：一键张合——张开 = 行程上限；合上 = 从上限往回 SERVO_GRIP_ONESHOT_DEG 度。
                     * 直接使用 servo.h 换算好的绝对脉宽（已含零点偏移与两端收进量）。 */
                    if (PS2_BtnDown(buttons, PS2_BTN_SQUARE))
                        set_abs_target(SERVO_CH_GRIPPER, SERVO_GRIP_OPEN_US);
                    else
                        set_abs_target(SERVO_CH_GRIPPER, SERVO_GRIP_CLOSE_US);
                }
                else
                {
                    const uint16_t step = ch_step_us(SERVO_CH_GRIPPER);

                    if (step > 0u)
                    {
                        if (PS2_BtnDown(buttons, PS2_BTN_SQUARE))
                            set_rel_target(SERVO_CH_GRIPPER, +(int16_t)step);   /* 向张开方向 */
                        else
                            set_rel_target(SERVO_CH_GRIPPER, -(int16_t)step);   /* 向闭合方向 */
                    }
                }
                pressed = 1u;
            }

#if SERVO_GRIP_RELEASE_MS > 0
            /* 到位后仍一直按住 -> 断开该通道 PWM（舵机失力），避免顶着机械限位持续堵转发烫 */
            if (pressed && s_pulse_us[SERVO_CH_GRIPPER] == s_target_us[SERVO_CH_GRIPPER])
            {
                s_grip_hold_ms += (uint32_t)SERVO_TICK_MS;   /* 按真实 tick 累计 */
                if (s_grip_hold_ms >= SERVO_GRIP_RELEASE_MS) gripper_attach(0);
            }
            else
            {
                s_grip_hold_ms = 0;
                if (!s_grip_attached) gripper_attach(1);
            }
#else
            (void)pressed;
#endif
        }
        }
    }   /* end: 一键放下序列空闲时才执行手动控制 */

    for (uint8_t ch = 0; ch < SERVO_CH_NUM; ch++)
        slew_toward_target(ch);
#endif
}
