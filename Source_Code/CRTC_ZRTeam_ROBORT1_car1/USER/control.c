//
// Created by SukuraFalling on 2026/10/14.
// 遥控操作映射 v3：档位 + 方向键平移 + 自转/绕后轮 + 舵机（详见 control.h）
//
#include "control.h"

/* ---------------- 档位状态 ---------------- */
static uint8_t  s_gear           = CTRL_GEAR_DEFAULT;
static uint32_t s_gear_lock_ms   = 0;
static uint8_t  s_prev_gear_up   = 0;      /* 上一帧 △ 是否按下（用于边沿检测） */
static uint8_t  s_prev_gear_down = 0;      /* 上一帧 ✕ 是否按下 */

static const int16_t s_gear_lo[CTRL_GEAR_COUNT] = { CTRL_GEAR0_LO, CTRL_GEAR1_LO, CTRL_GEAR2_LO };
static const int16_t s_gear_hi[CTRL_GEAR_COUNT] = { CTRL_GEAR0_HI, CTRL_GEAR1_HI, CTRL_GEAR2_HI };

/* ---------------- 摇杆原始值 -> 有符号偏移 ---------------- */
/* PS2 惯例：推到上/左 = 小值(≈0)，推到下/右 = 大值(≈255) */
static int16_t axis_up_pos(uint8_t raw)    { return (int16_t)(127 - (int16_t)raw); }   /* 推上 = 正 */
static int16_t axis_right_pos(uint8_t raw) { return (int16_t)((int16_t)raw - 127); }   /* 推右 = 正 */

/* 死区 + 线性映射到 ±CTRL_SCALE（死区外从 0 开始，推杆距离线性对应速度） */
static int16_t shape(int16_t v)
{
    int16_t a = (v < 0) ? (int16_t)(-v) : v;
    int32_t out;

    if (a <= CTRL_DEADZONE) return 0;

    out = (int32_t)(a - CTRL_DEADZONE) * CTRL_SCALE / (127 - CTRL_DEADZONE);
    if (out > CTRL_SCALE) out = CTRL_SCALE;

    return (v < 0) ? (int16_t)(-out) : (int16_t)out;
}

static int16_t iabs16(int16_t v) { return (v < 0) ? (int16_t)(-v) : v; }

void Ctrl_Init(void)
{
    s_gear           = CTRL_GEAR_DEFAULT;    /* 上电默认中速档 */
    s_gear_lock_ms   = 0;
    s_prev_gear_up   = 0;
    s_prev_gear_down = 0;
}

uint8_t Ctrl_GetGear(void)      { return s_gear; }
int16_t Ctrl_GetGearLimit(void) { return s_gear_hi[s_gear]; }

const char *Ctrl_ModeName(Ctrl_Mode_t mode)
{
    switch (mode)
    {
    case CTRL_MODE_STICK: return "STICK";
    case CTRL_MODE_DPAD:  return "DPAD";
    case CTRL_MODE_BRAKE: return "BRAKE";
    default:              return "IDLE";
    }
}

/* 绕后轮的输出上限：把档位映射到"物理可用区间"[CTRL_PIVOT_LIMIT_LO, CTRL_PIVOT_LIMIT_HI]
 * 慢速档 -> LO，快速档 -> HI，中间档线性插值。见 control.h 里的说明。 */
static int16_t pivot_limit_for(int16_t gear_hi)
{
    const int32_t span_gear = (int32_t)CTRL_GEAR2_HI - CTRL_GEAR0_HI;
    const int32_t span_lim  = (int32_t)CTRL_PIVOT_LIMIT_HI - CTRL_PIVOT_LIMIT_LO;
    int32_t t;

    if (span_gear <= 0) return CTRL_PIVOT_LIMIT_HI;

    t = ((int32_t)gear_hi - CTRL_GEAR0_HI) * 1000 / span_gear;   /* 0..1000 */
    if (t < 0)    t = 0;
    if (t > 1000) t = 1000;

    return (int16_t)(CTRL_PIVOT_LIMIT_LO + span_lim * t / 1000);
}

/* △ 加档 / ✕ 减档：边沿触发 + 去抖，按住不会连续跳档 */
static void gear_task(uint16_t buttons, uint32_t now)
{
    uint8_t up   = PS2_BtnDown(buttons, PS2_BTN_TRIANGLE);   /* △ */
    uint8_t down = PS2_BtnDown(buttons, PS2_BTN_CROSS);      /* ✕ */

    if ((uint32_t)(now - s_gear_lock_ms) >= CTRL_GEAR_LOCKOUT_MS)
    {
        if (up && !s_prev_gear_up)
        {
            if (s_gear + 1u < CTRL_GEAR_COUNT) s_gear++;
            s_gear_lock_ms = now;
        }
        else if (down && !s_prev_gear_down)
        {
            if (s_gear > 0u) s_gear--;
            s_gear_lock_ms = now;
        }
    }

    s_prev_gear_up   = up;
    s_prev_gear_down = down;
}

uint8_t Ctrl_Update(const PS2_Data_t *ps2, uint8_t enabled, Ctrl_Output_t *out)
{
    int16_t ly, lx, rx;
    int16_t lo, hi;

    if (out == NULL) return 0;

    out->vx    = 0;
    out->vy    = 0;
    out->vw    = 0;
    out->brake = 0;
    out->mode  = CTRL_MODE_IDLE;
    out->gear  = s_gear;
    out->limit = s_gear_hi[s_gear];

    if (ps2 == NULL || ps2->mode != 1) return 0;   /* 绿灯(数字)模式等：不驱动 */

    /* ---- 档位：只要手柄在模拟模式就允许换档（即使还没 armed 也生效） ---- */
    gear_task(ps2->buttons, HAL_GetTick());
    out->gear  = s_gear;
    out->limit = hi = s_gear_hi[s_gear];
    lo = s_gear_lo[s_gear];

    if (!enabled) return 0;

    /* ---- 优先级 1：START 制动 ---- */
    if (PS2_BtnDown(ps2->buttons, PS2_BTN_START))
    {
        out->brake = 1;
        out->mode  = CTRL_MODE_BRAKE;
        return 1;
    }

    /* ---- 优先级 2：方向键平移（可组合成斜向；仍受档位上限约束） ---- */
    {
        int16_t dx = 0, dy = 0;
        int16_t d = CTRL_DPAD_SPEED;

        if (PS2_BtnDown(ps2->buttons, PS2_BTN_UP))    dx += d;    /* 上 = 前进 */
        if (PS2_BtnDown(ps2->buttons, PS2_BTN_DOWN))  dx -= d;    /* 下 = 后退 */
        if (PS2_BtnDown(ps2->buttons, PS2_BTN_LEFT))  dy -= d;    /* 左 = 左移 */
        if (PS2_BtnDown(ps2->buttons, PS2_BTN_RIGHT)) dy += d;    /* 右 = 右移 */

        if (dx != 0 || dy != 0)
        {
            out->vx   = dx;
            out->vy   = dy;
            out->mode = CTRL_MODE_DPAD;
            return 1;
        }
    }

    /* ---- 优先级 3：摇杆 ---- */
    ly = (int16_t)(shape(axis_up_pos(ps2->left_y))    * CTRL_LY_SIGN);
    lx = (int16_t)(shape(axis_right_pos(ps2->left_x)) * CTRL_LX_SIGN);
    rx = (int16_t)(shape(axis_right_pos(ps2->right_x))* CTRL_RX_SIGN);

    /* 左摇杆 Y：前后。推杆幅度在当前档位区间 [lo, hi] 内线性映射 */
    if (ly != 0)
    {
        int16_t mag = iabs16(ly);                              /* 0..1000 */
        int16_t spd = (int16_t)(lo + (int32_t)(hi - lo) * mag / CTRL_SCALE);
        out->vx = (ly < 0) ? (int16_t)(-spd) : spd;
    }

    /* 左摇杆 X：原地自转（绕底盘中心，纯旋转；按档位限制，与平移一致） */
    if (lx != 0)
    {
        out->vw += (int16_t)((int32_t)lx * hi / CTRL_SCALE);
    }

    /* 右摇杆 X：绕（对侧）后轮水平旋转
     * 推导：绕点 p 转时车体中心速度 v = ω × (C - P)；本模块的麦轮公式
     *       LF=vx+vy+vw, RF=vx-vy-vw, LR=vx-vy+vw, RR=vx+vy-vw
     *       对应标准式需做 vy=-vy_std、vw=-ω_std，于是：
     *         vx = ω·p_y/arm, vy = ω·p_x/arm, vw = -ω,  arm = 半轴距+半轮距
     *       选 p = (-半轴距, ±半轮距)（后轴），推左取对侧(右)、推右取对侧(左)。
     *       此时被绕的那个轮子的指令恰好为 0（见主机端测试）。 */
    if (rx != 0)
    {
        int16_t dir   = (rx > 0) ? +1 : -1;                    /* +1 = 推右 */
        int16_t mag   = iabs16(rx);
        int16_t side  = (int16_t)(-dir * CTRL_HALF_WID_MM);    /* 同侧后轮 */
        int16_t plim  = pivot_limit_for(hi);                   /* 绕转的档位上限（物理可用区间） */
        int32_t omega;
        const int32_t arm = (int32_t)CTRL_HALF_LEN_MM + CTRL_HALF_WID_MM;

#if CTRL_PIVOT_OPPOSITE_SIDE
        side = (int16_t)(-side);                               /* 取对侧后轮 */
#endif

        omega = (int32_t)CTRL_PIVOT_SIGN * (-dir) * mag * plim / CTRL_SCALE;

        out->vx += (int16_t)(omega * side / arm);
        out->vy += (int16_t)(omega * (-(int32_t)CTRL_HALF_LEN_MM) / arm);
        out->vw += (int16_t)(-omega);

        /* 绕转的输出上限 = 该档位对应的绕转上限（见 pivot_limit_for 说明） */
        out->limit = plim;
    }

    out->mode = CTRL_MODE_STICK;
    return 1;
}
