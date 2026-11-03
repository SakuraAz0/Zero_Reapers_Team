//
// Created by SukuraFalling on 2026/10/14.
// 遥控失联保护 + 上电安全启动（详见 rc.h）
//
#include "rc.h"

volatile RC_Debug_t g_rc_dbg = {0};

static uint32_t s_last_change_ms = 0;
static uint8_t  s_has_last       = 0;

// 安全启动状态
static uint8_t  s_armed        = 0;
static uint8_t  s_arm_started  = 0;
static uint32_t s_arm_since_ms = 0;

void RC_Init(void)
{
    s_has_last       = 0;
    s_last_change_ms = HAL_GetTick();
    s_armed          = 0;
    s_arm_started    = 0;
    s_arm_since_ms   = 0;

    g_rc_dbg.has_last      = 0;
    g_rc_dbg.stale         = 0;
    g_rc_dbg.armed         = 0;
    g_rc_dbg.unchanged_ms  = 0;
    g_rc_dbg.protect_count = 0;
    g_rc_dbg.reject_count  = 0;

    for (uint8_t i = 0; i < 6; i++)
        g_rc_dbg.last[i] = 0;
}

// 强制退回安全启动状态（保留累计统计计数，不清零 ok/fail/reject）
void RC_ForceReArm(void)
{
    s_armed       = 0;
    s_arm_started = 0;
    s_has_last    = 0;

    g_rc_dbg.armed        = 0;
    g_rc_dbg.stale        = 0;
    g_rc_dbg.unchanged_ms = 0;
}

// 数据是否合理：接收器欠压/失联时常见的"四轴全 0x00 或全 0xFF"要拒绝
static uint8_t rc_payload_valid(const PS2_Data_t *d)
{
    if (d->left_x == 0x00 && d->left_y == 0x00 && d->right_x == 0x00 && d->right_y == 0x00)
        return 0;
    if (d->left_x == 0xFF && d->left_y == 0xFF && d->right_x == 0xFF && d->right_y == 0xFF)
        return 0;
    return 1;
}

// 四个摇杆是否都在中心附近（中心按 127/128 处理）
static uint8_t rc_sticks_centered(const PS2_Data_t *d)
{
    const uint8_t lo = (uint8_t)(128 - RC_ARM_DEADZONE);
    const uint8_t hi = (uint8_t)(128 + RC_ARM_DEADZONE);

    return (d->left_x  >= lo && d->left_x  <= hi) &&
           (d->left_y  >= lo && d->left_y  <= hi) &&
           (d->right_x >= lo && d->right_x <= hi) &&
           (d->right_y >= lo && d->right_y <= hi);
}

uint8_t RC_Feed(const PS2_Data_t *data)
{
    uint8_t  cur[6];
    uint8_t  changed = 0;
    uint32_t now;
    uint32_t dt;

    if (data == NULL) return 0;

    // ---- 1) 垃圾数据：立即解除使能并停车（防止把 0x00/0xFF 当成满速指令）----
    if (!rc_payload_valid(data))
    {
        s_armed       = 0;
        s_arm_started = 0;
        g_rc_dbg.armed = 0;
        g_rc_dbg.reject_count++;
        return 0;
    }

    now = HAL_GetTick();

    // ---- 2) 上电安全启动：连续 RC_STARTUP_ARM_MS 回中才允许驱动 ----
    if (!s_armed)
    {
        if (!rc_sticks_centered(data))
        {
            s_arm_started = 0;
            g_rc_dbg.reject_count++;
            return 0;
        }

        if (!s_arm_started)
        {
            s_arm_started  = 1;
            s_arm_since_ms = now;
        }

        if ((uint32_t)(now - s_arm_since_ms) < RC_STARTUP_ARM_MS)
            return 0;

        s_armed        = 1;
        s_has_last     = 0;            // 重新建立参考帧，避免刚使能就被判超时
        s_last_change_ms = now;
        g_rc_dbg.armed = 1;
    }

    // ---- 3) 数据变化 / 失联超时检测 ----
    cur[0] = data->left_x;
    cur[1] = data->left_y;
    cur[2] = data->right_x;
    cur[3] = data->right_y;
    cur[4] = (uint8_t)(data->buttons >> 8);
    cur[5] = (uint8_t)(data->buttons & 0xFFu);

    if (!s_has_last)
    {
        changed = 1;
    }
    else
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            if (cur[i] != g_rc_dbg.last[i])
            {
                changed = 1;
                break;
            }
        }
    }

    if (changed)
    {
        for (uint8_t i = 0; i < 6; i++)
            g_rc_dbg.last[i] = cur[i];

        s_has_last            = 1;
        s_last_change_ms      = now;
        g_rc_dbg.has_last     = 1;
        g_rc_dbg.unchanged_ms = 0;
        g_rc_dbg.stale        = 0;
        return 1;
    }

    dt = now - s_last_change_ms;                       // 无符号相减，可正确处理 tick 回绕
    g_rc_dbg.unchanged_ms = (dt > 0xFFFFu) ? 0xFFFFu : (uint16_t)dt;

    if (dt >= RC_STALE_TIMEOUT_MS)
    {
        if (!g_rc_dbg.stale)
            g_rc_dbg.protect_count++;
        g_rc_dbg.stale = 1;
        return 0;                                      // 失联保护：停车
    }

    return 1;
}
