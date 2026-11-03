//
// Created by SukuraFalling on 2026/10/14.
// 修订：整包 CS 时序 + 正确的模式/ID 判定 + 掉线自恢复
// 依据 Arduino PS2X 参考实现（read_gamepad / sendCommandString / _gamepad_shiftinout）
//
#include "ps2.h"

// ===== 调试可观测变量 =====
volatile PS2_Debug_t g_ps2_dbg = {0};

// 连续失败计数 / 重配置重入保护 / 上次重配置时刻 / 连续掉模拟模式计数 / 重新初始化防重入
static uint8_t  s_fail_streak   = 0;
static uint8_t  s_in_config     = 0;
static uint8_t  s_drop_streak   = 0;
static uint8_t  s_reinit_guard  = 0;
static uint32_t s_last_cfg_ms   = 0;

// 振动状态：必须在每次轮询包里持续发送，否则手柄会在一个周期内把它清掉
static uint8_t s_rumble_small = 0;
static uint8_t s_rumble_large = 0;

// 配置命令
#if PS2_ENABLE_RUMBLE
static const uint8_t enable_rumble_cmd[5] = {0x01, 0x4D, 0x00, 0x00, 0x01};
#endif

// ===== 微秒级延时（基于 DWT，不依赖 HAL_Delay）=====
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { }
}

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// ===== 底层GPIO操作 =====
static void PS2_CS_Low(void)   { HAL_GPIO_WritePin(PS2_CS_PORT,  PS2_CS_PIN,  GPIO_PIN_RESET); }
static void PS2_CS_High(void)  { HAL_GPIO_WritePin(PS2_CS_PORT,  PS2_CS_PIN,  GPIO_PIN_SET);   }
static void PS2_CLK_Low(void)  { HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_RESET); }
static void PS2_CLK_High(void) { HAL_GPIO_WritePin(PS2_CLK_PORT, PS2_CLK_PIN, GPIO_PIN_SET);   }
static void PS2_DO_Low(void)   { HAL_GPIO_WritePin(PS2_DO_PORT,  PS2_DO_PIN,  GPIO_PIN_RESET); }
static void PS2_DO_High(void)  { HAL_GPIO_WritePin(PS2_DO_PORT,  PS2_DO_PIN,  GPIO_PIN_SET);   }
static uint8_t PS2_DI_Read(void){ return HAL_GPIO_ReadPin(PS2_DI_PORT, PS2_DI_PIN); }

// ===== 收发一个字节：置DO -> CLK拉低 -> 采样DI -> CLK拉高（LSB first）=====
static uint8_t PS2_Transfer(uint8_t txData)
{
    uint8_t rxData = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        // 先准备发送位
        if (txData & 0x01) PS2_DO_High();
        else               PS2_DO_Low();
        txData >>= 1;

        // 时钟下降沿，手柄在此采样DO、并送出DI
        PS2_CLK_Low();
        delay_us(PS2_CLK_US);

        // 时钟低电平期间采样DI（LSB优先）
        if (PS2_DI_Read()) rxData |= (uint8_t)(1u << i);

        // 时钟上升沿
        PS2_CLK_High();
        delay_us(PS2_CLK_US);
    }

    PS2_DO_High();                  // 空闲时DO保持高
    delay_us(PS2_BYTE_DELAY_US);
    return rxData;
}

// ===== 发送一整包：必须在同一次 CS 低电平窗口内完成 =====
// 这是原实现最大的问题：每个字节都拉低/拉高一次 CS，手柄会把包丢弃
static void PS2_SendPkt(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    PS2_CS_Low();
    delay_us(PS2_BYTE_DELAY_US);

    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t v = PS2_Transfer(tx[i]);
        if (rx) rx[i] = v;
    }

    PS2_CS_High();
    delay_us(PS2_PKT_DELAY_US);     // 包间隔，参考实现用 read_delay(>=1ms)
}

// ===== 进入模拟（红灯）模式的标准配置序列 =====
static void PS2_EnterAnalogMode(void)
{
    static const uint8_t enter_cfg[5] = {0x01, 0x43, 0x00, 0x01, 0x00};
    static const uint8_t type_read[9] = {0x01, 0x45, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
    static const uint8_t set_mode[9]  = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
#if PS2_ENABLE_RUMBLE
    static const uint8_t set_bytes_large[9] = {0x01, 0x4F, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00};
#endif
    static const uint8_t exit_cfg[9]  = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
    uint8_t rx[9];

    if (s_in_config) return;        // 防止读取失败时递归重配置
    s_in_config = 1;

    PS2_SendPkt(enter_cfg, rx, sizeof(enter_cfg));
    PS2_SendPkt(type_read, rx, sizeof(type_read));
    g_ps2_dbg.ctrl_type = rx[3];    // 0x03=有线DualShock，0x0C=2.4G无线
    PS2_SendPkt(set_mode,  rx, sizeof(set_mode));
#if PS2_ENABLE_RUMBLE
    PS2_SendPkt(enable_rumble_cmd, rx, sizeof(enable_rumble_cmd));
    PS2_SendPkt(set_bytes_large, rx, sizeof(set_bytes_large));   // 大马达需要 0xFF/0xFF/0x03
#endif
    PS2_SendPkt(exit_cfg,  rx, sizeof(exit_cfg));

    s_in_config = 0;
}

// ===== 限速地重新进入模拟模式 =====
// 用途：手柄自己掉出模拟模式（id=0x41 绿灯，多键连按时常见）、或读取连续失败时，
//       自动把它拉回红灯模式，操作手不用去按手柄上的 ANALOG 键。
static void ps2_restore_analog(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_last_cfg_ms) < PS2_RECONFIG_INTERVAL_MS)
        return;                                  // 限速，避免每 10ms 都发配置包

    s_last_cfg_ms = now;
    g_ps2_dbg.mode_recover_count++;
    PS2_EnterAnalogMode();
}

// 无视限速，立刻拉回一次（掉模式的第一帧就用它，尽量缩短"失控"时间）
static void ps2_restore_analog_now(void)
{
    s_last_cfg_ms = HAL_GetTick() - (uint32_t)PS2_RECONFIG_INTERVAL_MS;   /* 让限速立即放行 */
    ps2_restore_analog();
}

// ===== 初始化：进入并确认模拟（红灯）模式 =====
void PS2_Init(void)
{
    DWT_Init();

    // 清除上一次通信的残留：失败计数、调试缓存里的旧包
    s_fail_streak = 0;
    s_in_config   = 0;
    s_last_cfg_ms = HAL_GetTick();
    for (uint8_t i = 0; i < 9; i++)
        g_ps2_dbg.raw[i] = 0x00;
    g_ps2_dbg.id    = 0x00;
    g_ps2_dbg.valid = 0;

    PS2_CS_High();
    PS2_CLK_High();
    PS2_DO_High();

    HAL_Delay(PS2_BOOT_DELAY_MS);   // 等手柄/接收器上电稳定

    // 先空读两包把接收器可能残留的旧帧读掉（参考实现先 read 两次再进配置模式）
    for (uint8_t i = 0; i < 2; i++)
    {
        PS2_Data_t dummy;
        (void)PS2_ReadData(&dummy);
    }

    // 最多尝试 3 轮：配置 -> 读一包 -> 确认 mode==1
    g_ps2_dbg.init_ok = 0;
    for (uint8_t retry = 0; retry < 3; retry++)
    {
        PS2_Data_t d;
        PS2_EnterAnalogMode();
        if (PS2_ReadData(&d) && d.mode == 1)
        {
            g_ps2_dbg.init_ok = 1;
            break;
        }
    }
}

// ===== 读取一包手柄数据 =====
uint8_t PS2_ReadData(PS2_Data_t *data)
{
    // 轮询包：同时把振动值带下去（大/小马达必须每个周期刷新，否则会被手柄清掉）
    uint8_t poll[9] = {0x01, 0x42, 0x00,
                       (uint8_t)(s_rumble_small ? 0x01 : 0x00),
                       s_rumble_large,
                       0x00, 0x00, 0x00, 0x00};
    uint8_t buf[9];
    uint8_t id;

    if (data == NULL) return 0;

    PS2_SendPkt(poll, buf, sizeof(poll));

    for (uint8_t i = 0; i < 9; i++)
        g_ps2_dbg.raw[i] = buf[i];

    // 读取模式下各字节含义：
    //   buf[0]=0xFF(头)  buf[1]=模式/型号ID  buf[2]=0x5A("数据就绪"标记)
    //   buf[3]=按键低字节  buf[4]=按键高字节  buf[5..8]=右X/右Y/左X/左Y
    // 注意：0x5A 是 0x42 轮询响应的第 3 字节(buf[2])，也是退出配置命令的发送填充值；
    //      它不是 buf[1]，不能拿 buf[1]!=0x5A 当连接判据（原实现的 bug 就在这里）
    id = buf[1];
    g_ps2_dbg.id = id;

    if (id != 0x41 && id != 0x42 && id != 0x73 && id != 0x79)
    {
        // 通信异常或手柄掉线：回中并累计失败，必要时自动重配置
        g_ps2_dbg.valid = 0;
        g_ps2_dbg.fail_count++;

        if (++s_fail_streak >= PS2_RECONFIG_FAILS)
        {
            s_fail_streak = 0;
            g_ps2_dbg.reconfig_count++;
            ps2_restore_analog();
        }

        data->left_x  = 0x80;
        data->left_y  = 0x80;
        data->right_x = 0x80;
        data->right_y = 0x80;
        data->buttons = 0;
        data->mode    = 0;
        return 0;
    }

    s_fail_streak   = 0;
    g_ps2_dbg.valid = 1;
    g_ps2_dbg.ok_count++;

    // 0x7x = 模拟模式（红灯）；0x41/0x42 = 数字模式（绿灯）
    // 注：0x79 为"全量模式"（含按键压力值），参考实现会在同一 CS 窗口内再续读 12 字节；
    //     本驱动不启用压力模式，故按 9 字节处理即可，若实测遇到错位再补续读
    data->mode    = ((id & 0xF0) == 0x70) ? 1 : 0;
    // 按键字节序：buf[3] 是低字节、buf[4] 是高字节（PS2X: (PS2data[4]<<8)+PS2data[3]）
    data->buttons = (uint16_t)(((uint16_t)buf[4] << 8) | buf[3]);

    if (!data->mode)
    {
        // 掉出模拟模式（多键连按/接收器干扰/轮询太快时常见）：自动重发配置序列把它拉回红灯模式，
        // 操作手不用去按手柄上的 ANALOG 键。
        // 策略：① 掉模式的**第一帧**立刻拉回（无视限速，尽量缩短失控时间）；
        //       ② 连续多次都拉不回来（>= PS2_MODE_ESCALATE_AFTER）-> 升级为整包 PS2_Init() 重初始化；
        //       ③ 其它情况按 PS2_RECONFIG_INTERVAL_MS 限速慢慢试。
        s_drop_streak++;
        if (s_drop_streak == 1u)
        {
            ps2_restore_analog_now();
        }
        else if (s_drop_streak >= (uint8_t)PS2_MODE_ESCALATE_AFTER && !s_reinit_guard)
        {
            s_drop_streak  = 0u;
            s_reinit_guard = 1u;                     /* 防重入：PS2_Init 内部还会读包 */
            PS2_Init();                              /* 整包重新初始化（重新识别型号 + 配置） */
            s_reinit_guard = 0u;
        }
        else
        {
            ps2_restore_analog();
        }
    }
    else
    {
        s_drop_streak = 0u;                          /* 回到模拟模式：掉模式计数清零 */
    }

    if (data->mode)
    {
        data->right_x = buf[5];
        data->right_y = buf[6];
        data->left_x  = buf[7];
        data->left_y  = buf[8];
    }
    else
    {
        // 数字模式下摇杆字节无效（常为0xFF），强制回中，配合 main 的 mode 判断双保险
        data->left_x  = 0x80;
        data->left_y  = 0x80;
        data->right_x = 0x80;
        data->right_y = 0x80;
    }

    return 1;
}

// ===== 振动 =====
// 只记录状态，真正的马达字节由之后每一次 PS2_ReadData 的轮询包持续发出。
// 需要 PS2_ENABLE_RUMBLE=1（配置序列里开启振动）且手柄/接收器支持，第三方手柄常无此功能。
void PS2_Vibrate(uint8_t motor1, uint8_t motor2)
{
#if PS2_ENABLE_RUMBLE
    s_rumble_small = (uint8_t)(motor1 ? 0x01 : 0x00);
    // 大马达 0x00 关闭；0x01~0x3F 转不动，统一抬到 0x40（对应 PS2X 的 map(0,255,0x40,0xFF)）
    if (motor2 == 0)          s_rumble_large = 0x00;
    else if (motor2 < 0x40)   s_rumble_large = 0x40;
    else                      s_rumble_large = motor2;
#else
    (void)motor1;
    (void)motor2;
#endif
}
