//
// servo.h —— 舵机 PWM（TIM3 四通道，50Hz）
//
// 引脚（TIM3 默认映射，全部为本次新增占用）：
//   PA6 = TIM3_CH1 -> 关节舵机1  MG995   （L1 增 / L2 减）
//   PA7 = TIM3_CH2 -> 关节舵机2  SG90    （R1 增 / R2 减）
//   PB0 = TIM3_CH3 -> 机械爪舵机 MG90S   （□ 张开 / ○ 合拢）
//   PB1 = TIM3_CH4 -> 备用
//
// 注意：舵机必须用独立 5V 供电（堵转电流可达 0.5~1A），并与 MCU 共地；
//       绝不能从板载 3.3V/5V 取电，否则会重演"一上负载就掉压"。
//
#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "ps2.h"

#define SERVO_CH_NUM      4
#define SERVO_CH_JOINT1   0        /* CH1 PA6 = MG995（最下面那段臂，右摇杆Y控制） */
#define SERVO_CH_JOINT2   1        /* CH2 PA7 = SG90（下面那个，L1/L2） */
#define SERVO_CH_GRIPPER  2        /* CH3 PB0 = MG90S 机械爪（□/○ + SELECT） */
#define SERVO_CH_ARM_UP   3        /* CH4 PB1 = SG90（上面那个，R1/R2） */

// ===== 三路机械臂舵机：范围 + 初始角（统一按"角度"配置，只改这一块就够）=====
// 舵机是 180° 位置舵机：电气 0° = 500us、180° = 2500us，即 **1° = 11.11us**。
//
// 每路 4 个宏，单位都是"度"，且都用 **机械角坐标系**（你自己拿量角器量的角度）：
//   SERVOx_OFFSET_DEG     : 机械零点偏移 = 机械角 - 电气角。舵盘装正了填 0；
//                           装偏了（例如装好后"机械 0°"对应脉宽 800us）就填 +27。
//   SERVOx_MIN_ANGLE_DEG  : 行程下限（机械角）—— 舵机永远走不到这以下
//   SERVOx_MAX_ANGLE_DEG  : 行程上限（机械角）—— 舵机永远走不到这以上
//   SERVOx_START_ANGLE_DEG: 上电初始角（机械角）—— 会**自动限幅**到 [下限, 上限] 内
//
// 换算关系：电气角 = 机械角 - 偏移；脉宽 = 500 + 电气角 × 11.11（自动兜底限幅 500~2500us）
// 改动后 MIN_US/MAX_US/START_US/SPAN_DEG 全部自动跟着算，不需要动别处。
// ---------------------------------------------------------------------------
// ===== 本车机械臂"装了哪几个舵机"（决定一键序列会动哪些轴 / 哪些点动有效）=====
// 车2 = 3 个舵机：MG995(CH1) + 上面 SG90(CH4) + 机械爪(CH3)，**没有下面那节 SG90(CH2)**
#ifndef SERVO_HAS_SG90_LOW
#define SERVO_HAS_SG90_LOW      0      /* 1=装了 CH2 SG90(下)，0=没装（车2） */
#endif
#ifndef SERVO_HAS_SG90_UP
#define SERVO_HAS_SG90_UP       1      /* 1=装了 CH4 SG90(上) */
#endif
// ---------------------------------------------------------------------------
// CH1 = MG995（最下面那段臂，右摇杆 Y 控制）
#define SERVO0_OFFSET_DEG       0      /* 机械零点偏移（度） */
#define SERVO0_MIN_ANGLE_DEG    0      /* 行程下限 */
#define SERVO0_MAX_ANGLE_DEG    140    /* 行程上限（实测机械极限 160°，取 140° 留 20° 余量） */
#define SERVO0_START_ANGLE_DEG  0      /* 上电初始角 */

// CH2 = SG90（下面那段臂）—— 车2 没有这个舵机（SERVO_HAS_SG90_LOW=0），配置仅保留备用
#define SERVO1_OFFSET_DEG       -10
#define SERVO1_MIN_ANGLE_DEG    5
#define SERVO1_MAX_ANGLE_DEG    170    /* 170-(-10)=180 正好到电气上限 */
#define SERVO1_START_ANGLE_DEG  170

// CH4 = SG90（上面那段臂，R1/R2 控制）
// 现场选定"方案2"：偏移 88° 是真实安装位置 -> 机械角 = 电气角 + 88，
// 所以可达的机械角范围是 **88° ~ 268°**（对应电气 0°~180° = 500~2500us）。
// 原来填 5~170 时，5-88=-83 越界被限幅，导致实际只能转约 82°。
#define SERVO3_OFFSET_DEG       0
#define SERVO3_MIN_ANGLE_DEG    5
#define SERVO3_MAX_ANGLE_DEG    170
#define SERVO3_START_ANGLE_DEG  170
// ---------------------------------------------------------------------------
// ===== 编译期自检：行程是否落在舵机物理范围（电气 0~180°）内 =====
// 若"下限-偏移 < 0"或"上限-偏移 > 180"，说明这一段行程物理上到不了（会被限幅），
// 表现就是"舵机只能转一小段"。看到下面这些警告就去改 OFFSET 或 MIN/MAX。
#if (SERVO0_MIN_ANGLE_DEG - SERVO0_OFFSET_DEG) < 0
#warning "SERVO0(MG995): 下限-偏移 < 0 -> 低于 500us 的部分不可达"
#endif
#if (SERVO0_MAX_ANGLE_DEG - SERVO0_OFFSET_DEG) > 180
#warning "SERVO0(MG995): 上限-偏移 > 180 -> 高于 2500us 的部分不可达"
#endif
#if (SERVO1_MIN_ANGLE_DEG - SERVO1_OFFSET_DEG) < 0
#warning "SERVO1(SG90下): 下限-偏移 < 0 -> 低于 500us 的部分不可达"
#endif
#if (SERVO1_MAX_ANGLE_DEG - SERVO1_OFFSET_DEG) > 180
#warning "SERVO1(SG90下): 上限-偏移 > 180 -> 高于 2500us 的部分不可达"
#endif
#if (SERVO3_MIN_ANGLE_DEG - SERVO3_OFFSET_DEG) < 0
#warning "SERVO3(SG90上): 下限-偏移 < 0 -> 低于 500us 的部分不可达"
#endif
#if (SERVO3_MAX_ANGLE_DEG - SERVO3_OFFSET_DEG) > 180
#warning "SERVO3(SG90上): 上限-偏移 > 180 -> 高于 2500us 的部分不可达"
#endif
// ---------------------------------------------------------------------------
// ===== 新车/重装后的"安全上电姿态"开关 =====
// 换一辆车、或重新装过舵盘后，上面的标定值全部失效（还是上一辆车的），此时**首次上电**很危险：
// 上电瞬间会按旧标定把舵机顶到机械死点（嗡嗡响、发烫、烧舵机）。
// 置 1 后：三路臂舵机上电都停在 **电气 90° = 1500us（中间位）**，不管旧标定是什么。
// 调试步骤：① 置 1 烧录 -> ② 看三路都停在中间、不顶死 -> ③ 逐路重新标定（改 OFFSET/MIN/MAX/START）
//           -> ④ 标定完成后改回 0。
#ifndef SERVO_BRINGUP_SAFE_POSE
#define SERVO_BRINGUP_SAFE_POSE  0
#endif
#if SERVO_BRINGUP_SAFE_POSE
#undef  SERVO0_START_ANGLE_DEG
#define SERVO0_START_ANGLE_DEG  (SERVO0_OFFSET_DEG + 90)     /* 电气 90° -> 1500us */
#undef  SERVO1_START_ANGLE_DEG
#define SERVO1_START_ANGLE_DEG  (SERVO1_OFFSET_DEG + 90)
#undef  SERVO3_START_ANGLE_DEG
#define SERVO3_START_ANGLE_DEG  (SERVO3_OFFSET_DEG + 90)
#endif
// ---------------------------------------------------------------------------

// ===== 一键放下（**模式 0** 下按 L1 触发）=====
// 动作顺序固定为 **从下往上依次动**：MG995 -> SG90(上) -> 等一会儿 -> 机械爪张开；
// （车2 没有 SG90(下)，这一步会自动跳过；前一个舵机到位后下一个才开始动）
// 角度都是 **偏移后的机械角**，与上面 SERVOx_*_ANGLE_DEG 同一个坐标系；超出行程会自动限幅。
#ifndef SERVO_DROP_MG995_DEG
#define SERVO_DROP_MG995_DEG      35      /* CH1 MG995 目标机械角 */
#endif
#ifndef SERVO_DROP_SG90_LOW_DEG
#define SERVO_DROP_SG90_LOW_DEG   5       /* CH2 SG90(下) 目标机械角（本车没有该舵机，忽略） */
#endif
#ifndef SERVO_DROP_SG90_UP_DEG
#define SERVO_DROP_SG90_UP_DEG    10      /* CH4 SG90(上) 目标机械角 = 该路最低点(电气0°/500us) */
#endif
#ifndef SERVO_DROP_GRIP_DELAY_MS
#define SERVO_DROP_GRIP_DELAY_MS  100     /* 上面三个舵机都到位后，再等这么久机械爪才张开 */
#endif

// ===== 一键摆臂（**模式 0** 下按 L2 触发）=====
// 顺序：**SG90(上) -> MG995 -> 等延时** -> 机械爪张开（先上后下，避免卡住；本车没有 SG90(下)）
// 角度都是"偏移后的机械角"，改这里就能调姿态，不用动代码。
#ifndef SERVO_POSE_MG995_DEG
#define SERVO_POSE_MG995_DEG      138      /* CH1 MG995 目标机械角 */
#endif
#ifndef SERVO_POSE_SG90_LOW_DEG
#define SERVO_POSE_SG90_LOW_DEG   110     /* CH2 SG90(下) 目标机械角（本车没有该舵机，忽略） */
#endif
#ifndef SERVO_POSE_SG90_UP_DEG
#define SERVO_POSE_SG90_UP_DEG    80    /* CH4 SG90(上) 目标机械角 */
#endif
#ifndef SERVO_POSE_GRIP_DELAY_MS
#define SERVO_POSE_GRIP_DELAY_MS  60     /* 舵机到位后等 50ms 再张开机械爪 */
#endif

// ===== 联动限位（防机械干涉）：MG995(CH1) 与 上面的 SG90(CH4) 不能同时在高位 =====
// 规则：MG995 机械角 >= 138° 时，SG90(上) 最多到 87°；
//       反过来 SG90(上) > 87° 时，MG995 最多到 137°（保证"MG995 到 138° 且 SG90 超 87°"不会出现）。
// 只作用于"设置目标"（摇杆/按键/一键序列），不影响上电姿态与 Servo_SetPulse 调试直写。
#ifndef SERVO_LINK_ENABLE
#define SERVO_LINK_ENABLE          1
#endif
#ifndef SERVO_LINK_MG995_UP_DEG
#define SERVO_LINK_MG995_UP_DEG    138    /* MG995 的"高位"门限（机械角） */
#endif
#ifndef SERVO_LINK_SG90_UP_MAX_DEG
#define SERVO_LINK_SG90_UP_MAX_DEG 87     /* MG995 高位时 SG90(上) 的最大机械角 */
#endif

// ===== 一键序列编号（调试用 g_servo_seq_id 的取值）=====
#define SERVO_SEQ_NONE   0    /* 空闲 */
#define SERVO_SEQ_DROP   1    /* 模式0 的 L1：一键放下 */
#define SERVO_SEQ_POSE   2    /* 模式0 的 L2：一键摆臂 */

// 角度 -> 脉宽：先按 11.11us/° 换算，再兜底限幅到舵机物理范围 500~2500us
#define SERVO_US_CLAMP(us) \
    ((uint16_t)((int32_t)(us) < 500 ? 500 : ((int32_t)(us) > 2500 ? 2500 : (int32_t)(us))))
#define SERVO_ANGLE_TO_US(deg, offset_deg) \
    SERVO_US_CLAMP(500 + ((int32_t)(deg) - (int32_t)(offset_deg)) * 1000 / 90)

#define SERVO0_MIN_US      SERVO_ANGLE_TO_US(SERVO0_MIN_ANGLE_DEG, SERVO0_OFFSET_DEG)
#define SERVO0_MAX_US      SERVO_ANGLE_TO_US(SERVO0_MAX_ANGLE_DEG, SERVO0_OFFSET_DEG)
#define SERVO0_START_US    SERVO_ANGLE_TO_US(SERVO0_START_ANGLE_DEG, SERVO0_OFFSET_DEG)
#define SERVO1_MIN_US      SERVO_ANGLE_TO_US(SERVO1_MIN_ANGLE_DEG, SERVO1_OFFSET_DEG)
#define SERVO1_MAX_US      SERVO_ANGLE_TO_US(SERVO1_MAX_ANGLE_DEG, SERVO1_OFFSET_DEG)
#define SERVO1_START_US    SERVO_ANGLE_TO_US(SERVO1_START_ANGLE_DEG, SERVO1_OFFSET_DEG)
#define SERVO3_MIN_US      SERVO_ANGLE_TO_US(SERVO3_MIN_ANGLE_DEG, SERVO3_OFFSET_DEG)
#define SERVO3_MAX_US      SERVO_ANGLE_TO_US(SERVO3_MAX_ANGLE_DEG, SERVO3_OFFSET_DEG)
#define SERVO3_START_US    SERVO_ANGLE_TO_US(SERVO3_START_ANGLE_DEG, SERVO3_OFFSET_DEG)

// ===== 机械爪角度零点偏移（重要校准项）=====
// 舵盘装在舵机花键上，一个齿 = 360/25 = **14.4°**，装偏一齿会让整段行程整体偏移。
// 实测现象：张开端顶到/超过机械限位，同时闭合端合不到位
//   -> 说明机构可用窗口整体偏低，需要**整体向下平移**；实测最佳值 **13.5°**。
// 单位是 **0.1°**：-135 = -13.5°。负值 = 向"角度更小"方向平移（脉宽变小）。
// 13.5° = 150us。当前取 -135，得到：全闭 350us、全开 1350us。
// 现场微调：张开端还顶死就减（-140 / -145）；闭合端顶死发烫就加回来（-130 / -120）。
#define SERVO_GRIP_ANGLE_SHIFT_DEG10   (-130)
#define SERVO_GRIP_SHIFT_US            ((int32_t)SERVO_GRIP_ANGLE_SHIFT_DEG10 * 1000 / (90 * 10))  // -130 -> -144us
// 两端各收进这么多度（安全限位，防顶死 / 留余量）：5° = 55us，机械爪实际行程变成 80°（890us）
#ifndef SERVO_GRIP_TRIM_DEG
#define SERVO_GRIP_TRIM_DEG            5
#endif
#define SERVO_GRIP_TRIM_US             ((int32_t)SERVO_GRIP_TRIM_DEG * 1000 / 90)
#define SERVO2_MIN_US                  ((uint16_t)(500  + SERVO_GRIP_SHIFT_US + SERVO_GRIP_TRIM_US))   // 356+33 = 389us（全闭）
#define SERVO2_MAX_US                  ((uint16_t)(1500 + SERVO_GRIP_SHIFT_US - SERVO_GRIP_TRIM_US))   // 1356-33 = 1323us（全开）

// ===== 每路舵机的"角度跨度"（用于 Servo_SetAngle：0 = 下限，SPAN = 上限）=====
// 机械爪的 0°/SPAN 指**机构**全闭/全开（已含上面的零点偏移与两端收进量）
#define SERVO0_SPAN_DEG   ((uint16_t)(SERVO0_MAX_ANGLE_DEG - SERVO0_MIN_ANGLE_DEG))
#define SERVO1_SPAN_DEG   ((uint16_t)(SERVO1_MAX_ANGLE_DEG - SERVO1_MIN_ANGLE_DEG))
#define SERVO2_MAX_DEG    (90 - 2 * SERVO_GRIP_TRIM_DEG)   // 机械爪：0°=全闭、SPAN=全开（当前 84°）
#define SERVO3_SPAN_DEG   ((uint16_t)(SERVO3_MAX_ANGLE_DEG - SERVO3_MIN_ANGLE_DEG))

// 机械爪上电停在**完全张开**（90°，一键式模式的初始位置，见下面 SERVO2_POWERON_US）

// ===== 舵机移动速度 =====
// ⚠️ Servo_Task 的调用节拍 = 主循环的 PS2 读取周期（现在 **10ms**，见 main.c 的 PS2_READ_PERIOD_MS）。
// 下面的速度只填 **°/s**，每 tick 的步长会自动按 SERVO_TICK_MS 换算，不需要手算。
// 三个机械臂舵机都是 11.11us/°（2000us=180°、机械爪 1000us=90°）。
#ifndef SERVO_TICK_MS
#define SERVO_TICK_MS   10       /* Servo_Task 调用周期（ms），必须和 main.c 的读取周期一致 */
#endif
/* °/s -> 每 tick 步长（单位 0.1us）：us/tick = deg*TICK/90 -> 0.1us/tick = deg*TICK/9 */
#define SERVO_STEP_FROM_DEG(deg_per_sec) \
    ((uint16_t)(((uint32_t)(deg_per_sec) * SERVO_TICK_MS + 4u) / 9u))

// **MG995**（CH1，最下面的臂）：**114°/s**（上一版 95°/s × 1.2），右摇杆前后控制
#define SERVO_MG995_DEG_PER_SEC   114
#define SERVO_MG995_STEP_US10     SERVO_STEP_FROM_DEG(SERVO_MG995_DEG_PER_SEC)

// **SG90-下**（CH2，下面那段臂）：**126°/s**（上一版 105°/s × 1.2），L1/L2 按住才动
#define SERVO_SG90_LOW_DEG_PER_SEC    126
#define SERVO_SG90_LOW_STEP_US10      SERVO_STEP_FROM_DEG(SERVO_SG90_LOW_DEG_PER_SEC)

// **SG90-上**（CH4，上面那段臂）：**140°/s**（上一版 117°/s × 1.2），R1/R2 按住才动
#define SERVO_SG90_UP_DEG_PER_SEC     140
#define SERVO_SG90_UP_STEP_US10       SERVO_STEP_FROM_DEG(SERVO_SG90_UP_DEG_PER_SEC)

// **机械爪**（MG90S，CH3）：**190°/s**（现场设定；点动和一键张合都按这个速度）
#define SERVO_GRIP_DEG_PER_SEC    190
#define SERVO_GRIP_STEP_US10      SERVO_STEP_FROM_DEG(SERVO_GRIP_DEG_PER_SEC)

// ===== 机械爪：靠近"闭合端"自动减速（越靠近越慢，防夹手/撞击）=====
// 离闭合端(全闭脉宽) SERVO_GRIP_SLOW_RANGE_DEG 度以内开始线性降速，
// 到最后只剩 SERVO_GRIP_SLOW_MIN_PCT% 的速度（张开方向从闭合端出发时同样由慢到快）。
#ifndef SERVO_GRIP_SLOW_ENABLE
#define SERVO_GRIP_SLOW_ENABLE      1
#endif
#ifndef SERVO_GRIP_SLOW_RANGE_DEG
#define SERVO_GRIP_SLOW_RANGE_DEG   25   /* 进入"慢速区"的范围（度）：25° ≈ 278us */
#endif
#ifndef SERVO_GRIP_SLOW_MIN_PCT
#define SERVO_GRIP_SLOW_MIN_PCT     20   /* 贴着闭合端时的速度百分比（20 = 只剩 1/5 速度） */
#endif

// ===== 加减速曲线（起步/收尾慢，中间快；不改目标、不阻塞）=====
// slew_toward_target() 每个 tick 更新一次"当前速度"：
//   起步：从 0 开始，每 tick 最多 +（巡航速度 / SERVO_RAMP_ACCEL_DIV），约 DIV 个 tick 到巡航；
//   收尾：剩余距离不足 SERVO_RAMP_TICKS 个 tick 的行程时按剩余距离线性降速。
// 曲线只影响速度变化，目标位置照旧精确到达（最后一 tick 会补齐余量）。
#ifndef SERVO_RAMP_ENABLE
#define SERVO_RAMP_ENABLE        1
#endif
#ifndef SERVO_RAMP_TICKS
#define SERVO_RAMP_TICKS         10    /* 收尾减速段长度（tick 数；5ms/tick -> 50ms） */
#endif
#ifndef SERVO_RAMP_ACCEL_DIV
#define SERVO_RAMP_ACCEL_DIV     8     /* 起步加速：巡航速度/本值 = 每 tick 增速（约 8 tick 到巡航） */
#endif

// ===== MG995 右摇杆控制参数 =====
// 摇杆死区（原始码值，PS2 惯例：推前 = 小值）+ 方向标定（实测方向反了就改成 -1）
#define SERVO_STICK_DEADZONE      10
#ifndef SERVO_MG995_STICK_SIGN
#define SERVO_MG995_STICK_SIGN    (+1)   // +1 = 推前 -> 角度增大（脉宽增大）
#endif

// ===== 机械爪（MG90S，0°=完全闭合 / 90°=完全张开）=====
#define SERVO_GRIP_OPEN_DEG        90                                  // 完全张开
// 一键式行程（合上角 = 90° - 本值）。想让爪子合得更紧就调大（如 75 -> 合到 15°）；
// 别给 90（会合到完全闭合死点，夹空物时一直顶死堵转发烫）。
#ifndef SERVO_GRIP_ONESHOT_DEG
#define SERVO_GRIP_ONESHOT_DEG     54                                  // 一键式行程 60°
#endif
#define SERVO_GRIP_CLOSE_DEG       (SERVO_GRIP_OPEN_DEG - SERVO_GRIP_ONESHOT_DEG)   // 当前 = 30°

// 一键张合的两个端点（脉宽）
// 张开 = 行程上限；合上 = 从张开端往回收 SERVO_GRIP_ONESHOT_DEG 度（1° = 11.11us，
// 与"两端各收进"无关，所以直接减脉宽，不用按跨度比例算）
#define SERVO_GRIP_OPEN_US         SERVO2_MAX_US                        // 完全张开（行程上限）
#define SERVO_GRIP_CLOSE_US        ((uint16_t)(SERVO2_MAX_US - \
                                     (uint32_t)SERVO_GRIP_ONESHOT_DEG * 1000 / 90))

#define SERVO2_POWERON_US          SERVO_GRIP_OPEN_US                   // 上电：完全张开

// ===== 机械臂总控模式（SELECT 键切换）=====
// 模式 0 = **全点动 + 两个一键动作（上电默认）**：
//   - 机械爪：按住 □ 向张开方向、按住 ○ 向闭合方向，松开停在当前位置（0~90° 全程）
//   - **L1 = 一键放下**（角度用 SERVO_DROP_* 宏）
//   - **L2 = 一键摆臂**（角度用 SERVO_POSE_* 宏）
//   - SG90(上)：R1/R2 点动；MG995：右摇杆前后
// 模式 1 = **只有机械爪一键张合**（机械臂不再有一键动作）：
//   - 机械爪：按一次自动走到位（完全张开 ↔ 合上），不再点动
//   - L1/L2/R1/R2 无功能
#define SERVO_GRIP_MODE_JOG        0
#define SERVO_GRIP_MODE_ONESHOT    1
#ifndef SERVO_GRIP_MODE_DEFAULT
#define SERVO_GRIP_MODE_DEFAULT    SERVO_GRIP_MODE_JOG     /* 上电默认：全部点动 */
#endif
// 切换模式的按键去抖（两次切换最小间隔）
#define SERVO_MODE_TOGGLE_MS       250U

// 当前机械爪控制模式（SELECT 切换；调试器里可直接 Watch 这个变量）
extern volatile uint8_t g_servo_grip_mode;

// 一键序列状态（调试器 Watch）：
//   g_servo_seq_id  : 0=空闲 / 1=一键放下(L1) / 2=一键摆臂(R1) / 3=自定义1(R2) / 4=自定义2(L2)
//   g_servo_seq_step: 0=空闲；1..5=当前步骤（1=MG995 2=SG90下 3=SG90上 4=延时 5=机械爪）
// ⚠️ step 一直非 0 说明序列卡住 —— 此时**所有手动控制都被跳过**（舵机都不听话）。
extern volatile uint8_t g_servo_seq_id;
extern volatile uint8_t g_servo_seq_step;

// ===== 机械爪堵转保护 =====
// 机械爪到位后若还一直按住按键，超过这个时间就断开该通道 PWM（舵机失力、不再发热）。
// 0 = 关闭（一直保持夹持力）。松键后会重新使能。爪子机构有卡阻时建议设 1000~2000。
#ifndef SERVO_GRIP_RELEASE_MS
#define SERVO_GRIP_RELEASE_MS 0
#endif

// ===== 行程校准模式（调试用，强烈建议机械爪先跑一次）=====
// 0 = 关闭；3 = 机械爪通道（1~4 分别对应 CH1~CH4）。
// 打开后：该通道从 MIN 开始，每 SERVO_CALIBRATE_STEP_MS 毫秒跳 SERVO_CALIBRATE_STEP_US 微秒，
// 一直走到 MAX 再往回走。你只要盯着看：**从哪个脉宽开始卡住/发出堵转声**，
// 然后把安全值（两端各留 50~100us 余量）填回 SERVOx_MIN_US / MAX_US，并把这里改回 0。
#ifndef SERVO_CALIBRATE_CH
#define SERVO_CALIBRATE_CH 0
#endif
#define SERVO_CALIBRATE_STEP_US 30      // 每步 50us（约 4.5°）
#define SERVO_CALIBRATE_STEP_MS 2000    // 每步保持 2 秒，方便观察

void     Servo_Init(void);
// 每 10ms 调用一次（传入手柄数据：按键 + 右摇杆Y 都要用）：
//   MG995(CH1)    : 右摇杆前后（推前/拉后），100°/s，行程 180°
//   SG90-下(CH2)  : L1/L2 按住转动，90°/s，行程 180°
//   SG90-上(CH4)  : R1/R2 按住转动，90°/s，行程 180°
//   MG90S(CH3)    : □/○（一键张合或点动），SELECT 切换模式（行为不变）
void     Servo_Task(const PS2_Data_t *ps2);
// 直接指定脉宽（会按该通道 min/max 限幅，并立即生效，不做插值）
void     Servo_SetPulse(uint8_t ch, uint16_t pulse_us);
// 按"机械角度"设置（0~该通道 SERVOx_MAX_DEG，超范围自动限幅）
// 机械爪：0 = 完全闭合，90 = 完全张开
void     Servo_SetAngle(uint8_t ch, uint16_t deg);
uint16_t Servo_GetPulse(uint8_t ch);    // 当前实际脉宽
uint16_t Servo_GetTarget(uint8_t ch);   // 目标脉宽

// 机械爪控制模式
uint8_t  Servo_GetGripMode(void);       // 0=点动保持，1=一键张合
// 切换机械爪控制模式。注意：**切换时不会让机械爪立刻动作**（保持当前位置不变），
// 只有按下 □ / ○ 之后才会一次走到对应端点。
void     Servo_SetGripMode(uint8_t mode);

#endif
