# 相对角差（视觉伺服）CAN 协议

> 目的：**在视觉侧读不到电机/云台位姿（无编码器、无 IMU 四元数）的情况下驱动云台**。
> 视觉每帧只下发「目标相对相机光轴的角差」，电控把角差累加到自己的位置上，
> 图像上的偏差逐帧收敛到 0。
>
> 实现：`tasks/auto_aim/visual_servo.{hpp,cpp}`（角差）+ `io/cboard.{hpp,cpp}`（帧编码）
> 入口：`src/relative_aim.cpp`（目标 `relative_aim`）
> 自检：`tests/relative_servo_test.cpp`（目标 `relative_servo_test`，不需要任何硬件）

---

## 1. 与旧（绝对角）协议的区别

| | 旧模式 `can_command_mode: "absolute"` | 新模式 `can_command_mode: "relative"` |
| --- | --- | --- |
| CAN ID | `send_canid`（`standard4.yaml` = `0xff`） | `relative_canid`（默认 `0xFE`） |
| 语义 | 世界系**绝对** yaw / pitch | 相对相机光轴的**角差** Δyaw / Δpitch |
| 依赖 | 电控/电机要有姿态反馈才能闭环 | **不需要**任何姿态反馈 |
| 视觉来源 | `Aimer`（EKF + 弹道预测，世界系） | `VisualServo`（`armor.xyz_in_gimbal`，云台系） |
| 丢目标 | `control=0`，角度字段仍写 0 | `control=0`，角差字段写 0（不会跳变） |

两种模式**不会同时发送**：`CBoard::send()` 按 `can_command_mode` 二选一。
缺键时默认 `absolute`，所以升级本仓库不会改变已有机器人的行为。

---

## 2. 坐标系与符号

数据来自 `Solver::solve()` 的 `armor.xyz_in_gimbal`（目标在**云台系**下的坐标），
云台系为 **F-L-U**：`x` 向前（相机光轴）、`y` 向左、`z` 向上。于是

```
Δyaw   = atan2(y, x)                     // 目标在左 -> 正；在右 -> 负
Δpitch = -atan2(z, sqrt(x*x + y*y))      // 向上为负（与 Aimer / CBoard 旧协议一致）
```

- `Δyaw / Δpitch` **不是**目标的世界绝对角，而是「目标在图像里偏离光轴多少」。
  云台转过去以后这个值自然变小，所以它是比例伺服（增益见第 6 节）。
- `pitch` 向上为负是 `tasks/auto_aim/aimer.cpp` 既有约定
  （`pitch = -(trajectory.pitch)`，`Trajectory::pitch` 抬头为正）。
  如果电控侧实际是「抬头为正」，**只在一处**取反：要么 `servo_pitch_sign: -1`，
  要么固件里取反，不要两边都改。

---

## 3. CAN 帧格式（标准帧，DLC = 8）

```
ID  = relative_canid            (默认 0xFE，可在 configs/*.yaml 改)
DLC = 8

byte0: bit0 = control          1 = 本帧有效，请按角差移动
       bit1 = shoot            1 = 请求开火（当前 VisualServo 恒为 0）
       bit2..7 = 0
byte1: 0                       （保留）
byte2..3: Δyaw    int16 大端   单位 1e-4 rad（即数值 = 弧度 × 10000）
byte4..5: Δpitch  int16 大端   单位 1e-4 rad
byte6..7: 0                    （保留；旧 UAV 帧这里是 horizon_distance，相对模式不用）
```

示例（`Δyaw = +0.1234 rad`，`Δpitch = -0.0567 rad`，control=1，shoot=0）：

```
ID 0xFE  data = 01 00 04 D2 FD C9 00 00
                ^  ^  ^^^^^  ^^^^^  ^^^^^
                |  |  yaw    pitch  reserved
                |  reserved
                control=1, shoot=0
```

- **饱和而不是回绕**：编码时超出 `int16` 范围会截断到 `+32767 / -32768`
  （≈ ±3.2767 rad），避免把「向右转」变成「向左转」。
- 分辨率 `1e-4 rad ≈ 0.0057°`。每帧角差通常 < 0.1 rad，量化误差 ~1e-4 rad，
  且下一帧的角差是重新按图像算的，**不会累积**。
- 与旧的 `0xff` 帧对比：两者都是 int16 大端 ×1e4，固件里 `yaw/pitch` 的解码函数
  可以直接复用，只是**含义从绝对角变成增量**。

### 3.1 固件侧参考实现

```c
/* 收到 0xFE 时调用；在「收到帧」的事件里执行，不要用定时器重复累加同一帧 */
void on_relative_cmd(const uint8_t data[8]) {
    if (!(data[0] & 0x01)) {          /* control=0：没有目标，保持当前位置 */
        return;
    }

    int16_t dyaw   = (int16_t)((data[2] << 8) | data[3]);
    int16_t dpitch = (int16_t)((data[4] << 8) | data[5]);

    float delta_yaw   = dyaw   / 10000.0f;   /* rad */
    float delta_pitch = dpitch / 10000.0f;   /* rad，向上为负 */

    /* 增量叠加到「目标位置」，再由位置环/速度环去执行 */
    gimbal_yaw_target   += delta_yaw;
    gimbal_pitch_target += delta_pitch;

    /* 安全：目标位置限幅、速度限幅、超时保护都要保留 */
    gimbal_yaw_target   = clampf(gimbal_yaw_target,   YAW_MIN,   YAW_MAX);
    gimbal_pitch_target = clampf(gimbal_pitch_target, PITCH_MIN, PITCH_MAX);

    /* 建议再限制单帧增量（与视觉侧 servo_max_step_deg 独立的一道保护） */
}
```

要点：

1. **事件驱动累加**：只在收到新帧时加一次。若固件用 1 kHz 定时器反复累加同一帧，
   云台会失控。
2. `control=0` 时必须**停止累加**（停住，不要继续按上一条命令走）。
3. 建议加**接收超时**（例如 100 ms 没收到有效帧就停），防止视觉掉线后云台
   按最后的增量继续跑。
4. 增量叠加到「目标位置」，不要直接叠加到电流/力矩上；位置环/速度环的平滑
   会引入延迟，延迟越大越容易振荡（见第 7 节调参）。

---

## 4. 视觉侧配置键（全部为新增可选键，缺省值即下表）

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `can_command_mode` | `"absolute"` | 改成 `"relative"` 才启用相对角差帧 |
| `relative_canid` | `0xFE` | 相对角差帧 CAN ID（不要与 `0x100 / 0x101 / 0xff` 冲突） |
| `cboard_wait_imu` | `true` | `false`：构造不阻塞等四元数，`imu_at()` 恒返回单位四元数 |
| `servo_gain` | `1.0` | 单帧角差比例系数 |
| `servo_deadband_deg` | `0.1` | 死区（度），偏差小于该值不发；`0` = 关闭 |
| `servo_max_step_deg` | `20` | 单帧最大角差（度），防首帧/换目标猛甩 |
| `servo_yaw_sign` | `1` | 转向反了改 `-1` |
| `servo_pitch_sign` | `1` | 转向反了改 `-1` |
| `servo_yaw_offset_deg` | `0` | 静态偏置（相机-枪管安装误差） |
| `servo_pitch_offset_deg` | `0` | 静态偏置（弹道下坠近似补偿） |

> `cboard_wait_imu: false` 只影响 `CBoard`：构造不再 `queue_.pop()` 阻塞，
> `imu_at()` 直接返回单位四元数。**其余遥测（`0x101` 的弹速/模式）仍然照常解析**。

---

## 5. 启动与验证步骤（从"不动"到"闭环"）

```bash
# 0) 编译（最小目标）
make -C build/ relative_aim relative_servo_test -j$(nproc)

# 1) 纯离线自检：不需要相机 / CAN / 电机，必须全 ok
./build/relative_servo_test

# 2) 只算不发（不需要 CAN 硬件）：确认检测→跟踪→角差这条链路有输出
./build/relative_aim -c=configs/standard4.yaml -dry-run -n=200

# 3) 看 CAN 上是否真的发出相对帧（另开一个终端）
candump -tz can0,0FE:7FF
#    静止不动时应能看到 0xFE 帧；
#    control=0 时 data[2..5] 全 0；control=1 时 Δyaw/Δpitch 随目标偏移变化

# 4) 低速闭环：先把 servo_gain 调到 0.2、servo_max_step_deg 调到 3，
#    用手/纸板在画面里左右移动，确认：
#      - 云台朝目标方向转（方向反了就改 servo_yaw_sign / servo_pitch_sign）
#      - 转过去后 Δyaw/Δpitch 变小（说明闭环成立）
#      - 不振荡（振荡就降 gain、加 deadband）

# 5) 正常速度
./build/relative_aim -c=configs/standard4.yaml
#    Ctrl+C 退出（tools::Exiter 只处理 SIGINT，不要用 kill/timeout 默认的 SIGTERM）
```

⚠️ 板上若没有 `0x100` 四元数帧，`configs/*.yaml` 里必须 `cboard_wait_imu: false`，
否则 `CBoard` 构造会一直停在 `[Cboard] Waiting for q...`。

---

## 6. 调参顺序

1. **方向**：`servo_yaw_sign` / `servo_pitch_sign`。目标在画面左边 → 云台应向左转，
   `Δyaw` 应为正、云台向左（从上方看逆时针）。
2. **死区**：`servo_deadband_deg`，从 `0.1` 起。相机噪声大就加到 `0.2~0.3`。
3. **增益**：`servo_gain` 从 `0.3` 起，逐步加到 `1.0`。云台来回晃就是增益过大或
   **执行延迟过大**（电控侧平滑/限速太强），先降 gain。
4. **单帧限幅**：`servo_max_step_deg` 默认 `20`，近距离/换目标时防止猛甩。
5. **静态偏置**：`servo_yaw_offset_deg` 修正相机-枪管不同轴；`servo_pitch_offset_deg`
   近似补偿弹道下坠（远距离可以给 1~2°）。

---

## 7. 已知局限（重要）

1. **没有弹道/超前量**：`VisualServo` 只把当前观测到的装甲板中心对到光轴上，
   不含 `Aimer` 的飞行时间迭代与目标预测。打移动/旋转目标时会滞后，
   后续可以在云台系里按角速度做超前补偿（需要一个小的角速度估计器），
   或把 `Aimer` 的预测点用 `R_gimbal2world` 转回云台系（那样又需要 IMU）。
2. **没有绝对基准**：断电/重启、或电控侧目标位置被复位后，云台与目标的相对关系
   由「当前图像」重新建立，不存在"零点漂移"问题；但也**不能**用它做定点扫描。
3. **丢目标即停**：`targets` 为空或 `tracker` 丢锁时 `control=0`，云台保持当前位置
   （依赖固件的超时/保持逻辑）。
4. **换目标首帧不发**：`Tracker::set_target()` 只对选中的那一块装甲板调用
   `solver.solve()`，其余同型号装甲板的 `xyz_in_gimbal` 还没算过。`VisualServo`
   在换目标后的第一帧返回 `control=0`，第二帧（`update_target()` 已把所有命中
   装甲板解算过）才开始发命令。这是有意的安全策略，不是丢帧 bug。
5. **不做开火判定**：`shoot` 恒为 0；相对角差无法判断"是否对准"，开火请沿用
   电控侧逻辑或后续接入 `Shooter`（需要绝对角）。

---

## 8. 相关文件

| 文件 | 作用 |
| --- | --- |
| `io/cboard.hpp/.cpp` | `CommandMode`、`encode_relative_command()`、`send()` 分支、`cboard_wait_imu` |
| `tasks/auto_aim/visual_servo.hpp/.cpp` | `VisualServo::angles/shape/aim`（死区、限幅、符号、偏置） |
| `src/relative_aim.cpp` | 端到端入口（相机 → 检测 → 跟踪 → 角差 → CAN） |
| `tests/relative_servo_test.cpp` | 离线自检（角度符号、限幅、字节布局） |
| `configs/standard4.yaml` | `can_command_mode` / `relative_canid` / `servo_*` 键 |
| `documentation/handoff_relative_servo.md` | 交接文档：当前进度、下一步（直播页面显示角差）、已知 tracker 问题 |

