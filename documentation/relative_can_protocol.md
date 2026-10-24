# 相对角差（视觉伺服）CAN 协议

> 目的：**在视觉侧读不到电机/云台位姿（无编码器、无 IMU 四元数）的情况下驱动云台**。
> 视觉每帧下发「这一帧观测到的目标偏离参考轴的角差」。这是测量值，不是必须执行的步长。
> 电控自行选择位置/速度闭环策略；不得在定时器里反复累加最后一条测量值。
>
> ⚠️ 关键区分（旧版本这里写混了）：视觉侧算的是**每帧重新观测的角差**，
> 不是「位置目标」。将测量值转成控制量是**电控侧**的职责，不要求逐帧累加。
> 参考轴有两种，必须选一种并向电控说清用的是哪种，见 §2。
>
> 实现：`tasks/auto_aim/current_relative.{hpp,cpp}`（当前帧观测，默认路径）、
> `tasks/auto_aim/visual_servo.{hpp,cpp}`（旧的跟踪路径）+ `io/cboard.{hpp,cpp}`（帧编码）
> 入口：`src/relative_aim.cpp`（目标 `relative_aim`）
> 自检：`tests/relative_servo_test.cpp`（目标 `relative_servo_test`，不需要任何硬件）
> 验收记录：`documentation/relative_acceptance.md`
>
> **2026-10-24 更新**：握手/会话/启停/颜色及接板验收以同目录
> `vision_cboard_protocol.md`、`vision_cboard_acceptance.md`、`vision_cboard_handoff.md` 为准。
> `relative_aim` 默认 v1；实发必须显式配置 ID 并确认，`0xFE` 不是实机分配值。

---

## 1. 与旧（绝对角）协议的区别

| | 旧模式 `can_command_mode: "absolute"` | 新模式 `can_command_mode: "relative"` |
| --- | --- | --- |
| CAN ID | `send_canid`（`standard4.yaml` = `0xff`） | 显式 `relative_canid`（接板前由电控确认） |
| 语义 | 世界系**绝对** yaw / pitch | 目标相对**参考轴**的角差 Δyaw / Δpitch |
| 依赖 | 电控/电机要有姿态反馈才能闭环 | **不需要**任何姿态反馈 |
| 视觉来源 | `Aimer`（EKF + 弹道预测，世界系） | `CurrentRelativeEstimator`（当前帧观测，默认）或 `VisualServo`（`armor.xyz_in_gimbal`，旧跟踪路径） |
| 时间性质 | 对目标的**状态估计**（含预测） | 对**这一帧图像**的观测（不预测） |
| 丢目标 | `control=0`，角度字段仍写 0 | `control=0`，角差字段强制写 0（不会跳变） |
| 过期观测 | 不适用 | 超过 `relative_max_frame_age_ms` → 判为无效，`control=0` |

两种模式**不会同时发送**：`CBoard::send()` 按 `can_command_mode` 二选一。
缺键时默认 `absolute`，所以升级本仓库不会改变已有机器人的行为。

---

## 2. 参考轴：两种语义，别混称「相对相机光轴」

`relative_reference` 选一个，启动日志会打印实际用的那一个：

### 2.1 `camera_optical`（默认，**不需要手眼外参**）

- 参考点 = **相机光心**，参考轴 = **相机光轴**（相机光学系 +z）。
- 只用 `camera_matrix` / `distort_coeffs`：4 个角点 → PnP 得到目标中心在相机系下的
  `xyz_camera`（x 右、y 下、z 前）→ 一次性 90° 约定变换到「光轴当 +x」的语义坐标
  （x 前、y 左、z 上）：

  ```
  (x_f, y_f, z_f) = (z_cam, -x_cam, -y_cam)
  ```

- 然后与旧约定完全一致：

  ```
  Δyaw   = atan2(y_f, x_f)                 // 目标在左 -> 正；在右 -> 负
  Δpitch = -atan2(z_f, sqrt(x_f² + y_f²))  // 向上为负（与 Aimer / CBoard 旧协议一致）
  ```

- 本机（还没做手眼标定）就用这个。它衡量的是「目标偏离光轴多少」。

### 2.2 `gimbal`（**必须**有手眼外参）

- 参考点/轴取**云台系** F-L-U：`xyz_gimbal = R_camera2gimbal * xyz_camera + t_camera2gimbal`，
  再接同一套 `atan2` 公式。
- 缺 `R_camera2gimbal` / `t_camera2gimbal` 时构造**直接报错**（不会静默按 `camera_optical` 算）。
- 只有「相机光轴恰好与云台 x 轴重合、且光心就在云台原点」时，两者才等价。

**两者不是同一个量**：光心到云台原点的平移（本仓库 `demo.yaml` 里是 ~[95, 95, 51] mm）
在近距离会带来可观的角差差别。离线自检里有一条用例专门验证这一点
（`current: gimbal 用单位阵外参 -> 角差与 camera_optical 完全不同（不可混用）`）。

- `Δyaw / Δpitch` **不是**目标的世界绝对角，而是「目标偏离参考轴多少」（每帧重新观测）。
  云台转过去以后这个值自然变小 —— 所以它是比例伺服（增益见第 6 节）。
- `pitch` 向上为负是 `tasks/auto_aim/aimer.cpp` 既有约定
  （`pitch = -(trajectory.pitch)`，`Trajectory::pitch` 抬头为正）。
  如果电控侧实际是「抬头为正」，**只在一处**取反：要么 `servo_pitch_sign: -1`，
  要么固件里取反，不要两边都改。

---

## 3. CAN 帧格式（标准帧，DLC = 8）

```
ID  = relative_canid            (必须显式配置并与电控确认；下例 0xFE 仅为示例)
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

---

## 3.1 视觉侧强制的编码安全性（不依赖调用方）

`io::encode_relative_command()` 与 `io::quantize_angle_1e4()` 是**唯一**的编码实现
（`CBoard::send()` 与 `CurrentRelativeEstimator` 的 TX 值都用它），并且强制如下规则：

| 输入 | 编码结果 | 为什么 |
| --- | --- | --- |
| `control=0` | 角差写 0、`shoot` 也写 0（即使调用方传了非零角度） | 帧里不能残留「上一帧的旧角度」，否则电控会继续累加 |
| `yaw` 或 `pitch` 为 NaN / Inf | `control` 清零、角差写 0 | 非有限值进 `lround` 是未定义行为；这类帧一定是无效观测 |
| 角度超出 int16 | 饱和到 `+32767 / -32768` | 不回绕，避免「向右转」变成「向左转」 |

`shoot` 在相对角差模式下恒为 0：单看角差无法判断「是否对准」，开火必须由电控/后续
逻辑决定。

`CBoard::send()` 返回 **bool**：只有「整帧完整写进 SocketCAN」才算成功
（`SocketCAN::write()` 检查 `write()` 返回值 == `sizeof(can_frame)`，并把 errno 一起报出来）。
旧调用方忽略返回值不受影响，但 `relative_aim` 会统计成功/失败并在 HUD/CSV 里给出。

### 3.2 固件侧参考实现


```c
/* 仅演示测量解码；v1 还需握手/会话、本地启停与接收超时门控。
   电控自行实现受限闭环，不直接把这段示意当作运动控制代码。 */
void on_relative_cmd(const uint8_t data[8]) {
    if (!(data[0] & 0x01)) {          /* control=0：没有目标，保持当前位置 */
        return;
    }

    int16_t dyaw   = (int16_t)((data[2] << 8) | data[3]);
    int16_t dpitch = (int16_t)((data[4] << 8) | data[5]);

    float delta_yaw   = dyaw   / 10000.0f;   /* rad */
    float delta_pitch = dpitch / 10000.0f;   /* rad，向上为负 */

    /* 将 delta_yaw / delta_pitch 交给电控自己的受限闭环。
       保留位置/速度/加速度限制；丢目标/停车/接收超时都清除测量有效性。 */
}
```

要点：

1. **事件驱动更新测量**：一帧只更新一次。禁止在定时器里无限重复累加同一观测。
2. `control=0` 时必须**清除有效测量**（停住，不要继续按上一条命令走）。
3. 建议加**接收超时**（例如 100 ms 没收到有效帧就停），防止视觉掉线后云台
   按最后的增量继续跑。
4. 电控选择闭环策略，不将测量直接叠加到电流/力矩；考虑图像和执行延迟。

---

## 4. 视觉侧配置键（全部为新增可选键，缺省值即下表）

### 4.1 入口 / CAN

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `can_command_mode` | `"absolute"` | 改成 `"relative"` 才启用相对角差帧；`relative_aim` 启动时会先校验它 |
| `relative_canid` | 入口无隐式默认 | 相对角差帧 CAN ID；必须在 `0..0x7FF`，不得与其它协议 ID 冲突；非核实 vcan 需 `vision_can_ids_confirmed: true` |
| `cboard_wait_imu` | `true` | 相对角差模式**必须** `false`：`CBoard` 构造不再 `queue_.pop()` 阻塞等四元数，`imu_at()` 恒返回单位四元数 |
| `relative_send_interval_ms` | `0` | 有效帧的最小发送间隔；`0` = 每帧都发。无效帧不受限（立刻发出去让电控停） |

### 4.2 当前帧观测（`-source=current`，默认路径）

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `relative_reference` | `"camera_optical"` | `camera_optical`（光轴，不用外参）/ `gimbal`（云台系，需要外参） |
| `relative_max_frame_age_ms` | `120` | 观测年龄上限（`发送时刻 - 图像时间戳`），超过 → `stale_frame`、`control=0` |
| `relative_min_confidence` | `min_confidence` | 再上一道置信度阈值（过滤低质量检出） |
| `relative_max_reprojection_px` | `15` | PnP 解的**平均**重投影误差上限；超过说明这一帧几何不可信 |
| `relative_min_depth_m` | `0.1` | 目标在相机前方的最小距离（米）；更近判为无效 |
| `relative_filter_color` | 有 `enemy_color` 时为 `true` | 只保留 `enemy_color` 那一色的装甲板 |
| `enemy_color` | 无（不过滤） | `red` / `blue` / `extinguish` / `purple` |

### 4.3 整形（raw → TX，两条路径共用）

| 键 | 默认 | 说明 |
| --- | --- | --- |
| `servo_gain` | `1.0` | 单帧角差比例系数 |
| `servo_deadband_deg` | `0.1` | 死区（度），偏差小于该值发 0；验收/前馈调试时设 `0`，让 TX 与 raw 只差量化 |
| `servo_max_step_deg` | `20` | 单帧最大角差（度），防首帧/换目标猛甩 |
| `servo_yaw_sign` | `1` | 转向反了改 `-1` |
| `servo_pitch_sign` | `1` | 转向反了改 `-1` |
| `servo_yaw_offset_deg` | `0` | 静态偏置（相机-枪管安装误差） |
| `servo_pitch_offset_deg` | `0` | 静态偏置（弹道下坠近似补偿） |

`CurrentRelativeEstimator`（当前帧）与 `VisualServo`（跟踪）读的是**同一份**
`load_servo_limits()`，不存在两条路径参数各说各话的问题。

> `cboard_wait_imu: false` 只影响 `CBoard`：构造不再阻塞，`imu_at()` 直接返回单位四元数。
> **其余遥测（`0x101` 的弹速/模式）仍然照常解析**。

---

## 5. 启动与验证步骤（从"不动"到"闭环"）

```bash
# 0) 编译（最小目标）
make -C build/ relative_aim relative_servo_test camera_test -j$(nproc)

# 1) 纯离线自检：不需要相机 / CAN / 电机，必须全 ok（当前 54 checks）
./build/relative_servo_test

# 2) 只算不发（不需要 CAN）：验证「检测 → 当前帧选目标 → PnP → 角差 → 量化」
#    -csv 给出逐帧 raw/TX，-stream 给出 HUD（无显示器时用浏览器看）
./build/relative_aim -c=configs/relative_current_replay.yaml -dry-run -n=200 \
                     -csv=/tmp/rel.csv -stream=8080
#    判据：出现 control=1 且 raw/tx 非零的连续帧；无效帧的 reason 只有
#          no_target / out_of_image / color_filtered / reprojection_error 等可解释原因

# 3) 软件链路验收（隔离 netns + 独立模拟电控，不触碰 can0）：
unshare -Urn sh -c '
  ip link add vcan_test type vcan && ip link set vcan_test up &&
  python3 /home/irm/iRM-Vision-2026/scripts/vision_cboard_sim.py
'

# 4) 真机抓包（CAN 已起来：ip link set up can0，1Mbps）
candump -tz can0,0FE:7FF
#    静止不动时应能看到 0xFE 帧；
#    control=0 时 data[2..5] 全 0；control=1 时 Δyaw/Δpitch 随目标偏移变化

# 5) 低速闭环：在电控侧限速/限位，视觉默认不做整形；如确需旧整形，先置
#    relative_tx_shape: true，再调 servo_gain / servo_max_step_deg。
#    用手/纸板在画面里左右移动，确认：
#      - 云台朝目标方向转（方向反了就改 servo_yaw_sign / servo_pitch_sign）
#      - 转过去后 Δyaw/Δpitch 变小（说明闭环成立）
#      - 不振荡（振荡就降 gain、加 deadband）

# 6) 正常速度（默认 current 路径；旧跟踪路径加 -source=tracker）
./build/relative_aim -c=configs/relative_current.yaml
#    Ctrl+C 退出（tools::Exiter 只处理 SIGINT，不要用 kill/timeout 默认的 SIGTERM）
```

退出行为（`current` 与 `tracker` 两条路径一致）：

- 正常退出（Ctrl+C / `-n` 到点）前会**补发一帧 `control=0`**（best effort），
  日志里写明成功/失败；比等电控自己的接收超时更早。
- 相机断流 → `camera.read()` 阻塞，期间不发新帧；恢复后第一帧的观测年龄会变大，
  超过 `relative_max_frame_age_ms` 就是 `stale_frame`（`control=0`），不会把旧观测当新值。
- 发送失败（CAN 掉线、`ENETDOWN`、`ENOBUFS`…）→ `CBoard::send()` 返回 false，
  计入日志的「失败」计数并按 1Hz 限流打印 errno；`SocketCAN` 的守护线程会自己重连。

⚠️ 板上若没有 `0x100` 四元数帧，`configs/*.yaml` 里必须 `cboard_wait_imu: false`，
否则 `CBoard` 构造会一直停在 `[Cboard] Waiting for q...`。
`relative_aim` 已在**构造 CBoard 之前**校验这个键，不满足直接拒绝启动。

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

1. **没有弹道/超前量**：两条路径都只把「当前观测到的装甲板中心」对到参考轴上，
   不含 `Aimer` 的飞行时间迭代与目标预测。打移动/旋转目标时会滞后；要补超前量
   需要角速度估计器，或把 `Aimer` 的预测点用 `R_gimbal2world` 转回云台系（那又需要 IMU）。
2. **`current` 路径不做时间滤波**：每帧独立解算，单帧的 PnP 抖动会直接进角差
   （量化前通常 < 0.1°）。要平滑就在电控侧做低通，或改用 `-source=tracker`（有 EKF）。
3. **没有绝对基准**：断电/重启、或电控侧目标位置被复位后，云台与目标的相对关系
   由「当前图像」重新建立，不存在"零点漂移"问题；但也**不能**用它做定点扫描。
4. **丢目标即停**：无检出 / 颜色不符 / 角点越界 / 重投影超限 / 观测过期 → `control=0`，
   云台保持当前位置（依赖固件的超时/保持逻辑）。
5. **`current` 路径没有「换目标保护」但有确定性**：每帧按「离画面中心最近」重选目标，
   同一帧输入永远给出同一块；多块同色装甲板相邻时，画面里谁更靠中心就发谁，
   `target_id` 会随之变化（CSV 里可直接看到切换）。要「锁死一个目标」请用 `-source=tracker`。
6. **`tracker` 路径的旧行为保留**：需要连续命中（`min_detect_count`）才锁定，
   换目标后第一帧不发（同型号装甲板尚未 `solve`），且 EKF 在抖动大的片段上可能发散。
   这条路径的 raw 与 TX 相同（整形发生在 `VisualServo::aim()` 内部）。
7. **不做开火判定**：`shoot` 恒为 0；相对角差无法判断"是否对准"，开火请沿用
   电控侧逻辑或后续接入 `Shooter`（需要绝对角）。
8. **图像时间戳不是曝光时刻**：MindVision 的 `CameraGetImageBuffer` 返回的是**主机取到
   buffer 的时刻**，回放后端返回的是**读到这一帧的时刻**。所以 `age_ms` 衡量的是
   「检测 + 排队延迟」，不包含曝光与传输延迟。

---

## 8. 相关文件

| 文件 | 作用 |
| --- | --- |
| `io/command.hpp` | `Command`、`quantize_angle_1e4()`（唯一的量化实现） |
| `io/cboard.hpp/.cpp` | `CommandMode`、`encode_relative_command()`、`send()` 分支与 bool 返回、`cboard_wait_imu`、`relative_canid` 校验 |
| `io/socketcan.hpp` | 整帧写入校验、errno 报错、close 后 fd 复位 |
| `tasks/auto_aim/current_relative.hpp/.cpp` | 当前帧观测：过滤、确定性选目标、PnP、参考系、时效、量化 |
| `tasks/auto_aim/visual_servo.hpp/.cpp` | 旧跟踪路径的 `angles/shape/aim` + `load_servo_limits()` |
| `src/relative_aim.cpp` | 端到端入口（`-source`、`-dry-run`、`-csv`、`-stream`） |
| `tests/relative_servo_test.cpp` | 离线自检（符号、限幅、字节布局、编码安全、几何解算、选目标、时效） |
| `configs/relative_current.yaml` | 真机专用配置（相机 + 内参 + 相对角差 + CBoard） |
| `configs/relative_current_replay.yaml` | 同一套相对角差参数 + 视频回放（无硬件验收用） |
| `documentation/relative_acceptance.md` | 验收证据（线材/帧率、dry-run、vcan 字节比对、板端清单） |
| `documentation/handoff_relative_servo.md` | 交接文档：做到哪了 / 下一步 / 坑 |

