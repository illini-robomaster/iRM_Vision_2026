# 交接文档：CAN 相对角差（视觉伺服）— 2026-10-17

> 给下一次接手的自己（或别人）看的**工作状态快照**。协议与真机联调细节在
> `documentation/relative_can_protocol.md`，本文只记「做到哪了 / 下一步做什么 / 坑在哪」。
> 环境：Jetson（aarch64），仓库 `/home/irm/iRM-Vision-2026`，`build/` 已配置好（勿删）。

## 0. 下次开工第一件事（TL;DR）

> **2026-10-24 更新**：下面这段原始计划已经做完（角差 HUD + 逐帧 CSV 落在
> `src/relative_aim.cpp`，不再改 `detect_freq_visual_test`）。当前状态与实测数据见
> §0.1 和 `documentation/relative_acceptance.md`。

~~把相对角差（Δyaw / Δpitch）显示到 `detect_freq_visual_test` 的本地直播页面上~~
已完成，改为在**专用入口**里做（复用 `tools::MjpegServer`）：

```bash
./build/relative_aim -c=configs/relative_current.yaml -stream=8080        # HUD 直播
./build/relative_aim -c=configs/relative_current.yaml -csv=/tmp/r.csv     # 逐帧 CSV
```

### 0.1 2026-10-24 的实际进展（比 §1 的表新）

| 项 | 状态 |
| --- | --- |
| 新增 `tasks/auto_aim/current_relative.{hpp,cpp}`：当前帧观测（过滤 → 确定性选目标 → PnP → 参考系 → 时效 → 量化） | ✅ 已实现，`relative_servo_test` 覆盖 |
| `src/relative_aim.cpp`：`-source=current/tracker`、`-csv`、`-stream`、`-d`、退出补发 `control=0` | ✅ 已实现 |
| `relative_reference`：`camera_optical`（默认，无需外参）/ `gimbal`（必须外参，缺键报错） | ✅ 已实现 |
| `CBoard::send()` 返回 bool；`encode_relative_command()` 对 NaN/Inf/control=0 强制清零 | ✅ 已实现 |
| `SocketCAN::write()` 整帧校验 + errno；`close()` 后 fd 复位；EINTR/EAGAIN 处理 | ✅ 已实现 |
| `configs/relative_current.yaml`（真机）/ `configs/relative_current_replay.yaml`（回放验收） | ✅ 已加 |
| 离线自检 | ✅ **54 checks, 0 failures** |
| 回放 dry-run | ✅ 150 帧 / 有效 124（82.7%）/ 检测 23.3 ms/帧 |
| vcan 字节级验收 | ✅ 121 帧全 `0FE [8]`，99 个 `control=1` 与 CSV 逐帧一致（0 处不一致） |
| 新 USB3.0 线取流 | ✅ 1280×1024 **120.5 fps**（3×60s，p95 ≤8.5ms，丢帧 0/0/1） |
| 内参归属 | ✅ 已核实就是**现在这枚相机**（SN 043070620316）：8 个标定 run 的日志全是这个 SN，`intrinsics.yaml` 与 `configs/relative_current.yaml` 逐字符相同（证据见验收文档 §3.1） |
| 旧线 A/B、真 CBoard、云台闭环、开火 | ❌ 未验证（旧线规格已知：USB 2.0 Type-B 无锁线，但线不在手上；其余缺硬件，清单见验收文档 §7） |


## 1. 当前状态

| 项 | 状态 |
|---|---|
| `io/cboard.{hpp,cpp}` 新增 `can_command_mode: absolute/relative`（默认 absolute，旧行为不变） | ✅ 已改，已编译 |
| `encode_relative_command()`（ID `0xFE`/可配，DLC 8，Δyaw/Δpitch int16 BE ×1e4 rad，饱和不回绕） | ✅ 已实现 + 单测覆盖 |
| `cboard_wait_imu`（默认 true；false 时不等 `0x100` 四元数，`imu_at()` 返回单位四元数） | ✅ 已改 |
| `tasks/auto_aim/visual_servo.{hpp,cpp}`（`angles()` / `shape()` / `aim()`） | ✅ 已实现 + 单测覆盖 |
| `src/relative_aim.cpp`（入口：相机 → 检测 → 跟踪 → 伺服 → CAN，`-dry-run` / `-n`） | ✅ 能编译，可跑 |
| `tests/relative_servo_test.cpp`（纯函数 + 编码 + 流水线自测） | ✅ **22 checks, 0 failures** |
| `configs/standard4.yaml` 新增可选键（`can_command_mode` / `relative_canid` / `cboard_wait_imu` / `servo_*`） | ✅ 已加 |
| `documentation/relative_can_protocol.md`（协议 / 固件参考实现 / 配置表 / 联调步骤） | ✅ 已写 |
| 端到端 dry-run（离线 demo 回放跑出非零角差） | ✅ **2026-10-24 解决**：`-source=current` 用本帧检出做 PnP，不依赖 tracker；150 帧里 124 帧有效（见验收文档 §4.2） |
| 直播页面显示相对角差 | ✅ 2026-10-24 完成：`./build/relative_aim ... -stream=8080`（HUD 里 raw/tx 两行 + 选中目标绿框） |
| 固件侧（电控）按增量执行的 handler | ❌ 未做（参考实现在协议文档里） |
| git 提交 | ✅ 已提交并推送 `44d15df`（`origin/main` 与本地 HEAD 一致）；仅 `.vscode/c_cpp_properties.json` 留在本地未提交（见 §2） |

## 2. 改动清单与提交状态

**已提交并推送**：`44d15df feat(io): 新增 CAN 相对角差（视觉伺服）命令通道`
（`git --no-pager log --oneline -1 origin/main` 与本地 HEAD 相同）。

本次提交包含（tracked 改动 + 新文件）：

- `CMakeLists.txt`（新增 `relative_aim`、`relative_servo_test` 目标）
- `tasks/auto_aim/CMakeLists.txt`（`visual_servo.cpp` 进 `auto_aim` 库）
- `io/cboard.hpp`（`CommandMode` 枚举、`encode_relative_command()` 声明、`relative()`）
- `io/cboard.cpp`（构造读新键、`send()` 分支、相对帧编码、`wait_imu_`）
- `configs/standard4.yaml`（新增可选键，含注释）
- `src/relative_aim.cpp`
- `tasks/auto_aim/visual_servo.hpp`、`tasks/auto_aim/visual_servo.cpp`
- `tests/relative_servo_test.cpp`
- `documentation/relative_can_protocol.md`、`documentation/handoff_relative_servo.md`

**仍留在工作区、不要提交**：

- ⚠️ `.vscode/c_cpp_properties.json` —— 本机 IntelliSense 改动，按 AGENTS §9.4 不提交；
  不想要了就 `git checkout -- .vscode/c_cpp_properties.json`（会丢本机 include 调整，介意就留着别 `git add`）


## 3. 今天已经跑过的验证（可直接复现）

```bash
# 1) 编译（exit 0）
make -C build/ relative_aim relative_servo_test -j$(nproc)

# 2) 纯函数 / 编码 / 流水线自测：22 checks, 0 failures（exit 0）
./build/relative_servo_test | tail -3

# 3) 离线 dry-run（不打开 CAN）
timeout -s INT 90 ./build/relative_aim -c=configs/offline.yaml -dry-run -n=60
#   结果：每帧都是 "[relative_aim] no command (tracker=lost, armors=0/1)"
#   原因见 §5（demo 视频 conf 抖动 + tracker 发散），不是伺服代码的问题
```

用临时调试图（**已删除**，不要再残留）确认过单帧 PnP 结果是对的（`xyz_in_gimbal`，云台系 F-L-U，单位 m）：

| 帧 | `xyz_in_gimbal` | 解析角差（默认 gain=1、deadband=0.1°） |
|---|---|---|
| 1 | (0.975, 0.035, 0.100) | Δyaw ≈ **+2.06°**，Δpitch ≈ **-5.86°** |
| 3 | (1.010, 0.013, 0.010) | Δyaw ≈ **+0.74°**，Δpitch ≈ **-0.57°** |
| 4 | (1.002, 0.024, 0.027) | Δyaw ≈ **+1.37°**，Δpitch ≈ **-1.54°** |
| 5 | (1.022, 0.024, 0.004) | Δyaw ≈ **+1.35°**，Δpitch ≈ **-0.22°** |

→ 直播 HUD 做完后，用同一段视频（`/tmp/relative_test.yaml`，见 §5）复看，这几行数字应该对得上。

## 4. 下次第一件事：把相对角差显示在直播页面上

> ⚠️ **2026-10-24：本节已作废（保留作方案记录）。** 实际实现没有改
> `tests/detect_freq_visual_test.cpp`，而是放在**专用入口** `src/relative_aim.cpp`：
> 同一份代码既算角差又发 CAN，HUD / MJPEG / CSV 都在里面，`detect_freq_visual_test`
> 保持「只测帧率与检测」的职责，`-dump` 的 CSV 表头也没动（`scripts/replay_ab.sh` 不受影响）。
>
> 现用命令：
> ```bash
> ./build/relative_aim -c=configs/relative_current.yaml -stream=8080   # HUD 直播
> ./build/relative_aim -c=configs/relative_current.yaml -csv=/tmp/r.csv
> ./build/relative_aim -c=configs/relative_current_replay.yaml -dry-run -n=150 -csv=/tmp/d.csv
> ```
> 下面 §4.1~§4.4 的设计要点（尤其 §4.3 的四条坑：`armors` 要拷贝、`use_enemy_color`、
> `set_R_gimbal2world` 顺序、别动 `-dump` 表头）在 `relative_aim.cpp` 里同样适用。

### 4.1 目标画面（`detect_freq_visual_test` 的 HUD 增加 2 行）

现有 HUD 已占 y = 30 / 60 / 90 / 120（`tests/detect_freq_visual_test.cpp:354-374`），
`NO YOLO` 在 y = 150（`:375-377`，只在没有模型时画，与下面的 servo 行互斥）。新增：

```
servo raw  dyaw +1.35  dpitch -0.22 deg  armor sentry/small  norm(0.61,0.59)
servo cmd  dyaw +1.35  dpitch -0.22 deg  control 1  tracker tracking  targets 1
```

- 第 1 行（**raw**）：直接由当前帧最靠近画面中心的装甲板的 `xyz_in_gimbal` 算角差，
  **不依赖 tracker 是否锁定**——这样即使 §5 的问题没修，HUD 上也能看到正确数字，
  立刻能判断 PnP / 坐标系 / 符号是否正确。
- 第 2 行（**cmd**）：`VisualServo::aim()` 真正会发出去的命令（含 gain / 死区 / 限幅 / 符号），
  `control 0` 时用黄色，`control 1` 时用绿色。

### 4.2 改动点（全部在 `tests/detect_freq_visual_test.cpp`，CMake 不用改）

`detect_freq_visual_test` 已经链接 `auto_aim` 库（`CMakeLists.txt:176-177`），
`visual_servo.cpp` 也在 `auto_aim` 里，所以只需加 include 和代码。

1. **include**（放在 `#include "tasks/auto_aim/yolos/yolov5_postprocess.hpp"` 附近）：

```cpp
#include "io/command.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/visual_servo.hpp"
```

2. **新增命令行键**（写进 `keys` 字符串，`:69-83`；默认关闭 = 原行为，CSV/AB 脚本完全不受影响）：

```cpp
"{servo          |                             | 叠加相对角差 HUD（solver+tracker，需 yolo）}"
```

3. **构造**（放在 `yolo` 构造完之后，`:263` 之后；变量名用 `visual_servo`，别用 `servo`
   以免和别的头冲突）：

```cpp
bool hud_servo = cli.has("servo") && yolo != nullptr;
std::unique_ptr<auto_aim::Solver> solver;
std::unique_ptr<auto_aim::Tracker> tracker;
std::unique_ptr<auto_aim::VisualServo> visual_servo;
if (hud_servo) {
  solver = std::make_unique<auto_aim::Solver>(config_path);
  // 相对角差只用 xyz_in_gimbal（与姿态无关），世界系统一设成单位阵，
  // 没有 IMU 也能跑 Solver/Tracker（与 src/relative_aim.cpp:72 一致）
  solver->set_R_gimbal2world(Eigen::Quaterniond::Identity());
  tracker = std::make_unique<auto_aim::Tracker>(config_path, *solver);
  visual_servo = std::make_unique<auto_aim::VisualServo>(config_path);
  tools::logger()->info("[HUD] 已启用相对角差叠加（servo raw + cmd）");
}
```

4. **每帧计算**（在 detect 之后、draw 之前，即 `:337` 与 `:339` 之间）：

```cpp
// tracker.track() 会**原地**删掉非敌方颜色的装甲板（tasks/auto_aim/tracker.cpp:43），
// 所以给它一份拷贝，画框用的 armors 保持原样（否则框会凭空消失）
auto track_armors = armors;
std::list<auto_aim::Target> targets;
io::Command servo_cmd{false, false, 0, 0};
if (tracker) {
  // 第三个参数 use_enemy_color=false：demo / 自录片段的装甲板颜色未必等于 yaml 里的
  // enemy_color，用默认 true 会永远锁不上（这正是 §5 的一个坑）
  targets = tracker->track(track_armors, timestamp, false);
  servo_cmd = visual_servo->aim(track_armors, targets);
}
```

5. **raw 角差**（同一处，纯计算、不产生命令）：

```cpp
const auto_aim::Armor * hud_armor = nullptr;
double hud_best = 1e10;
for (const auto & a : track_armors) {
  if (a.name == auto_aim::ArmorName::not_armor) continue;
  double d = std::hypot(a.center_norm.x - 0.5f, a.center_norm.y - 0.5f);
  if (d < hud_best) { hud_best = d; hud_armor = &a; }
}
double raw_yaw = 0, raw_pitch = 0;
bool raw_ok = hud_armor != nullptr && hud_armor->xyz_in_gimbal.allFinite() &&
              hud_armor->xyz_in_gimbal[0] > 0;
if (raw_ok) auto_aim::VisualServo::angles(hud_armor->xyz_in_gimbal, raw_yaw, raw_pitch);
```


6. **画到画布上**（在 `:374` 的 `armors ...` 那行之后；黄 = 只测不发，绿 = 有命令）：

```cpp
if (hud_servo) {
  if (raw_ok) {
    tools::draw_text(
      canvas,
      fmt::format(
        "servo raw  dyaw {:+.2f}  dpitch {:+.2f} deg  {} / {}  norm({:.2f},{:.2f})",
        raw_yaw * 57.3, raw_pitch * 57.3, auto_aim::ARMOR_NAMES[hud_armor->name],
        auto_aim::ARMOR_TYPES[hud_armor->type], hud_armor->center_norm.x,
        hud_armor->center_norm.y),
      {10, 150}, {0, 220, 255});
  } else {
    tools::draw_text(canvas, "servo raw  (no valid armor)", {10, 150}, {0, 220, 255});
  }
  cv::Scalar hud_color = servo_cmd.control ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 220, 255);
  tools::draw_text(
    canvas,
    fmt::format(
      "servo cmd  dyaw {:+.2f}  dpitch {:+.2f} deg  control {}  tracker {}  targets {}",
      servo_cmd.yaw * 57.3, servo_cmd.pitch * 57.3, servo_cmd.control ? 1 : 0,
      tracker ? tracker->state() : "-", targets.size()),
    {10, 180}, hud_color);
}
```

7. **日志也打一行**（`frame % interval == 0` 那段，`:451-456`），方便 SSH 下看：

```cpp
if (hud_servo) {
  tools::logger()->info(
    "servo raw dyaw {:+.2f} dpitch {:+.2f} deg | cmd {:+.2f}/{:+.2f} control {} | tracker {} "
    "targets {}",
    raw_yaw * 57.3, raw_pitch * 57.3, servo_cmd.yaw * 57.3, servo_cmd.pitch * 57.3,
    servo_cmd.control ? 1 : 0, tracker->state(), targets.size());
}
```

### 4.3 四个必须注意的点

1. **`armors` 要拷贝一份给 tracker**：`Tracker::track()` 对传入的 `std::list<Armor> &` 做
   `remove_if`（非敌方颜色直接删，`tracker.cpp:43`），共用一份会让画面上的框随机消失。
2. **`track(..., use_enemy_color=false)`**：`tracker.hpp:22-25` 第三个参数默认 `true`，
   demo 视频 / 自录片段的颜色标签未必匹配 `enemy_color`，用默认会「永远 lost」。
3. **`set_R_gimbal2world(Identity)` 要在第一次 `solve()` 之前**：`Solver` 构造里已经是单位阵，
   但显式设一次更明确（`src/relative_aim.cpp:72` 就是这么写的）。
4. **不要动 `-dump` 的 CSV 表头**（`:280`）：`scripts/csv_ab.py` 用 `csv.DictReader` 按列名读，
   改表头会打断 `scripts/replay_ab.sh` 的 A/B。真要导角差就另开 `-dump-servo=<csv>`。

### 4.4 验证命令（照这个顺序做）

```bash
make -C build/ detect_freq_visual_test -j$(nproc)

# a) 无相机、无显示：回放 demo，存图看 HUD（bench 默认不开窗，但 -save 会强制走 draw）
sed 's/^min_confidence: 0.8/min_confidence: 0.6/; s/^min_detect_count: 5/min_detect_count: 3/' \
  configs/offline.yaml > /tmp/relative_test.yaml
./build/detect_freq_visual_test -c=/tmp/relative_test.yaml -m=bench -n=60 -servo \
  -save=/tmp/servo.png -dump=/tmp/servo.csv
#   期望：/tmp/servo.png 上第 5 行 "servo raw dyaw ..." 有数字，
#         前几帧与 §3 的表一致（第 1 帧 ≈ +2.06 / -5.86 deg）；
#         第 6 行 servo cmd 的 control 很可能还是 0（§5 的 tracker 问题）

# b) 浏览器看直播（本次的真正目标）
#    注意 replay/bench 下 -n=0 会被当成 300 帧（`:218`），要一直看就用 -m=live，Ctrl+C 退出
./build/detect_freq_visual_test -c=configs/detect_freq.yaml -m=live -servo -stream=8080
#   本机浏览器：http://127.0.0.1:8080/        脚本自检：
curl -s -o /tmp/snap.jpg http://127.0.0.1:8080/snapshot.jpg
#   画面 = 检测框 + 4 行原有指标 + 2 行 servo（与 imshow 完全一样）

# c) 真机相机（有相机再看；千万别用 timeout/kill 强杀，见 AGENTS §8.7）
./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=live -servo -stream=8080
```

验收标准：raw 行数字随目标在画面里的位置变化，目标在光轴上时收敛到 0 ± 死区；
符号约定「左转为 +Δyaw、向上为 -Δpitch」（与 `CBoard` / `Aimer` 一致）。


## 5. 已知问题：离线 demo 回放时 tracker 锁不上（挡住 e2e 验证）

> ✅ **2026-10-24：已绕过。** `-source=current` 不经过 Tracker，直接用本帧检出做 PnP，
> 回放 demo 视频时首帧就能 `control=1`（150 帧里 124 帧有效）。本节的根因分析仍有效：
> 想用 `-source=tracker` 那条路（EKF + 预测）就必须先解决 `use_enemy_color=true`
> 把 demo 里的蓝甲板全过滤掉的问题，否则 tracker 恒 `lost`。

**现象**（可复现）：

```bash
timeout -s INT 90 ./build/relative_aim -c=configs/offline.yaml -dry-run -n=60
#   → 每帧 "[relative_aim] no command (tracker=lost, armors=1)"，没有一次 control=1
```

诊断（今天用临时日志确认，**日志已删**，需要时按下面重加）：

- 检测本身没问题：`armors` 在 0 / 1 之间跳（`configs/offline.yaml` 是
  `min_confidence: 0.8`、`min_detect_count: 5`），单帧 PnP 结果正常（§3 表）。
- 用派生配置放宽阈值后能进 `tracking`，但下一帧又回到 `lost`：

```bash
sed 's/^min_confidence: 0.8/min_confidence: 0.6/; s/^min_detect_count: 5/min_detect_count: 3/' \
  configs/offline.yaml > /tmp/relative_test.yaml
```

- 日志里持续刷 `[Target] r=nan, l=nan`（来自 `tasks/auto_aim/target.cpp:243-252` 的 `diverged()`，
  logger 级别是 debug，所以能看到）。`r` 应该是 0.2 这样的半径值，出现 `nan` 说明 EKF 状态已经发散。

**初步推断（未确证，属既有算法，按 AGENTS §5 不要顺手改）**：

- `Target::update()`（`target.cpp`）里 `int id;` **未初始化**；只有当
  `|angle_error| < |min_angle_error|` 成立时才赋值。若 `armor_xyza_list()` 里有 NaN，
  比较永远为假 → `id` 保持垃圾值 → 用垃圾 id 求 `h_armor_xyz()` → 状态变 NaN → 之后每帧都发散。
- NaN 的最初来源还没定位（可能与 demo 画面 conf/关键点抖动、单目 IPPE 退化有关）。

**下次排查顺序**（先只读诊断，别改算法）：

1. 在 `Target::update()` 里临时打印 `ekf_.x.transpose()`、`angle_error`、`id`（用完删掉）。
2. 确认是「首帧就 NaN」还是「第 2~3 帧更新后变 NaN」。
3. 如果确认是 `int id;` 未初始化触发，再讨论修法（`int id = 0;` 是最小改动，但**必须先确认**
   ——这是 tracker 算法文件，按 §5 要给出影响面再改）。

**规避方式（不影响做 §4）**：§4 的 raw 行不依赖 tracker，直播页面上照样能看到角差收敛；
真正闭环的数值验证建议放到真机 / 自录稳定片段上做（`configs/mv_sua133gc.yaml` +
`record_video: true` 录一段，再 `camera_name: "video"` 回放）。

## 6. 真机联调 checklist（等 §4 做完再上）

1. **配置**：在用的那份 config 里加（缺键默认值见协议文档 §4）：
   `can_command_mode: "relative"`、`relative_canid: 0xFE`；
   板上不发 `0x100` 四元数时再加 `cboard_wait_imu: false`（否则 `CBoard` 构造会阻塞）。
2. **CAN 起来**：`sudo ip link set can0 type can bitrate 1000000 restart-ms 100 && sudo ip link set up can0`
3. **抓包确认**：`candump -tz can0,0FE:7FF` —— 每帧 DLC 8，byte0 bit0=control、bit1=shoot=0，
   byte2-3 = Δyaw int16 大端（×1e4 rad），byte4-5 = Δpitch int16 大端；`control=0` 时角差必须是 0。
4. **慢闭环**：先 `servo_gain: 0.2`、`servo_max_step_deg: 3`，用手把云台偏一点，看它是否朝目标转；
   方向反了就改 `servo_yaw_sign` / `servo_pitch_sign`（**只在一处反号**，不要在固件里再反一次）。
5. **判据**：目标在画面中心时 Δyaw/Δpitch 收敛到 ±死区；不振荡（振荡先降 gain）、
   不来回甩（甩 → 固件重复累加，检查固件是否按「收到帧」累加而不是按定时器）。
6. **安全**：`relative_aim` 在 `can_command_mode != relative` 时会直接拒绝启动（不会把相对角当绝对角发出去）。

## 7. 固件侧还没做

协议帧格式、参考 C 实现、RX 超时、`control=0` 必须停止累加、不能与 `0x100/0x101/0xff`
冲突等等，全都写在 `documentation/relative_can_protocol.md`（§3 帧格式、§5 验证步骤、§6 调参顺序、§7 局限）。
**固件没合入之前不要开闭环**（否则视觉侧发的 `0x0FE` 会被当成未知帧或误触发）。

## 8. 之后提交时的 checklist（AGENTS §7 / §9）

第一次提交已完成（`44d15df`，已推送）。下面是**后续**（§4 的 HUD / §5 的排查）提交时照抄：

- [x] `make -C build/ relative_aim relative_servo_test -j$(nproc)` 通过（`detect_freq_visual_test` 改完后再加进这条）
- [x] `./build/relative_servo_test` → 22 checks, 0 failures
- [x] 没有残留临时调试打印（今天那段 `[dbg]` 已删）
- [x] `git push origin main` 后 `origin/main` == 本地 HEAD（`44d15df`）
- [ ] 改 `tests/detect_freq_visual_test.cpp` 后：`clang-format -i`，`awk 'length > 100'` 查行宽
- [ ] 无 Eigen 3.4 API（`Eigen::last` / `seq` / `reshaped` …）、无 `{{}}` 矩阵初始化、无 fmt 直接格式化 chrono
- [ ] `git add` **指定路径**，**不要** `git add .vscode/` / `build/` / `logs/` / `third_party/` / `*.engine`
- [ ] commit 首行 `<type>(<scope>): <中文摘要>`，正文写实测命令与结果（数字必须是真跑出来的）
- [ ] push 后核对 `git --no-pager log --oneline -1 origin/main` 与本地 HEAD SHA 一致

后续如果改动跨多个模块，按下面拆（可审阅性更好）：
1. `feat(test): detect_freq_visual_test 叠加相对角差 HUD`（只动 `tests/detect_freq_visual_test.cpp`）
2. `fix(auto_aim): 修 tracker 在 demo 上的 NaN 发散`（确证后单独一次，只动 `tasks/auto_aim/target.*`）
3. `feat(auto_aim): relative_aim 接入 <真机配置>`（如需要）


## 9. 速查

| 事情 | 命令 / 位置 |
|---|---|
| 协议与固件参考实现 | `documentation/relative_can_protocol.md` |
| 角差纯函数 | `tasks/auto_aim/visual_servo.cpp:36-47`（`angles`）、`:48-66`（`shape`）、`:11-25`（`load_servo_limits`，两条路径共用） |
| 选目标 / 首帧不发（tracker 路径） | `tasks/auto_aim/visual_servo.cpp:67-107` |
| 当前帧观测（选目标 / PnP / 参考系 / 时效 / 量化） | `tasks/auto_aim/current_relative.cpp`（`solve_current_angles` `:120-206`、`CurrentRelativeEstimator::update` `:298` 起） |
| CAN 相对帧编码 | `io/cboard.cpp:91-110`（`encode_relative_command`）、`:112-140`（`send` 分支） |
| 量化唯一实现 | `io/command.hpp`（`io::quantize_angle_1e4`） |
| 配置键读取 | `io/cboard.cpp:196-230`、`tasks/auto_aim/visual_servo.cpp:11-25`、`tasks/auto_aim/current_relative.cpp:207-290` |
| 单测 | `./build/relative_servo_test` |
| 端到端 dry-run | `./build/relative_aim -c=<cfg> -dry-run -n=60` |
| 直播页面 | `./build/relative_aim -c=configs/relative_current.yaml -stream=8080`（HUD 自带头） |
| 逐帧 CSV | `./build/relative_aim -c=<cfg> -csv=/tmp/r.csv`（列定义见 `src/relative_aim.cpp`） |
| 验收证据（线材/帧率/dry-run/vcan） | `documentation/relative_acceptance.md` |
| 本机注意 | 短选项必须 `-c=`；`timeout -s INT`；`build/` 别删（`/home/irm/sp_vision_25` 是它的旧路径符号链接） |

