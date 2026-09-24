# AGENTS.md — sp_vision_25 开发约束（Jetson / aarch64）

> 本文件是本仓库所有 agent 约束的**唯一正文**。`.clinerules/` 内只有一行引用，请勿在其中重复或修改规则。
> 适用范围：本仓库内所有代码生成、修改、重构与编译验证。
> 版本：v1，依据工作区当前 Jetson 移植状态实测（Eigen 3.3.7 / fmt 6.1.2 / spdlog 1.5.0 / aarch64）。

## 0. 硬性禁止项

违反任一条即视为无效输出，必须回退重做：

| # | 禁止 | 正确做法 |
|---|---|---|
| 0.1 | Eigen 3.4+ 专属 API：`Eigen::last`、`Eigen::placeholders`、`Eigen::seq`、`Eigen::seqN`、`Eigen::all`、`.reshaped()`、`.slice()` | 见 §2.2：`v[v.size() - 1]`、`A.block(...)`、`A.col(i)`、`A.row(i)`、`A.head(n)` |
| 0.2 | 矩阵/向量双层花括号列表初始化：`Eigen::Matrix3d R{{a,b,c},{d,e,f},{g,h,i}};` | 见 §2.1：先声明尺寸，再逗号流式赋值 `R << ...` |
| 0.3 | 用 fmt 直接格式化 `std::chrono` 时间对象：`fmt::format("{:%Y-%m-%d}", std::chrono::system_clock::now())` | 见 §3.1：`strftime` 格式化到 `char[]`，再交给 `fmt::format` |
| 0.4 | 引入或恢复 OpenVINO 运行时依赖：`ov::`、`core_.read_model()`、`compile_model()`、`find_package(OpenVINO REQUIRED)` | 见 §4：推理后端只能是 CUDA / TensorRT |
| 0.5 | 为绕过编译错误而重写 `Armor` / `Solver` / EKF / TinyMPC 的上层算法逻辑 | 只做 API/类型适配，排错顺序见 §6.3 |

## 1. 环境与版本基线

### 1.1 目标平台
- 平台：NVIDIA Jetson，ARM64 / aarch64（本机 `uname -m` = `aarch64`，非交叉编译）
- 系统：Ubuntu 20.04，JetPack 5.1.3
- 语言：C++17，`CMAKE_BUILD_TYPE=Release`
- 编译器：`/usr/bin/aarch64-linux-gnu-g++`

### 1.2 依赖版本（实测，改代码前先按此判断 API 可用性）

| 组件 | 版本 | 关键影响 |
|---|---|---|
| Eigen | **3.3.7** | 无 3.4 切片语法；无 `{{}}` 初始化 |
| fmt | **6.1.2** | 无 chrono formatter |
| spdlog | **1.5.0** | 无新版 API，`{}` 不能直接打印 chrono / Eigen 类型 |
| OpenCV | 4.x (aarch64) | — |
| CMake | 3.16.3，Unix Makefiles | `build/` 已配置好 |
| OpenVINO | **不存在** | 禁止依赖 |
| CUDA | **11.4**（`/usr/local/cuda-11.4`，V11.4.315） | 唯一允许的推理后端底层 |
| cuDNN | **8.6.0**（`libcudnn8 8.6.0.166-1+cuda11.4`） | — |
| TensorRT | **未安装**（无 `NvInfer.h` / `libnvinfer*` / `trtexec`） | 迁移前必须先装，见 §4.0 |

自检命令（需要时先核对版本，再决定写法）：
```bash
grep -E 'define EIGEN_(WORLD|MAJOR|MINOR)_VERSION' /usr/include/eigen3/Eigen/src/Core/util/Macros.h
grep -m1 'FMT_VERSION ' /usr/include/fmt/core.h
grep -m3 'SPDLOG_VER' /usr/include/spdlog/version.h
cat /proc/device-tree/model                       # Jetson 型号
/usr/local/cuda/bin/nvcc --version                # CUDA 版本
dpkg -l | grep -Ei 'cudnn|nvinfer'                # cuDNN / TensorRT
ls /usr/include/aarch64-linux-gnu/NvInfer.h 2>/dev/null || echo 'NO TensorRT headers'
```

### 1.3 禁止“升级依赖来解决问题”
不得通过升级系统库或引入 vcpkg / conan / 源码编译新版 Eigen、fmt、spdlog 来绕开版本限制；必须在现有 API 能力内改写代码。

## 2. Eigen 3.3.7 写法规范

### 2.1 初始化：先声明尺寸，再逗号流式赋值
```cpp
// 固定尺寸
Eigen::Matrix3d R_armor2world;
R_armor2world << cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch,
                 sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch,
                        -sin_pitch,        0,          cos_pitch;

// 动态尺寸
Eigen::VectorXd x0(11);
x0 << center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0;

Eigen::MatrixXd F(11, 11);
F << 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0,  1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0,  0, 1, dt, 0, 0, 0, 0, 0, 0, 0;
```
- 允许用 `// clang-format off/on` 保持矩阵对齐（仓库既有风格）。
- 若必须写成单表达式，只能 `Eigen::Matrix3d R = (Eigen::Matrix3d() << a, b, c, d, e, f, g, h, i).finished();`，**绝不**用 `{{}}`。

### 2.2 索引与切片
- 末元素：`v[v.size() - 1]`（**不是** `v[Eigen::last]`）
- 子块：`A.block<3,3>(0,0)`、`A.block(i, j, rows, cols)`、`A.col(i)`、`A.row(i)`、`A.head(n)`、`A.tail(n)`
- 逐元素：`A.array() * B.array()`；矩阵乘保持 `A * B`

### 2.3 其它
- 不要直接把 `Eigen::MatrixXd` / `VectorXd` 传给 `fmt::format` / `spdlog`，先转 `std::string` 或逐元素打印。
- 保留 `.vscode/eigen_fix.h`（IntelliSense 下 `#undef __ARM_NEON`）；该文件被 `.gitignore` 忽略，**不要提交它，也不要删除**。

## 3. 时间、日志（fmt v6 / spdlog 1.5）

### 3.1 时间戳统一用 strftime
```cpp
#include <ctime>
#include <chrono>

auto now = std::chrono::system_clock::now();
std::time_t now_c = std::chrono::system_clock::to_time_t(now);
std::tm now_tm = *std::localtime(&now_c);
char time_buf[64];
std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d_%H-%M-%S", &now_tm);

auto path = fmt::format("logs/{}.log", time_buf);
```
`tools/logger.cpp`、`tools/recorder.cpp` 已是此写法，新增/修改代码沿用同一模式。

### 3.2 日志
- 业务代码（`tasks/`、`io/`、`src/`、`tools/`）的新增或修改处，只通过 `tools::logger()`（spdlog）输出，不得引入新的 `std::cout` / `printf` 调试输出；`tests/`、`calibration/` 是可交互调试程序，允许保留控制台输出。
- spdlog 1.5 不能直接格式化 chrono / Eigen / 自定义类型（无对应 formatter 会编译失败），先转 `double` / `std::string`。
- 浮点显式指定精度：`fmt::format("yaw={:.3f}", yaw)`。

## 4. 神经网络推理：CUDA / TensorRT only

### 4.0 前置条件：本机 TensorRT 尚未安装
实测：CUDA 11.4.315 + cuDNN 8.6.0 可用，但**没有 TensorRT**——`find /usr /opt -name 'NvInfer*' -o -name 'libnvinfer*' -o -name 'trtexec'` 无结果，`dpkg -l | grep -i nvinfer` 为空。
开始迁移前必须先补齐（JetPack 5.1.3 对应 TensorRT 8.5.x）：
```bash
sudo apt update && sudo apt install -y tensorrt libnvinfer-dev libnvinfer-plugin-dev
# 包名以 `apt-cache search nvinfer` 结果为准；也可用 SDK Manager 单独安装
ls /usr/include/aarch64-linux-gnu/NvInfer.h && trtexec --version   # 两个都要有输出
```
若 TensorRT 装不上，**不要**擅自退回 OpenVINO：把该阻塞点明确报告给用户，等待指示。
`.engine` / `.plan` 由 `trtexec` 或 TensorRT API 离线生成，**不提交进仓库**。

### 4.1 迁移要求
- 本机**没有任何 OpenVINO 运行时**，所有推理必须面向 CUDA / TensorRT（TensorRT `.engine` / `.plan`，或自写 CUDA kernel）。
- **待迁移清单**（现有 OpenVINO 代码，不要新增同类代码）：
  - `tasks/auto_aim/yolos/yolov5.{cpp,hpp}`
  - `tasks/auto_aim/yolos/yolov8.{cpp,hpp}`
  - `tasks/auto_aim/yolos/yolo11.{cpp,hpp}`
  - `tasks/auto_aim/classifier.{cpp,hpp}`
  - `tasks/auto_aim/multithread/mt_detector.{cpp,hpp}`
  - `tasks/auto_buff/yolo11_buff.{cpp,hpp}`
  - 上述文件中的 `ov::Core`、`ov::CompiledModel`、`core_.read_model()`、`compile_model()` 需替换为 TensorRT 封装。
  - 现状：`tasks/auto_aim/CMakeLists.txt` 已把 `classifier.cpp`、`detector.cpp`、`yolo.cpp`、`yolos/*.cpp`、`multithread/mt_detector.cpp` 注释掉（未参与编译）；但 `tasks/auto_buff/CMakeLists.txt` **仍在编译 `yolo11_buff.cpp`**，且 `buff_detector.hpp` / `buff_target.hpp` / `buff_aimer.hpp` 都直接 `#include "yolo11_buff.hpp"`，因此 `auto_buff` 目标当前**编译失败**——这是预期状态，不是回归。迁移时需同步放开上述源文件并接上 TensorRT；在此之前改动这些文件不会有完整编译反馈。
- 迁移硬约束：
  1. **对外接口与类名不变**：`auto_aim::YOLO` / `YOLOV5` / `YOLOV8` / `YOLO11` 的构造签名
     `(const std::string & config_path, bool debug = false)` 与 `detect(...)` 返回类型保持不变。
  2. **YAML 键保持兼容**：`configs/*.yaml` 已有键不得重命名或删除；新增引擎路径/精度相关键使用新键名并提供默认值。
  3. **CMake**：不恢复 `find_package(OpenVINO REQUIRED)`；顶层保留空的 `INTERFACE` 目标 `openvino::runtime` 作为过渡兜底，只有全部链接项清理完毕才可移除。TensorRT 链接装好后的 `nvinfer` / `nvinfer_plugin`（aarch64 库路径 `/usr/lib/aarch64-linux-gnu`）+ CUDA 的 `cudart`（`/usr/local/cuda-11.4/lib64`），安装前提见 §4.0。
  4. **精度与验证**：默认 FP16（必要时 INT8 + calibration）；合入前必须给出与参考实现的一致性对比（检测框 / 关键点误差可接受），不得只凭“能跑通”合入。
  5. **复用预处理/后处理**：letterbox、`cv::resize` 到输入、NMS、关键点解码逻辑尽量沿用现状，只替换推理调用点。
- 不把 `.engine` / `.plan` 大文件提交进 git（必要时先说明再改 `.gitignore`）。

## 5. 不可改动的架构约定

| 模块 | 文件 | 约束 |
|---|---|---|
| Armor / Lightbar | `tasks/auto_aim/armor.hpp` | 字段（含 `xyz_in_gimbal` / `xyz_in_world` / `ypr_*` / `ypd_in_world` / `yaw_raw`）与全部构造函数签名不变 |
| Solver 位姿解算 | `tasks/auto_aim/solver.{hpp,cpp}` | `solve()`、`optimize_yaw()`、`reproject_armor()`、`oupost_reprojection_error()`、`SJTU_cost()` 的签名与解算流程不变 |
| EKF | `tools/extended_kalman_filter.{hpp,cpp}` | 状态量含义与协方差语义不变 |
| 目标跟踪 | `tasks/auto_aim/target.*`、`tracker.*` | 11 维状态量与 EKF 更新顺序不变 |
| TinyMPC 规划器 | `tasks/auto_aim/planner/**`（含 `tinympc/`） | 仅允许数值/类型/内存布局适配；不得改动代价函数、约束与 ADMM 迭代结构 |

确需改动上述签名时，必须先给出「为什么必须改 + 调用方影响面清单 + 替代方案」并得到确认。

## 6. 编译与调试流程

### 6.1 增量编译（首选）
```bash
make -C build/ <target> -j$(nproc)
```
`build/` 已配置的目标（ROS 未启用时约 80 个）。按实测（v1.1，在 Jetson 上以 `make -C build/ -k` 全量验证）分成两类：

- **实测编译通过**：可执行 `planner_test`、`planner_test_offline`、`camera_test`、`usbcamera_test`、`multi_usbcamera_test`、`gimbal_test`、`gimbal_response_test`、`dm_test`、`fire_test`、`handeye_test`、`cboard_test`、`calibrate_camera`、`calibrate_handeye`、`calibrate_robotworld_handeye`、`capture`、`split_video`；库目标 `auto_aim`、`omniperception`、`tools`、`io`、`tinympcstatic`、`serial`。
- **当前编译失败（依赖推理后端，属预期状态，见 §4）**：`uav`、`uav_debug`、`minimum_vision_system`、`auto_buff`、`auto_buff_test`、`auto_buff_debug_mpc`、`auto_aim_test`、`camera_detect_test`、`camera_thread_test`、`usbcamera_detect_test`、`detector_video_test`。原因只有两类：缺 `<openvino/openvino.hpp>`（来自 `classifier.hpp` / `yolos/*.hpp` / `multithread/mt_detector.hpp` / `auto_buff/yolo11_buff.hpp`），或 `auto_aim::YOLO` 符号未参与编译。**不要**为了让它们“编过”而注释逻辑或加临时桩；按 §4.1 完成 TensorRT 迁移后再放开。

- **不要**默认执行 `make -C build/` 全量编译；先编译受影响的最小 target。
- 仅在必要时 `cmake -B build`；**不要删除 `build/`**（重新全量编译成本高）。
- `sentry*`、`publish_test` 等目标只在 ROS2 环境存在时生成，本机通常没有，不要当作必须编译的对象。

### 6.2 验证要求
- 改完 C++ 代码必须至少编译通过受影响的 target，并在回复中给出**真实执行过的命令与结果**。
- 若无法编译验证（如缺 `.engine` 文件），必须显式说明“未编译验证”及原因。

### 6.3 排错顺序（禁止乱猜乱改）
1. 读报错原文，先定位**头文件**（缺失 `#include`、路径大小写、`Eigen/Dense` 与 `opencv2/core/eigen.hpp` 的包含顺序）
2. 核对**函数签名**（参数类型、`const`、返回类型、默认实参）
3. 核对**宏定义**（`#define` 污染、版本宏、条件编译分支）
4. `git --no-pager diff <file>` 判断该文件是否已有移植改动，避免覆盖他人工作
5. `grep -rn "<符号>" --include=*.hpp --include=*.cpp .` 定位定义与全部调用点
6. 确认只是语法/类型问题时才动手；**不得**为“编过”删除功能、注释掉逻辑或放宽约束

### 6.4 编辑纪律
- 禁止 `sed -i` / 脚本批量替换算法文件；使用精确的定点编辑。
- 每次改动尽量聚焦单一问题，保持 diff 可审阅。
- 风格遵循 `.clang-format`（Google 基础，`ColumnLimit: 100`，`PointerAlignment: Middle`，2 空格缩进，`BreakBeforeBraces: Custom`）；改完可 `clang-format -i <file>`。

## 7. 交付前自检清单
- [ ] 无 Eigen 3.4+ API（§0.1）
- [ ] 无 `{{...}}` 初始化，矩阵/向量均为“先声明尺寸 + `<<`”（§0.2、§2.1）
- [ ] 无 fmt 直接格式化 chrono，时间戳走 `strftime`（§0.3、§3.1）
- [ ] 无新增/恢复 OpenVINO 依赖，推理走 CUDA / TensorRT（§0.4、§4）

## 8. 笔记本 / x86_64 开发机（Jetson 不在手上时）

目标：在没有 Jetson 的情况下也能编译、跑离线回放与调试。

### 8.1 平台前提（硬限制）
- **只支持 x86_64 Linux**（原生 Ubuntu 或 WSL2）。不支持 macOS / Windows 原生：
  - `io/CMakeLists.txt` 对非 `x86_64` / `aarch64` 直接 `message(FATAL_ERROR)`；
  - `io/socketcan.hpp` 依赖 `<linux/can.h>` / `<sys/epoll.h>`（`io/cboard.cpp` 链路）；
  - MindVision / HikRobot SDK 只提供 Linux `.so`（仓库内已有 `lib/amd64` 与 `lib/arm64`）。
- **WSL2 补充**（Windows + WSL2 是最常见的笔记本形态）：
  - 仓库**必须 clone 到 WSL 的 ext4 文件系统**（如 `~/sp_vision_25`），不要放 `/mnt/c/...`：跨 9p 文件系统编译会慢一个数量级，且文件事件/权限行为异常。
  - USB 设备（工业相机 / UVC 相机 / 达妙 IMU 串口 / USB2CAN）不会自动出现在 WSL 里，需要 Windows 侧 `usbipd-win` + `usbipd attach`；不接硬件时走 §8.4 的回放模式。
  - GUI（`cv::imshow`）需要 WSLg（Win11 或已更新的 Win10）；没有 WSLg 就用 `ssh -X` / VcXsrv，或干脆不加 `-d display`。
  - WSL 里没有 `can0` 硬件，但 `io::CBoard` 打开失败只 `logger()->warn`（`io/socketcan.hpp` 的 `try_open`），不会退出，不影响纯视觉链路。
- 不要在机器之间复用 `build/`：`CMakeCache.txt` 记录了编译器（`/usr/bin/aarch64-linux-gnu-g++`）与库路径（`/usr/lib/aarch64-linux-gnu/...`）。每台机器各自 `cmake -B build`。
- 笔记本上 Eigen 3.4 / fmt 8+ / spdlog 1.9+ 都能编过（本仓库代码按 3.3.7 / fmt 6 兼容写法编写，向上兼容），但**基线仍是 Jetson 上的 Eigen 3.3.7 + fmt 6.1.2 + spdlog 1.5**，新增代码必须两边都能编。

### 8.2 环境准备与构建
```bash
bash scripts/setup_x86_dev.sh -y     # 装 apt 依赖 + 生成 .vscode/eigen_fix.h + configure + 编译可移植子集
```
该脚本会：
1. `apt install` OpenCV / fmt / Eigen / spdlog / yaml-cpp / nlohmann-json / libusb-1.0 / Ceres（Ceres 是 `tasks/auto_buff/CMakeLists.txt` 的 `find_package(Ceres REQUIRED)` 必需项）；
2. 本地生成 `.vscode/eigen_fix.h`（`.vscode/` 被 `.gitignore` 忽略，且 §2.3 要求不提交，所以每台机器都要生成）；
3. 编译「不依赖推理后端」的目标子集。

### 8.3 当前可移植目标子集
- 库：`serial`、`tools`、`io`、`auto_aim`、`tinympcstatic`、`omniperception`
- 可执行：`planner_test`、`planner_test_offline`、`camera_test`、`usbcamera_test`、`multi_usbcamera_test`、`cboard_test`、`dm_test`、`fire_test`、`gimbal_test`、`gimbal_response_test`、`handeye_test`、`capture`、`split_video`、`calibrate_camera`、`calibrate_handeye`、`calibrate_robotworld_handeye`
- 依赖推理后端、当前无法编译的目标：见 §6.1（预期状态）。
- CI：`.github/workflows/build-x86.yml` 在 ubuntu-22.04 上编译上述子集，并冒烟运行 `planner_test_offline configs/demo.yaml`（注意 `fire_thresh` 等键只有 `configs/demo.yaml` / `standard3.yaml` / `standard4.yaml` 有，其余 config 会因缺键 `exit(1)`）。

### 8.4 离线回放（已实现，x86_64 / WSL2 可用）

用**新增键**替代真实相机与 IMU，已有 `configs/*.yaml` 的键一个都没有改：

| 新增键 | 取值 | 说明 |
|---|---|---|
| `camera_name` | `"video"` | 新增的第 3 个相机后端：从视频文件取流 |
| `video_path` | `assets/demo/demo.avi` | 视频文件路径 |
| `video_loop` | `true` / `false` | 播到结尾是否回卷，默认 `true` |
| `video_frame_rate` | `0` | 取流帧率，`0` = 用视频文件自带帧率（默认 `0`），非法/取不到时兜底 30 |
| `imu_name` | `"none"` / `"/dev/ttyACM0"` | `"none"` = 无 IMU 回放模式，`imu_at()` 恒返回单位四元数（默认 `/dev/ttyACM0`） |

- 现成配置：`configs/offline.yaml`（= `configs/demo.yaml` + 上述 5 个键）。
- 实测（Jetson 上跑 x86 同一份代码）：`./build/camera_test -c=configs/offline.yaml` → **平均 29.97 fps**（目标 30，179 帧/6s）；`./build/dm_test -p=none` → `z0.00 y0.00 x0.00`；快放到结尾会正常回卷（`reached the end, rewinding`）不崩溃。注意 `assets/demo/demo.avi` 无索引（`CAP_PROP_FRAME_COUNT = 0`），回卷实现是 release + reopen，不能用 `set(CAP_PROP_POS_FRAMES, 0)`。
- ⚠️ **参数写法**：本仓库程序用 `cv::CommandLineParser`，短选项必须写 `-c=<path>` / `--config-path=<path>`；写成 `-c <path>`（空格分隔）会被解析成空值，报 `[YAML] Failed to load file: bad file`。
- `io::USBCamera::open()` 仍强制 `"/dev/" + name` + `cv::CAP_V4L`，**不能**读 `.avi`；读视频请走 `io::Camera` + `camera_name: "video"`。

### 8.5 仍然缺的降级（尚未实现）
- `io::Gimbal`：打不开 `/dev/gimbal` 仍会 `exit(1)`（`io/gimbal/gimbal.cpp`）→ `planner_test` / `fire_test` / `gimbal_test` 在笔记本上跑不了（`planner_test_offline` 不受影响）。
- `uav` / `uav_debug` / `minimum_vision_system` 已接入上述回放键，但在 §4 的 TensorRT 迁移完成前**无法编译**，所以笔记本上的端到端回放要等迁移完成。

- [ ] `Armor` / `Solver` / EKF / TinyMPC 签名与算法逻辑未变（§5）
- [ ] `make -C build/ <target> -j$(nproc)` 实际编译通过（§6.1、§6.2）
