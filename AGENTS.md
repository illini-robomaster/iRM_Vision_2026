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

### 4.2 装甲板检测模型：来源与输出格式（实测确认）

- 本仓库配套模型 = **RobotPilots P24-DetectionModel**（魔改 YOLOv5 + MobileNetV3 backbone）。
  已在 Jetson 上解包 `assets/yolov5.xml` 确认结构吻合：
  `HSwish×17 + HSigmoid×9 + GroupConvolution×11`（MobileNetV3 特征算子）、`Swish×24`、单输入
  `1×3×640×640`(FP32)、单输出 `Concat("output") = [1, 25200, 22]`（25200 = 80²+40²+20²）。
- 22 列含义与 `tasks/auto_aim/yolos/yolov5.cpp::parse()` 一一对应：
  - `col 0..7`：4 个关键点（代码按 0→1→2→3 映射为 左上/右下/右上/左下）
  - `col 8`：置信度 —— **模型输出 raw logits，代码里再 `sigmoid()`**（若把 sigmoid 固化进 ONNX 会二次 sigmoid）
  - `col 9..12`：颜色（红 / 蓝 / 灰 / 紫）；`col 13..21`：编号 9 类（G / 1 / 2 / 3 / 4 / 5 / O / Bs / Bb）
  - 说明：`parse()` 只做 `argmax`，不额外 softmax，因此颜色/编号列保持原始 logits 即可。
- **ONNX 权重已就位**：`assets/yolov5_0526.onnx`（4,359,932 B，md5 `45b111e2ddde9e0b588fef5a38917af4`；
  来源 `https://github.com/broalantaps/RobotDetectionModel` 的 `Model/0526.onnx`，pytorch 2.0.1 导出，
  输入名 `images`，输入 `1×3×640×640`）。已用 OpenCV DNN 实测确认：
  - 输出 `(1, 25200, 22)`，与仓库 IR 一致；
  - `col 8` 是 **raw logits**（实测 max `+3.29` → sigmoid `0.964`，min `-51.96` → `0.000`），
    所以 **必须保留 `parse()` 里的 sigmoid，不要把 sigmoid/softmax 固化进转换后的图**；
  - 官方参考 `OpenvinoInfer.cpp` 的 landmarks→points 映射 `(0,1)(6,7)(4,5)(2,3)` 与
    `tasks/auto_aim/yolos/yolov5.cpp::parse()` **完全一致**（连点序都是同一个约定）；
  - `assets/demo/demo.avi` 抽样 16 帧，15 帧在 conf ≥ 0.65 有检出（best 0.90~0.96）。
  `.engine` 由该 ONNX 离线生成，不入库（`.gitignore` 已忽略）。模型来自第三方（RobotPilots），
  **仓库保持 private**。
- 其它权重：`assets/yolo11.xml` 是 INT8 量化版（含 `FakeQuantize×110`）；`assets/best2-sim.onnx` 是
  打符用 YOLOv8n-pose（`names={0:'b'}`、`kpt_shape=[5,2]`、5 个扇叶关键点）；`assets/tiny_resnet.onnx`
  是装甲板数字分类器。新增/替换权重时不要删掉已有文件（`configs/*.yaml` 里还有 `yolo11_model_path` /
  `yolov8_model_path` / `classify_model` 等键指向它们）。

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

- **实测编译通过**：可执行 `planner_test`、`planner_test_offline`、`camera_test`、`usbcamera_test`、`multi_usbcamera_test`、`gimbal_test`、`gimbal_response_test`、`dm_test`、`fire_test`、`handeye_test`、`cboard_test`、`calibrate_camera`、`calibrate_handeye`、`calibrate_robotworld_handeye`、`capture`、`split_video`、`detect_freq_visual_test`（§8.7）；库目标 `auto_aim`、`omniperception`、`tools`、`io`、`tinympcstatic`、`serial`。
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

### 8.5 推理后端现状（x86_64 / WSL2 与 Jetson）

- **ONNX 后端（已实现）**：`tasks/auto_aim/yolos/yolov5_onnx.cpp` 用 `cv::dnn` 跑
  `assets/yolov5_0526.onnx`，`yolo.cpp` 按新增键 `yolov5_backend`（默认 `auto`）分发。
  预处理/后处理与 `yolos/yolov5.cpp::parse()` 完全一致（见 §4.2），无需 OpenVINO / TensorRT。
- ⚠️ **OpenCV DNN 版本要求（实测）**：本机 OpenCV 4.5.4 **可以 load 但 `forward()` 会断言失败**
  （`shape_utils.hpp total()`，模型尾部的 5D `Reshape/Transpose` 不被旧 DNN 支持）；
  原始 `0526.onnx` 还因为含 `FLOAT16` 张量连 load 都不行（python 的 cv2 5.0 两者都能跑）。
  已用 `onnx`(1.17, 隔离安装在 /tmp) 生成 FP32 版本 `0526_fp32.onnx`（数值逐位一致）验证：
  仍是同样的 forward 断言 → **结论是 OpenCV 4.5.x 的 DNN 跑不了这个图**。
  在 OpenCV ≥ 4.9/5.x 上该后端可用；本机（4.5.4）与 Ubuntu 22.04/24.04 的 `libopencv-dev`
  （4.5.4/4.6）**不可用**，需要等下一个后端。
- **TensorRT 后端（已实现，等 §4.0 装 TensorRT 才能编）**：`tasks/auto_aim/yolos/yolov5_trt.cpp`
  在 `HAVE_TENSORRT` 宏后面用 TensorRT 反序列化 `.engine`，`yolo.cpp` 按 `yolov5_backend: "trt"`
  分发；引擎路径用新增键 `yolov5_trt_engine_path`（默认 `assets/yolov5_0526_fp16.engine`）。
  本机没装 TensorRT 时构造函数抛明确异常（不静默退化），头文件不含 `<NvInfer.h>`（PIMPL）。
- **共享预处理/后处理**：`yolos/yolov5_postprocess.{hpp,cpp}` 是 letterbox / sigmoid / parse /
  NMS / check_name / center_norm 的**唯一实现**，ONNX 与 TensorRT 两个后端都调它，避免行为漂移
  （§4.1.4 的一致性对比就是对比它）。
- **下一步（ONNX Runtime）**：笔记本若 OpenCV 太旧则走 ONNX Runtime（x86_64/aarch64 都有官方
  预编译库）。在补上之前，`minimum_vision_system` / `auto_aim_test` / `camera_thread_test` /
  `usbcamera_detect_test` 虽然能**编译链接**，但运行到检测时会抛异常退出。
- `multithread/mt_detector` 已去掉 `ov::`（工作线程 + 队列，`push/pop/debug_pop` 接口不变），
  因此这几个 target 的编译不再依赖 OpenVINO。

### 8.6 仍然缺的降级（尚未实现）
- `io::Gimbal`：打不开 `/dev/gimbal` 仍会 `exit(1)`（`io/gimbal/gimbal.cpp`）→ `planner_test` / `fire_test` / `gimbal_test` 在笔记本上跑不了（`planner_test_offline` 不受影响）。
- `uav` / `uav_debug` / `minimum_vision_system` 已接入上述回放键，但在 §4 的 TensorRT 迁移完成前**无法编译**，所以笔记本上的端到端回放要等迁移完成。

### 8.7 检测频率可视化测试 + MindVision USB2.0 相机（新增）

- **测试程序**：`tests/detect_freq_visual_test.cpp`（目标 `detect_freq_visual_test`），imshow 前端 +
  三路曲线：`cap`（相机取流，蓝绿）、`model`（1000/detect_ms）、`pipe`（1000/(pre+detect+draw)），
  退出时打印各阶段 avg/p50/p95/max。用法（短选项必须 `-c=`）：
  ```bash
  ./build/detect_freq_visual_test -c=configs/detect_freq.yaml -m=replay -n=300 -d
  ./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=live -d
  ./build/detect_freq_visual_test -c=configs/detect_freq.yaml -m=bench -n=60 -save=/tmp/shot.png
  ```
  `-m=live/replay` 默认开显示；`-m=bench` 默认不显示（无 X 的 Jetson 上用 `-m=bench` + `-save`
  就能出图检查）。`-dump=<csv>` 导出每帧每个装甲板（一致性对比用）。**画面上只能写 ASCII**
  （`tools::draw_text` 是 Hershey 字体，中文会变 `????`）。
- 没有 TensorRT / OpenCV < 4.9 时验证本工具：用一个"输出恒为 `[1,N,22]` 常量"的合成 ONNX 当桩
  模型（只用 Slice/Sub/Reshape/Add，OpenCV 4.5.x 也能 forward），画框/CSV/曲线/汇总都会走到。
- **新增配置**：`configs/detect_freq.yaml`（回放 + `yolov5_backend: auto`）、
  `configs/mv_sua133gc.yaml`（真实 MindVision 相机 + `yolov5_backend: trt`）。
- **MindVision 新增键（都有默认值，已有 configs 不受影响）**：`mv_device_index`(0)、
  `mv_friendly_name`("")、`mv_frame_speed`(1；SDK: 0 低速/1 普通/2 高速)、`mv_resolution_width/height`(-1，改了要重标定)、
  `mv_media_type`(-1)、`mv_gain`(-1；SDK 值 100 = 1.0 倍)、`mv_frame_timeout_ms`(1000，原写死 100)、
  `mv_usb_reset`(true)。`io::MindVision` 打开时会打印：枚举到的设备、相机能力（分辨率/帧速/输出格式）、
  USB 链路速度、原始单帧大小 + 带宽估算、usbfs 内存上限，关闭时打印采集统计（共/有效/丢帧）。
- **实测（Jetson Orin Nano，MV-SUA133GC `f622:0001`，USB2.0 480M 口，1280x1024 Bayer8）**：
  35.7 fps、0 丢帧（原始 1.25 MB/帧 ≈ 45 MB/s，链路已跑满）；1024x768 ~59fps、640x480 ~134fps。
  `frame_speed` 0/1/2 在 1280x1024 下都是 ~35.7fps（瓶颈是 USB 链路）。⚠️ 同一枚相机只能被一个
  进程打开：`CameraInit` 返回 `-18`（设备已经打开）就是上次的程序没退干净，`pgrep -a` 杀掉即可。
- 注意仓库里 `configs/camera.yaml` 的 `vid_pid: "f622:d13a"` 与本机这枚相机（`f622:0001`）不符：
  启动日志会告警并给出实际 vid:pid，复位 USB 时会自动退回实际值。
- ⚠️ **别用 `timeout`/`kill` 强杀相机进程**：`tools::Exiter` 只处理 `SIGINT`，`timeout` 默认发
  `SIGTERM` 会跳过 `CameraUnInit`，把这枚 USB2.0 老相机留在半开流状态；之后每次 open 都返回
  "共 N 帧、有效 0、丢帧 N"（`Camera dropped!` 日志里会提示拔插 USB），实测 libusb 复位和
  正常退出都清不掉，**等 1~2 分钟会自己恢复**（也可以拔插一次 USB / 换 USB3.0 口）。
  所以：交互调试用 Ctrl+C，脚本里用 `timeout -s INT`，或直接 `detect_freq_visual_test -n=<帧数>`
  让它自己正常退出。守护线程重连间隔是递增的（0.3s→0.6s→…→5s 上限，恢复出图后重置），
  避免异常状态下每 100ms 疯狂 open/reset。

### 8.8 无显示器（headless）MJPEG 直播 + MindVision 固定 pipeline（新增）

#### 8.8.1 MJPEG 直播（`-stream=<port>`，零第三方依赖）

- 实现：`tools/mjpeg_server.{hpp,cpp}`（POSIX socket + `cv::imencode`），已列入
  `tools/CMakeLists.txt` 的 OBJECT 库源文件。路由：`/` = `multipart/x-mixed-replace` 实时流，
  `/snapshot.jpg` = 当前帧 JPEG，其它路径（含浏览器自动请求的 `/favicon.ico`）= 404，
  连接数超 8 → 503。
- `detect_freq_visual_test -stream=<port>`：`port > 0` 即启服务；**没显式给 `-d` 时自动关 imshow**
  （无 X 机器上不再抛异常）。`publish()` 只在有客户端时才 `imencode`，没人看只花一次原子读，
  对频率统计无影响；慢客户端只丢帧、不拖慢主链路（每客户端一个线程 + `SO_SNDTIMEO` +
  `MSG_NOSIGNAL`，客户端断开不会 SIGPIPE 杀掉整个进程）。
- 实测开销（1280x1194 画布）：无客户端 `draw` 10.2 ms/帧；1 个客户端 16.1 ms/帧（JPEG 质量 80
  的编码 ≈6 ms）。两者都小于相机帧间隔 28 ms（35.7 fps），所以端到端仍是相机瓶颈：
  取流 35.7 fps → 端到端 34.8 fps，不掉帧。
- 用法（本机实测）：
  ```bash
  ./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=live -stream=8080 -no-yolo
  curl -s -o /tmp/snap.jpg http://127.0.0.1:8080/snapshot.jpg   # 脚本自检
  # 浏览器 / VLC / ffplay：http://<本机IP>:8080/
  ```
  启动日志会列出本机所有 IPv4（`lo` / `wlan0` / `tailscale0` …）与对应 URL。实测（Jetson Orin Nano）：
  监听 `0.0.0.0:8080`；`/snapshot.jpg` 200 + `image/jpeg`（175 KB，1280x1194 = 1024 画面 +
  170 曲线面板）；`/` 5 秒收到 31 MB multipart 数据；`/nope`、`/favicon.ico` 均 404；退出打印
  「直播服务已关闭」。
- 无 X 环境下 `cv::imshow` 抛异常（`DISPLAY` 为空）现在被捕获 → 退化「不显示」+ 一条 warn，
  程序继续跑（配合 `-stream` / `-save` 看画面）。

#### 8.8.2 MindVision 固定 pipeline（逐项显式置位 + 回读校验）

- 为什么需要：原来只设 曝光 / 伽马 /（可选）数字增益，其余 ISP 参数沿用相机或 SDK 上一次会话
  留下的值 —— 换机器、或被官方演示程序动过，成像会悄悄变，标定与阈值跟着漂。
- 新增键（都有默认值，`-1` / 空 = 不改，已有 `configs/*.yaml` 行为不变）：`mv_analog_gain`、
  `mv_wb_mode`、`mv_clr_temp_mode`、`mv_clr_temp_gain`（`"R,G,B"`）、`mv_once_wb`、`mv_sharpness`、
  `mv_contrast`、`mv_saturation`、`mv_anti_flick`、`mv_light_frequency`、`mv_frame_rate`；
  参数来源：`mv_data_dir`、`mv_parameter_mode`、`mv_parameter_mask`、`mv_parameter_load_group`、
  `mv_parameter_file`、`mv_parameter_save_file`、`mv_parameter_save_group`。
- **顺序是实测要求，别改**：`setup_parameter_source()`（data_dir / mode / mask / 从文件或参数组加载）
  → 输出格式 / 触发 / 帧速 / 增益 / 分辨率 → **曝光 / 伽马** → `apply_pipeline()`（模拟增益、白平衡、
  色温、锐度、对比度、饱和度、抗闪、光源频率、帧率）→ `log_effective_settings()` 逐项回读。
  原因（实测）：`CameraSetParameterMode` 会让 SDK 按参数表重载一次，把「自动曝光=开、曝光≈10ms、
  伽马=100」冲回默认；曝光侧排在它之后才不会被静默覆盖（回读日志能直接看出来）。
- 校验：启动日志 `回读 ...` 一段 18 项（分辨率 / 帧速 / 输出格式 / 触发 / 期望帧率 / 自动曝光 /
  曝光时间 / 模拟增益 / 数字增益 / 伽马 / 白平衡模式 / 色温模式 / 色温增益 / 锐度 / 对比度 /
  饱和度 / 抗闪 / 光源频率）。连跑两次、去掉时间戳后 `diff` 应为空。
- 本机实测（MV-SUA133GC）：`configs/mv_sua133gc.yaml` 已固定 `mv_analog_gain=64`、
  `mv_clr_temp_mode=1`、`mv_clr_temp_gain="100,100,100"`、`mv_sharpness=0`、`mv_contrast=100`、
  `mv_saturation=100`、`mv_anti_flick=0`、`mv_light_frequency=0`；两次启动回读逐行一致。
  `mv_wb_mode` / `mv_frame_rate` 在这枚相机上 `CameraSet*` 返回 `-4`（机型不支持），**别写**
  （写了每次启动都告警）；这两项仍会出现在回读里（白平衡=手动、期望帧率=0 Hz 不限速）。
- 黄金方案（跨机器复制成像）：`mv_parameter_save_file: "xxx.config"` 退出时存盘（实测 74 KB），
  `mv_parameter_file` 启动时恢复。存/取文件走的正是上面那条"会重载"的路径，顺序依旧是关键。
- **写入生效性怎么验**（本次就是靠它定位）：做一份"故意与当前值不同"的临时 yaml
  （如 `mv_analog_gain: 96` / `mv_sharpness: 5` / `mv_light_frequency: 1`），跑一次看回读是否变成
  新值；再单独只留一个键，就能分辨是"写不进去"还是"被后面的调用覆盖了"。

### 8.9 文档分工（readme）

- `readme.md`：**纯英文**，目前只写「MindVision 相机 pipeline 的开启方式」（10 节：pipeline、
  依赖、编译、启动、headless 直播、回读一致性校验、`mv_*` 键表、排错、实测基线、相关文档）。
- `readme_zh.md`：原完整中文 readme 的**逐字副本**（未删改），尚未移植的内容都还只在那里。
- 后续新增：面向英文读者的使用说明写 `readme.md`；中文说明写 `readme_zh.md`。两边都不要删掉
  §8.8.2 里那两条实测要求（`setup_parameter_source` 必须在曝光设置之前、`mv_wb_mode` /
  `mv_frame_rate` 本机不支持）。

- [ ] `Armor` / `Solver` / EKF / TinyMPC 签名与算法逻辑未变（§5）
- [ ] `make -C build/ <target> -j$(nproc)` 实际编译通过（§6.1、§6.2）
