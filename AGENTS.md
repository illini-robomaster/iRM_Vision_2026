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
| 0.6 | 干完活不提交 / 不推送；或把 `build/`、`logs/`、`third_party/`、`.vscode/`、`*.engine\|*.plan\|*.trt` 等产物混进提交 | 见 §9：每个可验证的工作单元一次聚焦 commit，并 `git push origin main` |
| 0.7 | 对 `main` 做 `push --force` / `--force-with-lease`；rebase 已推送的提交；`reset --hard` 丢掉他人提交 | 见 §9.3：分叉先 `git fetch` 看清，默认 merge |

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
  - `tasks/auto_aim/yolos/yolov5.{cpp,hpp}`（原 OpenVINO 版；在 `tasks/auto_aim/CMakeLists.txt` 里注释着、不参与编译，能力已由 `yolov5_postprocess` + 三个后端文件覆盖）
  - `tasks/auto_aim/yolos/yolov8.{cpp,hpp}`
  - `tasks/auto_aim/yolos/yolo11.{cpp,hpp}`
  - `tasks/auto_buff/yolo11_buff.{cpp,hpp}`
  - 上述文件中的 `ov::Core`、`ov::CompiledModel`、`core_.read_model()`、`compile_model()` 需替换为 TensorRT 封装。
  - **已迁移（不再含 `ov::` / `openvino.hpp`）**：`tasks/auto_aim/classifier.{cpp,hpp}`（删掉 `ov::Core` / `ov::CompiledModel` 成员与 `ovclassify()`，只留 `cv::dnn` 分类器）、`tasks/auto_aim/detector.{cpp,hpp}`、`tasks/auto_aim/multithread/mt_detector.{cpp,hpp}`，以及 `yolo.cpp` / `yolos/yolov5*`（共享后处理 + ONNX Runtime / OpenCV DNN / TensorRT 三个后端，见 §8.5）。
  - 现状：全量 `make -C build/ -k` 现在**只剩一个失败构建单元 `auto_buff`**，唯一报错是
    `tasks/auto_buff/yolo11_buff.hpp:7:10: fatal error: openvino/openvino.hpp: No such file or directory`
    （`buff_detector.hpp` / `buff_target.hpp` / `buff_aimer.hpp` 都直接 `#include "yolo11_buff.hpp"`）；
    可执行 `uav` 链接 `auto_buff` 因此也编不过。这是预期状态，不是回归——这两个目标之外的代码已有完整编译反馈。
  - `use_traditional`（Detector + Classifier 二次矫正角点）已随迁移恢复：`YOLOBase::configure_traditional()`
    （`yolo.{hpp,cpp}`）持有 `std::unique_ptr<Detector>`，`yolos/yolov5_postprocess.cpp::parse()` 在
    `center_norm` 之前对每个存活检出调用 `traditional->detect(armor, bgr_img)`（与原 OpenVINO 版
    `yolov5.cpp` 的顺序一致），ONNX Runtime / OpenCV DNN / TensorRT 三个后端都已接上。启用后需要
    `configs/*.yaml` 里有传统方法那组键（`threshold` / `max_angle_error` / `min_lightbar_ratio` / …）
    与 `classify_model`，缺失时构造抛带后端名的异常，**不会静默退化成“矫正没生效”**。
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
`build/` 已配置的目标（ROS 未启用时约 80 个）。按实测（v1.2，x86_64 / WSL2 上以 `make -C build/ -k` 全量验证，与 Jetson 同一份代码）分成两类：

- **实测编译通过**：可执行 `planner_test`、`planner_test_offline`、`camera_test`、`usbcamera_test`、`multi_usbcamera_test`、`gimbal_test`、`gimbal_response_test`、`dm_test`、`fire_test`、`handeye_test`、`cboard_test`、`calibrate_camera`、`calibrate_handeye`、`calibrate_robotworld_handeye`、`capture`、`split_video`、`detect_freq_visual_test`（§8.7）、`camera_detect_test`、`detector_video_test`、`uav_debug`；库目标 `auto_aim`、`omniperception`、`tools`、`io`、`tinympcstatic`、`serial`。
- **能编译链接、运行到检测才需要推理后端**（x86_64 / WSL2，取决于 `SPVISION_HAS_ORT`）：`auto_aim_test`、`camera_thread_test`、`usbcamera_detect_test`、`minimum_vision_system`。缺 ONNX Runtime 时会回退 `cv::dnn`，而 OpenCV < 4.9 的 `forward()` 会断言失败（§8.5）。
- **当前编译失败（只剩 `auto_buff` 一条链路，属预期状态，见 §4.1）**：`auto_buff`、`auto_buff_test`、`auto_buff_debug_mpc`（`auto_buff/yolo11_buff.hpp`），以及链接 `auto_buff` 的 `uav`。报错只有一类：`fatal error: openvino/openvino.hpp`。**不要**为了让它们“编过”而注释逻辑或加临时桩；按 §4.1 完成迁移后再放开。

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
- [ ] 已按 §9 提交并推送，且回复中给出本地 / 远程 SHA 一致的证据（§9.3、§9.5）

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
3. 编译可移植目标子集（含端到端检测目标；`third_party/onnxruntime` 就位时自动启用 ORT 后端）。

### 8.3 当前可移植目标子集
- 库：`serial`、`tools`、`io`、`auto_aim`、`tinympcstatic`、`omniperception`
- 可执行：`planner_test`、`planner_test_offline`、`camera_test`、`usbcamera_test`、`multi_usbcamera_test`、`cboard_test`、`dm_test`、`fire_test`、`gimbal_test`、`gimbal_response_test`、`handeye_test`、`capture`、`split_video`、`calibrate_camera`、`calibrate_handeye`、`calibrate_robotworld_handeye`、`detect_freq_visual_test`（§8.7）、`camera_detect_test`、`detector_video_test`、`uav_debug`
- 端到端检测目标（`auto_aim_test`、`camera_thread_test`、`usbcamera_detect_test`、`minimum_vision_system`）能编译链接，**运行到检测时**需要可用后端（ORT 就位即真正跑，见 §8.5）。
- 仍然无法编译的只剩 `auto_buff` / `auto_buff_test` / `auto_buff_debug_mpc` / `uav`（未迁移的 `auto_buff/yolo11_buff.hpp`）：见 §6.1。`classifier` / `detector` 在本轮（v1.2）已迁移完成——`camera_detect_test`、`detector_video_test`、`uav_debug` 从“编译失败”转为可编译。
- CI：`.github/workflows/build-x86.yml` 在 ubuntu-22.04 上编译上述 26 个目标（含新增的 `camera_detect_test` / `detector_video_test` / `uav_debug`），并冒烟运行 `planner_test_offline configs/demo.yaml`（注意 `fire_thresh` 等键只有 `configs/demo.yaml` / `standard3.yaml` / `standard4.yaml` 有，其余 config 会因缺键 `exit(1)`）、`camera_test -c=configs/offline.yaml`、`dm_test -p=none`、`detect_freq_visual_test -m=bench -n=30 -no-yolo`。

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

- **OpenCV DNN 后端（兜底，已实现）**：`tasks/auto_aim/yolos/yolov5_onnx.cpp` 用 `cv::dnn` 跑
  `assets/yolov5_0526.onnx`，`yolo.cpp` 按新增键 `yolov5_backend`（默认 `auto`）分发。
  预处理/后处理与 `yolos/yolov5.cpp::parse()` 完全一致（见 §4.2），无需 OpenVINO / TensorRT。
- ⚠️ **OpenCV DNN 版本要求（实测）**：本机 OpenCV 4.5.4 **可以 load 但 `forward()` 会断言失败**
  （`shape_utils.hpp total()`，模型尾部的 5D `Reshape/Transpose` 不被旧 DNN 支持）；
  原始 `0526.onnx` 还因为含 `FLOAT16` 张量连 load 都不行（python 的 cv2 5.0 两者都能跑）。
  已用 `onnx`(1.17, 隔离安装在 /tmp) 生成 FP32 版本 `0526_fp32.onnx`（数值逐位一致）验证：
  仍是同样的 forward 断言 → **结论是 OpenCV 4.5.x 的 DNN 跑不了这个图**。
  在 OpenCV ≥ 4.9/5.x 上该后端可用；本机（4.5.4）与 Ubuntu 22.04/24.04 的 `libopencv-dev`
  （4.5.4/4.6）**不可用**，需要 ORT 或 TensorRT 顶上；本机当前 WSL 实测 OpenCV **4.10.0**，
  此版本 `cv::dnn` 与 ORT 都能跑。
- **TensorRT 后端（已实现，等 §4.0 装 TensorRT 才能编）**：`tasks/auto_aim/yolos/yolov5_trt.cpp`
  在 `HAVE_TENSORRT` 宏后面用 TensorRT 反序列化 `.engine`，`yolo.cpp` 按 `yolov5_backend: "trt"`
  分发；引擎路径用新增键 `yolov5_trt_engine_path`（默认 `assets/yolov5_0526_fp16.engine`）。
  本机没装 TensorRT 时构造函数抛明确异常（不静默退化），头文件不含 `<NvInfer.h>`（PIMPL）。
- **共享预处理/后处理**：`yolos/yolov5_postprocess.{hpp,cpp}` 是 letterbox / sigmoid / parse /
  NMS / check_name / center_norm 的**唯一实现**，`cv::dnn` / TensorRT / ONNX Runtime **三个**后端
  都调它，避免行为漂移（§4.1.4 的一致性对比就是对比它）；ORT 侧只保留 Session 构造与
  FP16/FP32 张量转换。
- **ONNX Runtime 后端（已实现，x86_64 / WSL2 主力）**：`tasks/auto_aim/yolos/yolov5_ort.{hpp,cpp}`，
  依赖 `third_party/onnxruntime`（`scripts/fetch_onnxruntime.sh` 下载，**不入库**），CMake 找到后定义
  `SPVISION_HAS_ORT`；`yolo.cpp` 在 `auto` 下按 TRT → ORT → `cv::dnn` 选后端，启动日志打印真实选中的
  那个。实测（`-dump=csv` 逐帧对比，本机 WSL2 / OpenCV 4.10.0 + ORT 1.17.3，`assets/demo/demo.avi`
  前 40 帧）：ORT **38/40** 帧有检出、最高置信度 **0.968**、平均 **40.5 ms/帧**（p95 47.4）；
  `cv::dnn` 同为 38/40 / 0.968、平均 **46.5 ms/帧**（p95 66.3）；两后端逐帧装甲板数量与
  颜色/编号/类型标签**完全一致**，最大置信度差 **5.8e-5**、最大关键点像素差 **0.07 px**。
- **Classifier / Detector 已迁移（v1.2）**：`classifier.{cpp,hpp}` 删掉了 `ov::Core` / `ov::CompiledModel`
  成员与 `ovclassify()`，只留 `cv::dnn`（`classify_model`，默认 `assets/tiny_resnet.onnx`）；
  `detector.{cpp,hpp}` 不再 include OpenVINO 头，只做编译期适配（`<numeric>` / `<ctime>`、
  `fmt::format`、`strftime`、`ARMOR_NAMES[...]` 查名）。因此 `use_traditional` 可以恢复：
  `YOLOBase::configure_traditional()`（`yolo.{hpp,cpp}`）+ `yolov5_postprocess::parse(..., Detector *)`。
  实测：`configs/offline.yaml` 本身就是 `use_traditional: true` + 全套传统方法键，因此本地回归的
  **用例 4（`auto_aim_test -e=60 -c=configs/offline.yaml`，检测→跟踪→规划 60 帧）已是这条路径的端到端验证**，
  日志首行即 `[YOLOV5_ORT] use_traditional=true：已启用传统方法二次矫正角点（Detector+Classifier…）`。
  另用 `configs/detect_freq.yaml` 派生 `use_traditional: true` 单测（WSL2 / ORT / `assets/demo/demo.avi`）：
  bench 20 帧 `detect` 平均 **32.01 ms**（关掉时 40 帧为 **31.94 ms**）、退出码 0、18 个装甲板；用临时计数日志
  （验证后已删除）确认 `Detector::detect(Armor&, img)` 恰好被调用 **18** 次 = 检出数。这份 demo 视频
  的画面过不了传统方法的几何检查（`detect()` 返回 false），所以角点与关掉时逐帧一致
  （`-dump=csv` 两份完全相同）——这是原实现的行为（矫正不成立就保留网络关键点），不是“没接上”。
- **仍然待补**：Jetson 侧 TensorRT 端到端回放（需按 §4.0 装 TensorRT 并离线生成 `.engine`）；
  ORT 复用共享后处理已完成（三个后端现在只有推理调用点不同）。
- `multithread/mt_detector` 已去掉 `ov::`（工作线程 + 队列，`push/pop/debug_pop` 接口不变），
  因此这几个 target 的编译不再依赖 OpenVINO。

### 8.6 仍然缺的降级（尚未实现）
- `io::Gimbal`：打不开 `/dev/gimbal` 仍会 `exit(1)`（`io/gimbal/gimbal.cpp`）→ `planner_test` / `fire_test` / `gimbal_test` 在笔记本上跑不了（`planner_test_offline` 不受影响）。
- `uav_debug` 已接入上述回放键，且随 `classifier` / `detector` 迁移完成（v1.2）已能编译；`uav` 仍因链接未迁移的 `auto_buff` 而编不过，所以笔记本上的端到端回放走 `minimum_vision_system` / `auto_aim_test`（ORT 就位即真正跑检测，见 §8.5）。

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

- [ ] `Armor` / `Solver` / EKF / TinyMPC 签名与算法逻辑未变（§5）

## 9. Git 提交与推送规则（交付即提交）

> 与 §0.6 / §0.7、§6.2、§7 配套：**验证通过 → 提交 → 推送 → 报告 SHA** 是本仓库每轮任务的固定收尾动作，
> 不要把未提交的改动留给用户手动处理。

### 9.1 触发时机：一个工作单元一次提交
- 一个「工作单元」= 一次可验证的改动：修一个 bug / 加一个后端 / 加一个测试或脚本 / 更新一处文档。
- 判据：`make -C build/ <受影响 target> -j$(nproc)` 通过（§6.2）；涉及 x86 可移植目标时，优先再跑一次
  `bash scripts/run_local_tests.sh`。
- **同一轮回复内**完成 commit + push；不要攒批，不要以「等会儿一起提」为由留下改动。任务结束时
  `git status` 必须是 `nothing to commit, working tree clean`（只剩被 `.gitignore` 忽略的文件）。

### 9.2 提交（commit）
- 提交前复核：`git status` + `git --no-pager diff --stat`，确认没有 `build/`、`logs/`、`third_party/`、
  `.vscode/`、`*.engine` / `*.plan` / `*.trt` 等产物混入（§0.6）。用**指定路径** `git add`，不要
  `git add -A` 盲加。
- **身份必须先配好**（实测 local / global / system 三处都可能为空，不配则 commit 直接失败）：
  ```bash
  git config user.name  "koerimikan"
  git config user.email "104670756+koerimikan@users.noreply.github.com"   # 与推送账号一致
  ```
  临时用 `git -c user.name=... -c user.email=... commit` 也算合规，但优先配置到仓库。
- 信息格式（沿用本仓库既有实践）：首行 `<type>(<scope>): <中文摘要>`，`type` ∈
  `feat|fix|port|build|test|docs|refactor|chore|merge`，`scope` 用模块名（`yolo` / `io` / `tools` / `scripts` /
  `agents` …），摘要 ≤ 60 字；禁止 `update` / `fix bug` 之类无信息量标题。
- 正文写清「改了什么 / 为什么 / **实测命令与结果**」，数字必须是真跑出来的（与 §6.2 同一要求）；
  跨平台影响（Jetson / x86_64）要写明。
- 一次提交只解决一个问题，diff 保持可审阅（§6.4）；不要把无关的格式化 / 重排混进来。

### 9.3 推送（push）与分叉处理
- 提交后立即 `git push origin main`（本仓库唯一长期分支，未启用 PR 流程）。
- 被拒 / 远程有新提交时，**先看清再动手**：
  ```bash
  git fetch origin
  git --no-pager log --oneline --left-right --graph main...origin/main
  ```
  - **默认策略：merge**（`git pull --no-rebase origin main`）——保留双方历史，冲突只解一遍；
  - 仅当本地提交**尚未推送**、且明确要求线性历史时，才用 `git rebase origin/main`；
  - **已推送的提交一律 merge，禁止 rebase**。
- 解冲突用定点编辑（§6.4，禁止 `sed -i` / 脚本批量替换）；解完**必须重新执行 §6.2 的编译 / 运行验证**，
  再提交（合并结果也是一次聚焦提交）并 push。
- **禁止** `git push --force` / `--force-with-lease` 到 `main`；**禁止** `git reset --hard` 丢掉他人提交；
  未经确认不得 `git clean -fd`。
- push 后自检并汇报证据：`git --no-pager log --oneline -1 origin/main` 与本地 HEAD **SHA 一致**。

### 9.4 仓库卫生
- 不提交：`build/`、`third_party/`（由 `scripts/fetch_onnxruntime.sh` 按机器下载）、`logs/`、`.vscode/`
  （含 `eigen_fix.h`，§2.3 要求不提交也不删除）、`*.engine` / `*.plan` / `*.trt`、`records*/`、
  `CMakeCache.txt`、`compile_commands.json`。
- 大文件（模型 / 引擎 / 视频）不新增；`assets/` 现有权重只在必要时更新，并在提交正文里写明来源与 md5
  （§4.2；仓库保持 private）。
- 需要暂存不完整的工作时用 `git stash`（留在本地），不要把半成品提交到 `main`。

### 9.5 与其它章节的关系
- §9.1 是 §7 自检清单的收尾项：§7 全部通过才算一个可提交的工作单元，commit + push 完成后该单元才算交付。
- 与 §6.4「编辑纪律」一致：解冲突 / 合并同样使用定点编辑。
- §0.6 / §0.7 是本节的硬约束版本：违反即视为无效输出。

- [ ] `make -C build/ <target> -j$(nproc)` 实际编译通过（§6.1、§6.2）
