# 相对角差链路验收记录 — 2026-10-24

> 记录「新 USB3.0 线 → 相机取流 → 检测 → 当前帧角差 → CAN 字节」这条链路**实际跑出来**的
> 数字与判据。所有命令都是在本机（Jetson Orin Nano，aarch64，`/home/irm/iRM-Vision-2026`）
> 真跑的；没跑过的一律标「未验证」，不写推测值。
>
> 协议/配置见 `documentation/relative_can_protocol.md`，交接状态见
> `documentation/handoff_relative_servo.md`。

---

## 0. 一句话结论

| 项 | 结果 |
| --- | --- |
| 新 USB3.0 线（**唯一变量**）兼容性 | ✅ 枚举 USB3.0 5Gbps，1280×1024 **120.5 fps**、3×60 s 无长间隔、丢帧 0/0/1 |
| 旧线 A/B 对照 | ❌ **未验证**（旧线不在手上：型号/长度/标称规格/两端接口均未知） |
| 当前帧角差链路（检测→PnP→角差→量化） | ✅ 150 帧回放，有效帧 124（82.7%），TX 与 raw 只差 ≤0.5×1e-4 rad |
| CAN 字节级验收（vcan 独立接收） | ✅ 121 帧全部 `0FE [8]`，99 帧 `control=1` 逐字节与 CSV 一致，0 处不一致 |
| HUD（无显示器直播） | ✅ `/snapshot.jpg` 200 + `image/jpeg`，绿框选中目标 + ASCII 角差可读 |
| 板端（真 CBoard / 云台闭环 / 开火） | ❌ **未验证**（无 CBoard、无电机、无编码器；清单见 §7） |

---

## 1. 环境

| 项 | 值 |
| --- | --- |
| 平台 | Jetson Orin Nano，aarch64，Ubuntu 20.04 / JetPack 5.1.3 |
| 相机 | MindVision MV-SUA133GC，`f622:0001`，SN 043070620316 |
| 链路 | 新 USB3.0 Type-B 线 → Genesys Hub → `bus 2 addr 4`，`speed=5000`（USB3.0 超速 5Gbps） |
| 推理后端 | TensorRT 8.5.2 + `assets/yolov5_0526_fp16.engine`（本机无 ONNX Runtime；OpenCV 4.5.4 的 `cv::dnn` 跑不了该图） |
| CAN | 板上无 CBoard；`can0` UP @1Mbps（本轮**未**在 can0 上发帧，见 §5） |

---

## 2. 线材 / 相机取流（`tests/camera_test.cpp`）

### 2.1 为什么先改这个测试

原来 `camera_test` **每帧**打印 `logger()->info("{:.2f} fps")`，而 `tools::logger()` 是
`flush_on(info)`：120fps 下每帧一次落盘本身就会拖慢采集循环，测出的帧率既偏低又看不出抖动。
现在默认每 1s 一行进度，退出时打印帧间隔 `p50/p95/p100`、抖动、长间隔（>50ms）次数；
逐帧打印保留在 `-v`。预热 `-w`（默认 5s）不计入统计，`-t` 到点自己正常退出
（让 SDK 打印「共/有效/丢帧」，不要用 `timeout` 默认的 SIGTERM 强杀）。

### 2.2 命令（连跑 3 次，每次 5s 预热 + 60s 统计）

```bash
for i in 1 2 3; do
  ./build/camera_test -c=configs/mv_sua133gc.yaml -w=5 -t=65 > /tmp/p1_run$i.log 2>&1
done
```

### 2.3 实测结果

| 运行 | 统计帧数 | 统计时长 | 平均 fps | p50 (ms → fps) | p95 | p100 | 抖动 std | 长间隔 >50ms | SDK 丢帧 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 7229 | 60.01 s | 120.47 | 8.29 → 120.67 | 8.46 | 23.99 | 0.40 ms | 0 | 0（7854/7854） |
| 2 | 7240 | 60.01 s | 120.65 | 8.29 → 120.70 | 8.49 | 15.60 | 0.33 ms | 0 | 0（7854/7854） |
| 3 | 7219 | 60.01 s | 120.30 | 8.29 → 120.69 | 8.50 | 25.41 | 0.57 ms | 0 | 1（7856/7855，开机首帧） |

- 三组平均帧率 **120.30 ~ 120.65 fps**，`p95` 都在 8.5 ms 以内 → 取流是稳定的等间隔。
- 原始数据量：1280×1024 Bayer8 ≈ 1.25 MB/帧 × 120 fps ≈ **150 MB/s**，USB3.0 Gen1 有足够余量。
- 历史文档里同机同相机的 USB2.0（480M）基线是 **~35.7 fps**（当时已跑满链路带宽）；
  本轮是 **120.5 fps ≈ 3.4×**。⚠️ 但这是**跨会话对比**，不是同条件 A/B：
  **旧线未对照 → 「换线带来的提升」记为未验证**。要出结论，需要旧线（型号/长度/标称规格 +
  两端接口）做同机、同口、同预设的对照跑。

---

## 3. 内参与标定

- 标定来源：`records/calibration/2026-10-10_16-23-58_0`（重投影 RMS 0.224 px），标定尺寸
  **1280×1024**，与 `configs/relative_current.yaml` 的 `camera_matrix` 一致。
- **换线不改变成像几何**（同一颗相机、同一镜头、同一 ROI），因此本轮未重标定。
- 手眼外参（`R_camera2gimbal` / `t_camera2gimbal`）**仍未标定**：默认路径用
  `relative_reference: "camera_optical"`，不需要它；改成 `gimbal` 会在启动时明确报缺键。


---

## 4. 离线自检与 dry-run

### 4.1 纯函数 / 编码 / 几何 / 选目标（`relative_servo_test`）

```bash
make -C build/ relative_servo_test -j$(nproc) && ./build/relative_servo_test
# → 54 checks, 0 failures（exit 0）
```

覆盖：角差符号（左正/上负）、死区/限幅/增益/符号/偏置、CAN 字节布局、`control=0` 强制清零、
NaN/Inf 清零、量化饱和不回绕、`camera_optical` 与 `gimbal` 两种参考系（含「用单位阵外参会得到
完全不同的角差」这条反例）、带畸变内参、min_depth、重投影超限、确定性选目标、过期帧。

### 4.2 回放 dry-run（不打开 CAN）

```bash
./build/relative_aim -c=configs/relative_current_replay.yaml -dry-run -n=150 -csv=/tmp/rel_dry.csv
```

实测（`assets/demo/demo.avi`，TensorRT FP16）：

| 指标 | 值 |
| --- | --- |
| 处理帧数 | 150 |
| 有效帧（`control=1`） | **124（82.7%）** |
| 无效原因分布 | `no_target` 24，`out_of_image` 2 |
| 检测耗时 | **23.3 ms/帧** |
| 观测年龄（发送 − 图像时间戳） | p50 **23.3 ms**，p95 **24.1 ms** |
| `max abs(TX_int − raw×1e4)` | **0.50**（= 半个 LSB，即只剩量化误差） |

有效帧样例（CSV，单位为度）：

```
frame,frame_t_ms,age_ms,candidates,target_id,valid,reason,raw_yaw_deg,raw_pitch_deg,tx_yaw_deg,tx_pitch_deg,tx_yaw_1e4,tx_pitch_1e4,reproj_px
2,90.25,22.66,1,blue:sentry:small@863/451,1,ok,-3.4802,-2.5057,-3.4781,-2.5040,-607,-437,1.590
4,158.16,22.73,1,blue:sentry:small@896/635,1,ok,-4.5750,3.2912,-4.5725,3.2890,-798,574,1.166
```

→ 这就是上一轮交接卡住的地方（当时 tracker 恒 `lost`，dry-run 恒 `control=0`）：
`-source=current` 用**本帧检出的装甲板自己的角点**做 PnP，不再依赖锁定，第一帧就能发。

### 4.3 HUD（headless 直播）

```bash
./build/relative_aim -c=configs/relative_current_replay.yaml -dry-run -n=400 -stream=8099 &
curl -s -o /tmp/hud.jpg -w '%{http_code} %{content_type} %{size_download}\n' \
     http://127.0.0.1:8099/snapshot.jpg
# → 200 image/jpeg 132175
```

快照内容核对（ASCII，Hershey 字体）：`src=current ref=camera_optical frame=115`、
`raw yaw=-6.22 pitch=-0.10 deg`、`tx yaw=-6.22 pitch=-0.10 deg`、`valid=1 reason=ok cand=1`、
`tgt=blue:sentry:small@952/528`、`age=24ms detect=24ms sent=0 ok=0 fail=0`，选中的装甲板画绿框。
（`sent=0` 是因为 dry-run 不打开 CAN。）

---

## 5. CAN 软件链路（vcan + candump 独立接收）

本机**没有** CBoard，也没有现成的 `vcan` 接口；`can0` 是真实接口（1Mbps，ERROR-ACTIVE，
本轮未发帧）。为了在不打扰总线的前提下验证字节布局，用**私有 user+net namespace** 建 `vcan0`
（不需要 sudo，退出后 vcan0 随 netns 消失）：

```bash
awk '{if ($1=="can_interface:") print "can_interface: \"vcan0\""; else print}' \
    configs/relative_current_replay.yaml > /tmp/rel_vcan.yaml

unshare -Urn --map-root-user bash -c '
  ip link add dev vcan0 type vcan && ip link set up vcan0
  candump -tz vcan0 > /tmp/candump.txt &
  sleep 1
  ./build/relative_aim -c=/tmp/rel_vcan.yaml -n=120 -csv=/tmp/rel_vcan.csv
'
```

实测：

| 项 | 值 |
| --- | --- |
| 程序侧 | 120 帧：有效 99（82.5%），发送 120，成功 **120**，失败 0，限频跳过 0；退出补发 `control=0` **成功**（0.02 ms） |
| candump 收到 | **121** 帧（120 + 退出帧），全部 `0FE [8]` |
| `control` 位分布 | `control=1` **99** 帧、`control=0` **22** 帧（21 无效 + 1 退出帧） |
| `shoot` 位 | **0** 帧置位 |
| 逐帧比对（candump 解码 vs CSV `tx_*_1e4`） | 99 个有效帧 **0 处不一致** |
| 首帧 | `00 00 00 00 00 00 00 00`（第 1 帧 `no_target`，符合预期） |

原始样例（`candump -tz`）：

```
 (000.033910)  vcan0  0FE   [8]  01 00 FD A1 FE 4B 00 00
                                      ^^^^^^ ^^^^^^
                                      0xFDA1 = -607 → -3.478°   0xFE4B = -437 → -2.504°
 (004.126923)  vcan0  0FE   [8]  00 00 00 00 00 00 00 00   ← 退出前补发的 control=0
```

⚠️ 关于 `can0`：真实总线在没有第二个节点 ACK 时，帧会重传并累计 error counter，
所以**本轮没有**在 `can0` 上发帧。真机联调时按协议文档 §5 用 `candump -tz can0,0FE:7FF` 抓包。

---

## 6. 代码层面的可观测性（本轮新增）

| 能力 | 位置 |
| --- | --- |
| 逐帧 CSV（帧号 / 图像时间戳 / 发送时刻 / 年龄 / 目标标识 / raw / TX / 有效性 / 原因 / 发送尝试与结果 / 检测耗时） | `src/relative_aim.cpp -csv=<path>` |
| HUD（ASCII）+ MJPEG 直播 | `src/relative_aim.cpp -stream=<port>` / `-d` |
| 发送结果（整帧写入才算成功） | `io::CBoard::send()` 返回 bool；`io::SocketCAN::write()` 校验 `sizeof(can_frame)` + errno |
| 编码层安全（control=0/NaN/Inf 清零、饱和不回绕） | `io::encode_relative_command()` / `io::quantize_angle_1e4()` |
| 配置在阻塞构造之前校验 | `src/relative_aim.cpp`（`can_command_mode` / `cboard_wait_imu` / `relative_canid`） |
| SocketCAN 健壮性（close 后 fd 复位、EINTR/EAGAIN、不完整帧丢弃） | `io/socketcan.hpp` |

---

## 7. 板端验收清单（**未验证**，需要 CBoard + 云台）

- [ ] 电控固件实现 `0xFE` handler：**按收到帧累加一次**（不要用定时器重复累加），
      `control=0` 立即停止累加，并有接收超时（建议 100 ms）→ 见协议文档 §3.2。
- [ ] `candump -tz can0,0FE:7FF`：DLC=8，byte0 bit0=control、bit1=shoot=0，
      byte2-3 / 4-5 = int16 大端。
- [ ] 方向：目标在画面左 → Δyaw 为正 → 云台向左转；反了改 `servo_yaw_sign` / `servo_pitch_sign`
      （只在一处反号）。
- [ ] 收敛：`servo_gain=0.2`、`servo_max_step_deg=3` 起步，对准后 Δyaw/Δpitch 收敛进死区。
- [ ] 不振荡、不甩头；掉线/无目标时云台停住（`control=0`）。
- [ ] 与电控确认 `0xFE` 未被其它节点占用；仓库里的 `relative_canid` 冲突检查只覆盖视觉侧自己的 ID。
- [ ] 开火：本轮**未**实现（`shoot` 恒 0），按电控侧逻辑或后续接 `Shooter`。

---

## 8. 未验证 / 已知缺口

1. **旧 USB 线同条件 A/B**：缺旧线 → 只能给当前线的绝对表现，不能给「换线收益」。
2. **真机 can0 + 真 CBoard**：无硬件，全部用 vcan 验证协议字节；电控侧解析未验证。
3. **云台闭环**：无电机/编码器，只验证到「角差正确且随目标变化」，未验证闭环稳定性。
4. **手眼外参**：未标定，`relative_reference: "gimbal"` 只有离线自检（合成数据）通过。
5. **时间戳语义**：`VideoFile` 给的是「读到这一帧的时刻」，MindVision 给的是「主机拿到 buffer
   的时刻」；两者都不含曝光/传输延迟，`age_ms` 只反映检测 + 排队。
6. **SocketCAN 并发**：`write()`（主线程）与守护线程的 `close()`/重连之间没有互斥；
   极端时序下这一帧可能写进已关闭的 fd（会返回失败并被计数，不会静默成功）。
