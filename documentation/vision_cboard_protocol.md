# vision ↔ CBoard v1 协议（2026-10-24）

## 1. 范围与安全边界

视觉输出**当前图像**中目标相对指定参考轴的 Δyaw/Δpitch，不预测、不依赖电机姿态、
不决定电控的控制律。它不是绝对位置、速度指令或必须逐帧累加的步长。shoot 编码层恒为 0。
当前默认输出 `camera_optical`：相机光心为原点、光轴为参考轴。目标左侧 yaw 正，
上方 pitch 负；单位 rad。云台坐标输出 `gimbal` 必须先标定手眼外参，不能混称光轴角差。

仅 Classical CAN，标准 11-bit ID，DLC=8；拒绝扩展帧、RTR、错误帧和不完整帧。
四个 ID 必须互不相同，且不能占用四元数、弹速或旧绝对角命令 ID。合法范围包含 0x000。
配置文件中的 `0xFE/0x310/0x311/0x312` 只是候选值，**不是实机批准分配**。
无身份认证、防重放或总线访问控制；session 是重启/会话隔离，不是安全令牌。

## 2. 线格式（所有多字节整数大端）

| 方向 / 配置 ID | byte0 | byte1 | byte2..5 | byte6 | byte7 |
|---|---|---|---|---|---|
| 电控→视觉 `vision_handshake_request_canid` | 01 HELLO | 01 版本 | session uint32 | 00 | 00 |
| 视觉→电控 `vision_handshake_response_canid` | 02 ACK | 01 版本 | 回显 session | status | 00 |
| 电控→视觉 `vision_control_canid` | 03 CONTROL | 01 版本 | session uint32 | flags | seq uint8 |

ACK status：0 READY、1 NOT_READY、2 VERSION_MISMATCH。
flags = enable(bit0) | color(bits1..3)，color=1 红、2 蓝，bits4..7 必须为 0。
合法 flags：红停车=02、红启动=03、蓝停车=04、蓝启动=05。停车也必须带合法颜色。
seq 每发送一次 CONTROL 递增，模256回绕；首条任意，之后仅接受前向差 1..127。
重复和倒序 CONTROL 不改变状态、不续心跳；seq 重置时应创建新 session。

角差 ID 为 `relative_canid`，没有类型字节：

| 字节 | 含义 |
|---|---|
| 0 | bit0 control：1 当前观测有效，0 清除有效测量；其它位全0（含 shoot） |
| 1 | 00 保留 |
| 2..3 | Δyaw int16 BE，单位 1e-4 rad |
| 4..5 | Δpitch int16 BE，单位 1e-4 rad |
| 6..7 | 00 00 保留 |

示例 session=12345678：HELLO `01 01 12 34 56 78 00 00`；READY ACK
`02 01 12 34 56 78 00 00`；蓝启动 seq=7 `03 01 12 34 56 78 05 07`。
有效 yaw=0.1234、pitch=-0.0567：`01 00 04 D2 FD C9 00 00`。
停止是八字节全0；NaN/Inf 同样编码为停止；超范围饱和到 int16 边界而不回绕。

**角差帧不含 session/seq。** 电控必须本地门控，并在停车/会话更换时清接收缓存、清有效测量；
软件无法识别已进入内核或硬件 TX 队列的旧有效角差。需要线级绝对隔离时必须升级帧布局，
不能宣称 v1 已具备该能力。

## 3. 握手与状态

1. 电控启动/重启时生成新 session；保持运动禁止，周期发送 HELLO（建议100ms重试）。
2. 模型、几何配置、首张新鲜非空 BGR 图像就绪前，视觉收到 HELLO 回 NOT_READY，
   不创建会话；未知版本回 VERSION_MISMATCH；畸形 HELLO（含保留字节）丢弃。
3. READY 后进入 wait_control。电控校验 ACK ID、版本、session、status、保留字节，
   然后每约80ms发送递增 CONTROL（含停车时），小于视觉500ms控制超时。
4. enable=1→active；enable=0→stopped。active 才能发送有效观测；同session HELLO幂等，
   不清颜色/启停，不刷新控制心跳。新session使旧样本作废，已有有效发送时先停止再 ACK。
5. wait_control / active / stopped 任一状态控制超时，作废会话，必须重新 HELLO。
6. CONTROL ID上的畸形版本、DLC、类型、保留位或非法颜色立即 fail-closed：作废会话，
   不续心跳；合法但旧session的CONTROL仅拒绝。无关 ID 不影响状态。

## 4. 视觉处理与并发

current 路径：颜色/置信度/角点过滤→按中心距离确定性排序→最多尝试8个几何候选→
使用所选装甲板自身4角点PnP→角差→符号/静态偏置→1e-4rad量化。
默认 `relative_tx_shape: false`，gain/死区/步长限幅不改变测量；显式 true 才启用旧整形。
v1 实发只支持 current，tracker 不支持动态颜色，启动拒绝该组合。

接收回调只复制到容量64的有界队列；溢出清队列并作废会话。独立5ms线程处理接收和发送。
同一帧样本最多尝试一次有效发送，不重复灌角差；停止/无效观测不受有效发送限频影响。
停止失败保留 pending 重试；正常退出最多5次尝试（SocketCAN 单次写入超时20ms）。
SocketCAN写入成功仅表示内核接收，**不是电控接收或执行的ACK**。

取图前冻结generation；启停/颜色/会话更换使generation递增，检测期间旧样本拒绝。
还拒绝采集时间早于控制变更的缓存图像；发送时按**图像时间**判断观测过期，不能用检测结束
时间给旧图“续命”。观测过期独立发送停止，不等待下一张图。生产发送在状态锁内完成资格检查
与有界写入，接收队列和发送串行处理；已提交内核的帧不可撤回。

工业相机队列读取最多等待50ms后检查SIGINT；本地视频解码/SDK析构不提供硬实时截止保证。
SIGINT正常退出有best-effort停止；SIGKILL、掉电、进程崩溃、总线断开必须靠电控本地看门狗。

## 5. 配置

`vision_protocol: v1` 为入口默认；`legacy` 必须显式选择且无握手/远端颜色/启停，不能
绕过实机ID确认。两种模式均要求显式 `relative_canid`。`vision_can_ids_confirmed: true`
表示人工确认所有实际ID，仅 rtnetlink 核实 kind=vcan 可豁免；lo/docker/未知接口不能豁免。
`relative_output_frame: camera_optical|gimbal` 只选坐标系，优先于旧兼容键 `relative_reference`。
`can_command_mode: relative`、`cboard_wait_imu: false` 是实发前置条件。
控制超时 `vision_control_timeout_ms` 默认500；观测超时 `relative_frame_timeout_ms` 默认100，
当前配置120；`relative_tx_rate_hz` 默认0不限，有值时优先于 `relative_send_interval_ms`。
超时必须有限且大于0，发送频率/间隔必须有限且非负。

dry-run不构造CBoard、不打开CAN；`-dry-enable=0|1`、`-dry-color=red|blue` 显式模拟使能/颜色。
CSV的TX是编码后的测量值，`publish_ok`不是总线发送成功；实际发送看链路统计和独立抓包。
统计中的tx_frames是两行间所有角差/停止发送计数，不能与当前行观测强行一一对应。