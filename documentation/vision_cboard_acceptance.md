# vision ↔ CBoard 验收记录

日期：2026-10-24。工作区 `/home/irm/iRM-Vision-2026`，本机 Jetson/aarch64，
实际日志选中 TensorRT 与已有本地 FP16 engine。本轮未接 CBoard，未向真实 can0 发帧。
下述 A–E 为本交付的分层定义，不代表外部认证或完整安全证明。

| 等级 | 范围 | 结果 |
|---|---|---|
| A | 最小目标编译、协议/几何/状态机/配置/并发事务单测 | 通过：128 checks，0 failures |
| B | 实际 TensorRT 离线回放、当前帧角差CSV | 通过：60帧，55有效，无CAN发送 |
| C | 独立vcan电控端+实际视觉程序，握手/控制/停止/线字节对照 | 通过，见下列记录 |
| D | 物理CAN与真实电控固件，断线、复位、方向及超时 | **待验收** |
| E | 真实相机+机械闭环、限位/限速/发射互锁 | **待验收** |

## 实际执行的命令与证据

在上述工作区执行（命令成功返回码0，除回归脚本内部预期的timeout外）：

```bash
make -C /home/irm/iRM-Vision-2026/build relative_servo_test relative_aim camera_test -j4
/home/irm/iRM-Vision-2026/build/relative_servo_test
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current_replay.yaml -dry-run -n=60 -csv=/tmp/current_resume.csv
unshare -Urn sh -c 'ip link add vcan_test type vcan && ip link set vcan_test up && python3 /home/irm/iRM-Vision-2026/scripts/vision_cboard_sim.py --output=/tmp/vision_final_vcan'
LOG_DIR=/tmp/vision_resume_regression bash /home/irm/iRM-Vision-2026/scripts/run_local_tests.sh
```

- 编译/单测证据 `/tmp/vision_final_build.log`、`/tmp/vision_final_unit.log`：
  三个目标Built，128 checks/0 failures。
- 回放证据 `/tmp/current_resume.log`、`/tmp/current_resume.csv`：60帧、55有效，
  检测均值26.8ms、观测年龄p50=27.4ms、p95=31.3ms；明确不构造CBoard，发布计数0。
  当次回放与第一次vcan并行运行，时延不作为独占性能基线。
  额外执行相同回放配置 `-dry-enable=0 -n=12` 与 `-dry-color=red -n=12`，
  两者均12帧0有效、rc=0（`/tmp/vision_disabled.log`、`/tmp/vision_red.log`），无CAN发送。
- 最终vcan证据 `/tmp/vision_final_vcan_result.log` 与
  `/tmp/vision_final_vcan.{log,csv,json}`：VCAN PASS；蓝使能阶段42个有效帧；
  整场162图像帧，83有效角差帧写入SocketCAN、71停止帧、3 ACK、0发送失败；
  接受发布84，不要求等于实际发送83（异步覆盖/退出可丢弃尚未发送样本）。
  所有实际有效帧角差在CSV成功发布序列中按序匹配；停车全0、shoot=0；
  控制超时1次；旧session拒绝4次；畸形CONTROL拒绝1次；SIGINT正常退出停止被独立端接收。
  最后补充原始发送ID/接收帧标志检查后，再次完整编译与vcan复验同样rc=0：
  `/tmp/vision_delivery_vcan_result.log`、`/tmp/vision_delivery_vcan.{log,csv,json}`。
  该次158帧、83接受发布、82有效角差发送、69停止帧、2 ACK、0发送失败；
  蓝使能阶段42有效帧，全部6组PASS（含线字节/CSV及SIGINT停止）。
- 回归 `/tmp/vision_resume_regression.log`：5 PASS、0 FAIL、2 SKIP。
  planner/camera/dm为限时运行（内部rc=124，日志标记符合脚本标准），检测bench与传统矫正rc=0。
  auto_aim_test缺DISPLAY/Wayland/xvfb；ORT与cv::dnn A/B可用后端不足2个，明确未通过该对比项。
  原有回归二进制未全部重建，不将此结果解释为全仓重新编译通过。

## 已覆盖边界

- NOT_READY/READY/版本不兼容；同会话HELLO幂等；活动会话切换停止优先。
- enable停车/恢复、红蓝覆盖、旧generation与早于控制变更的缓存图像拒绝。
- CONTROL sequence回绕与倒序拒绝；wait_control同样500ms超时。
- 畸形DLC、类型、版本、保留位与非法停车颜色fail-closed；合法旧session仅拒绝。
- 观测过期独立停止；无效观测绕过限频；停止失败重试；退出停止优先于ACK。
- 单测用阻塞注入sender验证：资格检查→写入事务期间控制变更等待，之后停止先发送。
- 配置坐标/协议分离；合法0x000不被替换；lo与未知接口不冒充vcan；NaN超时拒绝。
- 几何候选回退使用不同目标编号，断言选中第二候选而不是仅检查同一目标ID。

## 明确限制与待补验收

1. vcan不验证物理CAN波特率/ACK/仲裁拥塞/总线关闭，SocketCAN写入成功不是电控已执行。
2. 无真实CBoard、电机与手眼标定；保持camera_optical；ID confirmed保留false。
3. 停止失败由单测注入；未实测物理发送失败、硬件复位及TX队列残留。
4. 接收队列容量64、溢出fail-closed已实现，尚无专项洪泛压力/饥饿/时延上界验收；
   未执行TSAN/ASAN，不宣称已证明全仓线程安全。旧CBoard公开遥测字段仍由历史调用方负责同步。
5. 工业相机pop等待已受限50ms，SDK析构与文件解码不提供硬截止；真实相机拔插/SIGINT待D/E验证。
6. session没有进入角差线格式，已进入内核/硬件的有效帧不可撤回。电控必须本地门控、
   停车清缓存和看门狗；若要求线上无旧会话帧，需另行升级协议。
7. 默认不整形测量。电控不得直接套用旧文档的逐帧累加控制律，也不能依赖视觉死区/步长限幅保护机械。

接板前逐项执行同目录 `vision_cboard_handoff.md`；D/E通过前不作“实机闭环完成”结论。