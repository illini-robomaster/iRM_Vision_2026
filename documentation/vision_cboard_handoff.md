# 电控交接速查

协议正文：同目录 `vision_cboard_protocol.md`；验证范围：`vision_cboard_acceptance.md`。

## 接板前共同确认（未完成项不得置 ID confirmed）

- [ ] 4个标准CAN ID（角差、HELLO、ACK、CONTROL）、方向、波特率、终端电阻、接口名。
- [ ] 当前 `camera_optical` 原点/参考轴；yaw左正、pitch上负；rad与int16 BE ×1e-4。
- [ ] 没有手眼标定，不按gimbal/世界绝对角执行，不将候选ID当作实机分配。
- [ ] shoot恒0；先禁用发射机构、电机输出或机械脱开，再验抓包。
- [ ] 电控本地使能门、有效测量看门狗、限位/限速/限加速度、停车清缓存已实现。
- [ ] 角差无session：停车后已经在总线上的旧帧可能到达，电控必须本地拒绝。

## 电控最小流程

1. 上电禁止运动，生成新uint32 session，周期HELLO `01 01 session_BE 00 00`。
2. 只接受回显session且status=0的ACK；NOT_READY继续重试，版本不兼容报错。
3. 每80ms发CONTROL `03 01 session_BE flags seq`，seq每帧递增（255→0）。
   flags：红停车02/启动03，蓝停车04/启动05；停车不能写颜色0。
4. 收角差：先校验标准帧/DLC/保留位；本地未使能/会话未就绪则忽略有效角差。
   八字节全0立即清有效测量。有效帧将yaw/pitch解码后交电控自己的受限控制器。
5. 超时（建议按现场周期制定约100–120ms）清有效测量，绝不重复累加上一帧。
   视觉掉线/重启、CONTROL超时或畸形帧后，重新HELLO并重新使能。

## 视觉侧操作顺序

仓库路径 `/home/irm/iRM-Vision-2026`。先完成纯软件验收，再在
`/home/irm/iRM-Vision-2026/configs/relative_current.yaml` 填写批准ID和
`vision_can_ids_confirmed: true`。首次接板仅观察抓包，不允许运动。

```bash
cd /home/irm/iRM-Vision-2026
make -C /home/irm/iRM-Vision-2026/build relative_servo_test relative_aim -j4
/home/irm/iRM-Vision-2026/build/relative_servo_test
/home/irm/iRM-Vision-2026/build/relative_aim -c=/home/irm/iRM-Vision-2026/configs/relative_current.yaml -csv=/tmp/board.csv -stream=8080
```

再依次验证：NOT_READY→READY→蓝启动→停车→换红→恢复蓝→停止CONTROL心跳→重新握手→
拔断相机/通信→SIGINT。各阶段独立抓包并核对CSV量化值、停止全0、shoot=0；先确认左右/上下
方向，再低速受限闭环。Ctrl+C退出；不要用默认SIGTERM的timeout强杀相机。

故障定位：无ACK先查方向/ID/DLC/版本；一直NOT_READY查模型、几何与新鲜图像；
有ACK无有效角差查CONTROL的session/flags/seq、颜色、图像年龄与PnP；
CSV有观测而总线没有看publish_ok、generation和tx_fail。不要靠关闭超时保护“修复”。