# 队友重构核查

核查日期：2026-10-03。依据本地完整 Git 历史，审查远端 `e7404c592df92c7c5e7a9d09562e302ebf52e2b3`，主要比较 `3021961 → e7404c5`，并回看 `db9c1d6 → 3021961`。下文链接固定到被审查的历史提交，避免与本次机械臂微调接入混淆。这里是源码及调用链核查，不能据此判定实车效果。

**这次改动包括接口重构、控制行为调整和旧功能退役。核心方向是统一串口基础设施、集中舵机命令及姿态数据、拆出输入层，并改善遥控启动与航向控制。它没有把旧机械臂控制器变成通用运动模型，也没有完成自动任务的硬件接入。**

## 提交脉络

- [`3021961 repaired servo`](https://github.com/gpnu-in-jnds/e-control/commit/3021961)：相对 `db9c1d6`，正式构建和主循环移除 `arm_control/arm_tuner/arm_kinematics`，转向固定姿态直接发送；手机协议、姿态参数及遥控速度也有变化。旧机械臂退役早于后面两次“redesign”。
- [`3f7bf42 redesign servo saving method and UART`](https://github.com/gpnu-in-jnds/e-control/commit/3f7bf42f5d202fb9724f4eecc4debdfa371f7fd7)：引入共享 UART/发送队列、姿态表、`remote_heading`、启动键与可选 OLED，并修改相机模式管理。
- [`976efc6 redesign servo module`](https://github.com/gpnu-in-jnds/e-control/commit/976efc6bd842fa83d6965b073697295843c349ea)：进一步合并 Servo、拆出 `board_inputs`、车控改收中立输入快照。
- `9522ce0` 删除机械设计目录；`e7404c5` 合并该分支。大量 STL 删除造成很大的删行统计，不代表固件重写规模。

## 具体改变了什么

| 部分 | 性质与实际结果 | 历史源码证据 |
|---|---|---|
| Servo | `servo_code/servo_pose/servo_remote/zlis2_driver` 收敛为 `servo.c/h`：统一格式化、发送队列、预设表与状态码。`codes[][4]` 成为姿态数值来源；REFERENCE 只发三路。Servo 不读取蓝牙或 GPIO，适合其他调用方复用。 | [servo.c:132](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/servo.c#L132)、[servo.c:213](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/servo.c#L213) |
| 串口基础设施 | 各模块重复维护的 HAL 回调、缓冲及发送状态收拢到 `uart_driver` 和 `uart_tx_queue`。统一基础实现，但电机仍是 FIFO，舵机仍是当前帧加一个最新待发帧；日志、PID、桥接各有队列策略。**统一实现并不等于统一排队语义。** | [uart_driver.c:17](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/uart_driver.c#L17)、[uart_tx_queue.c:95](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/uart_tx_queue.c#L95) |
| 车控输入边界 | 车控由直接读取 Bluetooth 类型/getter，改收 `CarRemoteInput_t/CarLocalInput_t` 快照。UART 错误只在中断中标记输入失效，电机停止由前台处理；蓝牙负责发布快照和后段舵机分发，尚不是完全通用的输入调度框架。 | [car_control.h:10](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Inc/car_control.h#L10)、[bluetooth_driver.c:68](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/bluetooth_driver.c#L68) |
| 板级输入 | GPIO 初始化、采样、消抖、按下事件集中到 `board_inputs`；车控前采 PE0/PC1/PD10，舵机阶段采 PE4，辅助阶段采 PB8/显示按钮。Laser 接收手动请求，自己不再读 PB8。 | [board_inputs.c:25](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/board_inputs.c#L25) |
| 航向控制 | 新 `remote_heading` 管理共同原点与 FRONT/RIGHT/BACK/LEFT：转向后更新目标，普通松键和 STOP 暂停输出并保留原点。静止以 1°/0.5°、最多10 RPM纠偏；行驶以3°/1.5°、最多4 RPM叠加纠偏；≥10°、至少3个新采样且持续300ms才暂停平移对准。属于运动行为修改。 | [remote_heading.c:38](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/remote_heading.c#L38)、[heading_config.h:5](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Inc/heading_config.h#L5) |
| MaxiCam | 模式切换从立即发送改为记录期望值，收到独立 QR `0x80` 后后台发送；失败重试，切换后按到达时间丢弃50ms旧流，残包超时重同步。路线只提交模式请求，不再被一次 HAL_BUSY/发送失败卡住。发送成功仍没有相机应答确认。 | [maxicam.c:89](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/maxicam.c#L89)、[route_fsm.c:163](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/route_fsm.c#L163) |
| 启动与 OLED | 蓝牙/相机先启动接收，再初始化显示；OLED 取消100ms固定等待，单次50ms探测，首屏异步更新；I2C初始化失败返回状态，不进入 Error_Handler。蓝牙启动 AT 轮询也移除，默认不开启改名。 | [main.c:21](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/main.c#L21)、[ssd1306.c:116](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/ssd1306.c#L116) |

## 不是单纯重构的变化

1. **输入功能收缩。** JOY轴仍解析并校验，但不再驱动车辆；旧机械臂双摇杆和 `@ARM` tuner 没有正式调用链。当前主要使用方向按钮遥控，速度35 RPM；该速度在 `3021961` 已从100 RPM下调。[协议说明](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Inc/bluetooth_driver.h#L73)、[配置](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Inc/car_config.h#L13)。
2. **固定姿态数值变化。** 比较 `3021961` 与 `e7404c5`：TB_M三关节从 `1421/2071/812` 改为 `1356/1850/698`；BD_U从 `1717/2297/884` 改为 `1566/1896/673`；TH_C改为 `1667/1882/651`，原 `1684/2136/785` 另保留为 TH_PRE；放球夹爪从1800改为1200。不能把这批提交当作机械动作完全等价的文件改名。[旧配置](https://github.com/gpnu-in-jnds/e-control/blob/3021961/Core/Inc/servo_pose_config.h)、[新姿态表](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/servo.c#L213)。
3. **正式物理输入被门控。** 默认 `CAR_TEST_INPUTS_ENABLE=0`，只保留PD10启动键；它消抖后仅打印一次 `[START] press`，没有绑定比赛启动。PE0视觉、PC1射靶、PE4 AIM、PB8手动激光和显示按钮属于台架配置。[配置](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Inc/car_config.h#L27)、[事件处理](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/board_inputs.c#L47)。
4. **上电四轮仍自动使能。** `CAR_BOOT_AUTO_ENABLE=1`；上电使能后等待合法归中帧。启动键尚未控制这一行为。[car_config.h:46](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Inc/car_config.h#L46)。

## 还没有完成的接入

- `action_fsm/mission_fsm/route_fsm` **列在构建清单中，但正式 `main()` 没有启动或更新路线**。任务代码和主机测试存在，不等于自动比赛流程已经运行。[构建清单](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/CMakeLists.txt#L24)、[完整主循环](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/main.c#L30)。
- `mission_fsm.c:283` 的 `Servo_Start()` 只记录请求并清完成标志，仍有 TODO；没有调用新 Servo 硬件发送器。抓取、释放、救援抓取需要适配器执行，再经 `Servo_NotifyDone()` 通知；不能把这里理解为已完成机械臂动作。[占位接口](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/mission_fsm.c#L283)。
- 路线中的 `ACTION_TURN_LEFT` 仍是等待显式完成通知的安全占位；扫码段 `ACTION_QR_LEFT_TURN` 则已经调用实际左转控制，二者应区分。完整自动流程还需绑定启动、路段终点、任务动作，并与遥控取得互斥的运动控制权。[占位实现](https://github.com/gpnu-in-jnds/e-control/blob/e7404c592df92c7c5e7a9d09562e302ebf52e2b3/Core/Src/action_fsm.c#L218)。
- `e7404c5` 的 Servo 是协议发送和固定预设组件，没有笛卡尔轨迹、碰撞检查、姿态同步或运动完成反馈；旧机械臂模块虽保留源码/测试，已不在正式构建中。**本次合入的微调核心因此应保持独立，通过服务层接其 Servo，而不是恢复旧 arm_tuner 整条依赖链。**
