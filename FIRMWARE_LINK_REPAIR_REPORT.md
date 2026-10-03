# 固件链路精简与修复实施记录

> 当前模块和输入边界已更新，见 `SERVO_INPUT_LAYER_REPORT.md`；本文保留原阶段验收数据。

> 本文为 2026-09-30 的历史验收记录；2026-10-01 的公共 UART、队列及枚举变化见 `UART_QUEUE_SERVO_REFACTOR_REPORT.md`，当前代码结构以 README 为准。

日期：2026-09-30。依据用户提供的《固件链路精简与修复计划（v2）》及本会话三项确认实施。接手时工作区已有该计划的部分修改及其他遥控航向改动；保留原有改动，没有撤销其他工作。改动保留在工作区，未创建提交、未烧录、未连接或使能硬件。

## 已确认的边界

- A4：仅 boot_enable_pending=1 时 STOP/BRAKE 保留上电待使能请求并等待中立；使能成功后立即进入正常刹车锁优先级。DISABLE 始终急停并失能。测试使用真实 STOP 控制帧，覆盖使能前、使能后、两帧内归中恢复及原失联/故障行为。
- C4：发送期间指令成功入队或替换最新待发项返回 OK；HAL 拒绝且未入队返回 BUSY，ServoRemote 的 dropped 加一。
- E3：旧 Bluetooth_SetExtended/IsExtended/SetTextHandler/GetArmControl/ArmIsConnected/GetArmSequence 本次保留，因为历史 arm_tuner.c 与 tests/arm_bt 仍引用。
- 启动键只输出一次 [START] press，不绑定任务动作。路由/任务状态机不接入正式主循环。蓝牙保持 9600，PWM 与 2000 ms 时间保持计划指定值。C5 可选的其他日志 writer 替换未做，当前 Flash 使用率 8.74%，没有容量需求。

## 实施结果

| 工作流 | 软件结果 |
| --- | --- |
| A 启动 | 删除 AT 轮询/固定等待与无人读取状态；Bluetooth_ProvisionName 默认关闭，开启只发一次。Board_InitDisplay 与硬件初始化拆分，先打开 RX；I2C/OLED 失败非致命。OLED 一次 50 ms 探测，无 HAL_Delay、无启动整屏同步发送，首屏走 IT。首帧记录解析时刻，READY 记录时间，等待中立每秒提示。 |
| B MaxiCam | 独立二维码通知计数与锁存；重复通知仍通知 ActionFSM。请求先记录，无二维码只提示一次；收到通知后发送，快速最多 3 次/20 ms，之后每 200 ms 再试。成功后 50 ms 入流按到达时间丢弃，残包静默超过 200 ms 主动发布失效。UART 错误清 ORE，保留原 Serial_Recover。宏可覆盖且无嵌套错误。 |
| C 预设/协议 | 唯一表 Core/Src/servo_pose_table.c，一行包含中文名称、PWM、时间、路数及中文动作注释。日志查表。所有组合带 T2000，删除 Untimed。发送路径无 snprintf/vsnprintf/363 B 栈缓冲，保留历史任意 ID/24 路能力及 GAP 的单路组合格式。 |
| C4 异步 | USART3 IRQ 优先级 3，TxCplt 分派至 ZLIS2；拥有当前帧和一格最新待发帧，提交后调用方可修改原存储。IsIdle 只表示 UART 链路空闲。queued HAL 发送失败不重放可能部分发送的命令。 |
| D 按键 | 正式配置仅 PD10 启动键，20 ms 消抖/一次性事件；PE0、PC1、PE4、PB8、PD11/14/15 的额外输入与 GPIO 初始化编译期移除。台架开关恢复原输入。麦轮/航向/PD10 测试模式要求测试输入开启。 |
| E 构建/文档 | GCC/Keil 移除 x42.c 构建项，源码/历史测试保留；死诊断宏删除；旧 arm 三文件添加历史说明；双次 Car_Control_Process 保留并说明安全顺序。README PWM、带 T、相机请求、启动/按键、AGENTS 构建清单已同步。 |
| 任务 0 | 每套 run.ps1 支持 CAR_TEST_BUILD_DIR，默认 build，使用 build-local 避开旧权限；可执行文件调用支持绝对路径/空格。build-local 已忽略。未修改原 build 权限或 stale snapshot。 |

C2/C4 的缓冲实现直接构建到驱动拥有的两帧中，无需额外 62 B 或 363 B 组合帧栈副本；每帧保持 362 B 上限，兼容历史 24 路 API。这一异步生命周期需要静态 RAM，不能满足计划中“RAM 持平”的预期。

## 验证

以下命令均通过，应用 -Wall -Wextra -Werror；主机编译另含 -pedantic：

```powershell
$env:CAR_TEST_BUILD_DIR = 'build-local'
powershell -ExecutionPolicy Bypass -File tests/car/run.ps1
powershell -ExecutionPolicy Bypass -File tests/vision/run.ps1
powershell -ExecutionPolicy Bypass -File tests/route/run.ps1
powershell -ExecutionPolicy Bypass -File tests/zlis2/run.ps1
powershell -ExecutionPolicy Bypass -File tests/servo/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_bt/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_kinematics/run.ps1
cmake -S . -B build-local -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build-local --parallel 4
cmake -S . -B build-local/bench -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCAR_TEST_INPUTS_ENABLE=ON
cmake --build build-local/bench --parallel 4
git diff --check
```

补充验证：

- CAR_TEST_BUILD_DIR 指向绝对路径 `D:\GDUTcom\stm32\build-local\host output`，zlis2 脚本通过。
- 外部定义 VISION_UART_BAUD_RATE=115200U 时 MaxiCam 编译及测试通过。
- 历史 x42 手工 gcc -std=c11 -Wall -Wextra -Werror -pedantic 编译与测试通过。
- CMake 拒绝测试输入关闭但开启航向模式；拒绝同时开启航向/麦轮模式。
- 正式 ELF 包含 USART3_IRQHandler/ZLIS2_TxCallback，无 Board_*Button/Board_PD10/x42 符号；编译清单无 x42.c。
- 舵机逐字节测试共 144 个检查：所有表项、AIM/RST/GAP 字符串、边界、HAL 错误、延迟 TC、当前帧不变、待发复制/替换、错误 UART 回调、BUSY 与重新初始化状态。ServoRemote 两种输入开关覆盖日志名称和 BUSY dropped。
- 新增 OLED 测试：一次 50 ms 探测、缺屏/指令失败、无启动延时或整屏同步发送、异步与坏总线恢复；新增生产/台架激光输入测试。
- MaxiCam 覆盖无通知不发送、重复请求不打破间隔、零 tick、3 次快速失败后慢速自恢复、50 ms 旧流延迟消费、200 ms 无新字节残包失效、缓冲完整包、时间回绕及载荷 0x80 不误判。

本地日志位于 build-local/*-validation.log，生成目录不进入提交。

## 资源实测

基线是接手时已部分实施计划的当前工作区构建，非 git HEAD 或历史原版。

| 镜像 | Flash/B | RAM/B |
| --- | ---: | ---: |
| 接手基线 | 44948 | 15704 |
| 本次正式配置 | 45804 | 16448 |
| 正式配置增量 | +856 | +744 |
| 台架输入配置 | 47080 | 16496 |

新增状态、诊断、中文名称及中断发送使整体 Flash 增加；两份 TX 帧共 724 B，加状态等使 RAM 增加。预设是 const，仍在 Flash。不能宣称整体 Flash 下降或 RAM 持平。

同正式构建参数附加 -fstack-usage：ZLIS2_SetServoValues 单函数栈 32 B，ZLIS2_SetServos 24 B，AppendServo 24 B，补零 writer 40 B，扩展命令 builder 72 B。该数据不是整个固件或中断嵌套的栈峰值，也不是主循环耗时测量。

## 待台架验收

软件验证已完成，以下计划中的物理指标未验证：

1. 安全架空四轮，复位 10 次，观察 [BT] first frame 与 [CAR] READY：首帧解析 ≤250 ms，READY ≤2 帧；复核使能前 STOP 与使能后 STOP/BRAKE、DISABLE 行为。
2. 无二维码时 SHOT 只延迟请求；给出二维码后 UART4 发 0x00，再切对象发 0x01，全程不需要板断电。协议无应答，软件成功只能说明 UART 接受发送。
3. 示波器/DWT 检查四舵机发送不再等待 5.38 ms 线时，核对实际舵机动作、PWM、T2000；不能将“中断提交”直接当作主循环耗时为零。
4. PD10 按下只产生一次日志、长按不重复；正式固件其余台架输入无响应。

未烧录或操作硬件，未自定启动任务语义，未修改蓝牙波特率或运动常量。
