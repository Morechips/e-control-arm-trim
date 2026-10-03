# 单一 Servo 层与输入分发重构验收记录

日期：2026-10-01。按本会话确认的方案实施：Bluetooth 驱动直接调用功能接口，舵机协议一并合入 Servo，物理按键输入全部移出功能层。接手时其他未提交修改保持。`D:\GDUTcom\mytry` 未修改；本次未提交、未烧录、未操作硬件。

## 模块边界

- 舵机仅 `Core/Src/servo.c` / `Core/Inc/servo.h`：13 个预设、名称、计数、协议 writer、状态及共享队列实例合为一处。Servo 不包含 Bluetooth/GPIO/按键仲裁。状态 `ServoStatus_t` 数值 0～6 保持；数据统一为 `ServoCommand_t {id,pwm,time_ms}`。
- 删除 servo_code、servo_pose、servo_remote、zlis2_driver 的源/头及 servo_remote_config。旧 ServoPose_t、映射和 ZLIS2/ServoRemote API 没有兼容残留。历史 arm 的任意通道/原始指令及控制器扩展操作改用 Servo_*。
- 原枚举 0～8，REFERENCE/RST/AIM/TH_PRE 的 9～12、MAX=13/NONE=-1 保持。静态对照接手快照，13 行 ID/PWM/time 完全相同；REFERENCE 三路、动态 GAP 为 ID003。所有组合带 T，writer 无 snprintf，队列保持当前帧＋一个最新待发。
- Bluetooth 接收有效控制帧后调用 Car_Control_SubmitRemoteInput，只复制中立输入与序号/接收时间，不直接派发运动。PID 短指令仍直接调用 PID 接口。舵机映射/重连/边沿/优先级由 Bluetooth_DispatchServoActions 在原主循环后段调用 Servo。
- CarCommand_t/CarRemoteInput_t/CarLocalInput_t 属于车控接口；CarControl 不再依赖 Bluetooth 类型、Getter 或 GPIO 采样。500 ms 使用接收时间，不使用提交/日志时间。Car_Control_InvalidateRemoteInput 可在 ISR 立即置失效标志，本身不发电机命令；错误和 RX 溢出均立即通知此接口。
- 台架 PE0/PC1/PD10/PE4/PB8、启动键及按钮显示采样归 board_inputs；配置在 board_input_config。PE0/PC1/启动键/PE4 上电已按住须先释放；PD10 按住持续消抖后运行、释放立即停止；PB8 首次辅助轮询开始计时、释放立即撤销。
- Laser 仅维护自动/手动请求及 PC3 输出。Laser_SetManualRequest 不读 PB8；输出恢复浮空时的 PC3 INPUT 配置是输出行为，不是输入采样。

Servo 对外的调用接口包括 Init、SendPreset、SendGap、GetName、IsIdle、纯 GetFullCommand/FormatCommands，以及 SetChannel/SetCommands/SetValues/SendRaw/控制器扩展 API。公共 UART、队列算法及各配置没有重写。

## 主循环与输入规则

```
Bluetooth_Process -> 提交 CarRemoteInput / PID 接口
MaxiCam_Process
BoardInputs_ProcessControl -> 提交本地输入及启动键日志
Car_Control_Process -> 安全取消待发
Motor_Process
Car_Control_Process -> 同轮处理回复/故障
PID_Tuner_Process
BoardInputs_TakeServoAimPress -> Bluetooth_DispatchServoActions -> Servo
Debug_Process
Board_Process -> BoardInputs_ProcessAux（PB8/显示）
```

保留每轮最多一个舵机选择、PE4 最高优先级、未选中边沿消费后不补发、首帧 GAP 仅建立基线、离线参考短指令消费以及 BUSY dropped 统计。AIM 输入序号由 Bluetooth_GetAimSequence 提供。短 PID/参考指令不刷新车控保活。

车控仍拥有原运动、航向、视觉、安全状态机；输入提交仅记录数据，双次处理不重放物理按下事件。未改变低速值、波特率、任务接入或启动键动作。历史 arm_tuner 的旧蓝牙 Getter 作为仅测试例外保留，未进入正式固件。

## 验证

```powershell
$env:CAR_TEST_BUILD_DIR = 'build-local'
powershell -ExecutionPolicy Bypass -File tests/uart/run.ps1
powershell -ExecutionPolicy Bypass -File tests/car/run.ps1
powershell -ExecutionPolicy Bypass -File tests/vision/run.ps1
powershell -ExecutionPolicy Bypass -File tests/route/run.ps1
powershell -ExecutionPolicy Bypass -File tests/zlis2/run.ps1
powershell -ExecutionPolicy Bypass -File tests/servo/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_bt/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_kinematics/run.ps1
cmake --build build-local --parallel 4
cmake --build build-local/bench --parallel 4
git diff --check
```

应用保持 -Wall/-Wextra/-Werror；主机另含 -pedantic。GCC 正式与台架配置通过；Keil 仅验证源文件清单/XML，未执行 Keil 编译。

- 纯 Servo 协议测试不编译 Bluetooth，保留全部逐字节断言、13 个枚举、三路/四路缓冲边界、动态 GAP、任意通道/原始扩展命令及队列错误用例。144 个原检查通过。
- Bluetooth/Servo 测试已改名 test_bluetooth_servo.c，保留输入/优先级及准确串口字符串断言，覆盖测试输入 0/1。新增发布测试：短 PID/参考不更新车控快照；UART 错误和环缓冲溢出在解析前即通知失效，无动作调用。
- 中立车控测试不发送任何 Bluetooth 帧：通过接口完成启动、运动、STOP、仅标志的失效及真正停止、500 ms 边界；Bluetooth 序号全程为 0。既有 BRAKE/DISABLE、错误、故障、突发 STOP、转向与视觉回归保留，内部安全字段改为通过车控接口提交，不再篡改 Bluetooth 全局变量。
- 新增 test_board_inputs.c，覆盖 0/1 配置、20 ms 边界/回绕、单次事件、上电保持、PD10 立即释放、PE4 重按和 PB8 自动/手动重叠。GPIO 初始化与处理也经过真实板级输入代码。
- 静态检查：Servo 无 Bluetooth/GPIO；正式功能层无 Bluetooth Getter；输入 GPIO 读取只在 board_inputs/start_button；旧模块、接口及构建引用没有残留；13 行数值与接手快照一致。

GCC/Keil、README、AGENTS 与测试清单已同步。日志为 build-local/*-input-final.log；接手比对快照为 build-local/pre-servo-layer，均为本地生成目录，不提交。

## 资源结果

基线为本次接手工作区的已构建镜像，不是 git HEAD。

| 配置 | Flash/B | RAM/B |
| --- | ---: | ---: |
| 正式基线 | 46628 | 14856 |
| 正式重构后 | 46772 | 14904 |
| 正式增量 | +144 | +48 |
| 台架基线 | 47888 | 14904 |
| 台架重构后 | 48028 | 14960 |
| 台架增量 | +140 | +56 |

增加主要来自中立输入快照与板级状态；删除旧适配及命令 DTO 转换降低部分开销。上述为链接资源，不是硬件耗时或机械动作验证。

未烧录。硬件需复核 GPIO 接线/上电保持、发送时序、车控失联与急停；软件通过不等于机械停止、舵机到位或启动时限已达标。
