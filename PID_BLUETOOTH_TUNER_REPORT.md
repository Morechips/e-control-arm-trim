# PID BLUETOOTH TUNER RESULT

> 历史实现记录：本文提到的 13 字节摇杆控制帧已被 `README.md` 中的 25 字节独立 short 控制协议取代。本文的 PID 5 字节短帧仍有效。

## 实际工程检查

- Bluetooth UART：**USART6，PC6 TX / PC7 RX，9600 8N1**；未修改引脚、波特率或启动改名流程（AIOTCAR）。
- RX：`HAL_UART_Receive_IT()` 单字节中断 → `HAL_UART_RxCpltCallback()` → `Bluetooth_RxCallback()` → `SerialRx` 的 256 字节环形缓冲 → 前台 `Bluetooth_Process()`。
- 现有摇杆为 13 字节 ValuePack，五个小端 short 依次为 RESERVED、BRAKE、DISABLE、JOY_X、JOY_Y。原 ENABLE 已改为保留位，填 0，兼容旧值 1；帧长度、原校验、按包交付和失联保护不变。
- 控制器：`Core/Src/heading_control.c` 的 `Heading_Update()`。原来是航向 **PD**，默认 Kp=1.0、Kd=0.2，无积分项；不是轮速反馈 PID。
- 原目标为 JY61 归零后的 0°，actual 为 JY61 yaw，error 为原 `h.yaw_error`，output 为原 `h.omega_correction`。
- `Heading_Update()` 按原主循环/状态机调用，不是独立固定周期定时器；`Car_Control_Process()` 每轮可调用两次。电机指令节拍仍为 `CAR_CONTROL_PERIOD_MS=20ms`，JY61 DMA 服务仍为 5ms。100ms 仅用于新增蓝牙调试输出。
- USART1 原有非阻塞日志队列 `Debug_Log()/Debug_Process()` 保留。工程未配置 printf 串口重定向，newlib-nano 也未显式启用 `_printf_float`。

## 参数与控制行为

唯一参数集位于 `heading_control.c`：

```text
Kp = pid.kp，默认 HEADING_KP = 1.0
Ki = pid.ki，默认 HEADING_KI = 0.0
Kd = pid.kd，默认 HEADING_KD = 0.2
```

通过 `Heading_GetPID()` 只读访问，`Heading_SetPID()` 修改。调参器没有独立的增益副本。
参数只存 RAM，重启恢复上述默认值；不写 Flash/EEPROM。按钮加减后下一次 `Heading_Update()` 使用新参数。

默认 Ki=0 时保持原 PD 计算、浮点舍入、5°/2°纠偏滞回、±10 输出限幅与原安全状态机。
启用 I 后，使用实际经过的毫秒数积分，不按函数调用次数积分，同一 tick 重复调用不重复累积。
积分贡献被限制在原 ±10 输出范围内，并在输出饱和时阻止继续同向积累。
手动转向、禁止运动/刹车、IMU 故障、等待归零和 HOLD 都清除积分；改变 Ki 也清零积分。
调用间隔超过 JY61 超时窗口时不累积该间隔，以免长暂停产生积分突变。

增益减到零后不再变负；不设置猜测的增益上限，拒绝 NaN/Infinity。
极大有限增益的 PD 运算使用 double 后备路径，避免 float 溢出产生无效输出；最终仍由原输出限幅约束。

## 手机按钮 Commands

已按用户确认采用完整 HEX 帧。每按一次发送一次，关闭附加换行和自动重复发送。
格式沿用 ValuePack 的帧头、累加校验和帧尾，但 payload 只有一个小端 short：

```text
A5  cmd_low  cmd_high  checksum  5A
checksum = (cmd_low + cmd_high) & 0xFF
```

| 命令 | 按钮 | 完整 HEX 帧 |
| --- | --- | --- |
| 1 | P+ | `A5 01 00 01 5A` |
| 2 | P- | `A5 02 00 02 5A` |
| 3 | I+ | `A5 03 00 03 5A` |
| 4 | I- | `A5 04 00 04 5A` |
| 5 | D+ | `A5 05 00 05 5A` |
| 6 | D- | `A5 06 00 06 5A` |
| 7 | COARSE | `A5 07 00 07 5A` |
| 8 | FINE | `A5 08 00 08 5A` |
| 9 | PID QUERY | `A5 09 00 09 5A` |
| 10 | DEBUG ON | `A5 0A 00 0A 5A` |
| 11 | DEBUG OFF | `A5 0B 00 0B 5A` |

不能只发裸字节 `01` 或文本 `1`。原 13 字节摇杆包继续发送，调参包可以在完整摇杆包之间插入；不要把两个包的字节交叉拼接。
短帧与合法摇杆帧的前缀不冲突：短帧第 5 字节为 5A，摇杆帧同一位置是 BRAKE 高字节，合法值为 00。
只在帧边界解码，合法摇杆 payload 内出现调参样式的字节序列也不会调参。

调参包不修改任何 BRAKE/DISABLE/X/Y 值，不刷新控制包序号或 500ms 失联计时，也不计入电机重新使能所需的连续归中包。
调参流量不能作为小车运动保活；操纵车辆时仍需原有周期摇杆包。
不完整帧按原 100ms 字节间隔规则丢弃；处理时已超过 500ms 的调参帧被忽略，非法命令、校验错误不会改参数。

## 粗调、细调及回包

| 模式 | P 步长 | I 步长 | D 步长 |
| --- | --- | --- | --- |
| COARSE（默认） | 0.1 | 0.01 | 0.01 |
| FINE | 0.01 | 0.001 | 0.001 |

命令 1–8 操作后由统一的 `PID_Tuner_PrintParameters()` 排队回复，命令 9 仅查询。
下列是格式示例，不是实物回包记录：

```text
PID P=1.100 I=0.000 D=0.200 STEP=COARSE
PID P=1.100 I=0.000 D=0.200 STEP=FINE
PID DEBUG=ON
PID DEBUG=OFF
```

手机接收区使用文本显示。所有输出以 CRLF 结尾，不发送结尾 NUL。

## 实时 Debug

默认关闭，命令 10 开启，11 关闭。独立 `PID_Tuner_Process()` 位于主循环原安全与电机处理之后，100ms 检查一次：

```text
CTRL T=0.0 A=6.0 E=-6.0 O=-6.0 P=1.000 I=0.000 D=0.200
```

字段映射：

| 字段 | 实际值 | 单位/说明 |
| --- | --- | --- |
| T | `h.target` | 原固定目标 0° |
| A | `h.actual` | 最近一次控制读取的 JY61 yaw，° |
| E | `h.yaw_error` | 原环绕到 ±180° 的航向误差 |
| O | `h.omega_correction` | 限幅后的航向修正量，轮等效 RPM；不是手动转向或电机反馈速度 |
| P/I/D | `pid.kp/ki/kd` | 当前 RAM 增益 |

退出纠偏时 O 为 0。IMU 失效时 A/E 可能是最后一次样本，不能凭这些数值判断 IMU 有效；原 USART1 日志仍提供 IMU_VALID/HEADING_STATE/FAULT。

### 非阻塞发送与长度保护

- 新模块独占启动 AT 交换之后的 USART6 TX，使用 `HAL_UART_Transmit_IT()`，没有新增 DMA、阻塞 UART 发送或 HAL_Delay。
- TX 完成回调仅清除 active 标志，所有格式化、排队、重试调度都在前台。发送缓冲在完成前保持不变。
- 参数回复使用 8 条 × 192 字节队列，优先于周期 CTRL；队列满丢弃新回复并增加 `replies_dropped`，参数修改仍立即生效。突发按钮操作后可发送 9 查询最终值。
- CTRL 不排队，串口忙/有参数回复时跳过本次，`debug_skipped` 累计跳过次数；不补发历史样本。
- 在通常数据长度下，9600 8N1 可以容纳 100ms 一行输出；按钮密集操作或行变长时，实际 CTRL 间隔会变长。
- DEBUG OFF 后不再生成 CTRL，但已经开始发送的一行可能继续传完。
- HAL_BUSY/ERROR 时保留尚未开始的参数回复供后续调度重试；已启动 TX 超过 500ms 未完成则只中止 TX 并计入 `tx_errors`，不停止 RX。
- 使用放大整数 + 整数 `snprintf` 输出小数，没有启用 float printf。非常大的有限值使用科学计数法，避免缩放整数溢出；这不限制存储参数。
- 固定缓冲、长度检查、无动态内存；不能因为调参或打印解除刹车锁、改变使能状态或刷新运动看门狗。

## 验证结果

```text
PID negative protection: PASS
Brake safety logic preserved: PASS
Motor UART5 preserved: PASS
Existing Bluetooth joystick control preserved: PASS
Build: PASS
Errors: 0
Warnings: 0
HARDWARE TEST: NOT PERFORMED
```

实际执行：

```powershell
./tests/car/run.ps1
cmake --build cmake-build-debug --parallel 4
```

主机 HAL 仿真全部通过，覆盖普通整车、PD10、麦轮测试模式、航向测试模式、原串口桥、原 JY61/归零/航向控制。
新增测试覆盖全部 11 命令与字节内容、默认与粗细步长、负值保护、参数查询、RAM 重置、参数即时生效、积分时间与抗饱和、100ms 输出及关闭、tick 回绕、分包/粘包/混合包、坏校验/超时/无效命令、合法摇杆包内嵌命令样式、UART 错误、TX 持有缓冲寿命、队列溢出与 TX 卡住恢复。
整车模拟还验证 DEBUG ON 下原 20ms 电机输出节拍，调参不能解除刹车锁或延长失联期限，TX 队列满时仍能处理立即刹车。

ARM GCC 正常固件编译链接通过，启用 `-Wall -Wextra -Werror`；Flash 29480B，RAM 13704B。
产物为 `cmake-build-debug/stm32f407_bt_oled.elf/.hex/.bin`。
Keil 已登记新源码/头文件并检查 XML，本次未执行 Keil 编译。
未烧录、未读取真实蓝牙回包、未验证实车 PID 效果或物理控制周期抖动。

## 文件清单

新增：

- `Core/Inc/pid_tuner.h`
- `Core/Src/pid_tuner.c`
- `tests/car/pid_tuner_cases.inc`
- `PID_BLUETOOTH_TUNER_REPORT.md`

修改：

- `Core/Inc/heading_config.h`
- `Core/Inc/heading_control.h`
- `Core/Src/heading_control.c`
- `Core/Src/bluetooth_driver.c`
- `Core/Src/uart_bridge.c`
- `Core/Src/main.c`
- `CMakeLists.txt`
- `MDK-ARM/stm32f407_bt_oled.uvprojx`
- `tests/car/test_car.c`
- `tests/car/test_heading.c`
- `tests/car/test_usart2_bridge.c`
- `tests/car/run.ps1`
- `README.md`

电机协议/映射、车体安全状态机、USART1 日志模块、GPIO/MSP、JY61 驱动、OLED、ZL-IS2 驱动均未修改。
