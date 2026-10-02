# v4.5 蓝牙与机械臂状态诊断

2026-10-01。原 v3 已由用户 Flash 校验确认烧入；旧协议能控制是兼容设计，不能据此判断烧错。原 v3 要求 BYTE0=84，但用户的“蓝牙调试器”没有找到设置此值的选项。本次修复接受默认 0 和旧 84，实际手机字节仍需实测确认。

当前目录为 v4.5：三个固定参考动作只发送 000～002，夹爪显式控制；抓人质旧姿态待重录，见 `ARM_FIXED_ACTION_V4_5.md`。按住连续微调、关节范围、箱体包络检查和 L2=(82+87.5)/2=84.75 mm 沿用 v4.4。当前行为见 `ARM_JOG_V4_TEST.md`。诊断脚本显式打印 `read_memory` 返回值，并提供类型化机械臂状态字段，地址/宽度来自匹配的 ELF。

若板上为 v4.4，追加 `-FirmwareDirectory rollback_v4_4_20261002`；若板上为 v4.3，追加 `-FirmwareDirectory rollback_v4_3_20261002`；若板上为 v4.2，追加 `-FirmwareDirectory rollback_v4_2_20261002`；若板上为 v4.1，追加 `-FirmwareDirectory rollback_v4_1_20261002`；若为 v4，追加 `-FirmwareDirectory rollback_v4_20261002`；若为 v3.1，追加 `-FirmwareDirectory rollback_v3_1_20261001`。默认读取当前目录 v4.5 镜像；验证不匹配则拒绝读状态并恢复运行。不可沿用之前手写的 RAM 地址。

## 保留现有页面

截图的类型为 8 bool、1 byte、1 signed short，int/float 为 0，总包长 7 字节；顺序正确。byte 只是保留字段，不设控件，不需设置 84。short 输入先置 0。

8 个 bool 按截图下标绑定：0 抓球前、1 抓人质前、2 放球前、3 微调、4 夹紧、5 松开、6 查询、7 停止。名称不参与协议；`start` 的下标 6 是查询，不是运行授权。

按钮按下发 True，松开发 False；每次发完整包，确保松开也发出。若只能周期发送，可用 100 ms，按住至少一个周期。请在实际发送/控制状态测试，布局编辑状态的操作不能证明已经发包。先全部松开，再点查询 3 次，每次松开后再点；查询不发送舵机运动。

## 烧入修复版及校验

在已确认的板卡、接线和安全条件下，由用户运行：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\flash_firmware.ps1" -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\verify_firmware.ps1"
```

新 HEX 哈希见 `FIRMWARE_SHA256.txt`。`CURRENT_FIRMWARE_VERIFIED` 说明板上与当前目录的参考 HEX 匹配。目录更新后，原 v3 的校验结果不能代替新版本校验。

## 无需新增串口接线的读取

保持停车，先松开全部按钮并等稳定，暂停手机周期发包，再运行现有 Horco 烧录器的读取脚本；读完再恢复手机发送。否则 halt 期间仍收包可能引发 UART 溢出，恢复后需要重新固定动作初始化：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1"
```

脚本先验证 Flash 与当前 HEX 一致，再按本次 ELF 符号地址读取 RAM。它不擦写 Flash、不复位、不发送舵机命令；会短暂暂停 MCU，最后恢复运行。读取期间不要操作底盘。末尾 `BT_DIAGNOSTICS_DONE` 表示读取完成；本脚本的硬件读取仍待用户运行。

数值直接跟随标签：如 `BT_TRIM_PRESSES=0xa` 为 10。脚本另输出 `ARM_*_DEC` 十进制关节/状态和 `ARM_*_MM` 浮点毫米值。原始包只取 `BT_RAW_LAST_LENGTH` 指定的前 N 个字节，后面的旧内容不用看。读取前先松开 wt 并等待稳定。

关键字段：`ARM_REFERENCE_VALID`、`ARM_STATE`、`ARM_EST_P`、`ARM_TARGET_P`、`ARM_P0/P1/P2_LIMITS`、`ARM_MODEL_MIN/MAX_MM`、`ARM_ENABLED_MIN/MAX_MM`、`ARM_OFFSET_MM`。v4 另有 `ARM_JOGGING` 和 `ARM_REQUEST_ERROR`，最近拒绝不会误当作运行故障。状态 2=移动、3=稳定等待、4=预计完成、7=故障；请求结果 2=忙碌、3=缺参考、4=超范围、5=轨迹无效、7=调度超时、9=按住包超时。

| 标签 | 说明 |
|---|---|
| BT_RX_BYTES | USART6 接收的字节总数 |
| BT_TRIM_VALID_FRAMES | 校验通过的微调测试包总数，包含松开/重复包 |
| BT_TRIM_PRESSES | 新页非零按钮上升沿次数；动作被互锁拒绝也计入 |
| BT_INVALID_FRAMES | 解析失败次数，包含流中的错误前缀 |
| BT_BASE_VALID_FRAMES | 旧底盘协议有效包数，独立新页不会刷新它 |
| BT_UART_RECOVERIES | UART 错误/接收缓存溢出的恢复次数 |
| BT_TRIM_LAST_LENGTH | 新页为 07，旧四 short 测试页为 0b |
| BT_TRIM_LAST_PRESS_BITS | 最近非零新按钮位图；查询是 40，抓球前是 01 |
| BT_RAW_LAST_LENGTH / FRAME | 最近被记录的原始包长度和字节，可能是按钮松开包 |

读取两次、在中间点查询，比较计数：

- RX_BYTES 不增加：检查当前工程是否确实发送、蓝牙连接与接收线路。
- RX_BYTES 增加，TRIM_VALID_FRAMES 不增加：看原始字节与错误计数，核对包头尾、长度、校验、byte 和 short；不继续猜控件设置。
- TRIM_VALID_FRAMES 增加，TRIM_PRESSES 不增加：通常是只发送了按钮 False 或按下值未绑定，核对按钮发送行为。
- 两者都增加：协议已经识别按钮；若固定动作仍不执行，进一步检查忙碌/故障/旧调参占用/车辆非空闲等拒绝原因。

v3.1 夹爪接口已修正独立页面缺少底盘保活时被 READY 条件挡住的问题：READY、OFF、WAIT_CENTER、BRAKE_LOCK、LINK_LOST 可在电机空闲且无故障时操作夹爪。运动、启动中及故障状态仍拒绝，车辆物理停车继续由外部确认。

将完整读取输出贴回即可。若有 `.pro` 文件，也可提供正在测试的工程和队友能控制的工程，用于核对实际字段绑定。

## 已连接调试 UART 时

固件调试输出为 USART1 PA9 TX，115200、8N1；仅在已经确认这路接线时使用，不把 CMSIS-DAP 显示支持 UART 等同于已连上 STM32 UART。串口接收会看到：

```text
ARM TRIM FW=V4.5 L2_MM=84.75 WT=HOLD_JOG BYTE0=0_OR_84
[BT RX] bytes=... test=... invalid=...
[BT RX] base=... test_len=... recover=...
```

原始 `BT RAW` 日志、解析拒绝参数和 BENCH/TRIM 回复也镜像到该端口。日志使用有界非阻塞队列，拥堵时可能丢日志；RAM 计数不依赖日志输出。原始包日志有节流，不能替代完整抓包。

按钮查询包（dx=0、byte=0）应为 `A5 40 00 00 00 40 5A`；松开为 `A5 00 00 00 00 00 5A`。必须先确认通信，再做“固定动作 → +2/−2 微调 → 夹紧/松开”的实物验收。
