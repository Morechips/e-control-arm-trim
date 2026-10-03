# UART、队列与舵机枚举重构验收记录

> 当前模块和输入边界已更新，见 `SERVO_INPUT_LAYER_REPORT.md`；本文保留原阶段验收数据。

日期：2026-10-01。按本会话确认方案实施：统一 UART 收发、回调、队列算法与发送状态机，模块持有独立实例及策略。`D:\GDUTcom\mytry` 仅作参考，没有修改。保留接手时其他未提交改动。本次未提交、未烧录或操作硬件。

## 实现与接口

- `uart_driver.c/h`：UART_SEND 返回 HAL 状态，明确 blocking/IT 模式；UART_RECV 非阻塞，空读 HAL_OK/0，支持逐字节及 DMA 批次时间戳，UART_RxTick 提供最近读取批次时间。全局 HAL 回调仅由此模块定义，通过 handle 注册表派发。
- RX 启动、续接、错误恢复共用实现；USART2 原循环 DMA/TIM6 后端保留，JY61 使用完整原批次并保留批次时间，没有追加 1 KB 时间戳栈数组。桥接独占切换失败恢复原绑定，并保留流中断通知给原协议消费者。RX 错误不终止独立 TX。
- `uart_tx_queue.c/h`：UartTxQueue_t 共用静态缓存、复制、FIFO/latest、发送中状态、间隔、超时、错误及完成通知。Init/Submit/Reserve/Commit/Process/CancelPending/IsIdle/Complete 和查询函数为唯一队列操作实现。标签随当前帧保存，用于电机停止代次及计数。
- 半成品仅 Commit 后发布；发送启动期间的立即完成回调先延后退休，再依次发布 STARTED/COMPLETED，防止错配帧和标签。超时与 TC 交接受短 IRQ 保护；事件只做有界状态更新，电机日志格式化移到前台。无堆分配、无 363 B 组合指令栈缓冲。
- 模块原 Rx/Tx/Error 回调 API 仅作兼容转发；真实 HAL I/O 及全局回调路由没有重复实现。MaxiCam 二维码等待/模式重试、JY61 归零确认仍属于设备业务。

## 独立队列策略

| 模块 | 固定配置与行为 |
| --- | --- |
| 电机 | FIFO 15 待发＋当前，前台派发，2 ms 间隔，超过 20 ms 超时；启动失败保留队首并报告电机故障；急停取消待发速度/同步帧，当前帧继续到 TC；故障仅允许停止/失能 |
| 舵机 | 一个最新待发＋当前；立即提交、TC 续发；替换待发返回 OK；未入队拒绝返回 BUSY/错误；不新增动作等待、超时或重发 |
| PID | FIFO 8 待发＋当前；500 ms 边界包含等于；启动失败保留回复，abort 失败维持当前；周期诊断只在空闲时发送，失败不形成待发诊断积压 |
| 日志 | FIFO 15 待发＋当前；满丢新日志，启动失败保留队首，超过 200 ms 恢复 |
| 桥接 | 每方向 1023 字节总预算，含当前字节；满丢新字节，启动失败保留队首；初始化失败回滚已取得的串口 |

主循环调用顺序及两次 Car_Control_Process 未改变；队列不会在 ISR 为电机派发下一普通帧。蓝牙波特率、低速常量、PWM、时间和启动键语义未改变。

## 预设存储

`servo_code.c/h` 使用手写 ServoCode 与 `codes[ServoCode_MAX][4]`，每路 `{id,pwm,time}`。原 0～8 及 BEGIN/END 别名保持，额外 REFERENCE/RST/AIM/TH_PRE 为 9～12，MAX=13、NONE=-1。人质组依次是 TH_U、TH_C、TH_G、TH_L。名称、通道数与旧保护常量同文件存放；数值仅来自 codes。

GetFullCommand 为纯构造函数，复用 ZLIS2_FormatServos 的补零 writer；四路容量 63 B/发送 62 B，REFERENCE 三路容量 48 B/发送 47 B。所有命令带 T，当前值均 2000 ms。Servo_SendPreset 使用绑定的舵机队列，动态 GAP 保留 ID003 单路组合格式。

旧 servo_pose_table.c/h 已删除；ServoPose_* 仅通过显式映射兼容旧编号，不强制转换枚举、不保留第二份数值表。手机分发直接使用 ServoCode；历史 arm 测试从新表读取 PWM。静态脚本逐动作名称对照接手快照，13 组 PWM/time/count 全部相同。

## 验证命令

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

正式目录保持 Debug、CAR_TEST_INPUTS_ENABLE=OFF；bench 保持 Debug、CAR_TEST_INPUTS_ENABLE=ON。应用 -Wall/-Wextra/-Werror；主机另含 -pedantic。GCC 与 Keil 源文件列表同步；Keil 工具链未执行。

公共 UART/队列专项覆盖二进制零字节、状态透传、IT/DMA 时间戳、独立串口、空/部分读取、RX 溢出/恢复、RX 错误不取消 TX、FIFO/latest、业务标签、复制生命周期、TC 构造交接、满队列、立即完成、失败保留/丢弃、超时边界/回绕、abort 失败与独占失败回滚。桥接回归新增未知 UART 忽略、已绑定 UART 三类事件路由及初始化回滚。

既有整车、电机急停、失联、故障、PID 优先级/积压、视觉重试和舵机逐字节断言保留。ZLIS2 144 检查及所有 13 枚举行/缓冲边界通过；ServoRemote 覆盖输入开关的 0、1。

测试 HAL 替身通过 tests/uart_hal_defaults.c 补充未使用的传输边界，选择宏仅位于测试；未提供的 TX 默认失败，不掩盖意外发送。测试也使用函数分节/GC；这没有生产 host 分支。

静态检查：正式功能模块无 HAL UART 收发/abort 调用（历史 X42 除外）；全局 UART 回调仅在公共驱动定义；旧数值表无残留引用；GCC/Keil 中公共模块及新枚举文件各一份。默认镜像中桥接缓存可被 GC，不再由全局回调强引用。

本地日志为 build-local/*-uart-final.log。接手文件快照在 build-local/pre-uart-refactor，仅为本地比对，不提交。

## 资源结果

基线为本次接手工作区的已构建镜像，非 git HEAD。

| 配置 | Flash/B | RAM/B |
| --- | ---: | ---: |
| 正式基线 | 45804 | 16448 |
| 正式重构后 | 46628 | 14856 |
| 正式增量 | +824 | -1592 |
| 台架基线 | 47080 | 16496 |
| 台架重构后 | 47888 | 14904 |
| 台架增量 | +808 | -1592 |

公共实现及枚举适配增加 Flash；回调解除对未启用桥接缓存的强引用后，默认及台架配置的总体 RAM 下降。此结果不能推广为每种桥接配置的 RAM 或实际主循环耗时。

同正式编译参数附加 `-fstack-usage`：ZLIS2_SetServoValues 32 B、ZLIS2_SetServos 24 B、UART_TxQueue_Process 32 B、JY61_Process 408 B（包含原 256 B 接收批次缓存）。这些是单函数静态栈，不是整机/嵌套中断栈峰值。

软件验收不代表机械停止、传感器安装极性、舵机到位或真实启动延迟已验证。此次未烧录；台架需复核 UART 波形、队列时序与电机急停行为。
