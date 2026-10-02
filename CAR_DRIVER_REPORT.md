# STM32 蓝牙四轮驱动交付报告

> 历史实现记录：本文的 13 字节蓝牙控制帧已被 `README.md` 中的 25 字节独立 short 控制协议取代。PID 的独立短帧仍保留。

VERDICT: PARTIAL

源码集成、ARM GCC/Keil 编译与主机 HAL 模拟测试已完成。未烧录、未连接实车，不能判定整车硬件 PASS。

## 最新修改：上电自动使能

蓝牙模式下 `CAR_BOOT_AUTO_ENABLE=1`：启动停止/失能队列完成后自动发送一次四轮 F3 使能，进入 WAIT_CENTER；首个合法归中包才解锁运动。已移除 ENABLE 按钮判断，协议首个 short 改为保留位，填 0，兼容旧值 1。首次蓝牙接入前只使能静止等待，首次合法包后执行 500 ms 失联保护。DISABLE 释放、失联或故障恢复后，BRAKE/DISABLE 均为 0、连续收到至少两个合法归中包且发送队列空闲时重新使能；使能发送完成后还需新的归中包。非归中、刹车或失联会清除归中恢复进度。主机测试覆盖启动顺序、未连接不运动、保留位不控制使能、持续 DISABLE、连续新包恢复及故障恢复；未进行实机烧录验证。

## 当前模式：蓝牙遥控

当前 `CAR_PD10_STANDALONE_TEST=0`，恢复 USART6 蓝牙整车模式：开机归中、摇杆控制、刹车锁、关机与 500 ms 失联急停全部启用。PD10 不直接驱动车轮。M1=-1、M2=+1、M3=+1、M4=-1 保持不变。以下 PD10 段落描述的是可选测试模式；下文蓝牙控制与安全规则适用于当前固件。

## 保留选项：PD10 独立按键测试

选择 `CAR_PD10_STANDALONE_TEST=1` 并重新编译时，无需蓝牙连接。PD10 持续低电平 20 ms 后，四轮自动使能并同步向前 100 RPM；高电平立即取消未发送的使能/速度/同步帧，发送四轮 FE 并失能，松开不延迟消抖。仅 PD10 触发行驶，其他三个按钮不触发行驶。

1、4 号方向已改为 -1，2、3 号为 +1。因此四轮逻辑前进 100 RPM 时，M1/M4 的 DIR=01，M2/M3 的 DIR=00，地址位置不变。电机故障会急停锁定，持续按住不能自动恢复，需先释放 PD10 再重新按下。初始化时 PD10 已为低电平也会按此规则启动。

独立模式不会执行蓝牙开机、摇杆、刹车/关机按钮或蓝牙失联状态机；当前已将该宏恢复为 0 并重新编译。

两种模式的主机回归均通过，新增测试覆盖无需蓝牙持续运行、按下消抖、四轮 100 RPM 和 M1/M4 反向、松开 FE/失能、使能途中松开、故障后释放重按、tick 回绕。ARM GCC 与 Keil 编译通过，未进行实车验证。

## FILES_CHANGED

新增：

- Core/Inc/car_config.h
- Core/Inc/board_app.h
- Core/Inc/serial_io.h
- Core/Inc/bluetooth_driver.h
- Core/Inc/motor_driver.h
- Core/Inc/car_control.h
- Core/Src/board_app.c
- Core/Src/serial_io.c
- Core/Src/bluetooth_driver.c
- Core/Src/motor_driver.c
- Core/Src/car_control.c
- tests/car/stm32f4xx_hal.h
- tests/car/test_car.c
- tests/car/run.ps1
- CAR_DRIVER_REPORT.md

修改：

- Core/Inc/main.h（USART1 用途注释）
- Core/Inc/ssd1306.h（异步刷新接口）
- Core/Src/main.c（简洁整车入口，保留可选诊断桥）
- Core/Src/ssd1306.c（运行期异步刷新及忙总线保护）
- Core/Src/stm32f4xx_hal_msp.c（新增 USART6 和 I2C1 中断使能）
- Core/Src/stm32f4xx_it.c（USART6、I2C1 IRQ 分发）
- Core/Src/uart_bridge.c（原唯一 HAL callback 中统一分发）
- CMakeLists.txt（添加源文件）
- MDK-ARM/stm32f407_bt_oled.uvprojx（添加源文件）
- README.md（当前行为说明，保留历史文档）

编译另生成 build 下固件、测试程序、日志，及 MDK-ARM/Objects、List 相关产物；修改前源码备份在 build/car-before。这些不是手写源码变更。

## 工程检查及实际差异

- 当前目录没有 .ioc、Core/Src/usart.c、Core/Inc/usart.h，也没有 .git。原 UART/时钟/GPIO/I2C 初始化集中在 main.c，本版迁移到 board_app.c。
- 原默认 UART_ONLY_TEST_MODE=1，运行 USART1↔UART5 原始串口桥；USART6/OLED/按钮初始化被该模式跳过。现在默认进入整车模式，恢复这些外设初始化。
- 蓝牙原配置就是 USART6 PC6/PC7 **9600**，USART1 PA9/PA10 也为 **9600**，均保持，没有擅自改为 115200。
- UART5 PC12/PD2、115200 与需求一致。没有现成 UART RX DMA 配置，因此选择 RX interrupt + ring buffer；新增 USART6 NVIC/IRQ，不重新分配 DMA 或 GPIO。
- HSI 16 MHz、I2C1 PB6/PB7 100 kHz、PD10/PD11/PD14/PD15 低有效按钮保留，SWD PA13/PA14 不动。
- 旧 x42.c 的 F3/F6 是历史错误协议配置，且 Stop 尚未实现。保留旧文件供追溯，但整车路径完全使用新的非阻塞 motor_driver，不调用旧事务和查询。原按钮显示保留，旧按钮直接调用电机的路径由整车状态机取代，避免绕过蓝牙安全锁。

## BLUETOOTH

- USART6：TX PC6、RX PC7，9600 8N1。
- 单字节 HAL_UART_Receive_IT + 256 槽环形缓冲（255 字节有效容量）；每字节记录接收时间。ISR 只入缓冲/重启接收，解析在主循环。
- ValuePack 帧长 13：A5 + 10 字节 payload + payload 累加低 8 bit + 5A。
- int16 小端顺序：RESERVED、BRAKE、DISABLE、JOY_X、JOY_Y，帧长度保持 13 字节。
- 保留位填 0（兼容 1，但忽略其控制含义）；按钮必须 0/1；摇杆限幅 -1000..1000；DISABLE 按上升沿发送停止/失能，持续电平抑制恢复。
- 滑动重同步支持拆包、粘包和损坏帧内嵌 A5。校验/尾字节/按钮值错误不更新控制。单包字节间隔超过 100 ms 重新同步；溢出/串口错误清空受损流并立即令链路无效。
- 每次 Bluetooth_Process 交付一个合法包，控制层逐包处理，不用最后一个包覆盖同批中的刹车/关机脉冲。失联按实际接收时间判断，延后解析不能延长旧包有效期。
- 手机每 100 ms 周期发送完整包，按下和松开按钮均要发送。启动保留原蓝牙 AT 命名逻辑，仅在开放遥控接收之前执行。

## MOTOR

- UART5，TX PC12、RX PD2，115200 8N1，地址只接受 1..4，固定校验尾 6B。
- F3：ADDR F3 AB EN 00 6B，EN=1 使能、EN=0 失能。
- F6：ADDR F6 DIR RPM_H RPM_L 0A SYNC 6B。signed RPM 在驱动内限幅并乘 motor_sign，自动转换 DIR；上层不拼 HEX。
- FE：ADDR FE 98 00 6B。
- Motor_SetSpeedSync4 按 1、2、3、4 发 SYNC=1 的 F6，最后发 00 FF 66 6B。
- HAL_UART_Transmit_IT 分帧发送；默认帧间隔 2 ms，正常速度批次不交错，忙时返回 HAL_BUSY 供下个控制周期重试。
- 急停清除未发送的速度帧及同步帧。当前在传输的一个完整帧发送完后执行四轮 FE，避免中途截断协议字节；已经发出的同步命令不能撤回。
- 回包识别 F3/F6/FE（以及 FF）的 02/E2/EE，保存每地址 command、response_status、timestamp、valid；地址越界返回 NULL。E2/EE、RX 错误/溢出、TX 错误或超过 20 ms 未完成会触发电机故障保护。
- API 的 HAL_OK 表示命令已入队，**不是设备执行成功**；Motor_GetLastStatus 提供设备回包诊断。本版没有缺失 ACK 超时判定，不能据成功发包推断电机在线。

## CAR_MAPPING

| 地址 | 车轮 | 混控输出 |
|---|---|---|
| 1 | 右前 | right |
| 2 | 左前 | left |
| 3 | 左后 | left |
| 4 | 右后 | right |

## CONTROL

- MOTOR_MAX_RPM=300；MOTOR_ACC=10。
- JOY_DEADZONE=50；JOY_RANGE=1000。
- TURN_GAIN_NUM/TURN_GAIN_DEN=700/1000。
- CAR_CONTROL_PERIOD_MS=20，正常输出约 50 Hz，安全处理每轮主循环执行。
- left=clamp(Y+X*700/1000)，right=clamp(Y-X*700/1000)，限幅 ±1000 后映射 ±300 RPM，全整数运算。
- X/Y 同时进入死区时目标为 0，通过 F6 正常减速，不发 FE。
- 手机摇杆 JOY_X 在进入麦轮运动学前取反，修正实车左右方向；前后轴、旋转方向及电机安装符号不变。
- MotorSign 唯一配置区：Core/Src/motor_driver.c。

## SAFETY

- 蓝牙模式启动自动使能一次并进入 WAIT_CENTER，合法帧归中且使能帧发送完成才 READY；尚未收到首包时仅静止等待，首包后启用失联计时。
- BRAKE 为电平有效，进入 BRAKE_LOCK 并 FE；松刹车不恢复旧速度，必须收到新的归中帧。
- 超过 BT_FAILSAFE_TIMEOUT_MS=500 进入 LINK_LOST 并 FE；500 ms 边界仍连接。恢复需 BRAKE/DISABLE=0、连续至少两个新的合法归中包且电机队列空闲，自动使能并进入 WAIT_CENTER；使能发送完成后新的归中包才进入 READY。
- DISABLE 上升沿先四轮 FE 再四轮 F3 失能，进入 OFF；持续按住抑制恢复而不重复发送，释放后采用与失联相同的归中恢复条件。
- 优先处理链路失效、关机；电机故障额外进入 CAR_FAULT，急停并采用与失联相同的连续归中恢复条件。
- 主循环先检查蓝牙安全状态再启动下一电机帧，并在电机回包处理后再检查故障，避免安全帧被普通速度队列拖后。
- USART1 日志使用有限非阻塞队列，满时丢日志；首个/恢复合法蓝牙包记录 valid frame，错误限频，正常 F6 ACK 不逐周期刷屏。
- OLED 保留初始化与按钮显示，运行期采用 I2C IT 双缓冲；预检查 BUSY，避免 HAL IT 入口对忙总线的内部轮询，显示超时复位 I2C 而不等待。

## MOTOR_SIGN

M1=-1，M2=+1，M3=+1，M4=-1。M1、M4 按用户反馈反转；修改后仍需实车确认。实车反向只能修改 motor_sign[]，不修改混控或地址映射。

## BUILD

- ARM GCC：CMake/Ninja，-Wall -Wextra -Werror，成功生成 build/stm32f407_bt_oled.elf/.hex/.bin，0 error、0 warning。
- Keil ARMCC 5.06 update 5：Rebuild，0 Error(s)、0 Warning(s)。日志 build/car-keil.log。
- 主机 GCC：tests/car/run.ps1，真实驱动源码 + HAL 模拟，所有测试通过。覆盖拆包/粘包/噪声/校验/尾字节/按钮值/摇杆限幅/重同步/1000 组噪声/缓冲溢出/UART恢复/过期包、协议字节、signed RPM 极值、四轮同步、队列急停抢占、按钮边沿、混控、归中锁、关机优先、500 ms 边界、tick 回绕、失联再使能、TX卡住与错误。
- git diff --check 已尝试：当前不是 Git 工作树，命令不可用。使用修改前快照执行 git diff --no-index --check，补充对新文件的空白和冲突标记检查。
- 以上是软件构建与模拟测试，未进行烧录和硬件测试。

## UNCONFIRMED

- 四个 motor_sign 的真实机械方向、四轮实际地址、正 RPM 是否整车前进。
- 设备固件是否采用需求中的 Emm V5.0/X42 协议、6B 校验、RPM 单位、加速度含义、F3/F6/FE 与 FF 同步行为。
- UART5 多设备回包的电气连接、共地、电平、总线冲突与回包延迟；2 ms 帧间隔需按实测调整。
- 蓝牙模块实际数据模式是否为 9600、手机专业模式是否严格每 100 ms 发完整 13 字节包、按键是否正确产生 0→1；AT 命名方言与开机时间。
- 实车开机归中锁、松刹车不恢复、摇杆归中正常减速、断联到物理停止的延迟、重新开机、四轮同步、轮胎滑移和转向增益。
- 电机使能成功与停止结果需通过回包及机械观察确认；线缆断开、掉电或驱动器不响应时，软件发送 FE 无法保证物理停止。
- OLED/按钮/SWD/USART1 现有硬件工作情况。首次确认方向应架空四轮、低速验证，再验证整车运动。
