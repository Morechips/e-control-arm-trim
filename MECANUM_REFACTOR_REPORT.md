# 麦克纳姆驱动层 / 运动控制层审查与重构

> 历史实现记录：本文提到的 13 字节蓝牙控制帧已被 `README.md` 中的 25 字节独立 short 控制协议取代。

VERDICT: 软件分层重构完成，正常/低速测试固件 Build PASS；实车运动方向 UNCONFIRMED。

## 审查前的结构与问题

- `board_app.c` 配置 UART5：PC12 TX、PD2 RX、115200 8N1；`main.c` 初始化蓝牙、协议驱动和控制状态机，在发送调度前后执行安全状态机。
- `motor_driver.c` 原先同时承担 F3/F6/FE/FF 组帧、UART5 收发队列、应答解析、方向修正和速度限制。已有 `motor_sign={0,-1,+1,+1,-1}`，无需重新猜测方向。
- `car_control.c` 原先使用 `left=clamp(y+0.7*x)`、`right=clamp(y-0.7*x)`，输出 `[right,left,left,right]`。这是差速转向，不是麦轮横移；分别截断左右速度也不是四轮共同归一化。
- `bluetooth_driver.c` 已有 13 字节 ValuePack、校验/帧重同步、五个 short（ENABLE/BRAKE/DISABLE/X/Y）、500ms 失联检测。无需重写协议或设计新摇杆 UI。
- 已有 `CAR_BRAKE_LOCK`：按下刹车清除未发送速度/同步队列；松开后必须有新的归中包且电机队列空闲才解锁。本次保留，并覆盖回归。
- 当前非阻塞驱动只解析命令应答。`x42.c/.h` 的旧查询是独占 UART 的阻塞原始字节捕获，回复长度/单位未验证，不接入当前 UART5 异步控制。

## 固定硬件基准

MOTOR_LAYOUT:

```text
M1 = RF = A = addr1 = sign -1
M2 = LF = B = addr2 = sign +1
M3 = LR = A = addr3 = sign +1
M4 = RR = B = addr4 = sign -1
```

MECANUM_LAYOUT:

```text
车头
B A    M2 M1
A B    M3 M4
```

COORDINATE:

- +X = 向前。
- +Y = 向右。
- +W = 顺时针。
- 单轮逻辑正方向 = 该轮按普通车轮方式向车头方向滚动的正转；不是驱动器未经修正的方向位。

## 三层及推导

1. `emm42_driver.c/.h`：从原协议代码提取，负责物理有符号 RPM、使能/失能、停止、同步、串口队列、命令应答和传输故障。协议字节和 UART5 参数保留。该层不引用轮子位置、A/B 类型或 motor_sign。
2. `motor_driver.c/.h`：单轮逻辑层，固定 RF/LF/LR/RR 的地址及 A/B 映射；限幅 ±300 RPM，电机速度死区配置默认 0（未测得可靠的起转阈值，不擅自设置非零值）。唯一方向转换为 `physical = limit(logical) * motor_sign[address]`。保留 Motor_* 兼容接口，并提供 `car_motor_set(MOTOR_RF, logical_rpm)`。
3. `mecanum.c/.h`：纯解算 `Mecanum_Calculate()` 和排队输出 `mecanum_drive(vx,vy,omega)`。所有量明确采用轮速等效 RPM。调用者必须先通过使能、刹车、链路安全门。

按用户确认的理想 45° X 布局，A 轮的滚子自由轴平行车体 `(1,+1)`，B 平行 `(1,-1)`；受约束投影方向分别为 A `(1,-1)`、B `(1,+1)`。这描述的是本次 X 型模型使用的滚子轴，不是另外猜测 A/B 厂商命名。实车是否符合该理想接触约束，仍以低速实测为准。

令半轴距为 a，半轮距为 b，轮子位置按 M1..M4 为 `(+a,+b),(+a,-b),(-a,-b),(-a,+b)`。在 +W 顺时针坐标中，每个轮位的速度为 `(Vx-W*y_i, Vy+W*x_i)`。投影约束：

```text
r*q_i = (Vx-W*y_i) + s_i*(Vy+W*x_i)
s_i = [-1,+1,-1,+1]
```

将 `(a+b)*W` 换算为轮速等效 omega，得到：

KINEMATICS_FORMULA:

```text
M1 = vx - vy - omega
M2 = vx + vy + omega
M3 = vx - vy + omega
M4 = vx + vy - omega
```

若最大绝对值超过 300，则四轮全部乘同一系数 `300/max_abs`（整数取整），保留轮速比例。之后才进入单轮方向层。

没有提供轮径、轴距、轮距，因此 API 不把输入冒充为 m/s 或 rad/s。将来得到半径 r、a、b 后：`vx=60*Vx/(2*pi*r)`、`vy=60*Vy/(2*pi*r)`、`omega=60*(a+b)*W/(2*pi*r)`。

## 蓝牙及安全

正常模式：Joystick Y → vx，Joystick X → vy，omega=0；每轴保留 ±50 的中心死区。现有二维摇杆不再用 X 做差速转向；没有添加旋转 UI。

刹车锁、失联、故障恢复、失能优先级、开机使能后等待归中均保留。串行链路上的“立即停止”沿用原实现：取消尚未发出的速度和同步命令，允许至多一个在途帧发送结束，接着优先发送四轮 FE。软件调度立即响应，不宣称机械轮速会瞬时归零。

正常固件保留 USART2 ↔ U1 透传；USART2 512B DMA/100ms 及 U1 9600 的吞吐限制不变。JY61 仍在 USART2 PD5/PD6、115200 8N1。本次未解析 55 51/52/53，也未新增 PID/EKF。

JY61_HOOK_RESERVED: PASS

未来由独立航向控制器产生轮速等效 `omega_correction`，通过同一个安全门调用 `mecanum_drive(vx,vy,omega_correction)`。编码器负责轮级闭环，JY61 负责车身航向，二者没有混成一个控制器。当前 `Emm42_GetLastStatus()` 只表示缓存命令 ACK；`Emm42_GetTelemetryCapability()` 明确返回 `EMM42_TELEMETRY_UNSUPPORTED`。旧查询代码保留但不运行，等待确认回复协议后再实现异步遥测与里程计。

## 低速测试入口

测试固件：`build/mecanum-test/stm32f407_bt_oled.hex`，调试符号为同目录 `.elf`。

```powershell
cmake -S . -B build/mecanum-test -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCAR_MECANUM_TEST_MODE=ON
cmake --build build/mecanum-test --parallel 4
```

正常固件：`build/stm32f407_bt_oled.hex`，`CAR_MECANUM_TEST_MODE=OFF`。Keil 已登记新源码，若用 Keil 测试，在预处理宏中定义 `CAR_MECANUM_TEST_MODE=1`；本次没有运行 Keil 编译。

测试模式仅占用 U1 输出日志，不运行 USART2↔U1 透传；USART2 DMA 仍接收。U1 串口助手设 **9600 8N1**。本入口通过调试器变量触发，不是串口文本命令。

1. 烧录低速测试镜像，保持蓝牙每 100ms 发送合法数据；摇杆归中、刹车松开，等待 `CAR_READY`。原有开机使能仍保留，但测试不会自动执行任何运动。
2. 在调试器实时变量视图（CPU 运行状态）先写 `mecanum_test_action`，再把 `mecanum_test_request` 加 1。每次请求只执行一次，最高轮速 30 RPM，最多 1000ms 后自动四轮 FE 停止。
3. 例如前进：`mecanum_test_action=1`，随后 `mecanum_test_request=1`。下一次后退：先改 action=2，再把 request 改为 2。不要只改 action 而不更新 request。
4. 紧急测试停止：action=0，再增加 request；蓝牙“立即刹车”始终优先有效。测试过程中不要暂停 CPU；定时停止和链路保护依赖主循环运行。
5. 每个动作等待 `mecanum_test_active=0` 且队列空闲后再发下一请求。蓝牙离线、刹车锁、未使能或摇杆不在中心时的请求会被消耗，不会在解锁后自动补执行。运动中摇杆离开中心立即取消测试。原刹车锁仍要求新鲜归中包解锁。

打印示例：`M1 logical=30 physical=-30 RPM`。`[TEST] ... QUEUED` 表示已加入发送队列；`[UART5 TX_START] addr=1 physical=-30 RPM` 从实际提交 UART 的 F6 字节中解码，表示传输已启动，不表示驱动器已应答或轮子达到该速度。STOP 使用 FE 命令，打印的零为停止目标，不是编码器反馈。

| action | 动作 | M1,M2,M3,M4 逻辑 RPM | M1,M2,M3,M4 物理 RPM |
|---:|---|---|---|
| 0 | STOP | 0,0,0,0 | 0,0,0,0（发送 FE） |
| 1 | FORWARD | 30,30,30,30 | -30,30,30,-30 |
| 2 | BACKWARD | -30,-30,-30,-30 | 30,-30,-30,30 |
| 3 | LEFT | 30,-30,30,-30 | -30,-30,30,30 |
| 4 | RIGHT | -30,30,-30,30 | 30,30,-30,-30 |
| 5 | FORWARD_LEFT | 30,0,30,0 | -30,0,30,0 |
| 6 | FORWARD_RIGHT | 0,30,0,30 | 0,30,0,-30 |
| 7 | BACKWARD_LEFT | 0,-30,0,-30 | 0,-30,0,30 |
| 8 | BACKWARD_RIGHT | -30,0,-30,0 | 30,0,-30,0 |
| 9 | ROTATE_CW | -30,30,30,-30 | 30,30,30,30 |
| 10 | ROTATE_CCW | 30,-30,-30,30 | -30,-30,-30,-30 |

对角线输入分量各 15 RPM，合成的非零轮速为 30 RPM。表格是模型预期，不是实测结果。

## 验证与交付

FILES_CHANGED:

- 新增：`Core/Inc/emm42_driver.h`、`Core/Src/emm42_driver.c`。
- 重构单轮层：`Core/Inc/motor_driver.h`、`Core/Src/motor_driver.c`。
- 新增运动学：`Core/Inc/mecanum.h`、`Core/Src/mecanum.c`。
- 新增测试入口：`Core/Inc/mecanum_test.h`、`Core/Src/mecanum_test.c`。
- 控制和配置：`Core/Src/car_control.c`、`Core/Inc/car_config.h`。
- 构建：`CMakeLists.txt`、`MDK-ARM/stm32f407_bt_oled.uvprojx`。
- 测试：`tests/car/test_car.c`、`tests/car/mecanum_cases.inc`、`tests/car/run.ps1`。
- 文档：`README.md`、本报告 `MECANUM_REFACTOR_REPORT.md`。

EMM42_DRIVER: MODIFIED（组织结构及测试日志；原协议字节 PRESERVED）

说明：复用原 F3/F6/FE/FF、队列、同步、接收应答、超时与停止抢占代码，提取为独立文件并移出安装方向/限幅。没有改成网上的另一套 Emm42 命令格式。没有新造编码器读取命令。

MOTOR_DIRECTION_LAYER: PASS（软件）

MECANUM_LAYER: PASS（模型与模拟）

BRAKE_LOCK_PRESERVED: PASS（主机回归）

JY61_HOOK_RESERVED: PASS（omega 输入；未实现姿态闭环）

BUILD: PASS

- 正常 ARM GCC：Flash 21948 B，RAM 12600 B。
- 低速测试 ARM GCC：Flash 22244 B，RAM 12176 B。
- 均启用 `-Wall -Wextra -Werror`，生成 ELF/HEX/BIN。
- `tests/car/run.ps1`：正常模式、原 PD10 模式、麦轮测试模式、USART2 透传测试全部通过。
- 覆盖 11 动作逻辑/物理表、极值、比例归一化、固定映射、原协议帧、刹车松开但未归中、失联重连、故障、失能、测试超时与被拒绝请求不重放。

HARDWARE_TEST: UNCONFIRMED

本次没有烧录、没有实际驱动车轮。Build PASS 不能证明实车前进/横移/旋转方向正确。

NEEDS_REAL_CAR_VERIFICATION:

请先按 **前进 → 后退 → 左移 → 右移 → 顺时针旋转 → 逆时针旋转**，逐项以 30 RPM 验车，每次结束确认停止，记录车身实际方向和四轮日志。

随后逐项验证左前、右前、左后、右后对角运动与 STOP；再验证“摇杆推着时刹车 → 松开刹车仍不动 → 摇杆归中一次后才允许新动作”、失联停止、失能和测试自动停止。归中规则实车测试应使用正常控制固件；测试固件本身禁止摇杆驱动，只接受安全门之后的测试请求。

若有方向不符，记录具体动作、车头朝向、四轮实际转向及 logical/physical/TX_START 日志后反馈，再据此核对运动学定义。不要先改地址或重复反转 M1/M4，也不在收到实车结果前猜测修改。
