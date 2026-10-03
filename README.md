# JNDS 电控固件

基于 STM32F407 和 STM32 HAL 的四轮麦克纳姆小车电控工程，包含蓝牙遥控、Emm42 电机通信、JY61 航向控制、MaxiCam 二维码通知、ZL-IS2 舵机驱动及路线/动作状态机。

本分支在 [Morechips/e-control-arm-trim](https://github.com/Morechips/e-control-arm-trim) v4.5 基线上导入 [gpnu-in-jnds/e-control](https://github.com/gpnu-in-jnds/e-control) 的 e7404c5，接入可移植机械臂微调。具体接口、手机协议、验证边界、固件与烧录命令见 [ARM_TRIM_INTEGRATION.md](ARM_TRIM_INTEGRATION.md)。下文保留队友原系统说明，当前接入状态以本表与该记录为准。

2026-10-03，用户确认本次 **v4.6 已实测**，并授权推送代码。发布保留已实测源码和固件参数；原 v4.5 可从 Git 历史提交 `e0b5bc9` 恢复。

## 当前功能与接入状态

| 功能 | 当前状态 |
| --- | --- |
| 蓝牙整车遥控 | 已接入主循环，手机四向按钮控制平移；JOY 只解析 |
| 四轮电机控制 | Emm42 协议、发送队列、四轮同步、急停和故障处理 |
| JY61 航向保持 | 已接入，USART2 DMA 接收，软件航向零点与 PID 修正 |
| 蓝牙 PID 调参 | 已接入，与摇杆共用 USART6；参数仅保存在 RAM |
| MaxiCam 二维码与目标偏移 | 已接入 UART4；区分独立 `0x80` 二维码包和 5 字节位置包 |
| 视觉任务三位值解码 | 独立接口已实现，尚未与完整视觉串口协议绑定 |
| 固定路线与任务状态机 | 路线及排爆/反恐/救援状态链已实现；正式主循环仍未启动路线 |
| 原地右转 90° | 独立状态机已实现；正式遥控未绑定触发入口 |
| ZL-IS2 舵机控制器 | USART3 中断发送：手机姿态、AIM/RST、GAP；PE4 仅台架配置；旧 ARM 模块不参与正式构建 |
| 独立机械臂微调 | 已接入：纯 C11 核心、项目参数、Servo 独占服务及 7/11 字节独立手机页；三关节微调与 003 夹爪分开 |
| XDK42 | 仅保留 USART6 候选配置，当前串口由蓝牙占用 |
| OLED / 按钮显示 | 可选 SSD1306，失败不阻断遥控；按钮文案仅台架配置 |
| PB8 红外触发按钮 | 仅 `CAR_TEST_INPUTS_ENABLE=1` 编译，正式配置无手动触发 |

**默认固件上电会自动使能四轮**，随后等待合法蓝牙归中数据才允许运动。确认目标板、接线和安全测试条件后才能烧录或上电联调。编译、主机测试通过不代表实车方向、机械停止或传感器安装极性已验证。

## 工程目录

```text
Core/Inc/           模块公共头文件、引脚及控制参数
Core/Src/           应用程序、驱动、控制逻辑与状态机
Drivers/           ST HAL、CMSIS Device 和 ARM CMSIS 依赖
cmake/             ARM GCC 工具链与运行库初始化
MDK-ARM/           Keil 工程、启动配置与链接脚本
openocd/           CMSIS-DAP / STM32F407 调试配置
tests/car/         整车、麦轮、航向、PID、右转与串口桥测试
tests/vision/      三位任务解码、视觉数据结构与 MaxiCam 测试
tests/route/       路线与动作状态机测试
tests/zlis2/       ZL-IS2 舵机驱动测试
tests/servo/       正式按钮直发与串口输出回归
tests/arm/         机械臂限位、状态机与停止测试
tests/arm_kinematics/  机械臂逆运动学测试
tests/arm_bt/      历史 @ARM 与组合帧独立测试（正式固件停用）
tests/x42/         历史 X42 阻塞驱动测试
CMakeLists.txt     固件构建入口
STM32F407_FLASH.ld ARM GCC 链接脚本
```

`build/`、`cmake-build-*`、Keil `Objects/` / `Listings/` 和个人 IDE 设置不属于源码交付内容。厂商依赖随仓库提供，无需额外下载才能编译；保留其原有许可证。

## 硬件接口

以下为当前默认配置，所有 UART 均使用 8N1、无流控；TX 接外设 RX，RX 接外设 TX，并共地。

| 外设 | 用途 | STM32 TX / RX | 波特率 | 配置位置 |
| --- | --- | --- | --- | --- |
| USART1 | 诊断日志 | PA9 / PA10 | 115200 | `Core/Inc/main.h` |
| USART2 | JY61 IMU | PD5 / PD6 | 9600 | `Core/Inc/usart2_dma.h` |
| USART3 | ZL-IS2 舵机控制器 | PB10 / PB11 | 115200 | `Core/Inc/servo.h` |
| UART4 | MaxiCam 视觉 | PC10 / PC11 | 115200 | `Core/Inc/vision_config.h` |
| UART5 | 四轮电机 | PC12 / PD2 | 115200 | `Core/Inc/main.h` |
| USART6 | 蓝牙 | PC6 / PC7 | 9600 | `Core/Inc/main.h`、`Core/Inc/usart6_config.h` |

- OLED：I2C1，PB6 SCL / PB7 SDA，100 kHz；SSD1306 128×64，7 位地址 `0x3C`。
- 正式配置 `CAR_TEST_INPUTS_ENABLE=0` 仅保留启动键 PD10：内部上拉、接地按下、20 ms 消抖，每次按下只输出一次 `[START] press`，长按不重复；本次没有绑定启动任务动作。换键由 `start_button_config.h` 配置。
- 台架配置 `CAR_TEST_INPUTS_ENABLE=1` 恢复 PD10/11/14/15 显示按钮、PE0 视觉、PC1 射靶、PE4 AIM、PB8 红外手动按钮。均沿用原接线；正式配置不初始化这些额外输入，也不读取它们。
- 红外触发输出：PC3 开漏低电平有效；无触发请求时配置为无上下拉输入，由外接模块完成红外发射。
- 时钟：内部 HSI 16 MHz，AHB / APB1 / APB2 不分频。
- GCC 链接范围：Flash `0x08000000` 起 512 KiB，SRAM `0x20000000` 起 128 KiB；不使用 CCM，无 bootloader 偏移。
- 蓝牙名称为 `AIOTCAR`。默认 `BT_AT_NAME_ENABLE=0` 不发送启动 AT 命令；产线置 1 烧写、上电一次，再置回 0。该配置只发送一次 `AT+NAME=AIOTCAR\r\n`，不延时、不轮询接收；模块是否接受须现场核对。

USART6 的蓝牙与 XDK42 用途互斥。当前选择 `USART6_OWNER_BLUETOOTH`；`XDK42_USART6_BAUD_RATE=115200` 只是候选配置，不能视为已实现 XDK42 接收和识别。

## 获取与构建

私有仓库需要具备访问权限的 GitHub 账号。

```powershell
git clone https://github.com/SMG0128/JNDS.git
cd JNDS
cmake -S . -B build-local -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build-local --parallel 4
```

需将 CMake 3.22 或以上、Ninja、`arm-none-eabi-gcc`、`arm-none-eabi-objcopy` 和 `arm-none-eabi-size` 加入 `PATH`。工程自动使用 `cmake/arm-none-eabi.cmake`，C 标准为 C11，应用代码启用 `-Wall -Wextra -Werror`。

| 产物 | 用途 |
| --- | --- |
| `build-local/stm32f407_bt_oled.elf` | 调试镜像 |
| `build-local/stm32f407_bt_oled.hex` | 含地址的固件镜像 |
| `build-local/stm32f407_bt_oled.bin` | 从 `0x08000000` 开始的裸镜像 |
| `build-local/stm32f407_bt_oled.map` | 链接映射 |

Keil 用户打开 `MDK-ARM/stm32f407_bt_oled.uvprojx`，选择 `STM32F407_BT_OLED` 并 Rebuild。工程沿用 ARM Compiler 5 配置；器件设置与下载算法需按实际芯片核对。Keil 与 GCC 使用各自的启动文件、链接脚本和运行库配置。

CLion 使用运行配置 `Horco CMSIS-DAP (OpenOCD)` 烧录 `stm32f407_bt_oled` 目标。该配置在烧录前构建固件、每次下载 ELF，并在下载后复位运行。板级配置位于 `openocd/horco-cmsis-dap-stm32f407.cfg`，使用 Horco CMSIS-DAP v2、SWD 和 STM32F4 目标；更换探针时需更新其中的序列号与接口。若 CLion 未自动找到 OpenOCD，请在 **Settings → Build, Execution, Deployment → Embedded Development** 将 OpenOCD 路径设为本机 `openocd.exe`。本机当前路径为 `D:\openocd-20260302\OpenOCD-20260302-0.12.0\bin\openocd.exe`。默认固件上电会使能四轮，确认目标板、接线及安全条件后再点击运行或调试。

## 主机回归测试

测试使用主机 `gcc` 和 HAL 替身，不连接硬件。Windows 环境需保证 `gcc` 及其运行库可从 `PATH` 找到。

```powershell
$env:CAR_TEST_BUILD_DIR = 'build-local'
powershell -ExecutionPolicy Bypass -File tests/uart/run.ps1
powershell -ExecutionPolicy Bypass -File tests/car/run.ps1
powershell -ExecutionPolicy Bypass -File tests/vision/run.ps1
powershell -ExecutionPolicy Bypass -File tests/route/run.ps1
powershell -ExecutionPolicy Bypass -File tests/zlis2/run.ps1
powershell -ExecutionPolicy Bypass -File tests/servo/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_kinematics/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_bt/run.ps1
```

所有 `tests/*/run.ps1` 使用 `CAR_TEST_BUILD_DIR` 覆盖输出目录（支持相对路径、绝对路径和空格），默认仍为 `build`。原目录权限不允许写入时使用上面的 `build-local`，该目录已忽略。整车台架测试显式开启测试输入；舵机和激光输入测试同时覆盖开关的 0、1。

历史 X42 驱动仅保留源码，已从 GCC / Keil 固件构建列表移除，另行测试：

```powershell
$env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
New-Item -ItemType Directory -Force build-local | Out-Null
gcc -std=c11 -Wall -Wextra -Werror -Itests/x42 -ICore/Inc Core/Src/x42.c tests/x42/test_x42.c -o build-local/test_x42.exe
./build-local/test_x42.exe
```

整车测试覆盖默认模式、PD10 独立模式、麦轮测试模式、航向测试模式、PID 调参、右转及串口桥。视觉测试覆盖三位值解码、数据布局与二维码通知；路线测试覆盖动作顺序、等待、刹车和异常路径。

## 整车运动与安全行为

### 电机布局与速度

| 地址 | 位置 | 安装方向修正 |
| --- | --- | --- |
| 1 | 右前 A | -1 |
| 2 | 左前 B | +1 |
| 3 | 左后 A | +1 |
| 4 | 右后 B | -1 |

X 型麦轮混控支持二维平移，航向 PID 以独立 `omega` 修正叠加。四轮统一缩放后限制在 ±500 RPM，航向修正限制为 ±100 RPM。正式遥控采用四向方向门控，先原地对准再按方向按钮平移；JOY_X/Y 不再驱动底盘。

包含 `mecanum.h` 后可使用 `up(rpm)`、`down(rpm)`、`left(rpm)`、`right(rpm)` 和 `brake()`。方向接口接收非负 RPM；返回 `HAL_OK` 只代表命令入队，忙时返回 `HAL_BUSY`。调用方负责初始化、使能、安全条件并持续调用 `Motor_Process()`。

`brake()` 取消排队的运动命令，在当前发送帧完成后优先发送四轮急停；它不等待机械停止、不失能，也不建立整车控制层的持续刹车锁。

### 路线动作的三档速度

`Core/Inc/car_config.h` 集中定义慢速 100 RPM、中速 133 RPM、全速 167 RPM。旧的 `SPEED_NORMAL` / `SPEED_PRECISE` 名称保留为兼容别名；`SpeedMode_GetRPM()` 返回三档固定值。

| 场景 | 速度档位 |
| --- | --- |
| 任务目标搜索（排爆物/桶）及所有对准 | 慢速，100 RPM |
| 反恐靶和人质搜索 | 中速，133 RPM |
| 任务2后前往右转口、右转和人质返回 | 全速，167 RPM |

速度属于当前任务状态或路线动作；蓝牙上下左右方向键平移为 35 RPM，路线任务速度不变。所有调用入口的右转和左转 90° 上限均为 20 RPM，右转 180° 上限为 30 RPM；右转 180° 接近目标和自动微调时降至不超过 20 RPM。扫码后的路线左转同样受 20 RPM 全局上限约束。PD10 诊断速度不受独立转向限速影响；遥控原地纠偏单独限制为 10 RPM。

### 启动链路与验收

顺序为 Board（Servo/板级输入初始化） → JY61 → Bluetooth/MaxiCam 接收中断 → Motor/Car/PID → 可选 OLED → 主循环。OLED 无 100 ms 固定等待，探测仅一次、50 ms 超时，I2C 初始化失败不进 Error_Handler，首屏采用异步更新。启动链路无 USART6 轮询接收。

首个有效帧解析输出 `[BT] first frame t=...`；进入 READY 输出 `[CAR] READY t=...`；WAIT_CENTER 超过 1 s 每秒说明等待条件。目标为复位后首帧 ≤250 ms、READY ≤2 帧，需架空车轮重复上电 10 次测量。本次只完成编译/主机验证，不据此判定硬件达标。9600 baud 下 41 B 帧线时约 42.7 ms，20 Hz 利用率约 85%；仅记录，实测后再考虑 115200，本次不改。

### 蓝牙控制协议

遥控方向状态机 `RemoteHeading` 维护相对正、相对右、相对后、相对左四个目标。首次遥控就绪且得到有效 JY61 数据时记录正前方；原始 yaw 在右转时减小，因此四个相对目标分别为 0°、-90°、±180°、+90°。成功完成右90°、右180°、左90°后累计切换目标；未完成或取消的转向不提交方向状态。

遥控起步前和静止待机采用原地纠偏：偏差达到 **1°** 时启动，回到 **0.5°以内**才执行当前仍按下的平移按钮。行驶中采用另一组门槛：偏差达到 **3°** 时将最多 **4 RPM** 的转向修正叠加到方向键的平移轮速，回到 **1.5°以内**停止叠加；轻微车身抖动不会发四轮停车指令。只有连续至少三个新航向帧偏差均达到 **10°**、持续至少 **300 ms** 时，才暂停平移、原地对准后恢复当前仍按下的按钮。单次异常帧会清除这段持续时间的计数。静止原地纠偏仍限 **10 RPM**。每个新有效航向采样更新一次 PID，电机提交周期仍为 20 ms，量化不足 1 RPM 时按正确方向输出 1 RPM。阈值和持续时间是软件初始值，需要实车在平路和减速带上调校；约 10 Hz 的传感器更新率和机械响应共同决定实际响应速度。

普通松手和 STOP 保留基准；STOP、失联、失能、电机故障期间禁止自动纠偏。失联后重新解锁、失能后重新进入遥控、或退出视觉模式时重新建立正前方。JY61 超过 150 ms 未更新或数据无效时进入 `NO_IMU`：停止自动纠偏，但仍允许方向按钮手动平移，拒绝新的90°/180°请求；恢复数据后保持已有目标并重新执行对准检查。航向反馈不可用期间，运动前对准保证不适用。

JOY_X/Y 仅保留协议解析与原有合法范围检查，不参与移动、转向或解锁。此前的 JOY 开环状态机、时长标定和专用同步完成接口已移除。方向键保持 35 RPM 平移；方向键是车身坐标的前后左右，不是选择四个目标朝向。

遥控转向调用 `TurnRight_StartRemote(-90/90/180, rpm)`，先对准当前目标，再由现有转向闭环独占控制，完成后切换目标并消除残余角差。遥控转向不再发送 JY61 硬件归零；自主路线的 `left90/right90/right180` 继续原先的停稳、归零与确认流程。遥控先对准期间已接收的转向按键边沿保留为单次请求，安全取消或冲突会清除请求；IMU 无效时拒绝的请求不会在恢复后重放。

`RemoteHeading_GetStatus()` 提供四向目标、阶段、基准 yaw、相对目标/实际角度、偏差及输出轮速。USART1 的 `[RHEAD]` 日志每秒报告 `FRONT/RIGHT/BACK/LEFT`、`WAIT/ALIGNED/ALIGNING/DRIFTING/TURN/NO_IMU/PAUSED`、目标/偏差乘100以及纠偏 RPM。主要配置位于 `Core/Inc/heading_config.h` 的 `REMOTE_HEADING_*`。

当前固件支持 `(20).pro` 的 41 字节控制帧，并兼容 A5 的 21、23、25、27 字节控制帧，以及带两个保留字节的 29 字节帧、最多带 12 个 bool 的 31 字节姿态帧和带 `GAP` 的 35 字节帧。手机专业模式周期发送完整数据包；现有手机工程的控制周期为 30 ms、发送周期为 50 ms。双摇杆也使用独立的 35 字节组合帧。蓝牙 USART6 保持 9600 波特率，舵机 USART3 为 115200 波特率。

`(20).pro` 的顺序为 `A5`、原有 12 个 bool（2 字节）、以下 16 个小端 short、GAP（4 字节）、累加校验和 `5A`，共 **41 字节**：

```text
JOY_Y、forward、backward、stop、strafe_left、strafe_right、right_90、right_180、Cam_T、Shot、LEFT_90、servo_mode、joy_x、armcmd、armx、army
```

GAP 位于字节 35～38，校验位于 39（累加字节 1～38 的低 8 位），帧尾位于 40，以上偏移从 0 开始。`joy_x` 从原来的第 1 个 short 移到了第 13 个；此格式不使用 -8 标记。`servo_mode/armcmd/armx/army` 校验后不执行机械臂动作。GAP 初值为 556，只记录不发送。日志中的静止 GAP=556 帧尾为 `2C 02 00 00 2E 5A`，仅按前进时为 `2C 02 00 00 2F 5A`。此前最多解析 35 字节的固件会拒绝该工程的数据包，出现持续 `seq=0` 和 `invalid control frame`。以下为历史格式说明。

```text
JOY_X、JOY_Y、FORWARD、BACKWARD、STOP、STRAFE_LEFT、STRAFE_RIGHT、RIGHT_90、RIGHT_180
```

旧格式为 `A5` + 上述 9 个 short 的 18 字节 + payload 字节累加低 8 位校验 + `5A`，共 21 字节。23 字节帧在后面增加 `Cam_T`，25 字节帧再增加 `Shot`。新增左转的完整字段顺序为：

```text
JOY_X、JOY_Y、FORWARD、BACKWARD、STOP、STRAFE_LEFT、STRAFE_RIGHT、RIGHT_90、RIGHT_180、Cam_T、Shot、LEFT_90
```

`Shot` 是小端 short 逻辑值，位于 25 字节及更长车控帧的字节 21–22，仅接受 0 或 1。31/35 字节布尔帧还可用第 10 个布尔位 `SHOT`（字节 2 的 bit 1）请求同一射靶模式；两者任意一个为 1 即视为按下。新收到的 0→1 切换进入射靶模式，持续发送 1 不重复进入；1→0 退出蓝牙启动的射靶模式，关闭激光并急停底盘。相机模式请求在后台保留：收到二维码通知后发送，暂忙或失败自动重试。退出后先停在 `CAR_WAIT_CENTER`，收到新的方向键释放的车控帧且电机空闲后恢复 `CAR_READY` 遥控。`SHOT=1` 期间不会被同帧的 `Cam_T` 切换到视觉跟随；蓝牙 `SHOT` 不接管 PC1 物理按钮启动的射靶模式。单次射击完成、故障或断线时仍按原安全流程退出；持续发送 1 不会自动重射，须先发送 0 再发送 1。

扩展格式沿用 `A5` 帧头，加 12 个 short 的 24 字节、payload 字节累加低 8 位校验及 `5A` 帧尾，共 27 字节。`LEFT_90` 占字节 23–24，校验占字节 25，帧尾占字节 26；按下时发送 1，松开时发送 0。旧短帧未包含左转字段时按 0 处理。`left90()` 请求 20 RPM，控制器输出负向 20 RPM 的转向命令；目标角为左转 90°，其余停稳、修正及故障条件与右转 90° 相同。`BRAKE` 和 `DISABLE` 是控制层内部字段，不在手机包内。开关字段仅接受 0/1；JOY_X/Y 在线路解析层兼容 -1000..1000；两轴不再参与运动和解锁判断。A5 归中帧示例：

```text
A5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 5A
```

29 字节帧在 `LEFT_90` 后保留 2 字节（原 `SERVO_MODE`，字节 25–26）；校验位于字节 27，帧尾为字节 28。该字段已废弃，固件忽略其任意取值，不触发舵机动作，也不屏蔽底盘。旧 21/23/25/27 字节帧继续接收，旧 43 字节八按钮帧不再支持。

手机布尔姿态帧为 **31 字节**：`A5`、2 字节 bool 位图、13 个小端 short、字节 1～28 的累加低 8 位校验、`5A`。bool bit 0～7 依次为 TB_B、TB_M、TB_G、BD_U、BD_D、TH_C、TH_G、TH_U；bit 8 为 RST，bit 9 为 SHOT，bit 10 为 AIM，bit 11 为 TH_L，bit 12～15 必须为 0。末尾 short 保留。SHOT 进入视觉射靶流程；其余舵机按钮在 0→1 时产生一次发送请求，持续按住不重复，松开后可重按。RST 发送四路 T2000 姿态，不发送控制器 `$RST!`。

正式舵机链路为 `Bluetooth 解析/舵机仲裁 → Servo 接口 → UartTxQueue → UART_SEND → USART3`。预设唯一修改位置为 `Core/Src/servo.c`：每行一个完整动作组，每个动作由枚举索引四路 `{id,pwm,time}`，同文件保存名称、有效通道数并附中文动作注释。以下数值与表一致，本次不调整 PWM 或发送时间。

| 标识 | 动作 | ID 000 | ID 001 | ID 002 | ID 003 | T 时间/ms | 路数 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `TB_B` | 放球前 | 1717 | 2297 | 884 | 500 | 2000 | 4 |
| `TB_M` | 抓球中 | 1356 | 1850 | 698 | 1480 | 2000 | 4 |
| `TB_G` | 夹取球 | 1356 | 1850 | 698 | 500 | 2000 | 4 |
| `BD_U` | 防爆桶上方 | 1566 | 1896 | 673 | 500 | 2000 | 4 |
| `BD_D` | 放球 | 1566 | 1896 | 673 | 1200 | 2000 | 4 |
| `TH_C` | 抓人质前 | 1667 | 1882 | 651 | 1483 | 2000 | 4 |
| `TH_G` | 夹住人质 | 1667 | 1882 | 651 | 500 | 2000 | 4 |
| `TH_U` | 抓起人质 | 1800 | 1855 | 528 | 500 | 2000 | 4 |
| `TH_L` | 放低人质 | 1673 | 1394 | 732 | 500 | 2000 | 4 |
| `REFERENCE` | 参考姿态 | 1532 | 2219 | 1202 | 0 | 2000 | 3 |
| `RST` | 机械臂复位 | 1524 | 1163 | 1693 | 1486 | 2000 | 4 |
| `AIM` | 瞄准 | 1058 | 821 | 554 | 1499 | 2000 | 4 |
| `TH_PRE` | 人质预抓取 | 1684 | 2136 | 785 | 1483 | 2000 | 4 |

`TH_L` 使用新工程中的实际字段名，对应本次消息中写作 `TB_L` 的最后一组 PWM；位图位置为 bit 11（数据包字节 2 的 bit 3），不占用 `RST`、`SHOT` 或 `AIM` 的位置。`TH_G` 与 `TH_C` 共用 ID 000～002，只将 ID 003 改为 500。所有姿态组合指令均带 T，当前均为 `T2000`，AIM/RST 也相同；REFERENCE 只发送三路，表中 ID003 的 0 不发送。`TH_PRE` 保留为直接姿态 API，手机没有新增映射。

`(15).pro` 的 **35 字节 GAP 帧**在相同的 12 个 bool 和 13 个 short 后追加一个小端 int GAP：字节 29～32 为 GAP、字节 33 为校验、字节 34 为 5A；保留 short 必须为 -8，以区别历史双摇杆组合帧。GAP=0 不发送，500～2500 按 `{#003PxxxxT2000!}` 发送，范围外整帧拒收。首次连接或重连的首个有效帧仅记录 GAP 初值，随后数值变化才产生请求，保持同值不重复，回 0 后可重发。新工程的 GAP、AIM、TH_L 都已绑定控件。

AIM 发送 `{#000P1058T2000!#001P0821T2000!#002P0554T2000!#003P1499T2000!}`。USART3 使用 `HAL_UART_Transmit_IT`：当前帧保持到发送完成；另存一格待发指令，新的请求替换旧待发指令。返回 OK 表示已经提交或入队，不代表串口发送完成、机械动作完成或控制器应答。`ZLIS2_IsIdle()` 只表示传输链路空闲。HAL 未接受且未入队返回 BUSY 或 UART_ERROR，不自动重发可能部分发送的舵机命令。初始化只绑定串口，不发送姿态。

每轮最多选一项，正式配置优先级为 **AIM → RST → 其余姿态按 bool 索引 → 参考短指令 → GAP**；台架配置在最前加入 PE4。未选中的同周期边沿不补发。日志名称来自 `Servo_GetName()`，例如 `[SERVO] 瞄准 AIM TX=0 dropped=0 ...`；TX=0 是提交成功，TX=5 是 UART_ERROR，TX=6 是 BUSY；dropped 计入未选中项及未入队的 BUSY 项。入队和替换待发项按已确认契约返回 OK。`Bluetooth_GetAimSequence()` 仍统计手机 AIM 上升沿。在线判定统一使用 `Bluetooth_IsConnected()`；超过 500 ms 离线后不接收新的手机发送请求，不追加停止舵机指令。

参考姿态独立短指令：按下 `A5 31 00 31 5A`，松开 `A5 30 00 30 5A`。按下边沿在蓝牙在线时发送三路参考姿态；短指令不改写车控字段、不刷新蓝牙保活，也不触发底盘停车。29 字节帧中的旧数值字段仍忽略。

台架 PE4 内部上拉、低电平有效，20 ms 消抖。每次按下请求 AIM（带 T2000），长按不重复，上电已按住须先释放。正式配置只保留 PD10 启动键；手机 AIM/RST/姿态/GAP 继续可用。`[BT RX]` 每秒输出收字节、帧序号、在线及恢复次数；`[PE4]` 仅台架配置输出，`last=0` 表示提交成功。

默认启用 `CAR_BOOT_AUTO_ENABLE=1`。上电使能后须收到合法的方向/转向/安全按钮释放帧才可运动；首个合法控制包（包括 STOP/BRAKE/DISABLE）后启用 500 ms 失联保护。仅 `boot_enable_pending=1` 时 STOP/BRAKE 保留上电请求并等待中立帧；使能成功后立刻恢复 STOP/BRAKE 刹车锁优先级，DISABLE 始终急停失能。优先级为 DISABLE > STOP/BRAKE > 主动转向请求 > 单个平移按钮；转向开始和普通平移均经过方向门控，待机时也纠偏。STOP 与 BRAKE 都走安全急停并进入 `CAR_BRAKE_LOCK`；只有新的全方向/转向/安全按钮为 0 的有效帧且电机发送空闲时才能解锁，JOY_X/Y 不参与判断。普通平移按钮松开仅调用一次普通 `brake()`，不进入安全锁。转向按钮仅在 0→1 边沿启动一次非阻塞转向，松开不会取消已启动转向。多个平移按钮、两个右转按钮或左右转同时请求时普通停车；转向冲突取消后，须释放再按下才能启动新转向。`DISABLE` 先急停再失能，持续按住时保持失能。旧 13 字节控制帧和 ACTION_CMD 不再接受；PID 调参仍接受其独立 5 字节短帧。

当前 `CAR_TEST_INPUTS_ENABLE=0`；PD10 独立、麦轮测试、航向测试模式均要求测试输入开关为 1，不一致会编译/配置报错。当前 `CAR_PD10_STANDALONE_TEST`、`CAR_UART_BRIDGE_TEST`、`CAR_USART2_U1_BRIDGE` 均为 0。PD10 独立模式及串口桥用于单独诊断，不能按正常蓝牙控制行为理解。

### 航向与在线 PID

JY61 使用 USART2 的 256 字节循环 DMA 接收，默认每 5 ms 提交新增数据。控制层用软件零点形成相对航向；默认 PID 为 P=0.500、I=0.000、D=0.050，IMU 新鲜度阈值 150 ms（适配实测约 10 Hz 的帧频）。实车正转向指令使原始 yaw 减小，因此航向反馈输出采用反号。正式遥控每个新航向帧更新四向目标偏差，起步和静止时超过1°先原地纠偏；行驶时超过3°叠加最多4 RPM转向，连续超过10°达300 ms才暂停平移并原地纠偏；普通松手及STOP保留目标。自主及原有航向接口继续使用原先5°/2°阈值。日志 `YAW_CORRECTION` 为 PID 原始值、`OMEGA_APPLIED` 为最近一次送入麦轮混控的限幅值；`FAULT=3` 表示航向模块无故障。IMU 无效或超时会关闭航向修正，普通平移仍可能继续。

USART1 每秒输出 `[CTRL]` 的车辆状态、蓝牙帧序号/年龄、方向输入和 PID 参数（P100/I100/D100 表示参数乘以 100），以及 `[MSTOP]` 的急停请求数、四电机急停发送/完成/回复位图（M1=1、M2=2、M3=4、M4=8）和 UART5 错误计数。`[CTRL] STOP`、`BUTTON_RELEASE`、`LINK_LOST` 记录触发安全动作的输入事件；`ack` 只说明驱动器回复了急停命令，不是车轮静止的测量值。

蓝牙 PID 调参使用单 short ValuePack 帧，例如查询 `A5 09 00 09 5A`，打开 100 ms CTRL 输出 `A5 0A 00 0A 5A`，关闭 `A5 0B 00 0B 5A`。调参帧不刷新运动保活、不解除刹车锁；参数掉电丢失。完整按钮表见 [PID 蓝牙调参报告](PID_BLUETOOTH_TUNER_REPORT.md)。

## 视觉与路线接口

### MaxiCam 二维码通知

`main.c` 已调用 `MaxiCam_Init()` 和 `MaxiCam_Process()`。UART4 接收/错误由 `uart_driver.c` 按绑定路由到 MaxiCam 接收流；接收使用中断和前台队列处理。

当前只消费二维码识别成功通知 **二进制单字节 `0x80`**，不是字符串 `"80"`。UART 回调只收取字节，前台 `MaxiCam_Process()` 调用 `ActionFSM_SetQRSuccess(true)` 锁存结果；接收路径不调用刹车，每个独立成功字节都会更新计数及通知标志。进入静止扫码动作时会清除扫码前的旧通知，只接受扫码阶段的新成功通知。

第一段直行只由 `RouteFSM_NotifyActionEnd()` 的路段结束事件完成。刹车停稳后复用 JY61 闭环右转 90°，再刹车停稳并原地扫码。收到扫码阶段的新 `0x80` 后再次刹车停稳，复用闭环左转 90° 恢复朝向；完成后进入原有左平移。

扫码期间未识别成功时持续静止等待；普通目标包和 `DETECT_UNKNOWN` 不触发路线运动或任务目标搜索。

位置包沿用 `DetectData` 的 5 字节紧凑布局：`type`、小端有符号 `offset_x`、小端有符号 `offset_y`。解析器只在等待新包首字节时把独立 `0x80` 识别为二维码成功；位置包载荷中的 `0x80` 不会触发二维码。类型 0～7 为有效目标，其中 6 是桶、7 是打靶圆环；`DETECT_UNKNOWN=8` 为无效目标。UART 错误会丢弃残缺位置包并将目标置为无效。解析结果及递增帧序号通过 `MaxiCam_GetTargetData()` 读取。模式字节只有在 MaxiCam 发过独立二维码通知后才生效：请求可提前记录，未通知时不发送、不失败，收到后自动补发。圆环模式发 `0x00`，对象模式发 `0x01`。失败最多连续快速尝试 3 次（20 ms 间隔），之后每 200 ms 慢速尝试，保留期望且不锁死。发送成功后的 50 ms UART4 入流按到达时间丢弃并发布目标失效；残包静默超过 200 ms 自动丢弃并重新同步。每个独立 `0x80` 递增计数且通知 ActionFSM，复位通知锁存不清诊断计数。每秒 `[MAXICAM] rx=... err=... mode=cur/desired qr=...` 记录状态。正式主循环只接入车控视觉/SHOT 请求，未启动路线；历史路线在任务一/二完成时分别记录上述模式请求。UART 发送成功不代表摄像头已确认切换，协议无应答。

### MaxiCam 目标对准

当前 `main()` 调用的蓝牙 `Cam_T` 视觉跟随和蓝牙 `SHOT` 打靶对准（PE0/PC1 仅台架配置）由 `Car_Control_Process()` 控制，微调速度为 8 RPM。PE0 仍以 `offset_x=+10` 为中心，死区为闭区间 `0..20`；仅打靶改为 `offset_x=0` 为中心，死区为闭区间 `-10..10`。停车并等待电机通信队列空闲后，再等待 300 ms 余振时间；只用此后的 3 个有效新帧判定，采样至少间隔 100 ms。3 帧均在对应死区内且最大最小偏差不超过 4 像素才确认归中，射靶此时才允许触发。若停车后持续同侧偏离，则重新微调；偏差来回跨越死区时保持停车。`Motor_IsIdle()` 不表示机械臂已实际静止，8 RPM 的实车起步能力也仍需验证。

实车验证前检查机械臂关节、相机与激光固定、线缆牵拉及舵机保持状态；确认目标板、接线和安全条件后，先在安全条件下验证 8 RPM 能稳定起步和微移。若不能可靠移动，应停止该速度的验收并记录结果，不自动提高转速。

`ACTION_ALIGN_TARGET` 是可复用的 Action FSM 入口，未加入固定路线数组，也未绑定 Task1/Task2。进入动作后只处理进入之后到达的新位置帧：

- `offset_x > 20`：按 `SPEED_SLOW_RPM`（当前 100 RPM）前进微调。
- `offset_x < 0`：按 `SPEED_SLOW_RPM`（当前 100 RPM）后退微调。
- `0 <= offset_x <= 20`：累计一次死区确认；连续 3 个有效新帧后进入 `BRAKE → WAIT_STOP`。
- 超出死区会清零确认计数；无效目标帧会打断连续确认。

一般对准的中心及死区由 `MAXICAM_ALIGN_TARGET_X=10`、`MAXICAM_ALIGN_DEADZONE_X=10` 配置；射靶中心由 `MAXICAM_SHOT_TARGET_X=0` 配置，沿用相同死区宽度。确认帧数和丢失帧数仍由 `MAXICAM_ALIGN_CONFIRM_FRAMES=3`、`MAXICAM_TARGET_LOST_FRAMES=5` 配置。PE0、独立对准动作及非射靶路线任务保持原 X 边界；PC1／蓝牙 `SHOT` 和任务二圆环射靶使用射靶边界。运动仅使用 `offset_x`；`offset_y` 保留解析但不参与控制。MaxiCam 接口公开目标 `type`，任务状态据此筛选目标类型。

### 三位任务值

`vision_data.h` 提供 `VisionData_DecodeDigits()` 和 `VisionData_DecodeAscii()`，分别接收数值字节或 ASCII 字符，长度必须恰好为 3，每位只能是 1～3。

| 位置 | 含义 | 1 | 2 | 3 |
| --- | --- | --- | --- | --- |
| 第一位 | 排爆物颜色 | 红 | 绿 | 蓝 |
| 第二位 | 反恐靶颜色 | 红 | 绿 | 蓝 |
| 第三位 | 救援目标形状 | 圆柱 | 圆锥 | 腰鼓 |

全部校验成功后才写入输出，失败保持输出原值。该模块没有 HAL 依赖或运动动作，尚未从 UART4 自动提取三位任务载荷；任务调度通过 `MissionFSM_SetTaskData()` 接收已解码的任务值。

### 任务完成状态机

`mission_fsm.c` 由现有 `ACTION_TASK_1` / `ACTION_TASK_2` / `ACTION_TASK_3` 路线动作推进。排爆依次搜索爆炸物、对准、等待拾取事件、复用 JY61 闭环转 180°、搜索并对准桶、等待释放事件；反恐只接受圆环类型 7，以 `offset_x=0` 为中心、`-10..10` 为死区进行前后微调，首次进入死区便刹车，等待电机发送队列空闲及 100 ms 后，再用 3 个新圆环帧确认居中；自动低电平触发持续 2000 ms。救援依次搜索/对准/新帧确认、等待抓取事件，再通过返回路线终点事件完成。

每次寻找新目标时先向右搜索；只有识别到当前任务要求的类型才进入对准。任务一和三在对准时连续 5 帧收到明确的 `DETECT_UNKNOWN`，会刹车并向左回退搜索；错误类型和 UART 错误不计入该连续计数。任务二纠偏时若圆环帧失效则立即刹车，停稳后重新搜索；自动触发期间若新帧显示失靶或偏离中心，则立即撤销任务二触发请求并进入错误状态。

`Servo_Start()` 和 `Servo_IsDone()` 仍是路线任务的舵机适配占位，硬件层须轮询舵机请求并调用 `Servo_NotifyDone()`；蓝牙手动姿态发送不改变路线任务完成标志。正式配置 PC3 由射靶自动请求控制。台架配置额外与 PB8 手动请求合并：任一请求存在输出低电平；任务错误或取消只撤销自动请求，PB8 仍按住时继续低电平。当前 QR 接收只提供独立 `0x80` 成功字节，未提供三位任务载荷；开始任务前调用方须在 `RouteFSM_Init()` 后设置 `MissionFSM_SetTaskData()`。任务一默认匹配协议中的桶类型 6，可通过 `MissionFSM_SetBucketDetectType()` 覆盖。任务状态机不在输入缺失时虚报完成。

### 固定路线

`route_fsm.c` 定义 17 步：前进 → 右转90° → 静止扫码 → 左转90° → 左平移 → 前进 → 左平移 → 右转90° → 任务1 → 左转占位 → 前进到任务2 → 任务2 → 前往右转口 → 右转90° → 任务3 → 返回终点 → 完成。

- `RouteFSM_Init()` 显式开始新路线，`RouteFSM_Update()` 在前台推进；当前正式主循环没有调用这两个接口。
- 平移结束由外部调用 `RouteFSM_NotifyActionEnd()` 通知；仅在已启动的有效平移动作期间接受。
- 路线完成由 `qr_success`、各任务实际完成事件和终点通知驱动；`task1_done` 只在释放舵机完成后置位，`task2_done` 只在激光计时结束后置位，`task3_done` 只在已抓取且终点到达后置位。
- 模式请求在任务完成时记录，路线继续下一步；二维码前置、UART 忙、错误及超时由 MaxiCam 延迟/重试处理，不因模式发送进入 ROUTE_ERROR。
- 左转仍是安全占位，只刹车并等待 `turn_left_done`，不会发出旋转命令。
- 刹车后先等待 `Motor_IsIdle()`，再非阻塞等待 100 ms；这不是机械停止反馈。
- `MISSION_DONE` 仅在三个任务完成且最终路线完成时进入；随后保持急刹并关闭激光。错误锁定 `ROUTE_ERROR`，不会自动跳步或重新启动。

路线运行时必须独占运动命令，不能同时让 `Car_Control_Process()` 或其他方向接口控制电机。接入自动路线前还需完成启动授权、视觉路段终点和任务执行模块的绑定。

### 右转 90° / 180°

`turn_right.h` 提供遥控专用 `TurnRight_StartRemote(degrees, rpm)`，以及自主用 `right90(rpm)`、`right180(rpm)`、`left90(rpm)`、`TurnRight_Process(motion_allowed)`、`TurnRight_Cancel()` 与 `TurnRight_GetStatus()`。左右转共用同一套 JY61 相对航向闭环与停稳确认流程。

动作从 `TURNING` 开始，距目标 30° 时把请求轮速降至对应动作的接近速度上限，且不高于初始转速：右转 180° 为 20 RPM，左转和右转 90° 均为 20 RPM。在目标 ±5° 范围内急停。`STOPPING` 期间继续累计滑行产生的 JY61 相对转角；四轮命令发送完毕且角速度稳定 100 ms 后，只有停稳误差不超过 ±5° 才允许完成：自主入口进入 `RESETTING → DONE`，遥控入口直接进入 `DONE`，再由四向状态机对最终目标做1°/0.5°精调。若超差，则进入 `CORRECTING`，仍按该动作的接近速度上限向目标修正并重新停稳检查，最多修正三次；反向累计达到 10°、连续 1 秒未取得 3° 进展、IMU 失效、电机故障、整次转向超过 10 秒、修正次数用尽或归零失败均请求急停并进入故障状态。跨 ±180° 的 yaw 跳变在整个停稳前均按相对角累计。

调用返回 `HAL_OK` 表示接受请求，完成以 `DONE` 为准。实车日志显示右转时 JY61 yaw 减小；固件按此极性累计，独立于航向保持 PID 的符号。±5° 是 JY61 停稳相对角的控制门槛，物理角度仍需实车标记验证。`[TURN]` 日志记录开始、进度、停稳角度、修正次数、停止原因及 FE 入队结果；`[M1]`～`[M4] FE OK` 表示各电机回复急停成功。动作期间需独占控制，每轮按 `JY61_Process()` → `TurnRight_Process(motion_allowed)` → `Motor_Process()` 推进；取消时应调用 `TurnRight_Cancel()` 或传入 `false`。

## 蓝牙分发与板级输入

正式输入流程为 `Bluetooth_Process → Car_Control_SubmitRemoteInput / PID_Tuner_HandleCommand`，舵机在原主循环后段由 `Bluetooth_DispatchServoActions(physical_aim_press)` 调用统一 Servo 接口。Bluetooth 驱动直接依赖功能接口，没有额外应用分发模块。解析有效控制帧只提交快照，不直接派发电机或舵机；PID 与参考短指令不刷新车控保活。

车控公开中立 `CarCommand_t/CarRemoteInput_t/CarLocalInput_t`，不包含 Bluetooth 类型，不读取蓝牙 Getter。远程快照包含命令、序号、接收时间和有效性；车控使用同一 500 ms 接收时间租约。UART 错误和接收溢出立即调用 `Car_Control_InvalidateRemoteInput()`，此 ISR 安全接口只置失效标志；实际停止仍由两次 `Car_Control_Process()` 的原状态机推进。历史 arm_tuner 的蓝牙 Getter 仅测试使用，未接入固件。

`board_inputs.c/h` 负责输入 GPIO 初始化、采样与消抖，配置集中在 `board_input_config.h`（启动键仍使用 `start_button_config.h`）。PE0/PC1/PD10/启动键在首次车控前处理，事件通过 `Car_Control_SubmitLocalInput()` 提交并只消费一次；PE4 在原舵机阶段采样，仍优先于同周期手机请求；PB8 和按钮显示在原 Board_Process 阶段处理。PE0/PC1/PE4 与启动键上电已按住须释放后重按；PD10 独立测试仍按住消抖后运行、释放立即停止；PB8 首次辅助采样启动 20 ms 消抖、释放立即撤销。

Laser 只处理自动/手动请求的合并和 PC3 输出；板级通过 `Laser_SetManualRequest()` 设置手动请求。正式配置不采样这些台架输入；启动键仍只输出一次 `[START] press`。公开 Servo 接口可由蓝牙以外的调用方使用，不包含整条 BT→SV 输入链。

## 公共 UART 与独立队列

`uart_driver.c/h` 是正式链路唯一 HAL UART 收发与全局回调入口；历史 X42 除外。`UART_SEND(data,length,uart,mode,timeout)` 接收二进制长度并返回 HAL 状态；IT 模式要求存储保持到 TC，blocking 模式沿用 MaxiCam 单字节、JY61 归零及可选 AT 的超时。`UART_RECV(data,capacity,uart,&received,ticks)` 非阻塞读取，空读返回 HAL_OK/0；ticks 可为空，中断流有逐字节时间戳，DMA 流保持原批次时间戳，也可通过 `UART_RxTick()` 读取。USART2 循环 DMA 与原错误统计保持不变。

UART 注册表按 handle 路由接收、TX 队列和错误事件。RX 错误/恢复不终止 TX。桥接显式独占相关串口，初始化失败恢复原绑定并报告 RX 流中断，让协议层重新同步。旧模块回调入口仅作兼容转发，不拥有 HAL 逻辑。

`uart_tx_queue.c/h` 共用算法与发送状态，各模块提供独立静态存储，无堆分配。接口为 Init、Submit、Reserve/Commit、Process、CancelPending、IsIdle 和 Complete，并支持业务 tag。半成品在 Commit 前不可被 TC 消费；正在发送的内容保持到完成。事件钩子只做有界状态/计数更新，日志格式化在前台。协议解析、设备重试和安全决策仍由模块负责。

| 实例 | 容量与策略 | 派发/失败行为 |
| --- | --- | --- |
| 电机 | FIFO，15 个待发，另有当前帧 | 前台派发；2 ms 帧间隔、超过 20 ms 超时；急停取消待发但保留当前帧；启动失败保留队首并进入电机故障 |
| 舵机 | 当前帧＋一个最新待发 | 提交时可立即发送，TC 可续发；启动拒绝且未入队返回 BUSY/错误；失败不重发，不新增动作保护时间 |
| PID | FIFO，8 个待发，另有当前帧 | 500 ms 超时边界包含等于；启动失败保留回复；周期诊断只在空闲时发送，失败不积压 |
| 日志 | FIFO，15 个待发，另有当前帧 | 满时丢弃新日志；超过 200 ms 恢复 TX；启动失败保留队首 |
| 桥接 | 每方向 1023 字节，包含当前字节 | 顺序发送；满时丢弃新字节；启动失败保留队首 |

队列不决定 MaxiCam 是否已扫码，也不判断 JY61 是否归零成功。主循环的两次车控处理及电机安全优先级不变。

### 舵机枚举编号

手写原值 0～8 保持，追加四个动作；BEGIN/END 为范围别名，不新增数据行。

| 值 | ServoCode | 动作标识 |
| --- | --- | --- |
| 0 | TakeBall_Before | TB_B |
| 1 | TakeBall_Mid | TB_M |
| 2 | TakeBall_Gap | TB_G |
| 3 | BarrelDown_Up | BD_U |
| 4 | BarrelDown_Down | BD_D |
| 5 | TakeHostage_Up | TH_U |
| 6 | TakeHostage_Catch | TH_C |
| 7 | TakeHostage_Gap | TH_G |
| 8 | TakeHostage_Leave | TH_L |
| 9 | Servo_REFERENCE | REFERENCE（三路） |
| 10 | Servo_RST | RST |
| 11 | Servo_AIM | AIM |
| 12 | TakeHostage_PreGrab | TH_PRE |

`ServoCode_MAX=13`，`ServoCode_NONE=-1`。`GetFullCommand()` 为纯协议构造：四路容量 63 B/发送 62 B，REFERENCE 三路容量 48 B/发送 47 B；使用共享补零 writer，包含 T，不使用 snprintf。`Servo_SendPreset(code)` 使用已绑定的舵机端口和队列；`Servo_SendGap(pwm)` 提交单路 GAP。ID003 动态 GAP 仍单独发送 `{#003PxxxxT2000!}`。所有数值与重构前一致。

## 舵机与历史驱动

`servo.h` 提供单一舵机接口，预设、ASCII 协议及队列全部在 `servo.c`。`Servo_SetValues()` 发送 ID000 起的最多四路 PWM；`Servo_SetCommands()` 支持任意协议 ID 和历史 24 路能力，统一类型为 `ServoCommand_t {id,pwm,time_ms}`。所有组合指令带 T，旧 ZLIS2/ServoPose/ServoRemote 接口和文件已删除。GAP 沿用 `{#003PxxxxT2000!}` 单路组合格式。发送路径无 snprintf/vsnprintf 和 363 B 栈缓冲，协议字符串保持逐字节一致。

USART3 115200 8N1、中断优先级 3，全局 HAL 回调由 uart_driver 统一分派。驱动拥有两份 362 B 帧缓存保存当前及最新待发项；预设仍为 Flash 常量。队列由公共 `UartTxQueue_t` 管理，Servo 构造协议并映射 `ServoStatus_t`（0～6 数值不变），不读取蓝牙或 GPIO。旧 arm_control/arm_tuner 及旧项目运动学帮助函数仅历史测试使用；正式微调使用独立运动学核心。微调持有 Servo 独占时拒绝其他姿态与 GAP；其传输必须确认实际 STARTED/TC，完整接入行为见 ARM_TRIM_INTEGRATION.md。

手机布尔姿态映射使用分组 `ServoCode`；旧 `ServoPose_t` 及其适配层已删除。旧蓝牙数值模式映射及其 0～10 编号已删除；29/31/35 字节帧的保留 short 仍占原有位置，以维持其他字段的线协议布局。

旧 `arm_control.c`、`arm_tuner.c` 及历史回归保留；正式固件不支持旧 @ARM 调参或双摇杆控制。新 `@ARM TRIM` 接口和微调停止由独立 core/service 处理，原接口与手机页说明见 ARM_TRIM_INTEGRATION.md。

实车联调须确认控制器供电、共地、PB10→控制器 RX、控制器实际波特率及运动空间。此前日志中的 `TX OK` 仅表示 STM32 完成发送，不能证明舵机控制器已接收或机械臂到位。

`x42.c/.h` 保留历史阻塞驱动及测试，当前整车电机控制使用 `motor_driver` / `emm42_driver`，不调用其旧请求流程。X42 电机历史驱动与 USART6 候选视觉设备 XDK42 是不同模块。

## 详细报告与依赖

以下报告保留开发过程及当时的验证范围；历史波特率、默认模式和接入状态可能与当前固件不同。

- [整车驱动报告](CAR_DRIVER_REPORT.md)
- [麦克纳姆驱动重构报告](MECANUM_REFACTOR_REPORT.md)
- [运动控制更新报告](MOTION_CONTROL_UPDATE_REPORT.md)
- [JY61 航向报告](JY61_HEADING_REPORT.md)
- [PID 蓝牙调参报告](PID_BLUETOOTH_TUNER_REPORT.md)
- [单一 Servo 与输入分发重构记录](SERVO_INPUT_LAYER_REPORT.md)
- [UART、队列与枚举重构记录](UART_QUEUE_SERVO_REFACTOR_REPORT.md)
- [ZL-IS2 驱动报告](ZLIS2_DRIVER_REPORT.md)

早期串口联调原始记录：`hardware-f3-com9.json`、`hardware-loopback-com9.json`、`hardware-status-com9.json`。这些记录不能作为当前整车固件的实车验收结论。

原工程记录的依赖版本为 STM32F4 HAL v1.8.5、CMSIS Device F4 v2.6.10、ARM CMSIS 5.9.0；实际随附内容位于 `Drivers/`，许可证随各依赖保留。工程不包含 CubeMX `.ioc` 或 CubeIDE 工程配置。

## 本次上传验证（2026-09-22）

- ARM GCC 全量编译、链接通过，生成 ELF / HEX / BIN；Flash 占用 28,664 B，RAM 占用 15,040 B。
- `tests/car/run.ps1`、`tests/vision/run.ps1`、`tests/route/run.ps1`、`tests/zlis2/run.ps1` 及 X42 主机测试通过。
- 本次仅更新 README 并整理 Git 提交，电控源码、配置和厂商依赖保持原始内容。
- 未重新验证 Keil 构建，未烧录或操作电机。
