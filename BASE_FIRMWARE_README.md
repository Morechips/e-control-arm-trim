# JNDS 电控固件

机械臂完整交接上下文（2026-10-02）：[ARM_HANDOFF_20261002.md](ARM_HANDOFF_20261002.md)，包含当前参数、协议、固件身份、待重录事项和新会话开场说明。

当前测试副本为 v4.5（2026-10-02）：蓝牙测试页的抓球前、抓人质前、放球前仅发送 000～002，保留夹爪。抓人质旧姿态待重录；本次修改、测试及烧录见 [固定动作说明](ARM_FIXED_ACTION_V4_5.md)。以下通用预设说明包含原团队入口，不能混同于三个独立测试页动作。

基于 STM32F407 和 STM32 HAL 的四轮麦克纳姆小车电控工程，包含蓝牙遥控、Emm42 电机通信、JY61 航向控制、MaxiCam 二维码通知、ZL-IS2 舵机驱动及路线/动作状态机。

仓库：<https://github.com/SMG0128/JNDS>。本文按 2026-09-22 的源码整理；历史联调过程见文末报告，运行配置以当前代码为准。

## 当前功能与接入状态

| 功能 | 当前状态 |
| --- | --- |
| 蓝牙整车遥控 | 已接入主循环，X 控制左右平移，Y 控制前后平移 |
| 四轮电机控制 | Emm42 协议、发送队列、四轮同步、急停和故障处理 |
| JY61 航向保持 | 已接入，USART2 DMA 接收，软件航向零点与 PID 修正 |
| 蓝牙 PID 调参 | 已接入，与摇杆共用 USART6；参数仅保存在 RAM |
| MaxiCam 二维码与目标偏移 | 已接入 UART4；区分独立 `0x80` 二维码包和 5 字节位置包 |
| 视觉任务三位值解码 | 独立接口已实现，尚未与完整视觉串口协议绑定 |
| 固定路线与任务状态机 | 路线及排爆/反恐/救援状态链已实现；正式主循环仍未启动路线 |
| 原地右转 90° | 独立状态机已实现；正式遥控未绑定触发入口 |
| ZL-IS2 舵机控制器 | USART3 接入机械臂状态机、逆运动学、`@ARM` 调参、35 字节双摇杆帧及 `SERVO_MODE`/PE4；路线任务仍使用舵机完成事件占位 |
| XDK42 | 仅保留 USART6 候选配置，当前串口由蓝牙占用 |
| OLED / 按钮显示 | 保留 SSD1306 显示及按钮状态显示 |
| PB8 红外触发按钮 | 按下接地；按住请求 PC3 低电平触发，松开撤销手动请求 |

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
tests/arm/         机械臂限位、状态机与停止测试
tests/arm_kinematics/  机械臂逆运动学测试
tests/arm_bt/      @ARM 与 35 字节组合帧测试
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
| USART3 | ZL-IS2 舵机控制器 | PB10 / PB11 | 115200 | `Core/Inc/zlis2_driver.h` |
| UART4 | MaxiCam 视觉 | PC10 / PC11 | 115200 | `Core/Inc/vision_config.h` |
| UART5 | 四轮电机 | PC12 / PD2 | 115200 | `Core/Inc/main.h` |
| USART6 | 蓝牙 | PC6 / PC7 | 9600 | `Core/Inc/main.h`、`Core/Inc/usart6_config.h` |

- OLED：I2C1，PB6 SCL / PB7 SDA，100 kHz；SSD1306 128×64，7 位地址 `0x3C`。
- 按钮：PD10、PD11、PD14、PD15，上拉输入，低电平有效，20 ms 消抖。
- 舵机模式 1 按钮：PE4 内部上拉，按钮按下接地时输入低电平 `0`，20 ms 消抖；松开后由内部上拉恢复高电平 `1`。PE4 每次按下均请求模式 1，无需外接 3.3 V。
- 红外手动按钮：PB8 内部上拉，按下接地；按下稳定 20 ms 后触发，松开立即撤销手动触发。
- 红外触发输出：PC3 开漏低电平有效；无触发请求时配置为无上下拉输入，由外接模块完成红外发射。
- 时钟：内部 HSI 16 MHz，AHB / APB1 / APB2 不分频。
- GCC 链接范围：Flash `0x08000000` 起 512 KiB，SRAM `0x20000000` 起 128 KiB；不使用 CCM，无 bootloader 偏移。
- 蓝牙目标名称为 `AIOTCAR`；启动改名命令为 `AT+NAME=AIOTCAR\r\n`，能否生效取决于模块的 AT 指令支持。

USART6 的蓝牙与 XDK42 用途互斥。当前选择 `USART6_OWNER_BLUETOOTH`；`XDK42_USART6_BAUD_RATE=115200` 只是候选配置，不能视为已实现 XDK42 接收和识别。

## 获取与构建

私有仓库需要具备访问权限的 GitHub 账号。

```powershell
git clone https://github.com/SMG0128/JNDS.git
cd JNDS
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 4
```

需将 CMake 3.22 或以上、Ninja、`arm-none-eabi-gcc`、`arm-none-eabi-objcopy` 和 `arm-none-eabi-size` 加入 `PATH`。工程自动使用 `cmake/arm-none-eabi.cmake`，C 标准为 C11，应用代码启用 `-Wall -Wextra -Werror`。

Windows也可直接运行`powershell -ExecutionPolicy Bypass -File .\scripts\build_firmware.ps1`，产物写入`firmware_direct`；该脚本包含队友的`servo_pose.c`/`servo_remote.c`与机械臂模块。

| 产物 | 用途 |
| --- | --- |
| `build/stm32f407_bt_oled.elf` | 调试镜像 |
| `build/stm32f407_bt_oled.hex` | 含地址的固件镜像 |
| `build/stm32f407_bt_oled.bin` | 从 `0x08000000` 开始的裸镜像 |
| `build/stm32f407_bt_oled.map` | 链接映射 |

Keil 用户打开 `MDK-ARM/stm32f407_bt_oled.uvprojx`，选择 `STM32F407_BT_OLED` 并 Rebuild。工程沿用 ARM Compiler 5 配置；器件设置与下载算法需按实际芯片核对。Keil 与 GCC 使用各自的启动文件、链接脚本和运行库配置。

CLion 使用运行配置 `Horco CMSIS-DAP (OpenOCD)` 烧录 `stm32f407_bt_oled` 目标。该配置在烧录前构建固件、每次下载 ELF，并在下载后复位运行。板级配置位于 `openocd/horco-cmsis-dap-stm32f407.cfg`，使用 Horco CMSIS-DAP v2、SWD 和 STM32F4 目标；更换探针时需更新其中的序列号与接口。当前脚本默认OpenOCD为`D:\Tool\xpack-openocd-0.12.0-7\bin\openocd.exe`，也可用`-OpenOcdExecutable`指定。默认固件上电会使能四轮，确认目标板、接线及安全条件后才可运行`scripts\flash_firmware.ps1 -ConfirmHardwareReady`；本轮整合未烧录硬件。

## 主机回归测试

测试使用主机 `gcc` 和 HAL 替身，不连接硬件。Windows 环境需保证 `gcc` 及其运行库可从 `PATH` 找到。

```powershell
powershell -ExecutionPolicy Bypass -File tests/car/run.ps1
powershell -ExecutionPolicy Bypass -File tests/vision/run.ps1
powershell -ExecutionPolicy Bypass -File tests/route/run.ps1
powershell -ExecutionPolicy Bypass -File tests/zlis2/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_kinematics/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_bt/run.ps1
```

历史 X42 驱动另行测试：

```powershell
$env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
New-Item -ItemType Directory -Force build | Out-Null
gcc -std=c11 -Wall -Wextra -Werror -Itests/x42 -ICore/Inc Core/Src/x42.c tests/x42/test_x42.c -o build/test_x42.exe
./build/test_x42.exe
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

X 型麦轮混控支持二维平移，航向 PID 以独立 `omega` 修正叠加。四轮统一缩放后限制在 ±500 RPM，航向修正限制为 ±100 RPM。普通摇杆不直接产生连续旋转指令。

包含 `mecanum.h` 后可使用 `up(rpm)`、`down(rpm)`、`left(rpm)`、`right(rpm)` 和 `brake()`。方向接口接收非负 RPM；返回 `HAL_OK` 只代表命令入队，忙时返回 `HAL_BUSY`。调用方负责初始化、使能、安全条件并持续调用 `Motor_Process()`。

`brake()` 取消排队的运动命令，在当前发送帧完成后优先发送四轮急停；它不等待机械停止、不失能，也不建立整车控制层的持续刹车锁。

### 路线动作的三档速度

`Core/Inc/car_config.h` 集中定义慢速 100 RPM、中速 133 RPM、全速 167 RPM。旧的 `SPEED_NORMAL` / `SPEED_PRECISE` 名称保留为兼容别名；`SpeedMode_GetRPM()` 返回三档固定值。

| 场景 | 速度档位 |
| --- | --- |
| 任务目标搜索（排爆物/桶）及所有对准 | 慢速，100 RPM |
| 反恐靶和人质搜索 | 中速，133 RPM |
| 任务2后前往右转口、右转和人质返回 | 全速，167 RPM |

速度属于当前任务状态或路线动作；按队友最新手机参数，蓝牙方向键平移为 35 RPM。所有调用入口的右转和左转 90° 上限均为 20 RPM，右转 180° 上限为 30 RPM；右转 180° 接近目标和自动微调时降至不超过 20 RPM。扫码后的路线左转同样受 20 RPM 全局上限约束。PD10 诊断速度和摇杆速度上限不受转向限速影响。

### 蓝牙控制协议

当前固件兼容 5 字节独立按钮帧、17 字节旧双摇杆帧，以及 A5 的 21、23、25、27、29、31、33、35、41 字节控制帧。29 字节 `SERVO_MODE`、31 字节 bool 姿态、35 字节 bool+`GAP`、35 字节双摇杆，以及本地和队友两种 41 字节布局均保留。手机专业模式周期发送完整数据包；蓝牙 USART6 保持9600波特率，舵机 USART3 为115200波特率。

```text
JOY_X、JOY_Y、FORWARD、BACKWARD、STOP、STRAFE_LEFT、STRAFE_RIGHT、RIGHT_90、RIGHT_180
```

旧格式为 `A5` + 上述 9 个 short 的 18 字节 + payload 字节累加低 8 位校验 + `5A`，共 21 字节。23 字节帧在后面增加 `Cam_T`，25 字节帧再增加 `Shot`。新增左转的完整字段顺序为：

```text
JOY_X、JOY_Y、FORWARD、BACKWARD、STOP、STRAFE_LEFT、STRAFE_RIGHT、RIGHT_90、RIGHT_180、Cam_T、Shot、LEFT_90
```

扩展格式沿用 `A5` 帧头，加 12 个 short 的 24 字节、payload 字节累加低 8 位校验及 `5A` 帧尾，共 27 字节。`LEFT_90` 占字节 23–24，校验占字节 25，帧尾占字节 26；按下时发送 1，松开时发送 0。旧短帧未包含左转字段时按 0 处理。`left90()` 请求 20 RPM，控制器输出负向 20 RPM 的转向命令；目标角为左转 90°，其余停稳、修正及故障条件与右转 90° 相同。`BRAKE` 和 `DISABLE` 是控制层内部字段，不在手机包内。开关字段仅接受 0/1；JOY_X/Y 接受 -1000..1000，死区 ±50。A5 归中帧示例：

```text
A5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 5A
```

29 字节帧在 `LEFT_90` 后追加一个 short `SERVO_MODE`，占字节 25–26；校验位于字节 27，帧尾为字节 28。模式允许 0～10。仅模式值变化时触发一次，持续发送同一值不重复。车辆未就绪、轮控未空闲或操控未归中时机械臂动作当次拒绝且不排队；它不会再停止或锁住底盘。

31 字节 bool 姿态帧为 `A5`、2 字节 bool 位图、原顺序 13 个小端 short、校验、`5A`。bool 0～7 为八个姿态，8 为 `RST`，9 为 `SHOT`，10 为 `AIM`，11 为 `TH_L`。`SHOT` 可发起射击；`AIM` 保留为独立状态位；`TH_L` 不会误映射为左转，本地无可靠标定姿态时安全拒绝。按钮只在 0→1 边沿触发，多个机械臂姿态同时按下不执行。

布尔姿态帧使用本地机械臂动作表。发送姿态要求蓝牙在线、车辆 `CAR_READY`、轮控空闲、操控归中、机械臂非忙且未进入调参会话；被拒绝的按下不排队，也不会影响底盘继续接收其自身控制。

| 布尔按钮 | 姿态 | ID 003 PWM |
| --- | --- | ---: |
| `TB_B` | 抓球前 | 2192 |
| `TB_M` | 抓球中 | 2192 |
| `TB_G` | 夹取球 | 500 |
| `BD_U` | 防爆桶上方 | 500 |
| `BD_D` | 放球 | 1800 |
| `TH_C` | 抓人质 | 1800 |
| `TH_G` | 夹住人质 | 500 |
| `TH_U` | 抓起人质 | 500 |

35 字节 bool+`GAP` 布局用 `servo_mode=-8` 与同长度双摇杆格式区分。`GAP=0` 表示不调整，500～2500 表示 ID003 绝对 PWM，发送时间 1000 ms；范围外整帧拒收。连接或重连后的首个 GAP 只作为基准，不会让夹爪突然动作；之后仅数值变化触发。车辆运动时该机械臂请求被拒绝，但底盘不被锁住。

41 字节同时支持两种已见实机布局：本地 `(7).pro` 的 `bool[12]+JOY_X 起始的13 short+arm_cmd/arm_x/arm_y+GAP`（以 `servo_mode=-8` 标记），以及队友最新 `(20).pro` 的 `bool[12]+JOY_Y/F/B/STOP/L/R/R90/R180/Cam_T/Shot/L90/servo_mode/JOY_X/armcmd/armx/army+GAP`。队友页面无需重排即可控制底盘；三项机械臂字段也接入本地机械臂核心。

模式 1 另有独立按钮指令：按下 `A5 31 00 31 5A`，松开 `A5 30 00 30 5A`。短指令不刷新车控保活；机械臂条件不满足时当次拒绝，不再锁住底盘。

PE4 松开时为高电平，按钮按下接地后稳定变为低电平，便请求模式1的三路姿态文本 `{#000P1532T1000!#001P2219T1000!#002P1202T1000!}`；持续按住只触发一次，松开后再次按下可重新发送。PE4无需连接蓝牙，也不检查底盘状态，但必须经过统一的机械臂状态机：机械臂正在运行、停止、故障或处于 `@ARM`/双摇杆会话时，本次按下直接拒绝且不排队，不再覆盖旧动作。蓝牙 `SERVO_MODE` 仍支持1～10，模式7为空动作。上电时按钮已接地不会立即触发，须先松开再按下。USART3发送失败会锁存机械臂传输故障；实际姿态文本从PB10（USART3 TX）输出，PB11是输入。

机械臂组合格式在底盘字段后携带 `ARM_CMD / ARM_X / ARM_Y`。`ARM_X/Y` 范围为 -1000～1000；`ARM_CMD` 的 0/1 为释放或归中，10/11 为腕部小步，12/13 为夹爪小步，14 为下一固定姿态，20～25 为会话控制。17/27/29/31/33/35 字节旧组合格式继续兼容；同长度格式按机械臂会话状态或 `servo_mode=-8` 标记消歧。两种41字节格式均同时刷新车控和机械臂序号。

默认启用 `CAR_BOOT_AUTO_ENABLE=1`。上电使能后须收到合法归中数据才可运动；首个合法控制包后启用 500 ms 失联保护。优先级为 DISABLE > STOP/BRAKE > 转向 > 单个平移按钮 > JOY。STOP 与 BRAKE 都走安全急停并进入 `CAR_BRAKE_LOCK`；只有新的全按钮 0 且双轴归中帧才能解锁。普通平移按钮松开仅调用一次普通 `brake()`，不进入安全锁。转向按钮仅在 0→1 边沿启动一次非阻塞转向，松开不会取消已启动转向。多个平移按钮、两个右转按钮或左右转同时请求时普通停车；转向冲突取消后，须释放再按下才能启动新转向。`DISABLE` 先急停再失能，持续按住时保持失能。旧 13 字节控制帧和 ACTION_CMD 不再接受；PID 调参仍接受其独立 5 字节短帧。

当前 `CAR_PD10_STANDALONE_TEST`、`CAR_UART_BRIDGE_TEST`、`CAR_USART2_U1_BRIDGE` 均为 0。PD10 独立模式及串口桥用于单独诊断，不能按正常蓝牙控制行为理解。

### 航向与在线 PID

JY61 使用 USART2 的 256 字节循环 DMA 接收，默认每 5 ms 提交新增数据。控制层用软件零点形成相对航向；默认 PID 为 P=0.500、I=0.000、D=0.050，IMU 新鲜度阈值 150 ms（适配实测约 10 Hz 的帧频）。实车正转向指令使原始 yaw 减小，因此航向反馈输出采用反号。手动平移的实际航向修正最多为较大平移分量的 20%；松手停稳后清除手动航向基准，下一段平移以当时车头朝向重新取零。日志 `YAW_CORRECTION` 为 PID 原始值、`OMEGA_APPLIED` 为最近一次送入麦轮混控的限幅值；`FAULT=3` 表示航向模块无故障。IMU 无效或超时会关闭航向修正，普通平移仍可能继续。

USART1 每秒输出 `[CTRL]` 的车辆状态、蓝牙帧序号/年龄、方向输入和 PID 参数（P100/I100/D100 表示参数乘以 100），以及 `[MSTOP]` 的急停请求数、四电机急停发送/完成/回复位图（M1=1、M2=2、M3=4、M4=8）和 UART5 错误计数。`[CTRL] STOP`、`BUTTON_RELEASE`、`JOYSTICK_CENTER`、`LINK_LOST` 记录触发安全动作的输入事件；`ack` 只说明驱动器回复了急停命令，不是车轮静止的测量值。

蓝牙 PID 调参使用单 short ValuePack 帧，例如查询 `A5 09 00 09 5A`，打开 100 ms CTRL 输出 `A5 0A 00 0A 5A`，关闭 `A5 0B 00 0B 5A`。调参帧不刷新运动保活、不解除刹车锁；参数掉电丢失。完整按钮表见 [PID 蓝牙调参报告](PID_BLUETOOTH_TUNER_REPORT.md)。

## 视觉与路线接口

### MaxiCam 二维码通知

`main.c` 已调用 `MaxiCam_Init()` 和 `MaxiCam_Process()`。UART4 完成/错误回调由 `uart_bridge.c` 分派到 MaxiCam；接收使用中断和前台队列处理。

当前只消费二维码识别成功通知 **二进制单字节 `0x80`**，不是字符串 `"80"`。UART 回调只收取字节，前台 `MaxiCam_Process()` 调用 `ActionFSM_SetQRSuccess(true)` 锁存结果；接收路径不调用刹车，标志已置位时不重复触发。进入静止扫码动作时会清除扫码前的旧通知，只接受扫码阶段的新成功通知。

第一段直行只由 `RouteFSM_NotifyActionEnd()` 的路段结束事件完成。刹车停稳后复用 JY61 闭环右转 90°，再刹车停稳并原地扫码。收到扫码阶段的新 `0x80` 后再次刹车停稳，复用闭环左转 90° 恢复朝向；完成后进入原有左平移。

扫码期间未识别成功时持续静止等待；普通目标包和 `DETECT_UNKNOWN` 不触发路线运动或任务目标搜索。

位置包沿用 `DetectData` 的 5 字节紧凑布局：`type`、小端有符号 `offset_x`、小端有符号 `offset_y`。解析器只在等待新包首字节时把独立 `0x80` 识别为二维码成功；位置包载荷中的 `0x80` 不会触发二维码。类型 0～7 为有效目标，其中 6 是桶、7 是打靶圆环；`DETECT_UNKNOWN=8` 为无效目标。UART 错误会丢弃残缺位置包并将目标置为无效。解析结果及递增帧序号通过 `MaxiCam_GetTargetData()` 读取。任务一完成且已扫码后，路线经 UART4 发送单字节 `0x00` 切到圆环模式；任务二激光结束后发送 `0x01` 切回对象模式。UART 发送成功不代表 MaxiCam 已确认切换，协议没有应答。

### MaxiCam 目标对准

当前 `main()` 调用的 PE0 视觉跟随和 PC1 打靶对准由 `Car_Control_Process()` 控制，微调速度为 8 RPM。统一以 `offset_x=+10` 为目标中心，死区为 `±10`，即闭区间 `0..20`。停车并等待电机通信队列空闲后，再等待 300 ms 余振时间；只用此后的 3 个有效新帧判定，采样至少间隔 100 ms。3 帧均在死区内且最大最小偏差不超过 4 像素才确认归中，PC1 此时才允许触发。若停车后持续同侧偏离，则重新微调；偏差来回跨越死区时保持停车。`Motor_IsIdle()` 不表示机械臂已实际静止，8 RPM 的实车起步能力也仍需验证。

实车验证前检查机械臂关节、相机与激光固定、线缆牵拉及舵机保持状态；确认目标板、接线和安全条件后，先在安全条件下验证 8 RPM 能稳定起步和微移。若不能可靠移动，应停止该速度的验收并记录结果，不自动提高转速。

`ACTION_ALIGN_TARGET` 是可复用的 Action FSM 入口，未加入固定路线数组，也未绑定 Task1/Task2。进入动作后只处理进入之后到达的新位置帧：

- `offset_x > 20`：按 `SPEED_SLOW_RPM`（当前 100 RPM）前进微调。
- `offset_x < 0`：按 `SPEED_SLOW_RPM`（当前 100 RPM）后退微调。
- `0 <= offset_x <= 20`：累计一次死区确认；连续 3 个有效新帧后进入 `BRAKE → WAIT_STOP`。
- 超出死区会清零确认计数；无效目标帧会打断连续确认。

目标中心、死区、确认帧数和丢失帧数分别由 `MAXICAM_ALIGN_TARGET_X=10`、`MAXICAM_ALIGN_DEADZONE_X=10`、`MAXICAM_ALIGN_CONFIRM_FRAMES=3`、`MAXICAM_TARGET_LOST_FRAMES=5` 配置。PE0、PC1、独立对准动作及路线任务使用同一组 X 边界。运动仅使用 `offset_x`；`offset_y` 保留解析但不参与控制。MaxiCam 接口公开目标 `type`，任务状态据此筛选目标类型。

### 三位任务值

`vision_data.h` 提供 `VisionData_DecodeDigits()` 和 `VisionData_DecodeAscii()`，分别接收数值字节或 ASCII 字符，长度必须恰好为 3，每位只能是 1～3。

| 位置 | 含义 | 1 | 2 | 3 |
| --- | --- | --- | --- | --- |
| 第一位 | 排爆物颜色 | 红 | 绿 | 蓝 |
| 第二位 | 反恐靶颜色 | 红 | 绿 | 蓝 |
| 第三位 | 救援目标形状 | 圆柱 | 圆锥 | 腰鼓 |

全部校验成功后才写入输出，失败保持输出原值。该模块没有 HAL 依赖或运动动作，尚未从 UART4 自动提取三位任务载荷；任务调度通过 `MissionFSM_SetTaskData()` 接收已解码的任务值。

### 任务完成状态机

`mission_fsm.c` 由现有 `ACTION_TASK_1` / `ACTION_TASK_2` / `ACTION_TASK_3` 路线动作推进。排爆依次搜索爆炸物、对准、等待拾取事件、复用 JY61 闭环转 180°、搜索并对准桶、等待释放事件；反恐只接受圆环类型 7，以 `offset_x=+10` 为中心、`0..20` 为死区进行前后微调，首次进入死区便刹车，等待电机发送队列空闲及 100 ms 后，再用 3 个新圆环帧确认居中；自动低电平触发持续 2000 ms。救援依次搜索/对准/新帧确认、等待抓取事件，再通过返回路线终点事件完成。

每次寻找新目标时先向右搜索；只有识别到当前任务要求的类型才进入对准。任务一和三在对准时连续 5 帧收到明确的 `DETECT_UNKNOWN`，会刹车并向左回退搜索；错误类型和 UART 错误不计入该连续计数。任务二纠偏时若圆环帧失效则立即刹车，停稳后重新搜索；自动触发期间若新帧显示失靶或偏离中心，则立即撤销任务二触发请求并进入错误状态。

`Servo_Start()` 和 `Servo_IsDone()` 仍是路线任务的舵机适配占位，硬件层须轮询舵机请求并调用 `Servo_NotifyDone()`；蓝牙手动姿态发送不改变路线任务完成标志。PC3 由任务二自动触发与 PB8 手动按钮共同控制：任一请求存在便输出低电平，两者都结束才恢复悬空。任务错误或取消只撤销自动请求；若 PB8 仍被按住，PC3 继续保持低电平。当前 QR 接收只提供独立 `0x80` 成功字节，未提供三位任务载荷；开始任务前调用方须在 `RouteFSM_Init()` 后设置 `MissionFSM_SetTaskData()`。任务一默认匹配协议中的桶类型 6，可通过 `MissionFSM_SetBucketDetectType()` 覆盖。任务状态机不在输入缺失时虚报完成。

### 固定路线

`route_fsm.c` 定义 17 步：前进 → 右转90° → 静止扫码 → 左转90° → 左平移 → 前进 → 左平移 → 右转90° → 任务1 → 左转占位 → 前进到任务2 → 任务2 → 前往右转口 → 右转90° → 任务3 → 返回终点 → 完成。

- `RouteFSM_Init()` 显式开始新路线，`RouteFSM_Update()` 在前台推进；当前正式主循环没有调用这两个接口。
- 平移结束由外部调用 `RouteFSM_NotifyActionEnd()` 通知；仅在已启动的有效平移动作期间接受。
- 路线完成由 `qr_success`、各任务实际完成事件和终点通知驱动；`task1_done` 只在释放舵机完成后置位，`task2_done` 只在激光计时结束后置位，`task3_done` 只在已抓取且终点到达后置位。
- 模式命令 UART 忙时重试；发送错误或超时使路线刹车并进入错误状态。单字节命令在对应任务完成时发送一次，发送成功后才进入下一路线动作。
- 左转仍是安全占位，只刹车并等待 `turn_left_done`，不会发出旋转命令。
- 刹车后先等待 `Motor_IsIdle()`，再非阻塞等待 100 ms；这不是机械停止反馈。
- `MISSION_DONE` 仅在三个任务完成且最终路线完成时进入；随后保持急刹并关闭激光。错误锁定 `ROUTE_ERROR`，不会自动跳步或重新启动。

路线运行时必须独占运动命令，不能同时让 `Car_Control_Process()` 或其他方向接口控制电机。接入自动路线前还需完成启动授权、视觉路段终点和任务执行模块的绑定。

### 右转 90° / 180°

`turn_right.h` 提供 `right90(rpm)`、`right180(rpm)`、`left90(rpm)`、`TurnRight_Process(motion_allowed)`、`TurnRight_Cancel()` 与 `TurnRight_GetStatus()`。左右转共用同一套 JY61 相对航向闭环与停稳确认流程。

动作从 `TURNING` 开始，距目标 30° 时把请求轮速降至对应动作的接近速度上限，且不高于初始转速：右转 180° 为 20 RPM，左转和右转 90° 均为 20 RPM。在目标 ±5° 范围内急停。`STOPPING` 期间继续累计滑行产生的 JY61 相对转角；四轮命令发送完毕且角速度稳定 100 ms 后，只有停稳误差不超过 ±5° 才进入 `RESETTING → DONE`。若超差，则进入 `CORRECTING`，仍按该动作的接近速度上限向目标修正并重新停稳检查，最多修正三次；反向累计达到 10°、连续 1 秒未取得 3° 进展、IMU 失效、电机故障、整次转向超过 10 秒、修正次数用尽或归零失败均请求急停并进入故障状态。跨 ±180° 的 yaw 跳变在整个停稳前均按相对角累计。

调用返回 `HAL_OK` 表示接受请求，完成以 `DONE` 为准。实车日志显示右转时 JY61 yaw 减小；固件按此极性累计，独立于航向保持 PID 的符号。±5° 是 JY61 停稳相对角的控制门槛，物理角度仍需实车标记验证。`[TURN]` 日志记录开始、进度、停稳角度、修正次数、停止原因及 FE 入队结果；`[M1]`～`[M4] FE OK` 表示各电机回复急停成功。动作期间需独占控制，每轮按 `JY61_Process()` → `TurnRight_Process(motion_allowed)` → `Motor_Process()` 推进；取消时应调用 `TurnRight_Cancel()` 或传入 `false`。

## 舵机与历史驱动

`zlis2_driver.h` 提供 ZL-IS2 UART ASCII 接口。`arm_control.c` 是正式主循环中唯一的舵机发送入口，管理限位、动作时长、停止及故障；`arm_kinematics.c` 提供平面三关节逆运动学，`arm_tuner.c` 接收 `@ARM` 调参及双摇杆输入。`servo_pose.c` 默认生成 ID 000～002 的三关节预设动作，由机械臂状态机在 USART3 上发送 `{#000PxxxxT1000!#001PxxxxT1000!#002PxxxxT1000!}`。8 个布尔姿态均在同一姿态帧中额外发送对应的 `#003PxxxxT1000!`；数值模式和 PE4 不发送 ID 003，旧值在源码注释中备份。蓝牙 `GAP` 仍可单独调整 ID 003，`@ARM` 调参和双摇杆夹爪功能保持现状。预设动作时间为 1000 ms。发送成功只表示控制器串口传输完成，状态机的完成标志按时长估算，没有舵机位置或夹持力反馈。主循环不自动执行路线任务的舵机请求。

v4.4 当前关节活动范围为 ID 000：915～1800，001：947～2500，002：500～1874，通用手动夹爪为 900～2192。它们是活动度，不保证无碰撞。模式1参考为 1532 / 2219 / 1202，模式6复用模式1。原 bool 固定姿态也必须遵守三关节范围，只有原夹爪通路继续允许 500～2500；原 P003 序列保持。旧 HOSTAGE_LIFT（数值模式10）含 000=P1800，按用户确认的新上限恢复通过关节限位检查，当前包络模型判定其与后方箱体有干涉；旧固定动作入口仍仅检查关节范围。其余记录不截断或改写。`@ARM LIMIT`更改仅在 RAM 生效。三个微调测试参考动作及最新范围见 `ARM_REAR_BOX_V4_4.md`。

代码中的 `ServoMode_t` 姿态枚举从 0 递增：0 无动作，1 抓球前，2 抓球中，3 夹取球，4 移动到防爆桶上方，5 放球，6 抓人质，7 夹住人质，8 抓起人质。它是姿态编号；蓝牙帧中的 `SERVO_MODE` 仍按下表使用原有 0～10 编号，蓝牙 6 复用姿态 1，蓝牙 7 为空动作。姿态枚举的新增不改变预设 PWM 或蓝牙帧格式。

| `SERVO_MODE` | 姿态 | ID 000 / 001 / 002 的 PWM | 当前状态 |
| --- | --- | --- | --- |
| 0 | 无动作 | 不发送 | 无动作 |
| 1 | 参考姿态（原抓球前） | 1532 / 2219 / 1202 | 可请求 |
| 2 | 抓球中 | 1421 / 2071 / 812 | 可请求 |
| 3 | 夹球 | 1421 / 2071 / 812 | 可请求 |
| 4 | 防爆桶上方 | 1717 / 2297 / 884 | 可请求 |
| 5 | 放球 | 1717 / 2297 / 884 | 可请求 |
| 6 | 参考姿态（复用 1） | 1532 / 2219 / 1202 | 可请求 |
| 7 | 占位 | 不发送 | 空动作 |
| 8 | 抓人质 | 1499 / 1855 / 528 | 可请求 |
| 9 | 夹住人质 | 1499 / 1855 / 528 | 可请求 |
| 10 | 抓起人质 | 1800 / 1855 / 528 | 限位可请求，箱体包络模型判干涉 |

`@ARM`文本命令以LF结束，由USART6接收，回复也走USART6。`@ARM SHOW`查看状态，`@ARM LINK`查看最近有效车控/机械臂帧长度、序号、错误帧计数和连接状态，`@ARM IK X Z PHI`只计算逆解；`@ARM SETUP`、`ENTER`、`SYNC`、`JOG`、`LIMIT`等调参命令需遵守机械臂状态机授权与限位。发送`@ARM DUAL START\n`会以2000 ms将ID 000～003移至参考值1532 / 2219 / 1202 / 1200；等待`ARM DUAL READY`，再发送摇杆归中的组合帧，收到`JOYSTICKS_ACTIVE`后可操作双摇杆。`ARM_CMD=14`或`@ARM PRESET NEXT`按八个原始固定姿态循环，每步1500 ms；也可用`@ARM PRESET 0..7`指定。固定动作仅在底盘`CAR_READY`、电机空闲、底盘操控归中、机械臂空闲且双摇杆会话已经授权时接受，否则拒绝且不排队。`@ARM STOP`停止，`@ARM EXIT`退出会话。`@ARM`会话期间队友的`SERVO_MODE`、bool、GAP及PE4请求均被拒绝；预设动作执行中启动会话会返回忙碌。上电不会由STM32自动移动机械臂。

实车联调须确认控制器供电、共地、PB10→控制器 RX、控制器实际波特率及运动空间。此前日志中的 `TX OK` 仅表示 STM32 完成发送，不能证明舵机控制器已接收或机械臂到位。

`x42.c/.h` 保留历史阻塞驱动及测试，当前整车电机控制使用 `motor_driver` / `emm42_driver`，不调用其旧请求流程。X42 电机历史驱动与 USART6 候选视觉设备 XDK42 是不同模块。

## 详细报告与依赖

以下报告保留开发过程及当时的验证范围；历史波特率、默认模式和接入状态可能与当前固件不同。

- [整车驱动报告](CAR_DRIVER_REPORT.md)
- [麦克纳姆驱动重构报告](MECANUM_REFACTOR_REPORT.md)
- [运动控制更新报告](MOTION_CONTROL_UPDATE_REPORT.md)
- [JY61 航向报告](JY61_HEADING_REPORT.md)
- [PID 蓝牙调参报告](PID_BLUETOOTH_TUNER_REPORT.md)
- [ZL-IS2 驱动报告](ZLIS2_DRIVER_REPORT.md)
- [队友最新版机械臂合并说明](ARM_INTEGRATION_REPORT.md)

早期串口联调原始记录：`hardware-f3-com9.json`、`hardware-loopback-com9.json`、`hardware-status-com9.json`。这些记录不能作为当前整车固件的实车验收结论。

原工程记录的依赖版本为 STM32F4 HAL v1.8.5、CMSIS Device F4 v2.6.10、ARM CMSIS 5.9.0；实际随附内容位于 `Drivers/`，许可证随各依赖保留。工程不包含 CubeMX `.ioc` 或 CubeIDE 工程配置。

## 本次本地整合验证（2026-09-29）

- ARM GCC全量编译、链接通过，生成ELF / HEX / BIN；`text=65,884 B`、`data=104 B`、`bss=16,184 B`。
- `tests/car/run.ps1`、`tests/vision/run.ps1`、`tests/route/run.ps1`、`tests/zlis2/run.ps1`、`tests/arm/run.ps1`、`tests/arm_kinematics/run.ps1`、`tests/arm_bt/run.ps1`及X42主机测试通过。
- `arm_bt`覆盖17/27/29/31/33/35/41字节机械臂协议、错误校验、拆包、双摇杆、八姿态循环、按钮边沿及底盘安全拒绝；队友29/31/35字节舵机功能由整车测试覆盖。
- 未验证Keil构建，未烧录或操作电机；实机舵机方向、碰撞空间、蓝牙长期稳定性仍须台架确认。
