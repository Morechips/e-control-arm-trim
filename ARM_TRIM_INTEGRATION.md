# 机械臂模块接入说明（v4.6 / team-main，2026-10-03）

> 本文保留v4.6交付记录。当前v4.7服务/接口/烧录见 [UNIFIED_MODULE_INTEGRATION.md](UNIFIED_MODULE_INTEGRATION.md)，手机见 [UNIFIED_CONTROL_GUIDE.md](UNIFIED_CONTROL_GUIDE.md)。旧版先END才能切入口的流程已由统一服务替代。

本目录 `D:\工科大\e-control-arm-trim-latest-20261003` 是新拉取的队友 `gpnu-in-jnds/e-control` main，基线 `e7404c592df92c7c5e7a9d09562e302ebf52e2b3`。在此基线上导入已有模块化机械臂、服务与输入适配、安装配置向导，并重新运行回归和完整构建。旧工作目录和已核对固件不改，本次未推送或操作硬件。

可烧录产物在 `firmware_direct`，默认启用机械臂模块。完整蓝牙配置见 [BLUETOOTH_CONTROLLER_GUIDE.md](BLUETOOTH_CONTROLLER_GUIDE.md)，精确基线、改动范围与本次验证见 [LATEST_INTEGRATION_REPORT.md](LATEST_INTEGRATION_REPORT.md)。

## 模块边界

2026-10-03补充：新增电脑端安装配置与总线舵机动作录入向导，见 [ARM_SETUP_GUIDE.md](ARM_SETUP_GUIDE.md)。配置适配层现在支持各关节两点标定角、轨迹策略和夹爪限位；默认值保持原安装参数。`scripts/setup_arm.ps1` 自动查询已调好姿态的P，三份配置先导出再应用；多步动作组为独立数据导出，未接入固件播放器。本次整合固件已重新构建到本目录 firmware_direct；旧目录的已核对镜像不改。

| 层 | 文件 | 负责什么 |
| --- | --- | --- |
| 可移植核心 | arm_kinematics、arm_collision、arm_trim 的 .h/.c | 正逆解、参数化障碍、整段预检、速度规划、微调及停止状态机；只依赖 C 标准库和 math |
| 当前机械安装 | arm_trim_project.h/.c、arm_trim_project_config.h、arm_collision_config.h | 显式构造几何、标定、限位、后箱模型；换机械臂时替换这些参数 |
| 项目服务 | arm_trim_service.h/.c | 三组参考姿态、定时等待、夹爪、Servo 独占及异步传输映射；不读蓝牙或 GPIO |
| 输入接入 | arm_trim_input.h/.c、arm_trim_input_config.h | 手机按钮和文本、按住租约、停车条件、回复与主循环接入 |
| 传输 | servo.h/.c、uart_tx_queue、uart_driver | 统一协议、可追踪的 STARTED/TC、独占保护和超时；保持队友固定动作表 |

移植纯微调只需前三对核心文件，提供 `ArmTrimConfig_t` 和 `ArmTrimIO_t`。核心不包含 HAL、蓝牙、Servo、car_control 或本项目配置头。`ArmTrim_t` 必须放静态区或调用方的长期存储区，约 8.5 KB，不能放当前 2 KB 主栈。可运行 `examples/arm_trim_portable/run.ps1` 验证没有 HAL 的独立用法。

原五字段 `ArmTrimIO_t` 同步接口保留。非阻塞串口另调用 `ArmTrim_SetAsyncIO`，提供 poll 与 cancel_pending：send/stop 成功表示接受；poll COMPLETE 必须提供实际开始和发完时间。普通分段按开始时间推进，末段和正常释放按发完时间加 T 等待。取消删除待发运动，并依次确认 000、001、002 停止命令。异步失败、停发超时和停止失败撤销参考。

`arm_kinematics_legacy_project.c` 仅供历史 arm_control/arm_tuner 主机测试，正式固件不编译这些旧业务。当前正式构建重新使用的是纯运动学核心。

## 当前行为

默认 `ARM_TRIM_ENABLE=1`（CMake、直接 GCC、Keil）；CMake `-DARM_TRIM_ENABLE=OFF` 或直接脚本 `-DisableArmTrim` 可关闭接入。主循环保留队友车控两次安全处理顺序，在原 Servo 阶段检查机械臂输入与停车条件后推进传输。上电不发送机械臂动作。

Servo 会话从空闲串口取得独占，直到 END、取消/故障停止及串口排空后才释放。持有参考期间，旧姿态、GAP、原始 Servo 命令会返回 BUSY；被拒绝的旧按钮不会自动补发。现有固定动作、41 字节整车协议、方向按钮与车控流程仍使用队友最新版逻辑。整车 STOP/BRAKE/DISABLE、蓝牙接收错误及溢出会取消微调并撤销参考。

参考固定动作只发送 000..002，T=2000 ms，再从 TC 等待 T+300 ms；READY 才自动建立参考。夹爪只发送 003，T=1500 ms，再从 TC 等待 T+300 ms，正常完成保留平面偏移。发完并不等于收到或物理到位，状态始终是估计值。

| 参考 | P000/P001/P002 | 当前模型允许 X 偏移（mm） |
| --- | --- | --- |
| BALL | 1356 / 1850 / 698 | -75..+4 |
| HOSTAGE | 1684 / 2136 / 785 | -11..+56 |
| BUCKET | 1566 / 1896 / 673 | -42..+14 |

HOSTAGE 沿用旧安装值，仍待用户重新记录，未臆造替代数值。夹爪关为 P500、开为 P1800。L1=104.85、L2=84.75、工具偏移=(121.1538,52.4) mm；关节限位分别 915..1800、947..2500、500..1874。微调保持 Z 和末端角度，仅改变 X；速度上限 10 mm/s、加速度 20 mm/s²、分段周期 50 ms。

当前后箱为 X[-70,-30]、Y[-50,50]、Z[-31.2,38.8] mm；包络半径 15/20/60 mm 是估计值，额外间隙 5 mm。用户此前对长按版本有定性联调反馈；本目录本次构建未做实车验收，上述偏移范围仍属于模型计算结果。READY/PREP 和队友已有固定姿态的整个移动路径没有被微调规划器做碰撞预检；它们不能据此称为安全轨迹。现有 HOSTAGE_LIFT=1800/1855/528 在当前箱体模型中被挡，队友原动作表仍保留原值。

预检仍同步完成，最多保存 512 段；本次没有测量它在 16 MHz MCU 上的计算时间。实机验收需记录预检对合作式主循环的阻塞时间及实际串口节拍，不能以主机测试耗时替代该测量。

## 手机与文本兼容

7 字节独立页：`A5 | buttons | byte | ydnum_lo | ydnum_hi | sum | 5A`，byte 为 0 或 84，sum 为字节 1..4 累加低八位。按钮位：BALL=1、HOSTAGE=2、BUCKET=4、wt/JOG=8、CLOSE=16、OPEN=32、STATUS=64、STOP=128。完整包每 100 ms 发送。

ydnum 是 -150..150 的有符号 short，只取正负方向；0 不启动。固定按钮只在按下沿执行；wt 持续按住刷新 500 ms 租约，松开正常减速，超时或 STOP 停止并撤销参考。方向改变要求松开后重新按下。冲突、陈旧、忙碌、非法或已到边界的按住请求不会自动重试。STOP 优先于非法数值。独立页不刷新整车蓝牙租约。

11 字节旧独立页也保留：四个 LE short 为 cmd、dx、close_p、open_p，加头尾与 payload sum。cmd 0 释放；1..3 PREP；4..6 BEGIN；7 DX；8 END；9/10 GRIP；11 STATUS；12 STOP；13 CLEAR。cmd 需变化才形成新按下沿。该页也不刷新车控租约。

为防止非法 7 字节包恰好带有合法 PID 前缀，开启微调时，5 字节 PID/独立参考按钮包需遇到下一个 A5 或超过原 100 ms 字节间隔才确认结束；关闭微调保持队友原处理方式。

文本需 LF（可用 scripts/arm_codec.ps1 转 HEX）：

```text
@BENCH READY BALL        # 移到三关节参考，再自动建立估计参考
@BENCH PREP HOSTAGE      # 只移动，不建立微调参考
@ARM TRIM BEGIN BALL     # 不移动；操作员明确断言已在该姿态且稳定
@ARM TRIM DX 2           # 相对当前偏移再移动 +2 mm（X 也是同义命令）
@ARM TRIM JOG 1          # 按住请求，后续 KEEP 1 每 100 ms
@ARM TRIM KEEP 1
@ARM TRIM RELEASE
@BENCH CLOSE
@BENCH OPEN
@BENCH GRIP 700
@ARM TRIM STATUS
@ARM TRIM STOP
@ARM TRIM END
@ARM TRIM CLEAR          # 故障停止结束并释放后，清故障；不恢复参考
```

代码要求车控处于 READY/OFF/WAIT_CENTER/BRAKE_LOCK/LINK_LOST，电机通信空闲且无故障。此软件条件不能确认真实机械稳定；BEGIN 的姿态断言仍由操作员负责。服务状态数字：0 IDLE、1 PROFILE、2 REFERENCE、3 MOTION、4 GRIP、5 STOPPING、6 FAULT。

## 构建与验收

在本目录，设置 ARM GCC 到 PATH 后运行 `powershell -ExecutionPolicy Bypass -File scripts/build_firmware.ps1`。产物为 firmware_direct/stm32f407_bt_oled.elf/.hex/.bin/.map，另有 arm-trim-v4.6-manifest.json，记录构建参数、固件及源文件 SHA256。

主机回归：tests/uart、zlis2、servo、arm_trim、arm_collision、arm_kinematics、arm_trim_service、arm_trim_input、arm、arm_bt、car、vision、route 各自的 run.ps1。服务测试同时包含替身边界及真实 Servo/UART 队列联动。主机验证不证明实机停止或到位。

已通过：同步微调 48708、异步微调 327、碰撞 40、运动学 61、服务 2762、真实 Servo/UART 1029、手机解析 266126、实际输入层 184、Servo 协议 186、原蓝牙 Servo 8103/8201、历史 arm 237 和 arm_bt 22652；uart/car/vision/route 整套回归通过。独立移植示例也已通过。整车测试有两处默认 PID 精确比较受 MinGW x87 中间精度影响，已用存储后的 float 默认参数比较并显式设置格式测试输入，生产 PID 参数未改。

本次直接 GCC 产物 text=80360、data=476、bss=24004 字节。BIN SHA256 为 `6CAFD7AFFB78A1AF0CCE500E4A0F798AC9EFF522C2855F5EAD8C3B30F0D01E9F`；HEX 为 `6880E52CE55393608580C23C8D04B8749D61FA6CC25878965F3A510A4C6CC71A`。ELF 哈希及精确构建输入见随固件生成的 manifest。C 编译使用 -Wall -Wextra -Werror；链接器仍输出 nosys 的未实现系统调用提示，这些不用于设备通信。

本次直接 GCC 完整构建及关闭微调的完整构建均成功。CMake/Keil 源清单已同步，Keil XML 已检查，但本次未执行 CMake/Keil 编译。

以下是供操作员使用的完整命令；本次没有连接、烧录或复位硬件：

```powershell
Set-Location 'D:\工科大\e-control-arm-trim-latest-20261003'
$env:PATH = 'C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin;' + $env:PATH
powershell -ExecutionPolicy Bypass -File .\scripts\build_firmware.ps1
# 操作员确认目标板、供电、SWD、机械臂支撑与车轮离地之后执行：
powershell -ExecutionPolicy Bypass -File .\scripts\flash_firmware.ps1 -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File .\scripts\verify_firmware.ps1
# 离线检查诊断地址（不连接硬件）：
powershell -ExecutionPolicy Bypass -File .\scripts\read_bluetooth_diagnostics.ps1 -SymbolsOnly
```

完整手机配置以 BLUETOOTH_CONTROLLER_GUIDE.md 为准；接入与本次测试记录见 LATEST_INTEGRATION_REPORT.md。
