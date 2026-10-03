# 队友最新代码与机械臂模块整合记录

日期：2026-10-03。工作目录：`D:\工科大\e-control-arm-trim-latest-20261003`。

## 基线与范围

从 `https://github.com/gpnu-in-jnds/e-control.git` 新克隆 main，HEAD 为 `e7404c592df92c7c5e7a9d09562e302ebf52e2b3`。本次拉取时远端没有比上次模块化接入更晚的 main 提交，因此可沿用已适配的 Servo 接口，再重新验证整合结果。

机械臂来源为本地 `e-control-arm-trim-integration` 的模块化版本，包含后续安装配置工具。旧工作目录、用户原 GitHub 源码目录和已核对固件均未改。本目录 origin 仍指向队友仓库；本次未 commit、push、烧录、复位或运行真实 COM 查询。

## 实际接入

- 六个纯核心文件：arm_kinematics、arm_collision、arm_trim；不依赖 HAL、蓝牙、底盘或项目参数。
- 安装参数层：arm_trim_project/config 和 arm_collision_config；保留现有几何、标定、关节范围、后箱模型和三组参考。
- 服务层：arm_trim_service；负责非阻塞动作、自动参考、夹爪和 Servo 独占。
- 输入层：arm_trim_input/config；7 字节页面、11 字节兼容页、文本、长按租约、停车条件及回复。
- 统一 Servo 增加可追踪的独占传输；保持队友原固定动作表和非会话发送行为。
- 蓝牙在原 USART6 接收和前台解析内增加独立入口；没有另装 UART 接收回调，也没有改蓝牙波特率/引脚。
- 主循环在原舵机阶段推进服务；保留队友车控在 Motor_Process 前后两次安全处理顺序。
- CMake、Keil 和直接 GCC 源清单启用 ARM_TRIM_ENABLE，可显式关闭。
- 附带电脑安装配置/动作录入工具，自动查询 #NNNPRAD!，按 #NNNPdddd! 返回读取。自定义多步动作组只录入/导出数据，尚未接入蓝牙动作播放器。

除新文件外，生产已有文件仅改变 `Core/Inc/bluetooth_driver.h`、`Core/Inc/servo.h`、`Core/Src/arm_kinematics.c`、`Core/Src/bluetooth_driver.c`、`Core/Src/main.c`、`Core/Src/servo.c`。历史运动学项目帮助函数拆到仅测试使用的 `arm_kinematics_legacy_project.c`。底盘、电机、JY61、MaxiCam、路线、共享 UART 驱动和 ST/CMSIS 内容与拉取基线一致（比较时仅忽略 CRLF/LF）；队友固定动作表逐项一致。

测试适配保留两处 MinGW 浮点默认值比较修正，仅改变测试，不改变生产 PID。新增 Servo 所有权/超时、核心/服务/蓝牙/串口录入等回归。具体修改可直接在本目录 Git diff 中查看；上游 CMSIS 的 CRLF 与 text 属性可能产生全行换行差异，不应把这类差异当成供应商升级。

## 当前用户行为

机械臂 7 字节页面：三个固定动作仅发送000～002并自动建立参考，夹紧003=P500，松开003=P1800；ydnum取正负，wt按住连续平移，松开减速停止。模型范围约 BALL -75..+4、HOSTAGE -11..+56、BUCKET -42..+14 mm；保留现有限位915..1800 / 947..2500 / 500..1874。当前源码 L2=84.75 mm，其他安装参数未重新标定。

HOSTAGE仍为1684/2136/785，待改装后重录。队友整车页面的旧姿态是另一套动作，多数包含003，详见蓝牙指引；它们未被新规划器自动做限位/碰撞检查。三组用户参考的移动路径也未做完整碰撞预检，微调模型只覆盖所配置后箱和估计包络。

当前固件无机械臂运行时位置反馈，预计完成不等于实物到位；视觉没有接入机械臂微调，左右回调只预留。新构建的完整实车验收仍待执行。

## 本次验证结果

| 项目 | 实际结果 |
| --- | --- |
| tests/arm_trim | 同步48708、异步327检查通过；包含整条轨迹、长按、边界与失败无发送 |
| tests/arm_collision | 40检查通过；只验证模型 |
| tests/arm_kinematics | 61检查通过 |
| tests/arm_trim_service | 2762服务检查、1029真实Servo/UART队列联动检查通过 |
| tests/arm_trim_input | 266126解析检查、184实际输入层检查通过 |
| tests/servo | 正式/台架两种输入配置，8103/8201检查通过；旧整车姿态及41字节协议兼容 |
| tests/zlis2 | 186协议检查通过 |
| tests/uart | 共享串口、队列、完成时间、错误与超时回归通过 |
| tests/car | 全套整车、航向、PID、视觉/射靶、转向、输入互锁及桥接回归通过 |
| tests/vision、tests/route | 全套回归通过 |
| 历史tests/arm、tests/arm_bt | 237 / 22652检查通过，历史模块仍只在主机测试 |
| examples/arm_trim_portable | 无HAL/项目参数的独立示例通过 |
| tests/arm_setup | 40项Python测试通过，含假串口、配置应用回滚、生成头的真实C编译检查 |
| 默认完整固件 | ARM GCC 14.3.1，C编译-Wall -Wextra -Werror通过；text80360、data476、bss24004 B |
| 关闭微调的完整固件 | -DisableArmTrim通过；text46936、data100、bss15140 B；产物另存build-local |
| 诊断脚本 | -SymbolsOnly解析新ELF地址/字段宽度通过；未连接探针 |
| Keil配置 | XML及源清单检查；未运行Keil编译 |
| 蓝牙指引 | 文中的完整HEX示例离线核对长度、累加和与负short编码 |

完整 GCC 链接仍有 newlib-nano/nosys 的未实现 `_close/_fstat/...` 提示；C 源码没有警告，这些系统调用不用于设备通信。本次未执行 CMake/Keil 构建，不将源清单检查说成编译通过。

主机命令可复跑：

```powershell
Set-Location 'D:\工科大\e-control-arm-trim-latest-20261003'
$env:PATH = 'D:\c++\MinGW\bin;C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin;' + $env:PATH
$env:CAR_TEST_BUILD_DIR = 'build-local'
foreach ($suite in @('arm_trim','arm_collision','arm_kinematics','arm_trim_service','arm_trim_input','servo','zlis2','uart','car','vision','route','arm','arm_bt')) {
    & "tests/$suite/run.ps1"
}
& examples/arm_trim_portable/run.ps1
& 'D:\Anaconda\python.exe' -X utf8 -m unittest discover -s tests/arm_setup -v
powershell -ExecutionPolicy Bypass -File scripts/build_firmware.ps1
powershell -ExecutionPolicy Bypass -File scripts/build_firmware.ps1 -DisableArmTrim -OutputDirectory build-local/firmware_without_trim
powershell -ExecutionPolicy Bypass -File scripts/read_bluetooth_diagnostics.ps1 -SymbolsOnly
```

## 产物与核对

默认可烧录版本为 `firmware_direct/stm32f407_bt_oled.elf/.hex/.bin`，构建标识 `v4.6-team-main-20261003`，手机回复仍使用模块接口标识 v4.6。manifest记录编译选项、基线和每个实际构建输入的SHA256。

| 文件 | SHA256 |
| --- | --- |
| ELF | `517D0E105135F1C09E38E5F7D66987246855622B50C9C16BEE5AEFCF9EC2C73F` |
| HEX | `6880E52CE55393608580C23C8D04B8749D61FA6CC25878965F3A510A4C6CC71A` |
| BIN | `6CAFD7AFFB78A1AF0CCE500E4A0F798AC9EFF522C2855F5EAD8C3B30F0D01E9F` |

操作员确认目标板、SWD/供电、支撑机械臂及车轮离地后使用：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-arm-trim-latest-20261003\scripts\flash_firmware.ps1" -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-arm-trim-latest-20261003\scripts\verify_firmware.ps1"
```

完整控制器设置：[BLUETOOTH_CONTROLLER_GUIDE.md](BLUETOOTH_CONTROLLER_GUIDE.md)。封装接口：[ARM_TRIM_INTEGRATION.md](ARM_TRIM_INTEGRATION.md)。录入工具：[ARM_SETUP_GUIDE.md](ARM_SETUP_GUIDE.md)。实车记录：[ARM_TRIM_ACCEPTANCE.md](ARM_TRIM_ACCEPTANCE.md)。
