# 队友最新版机械臂合并说明

当前测试副本为 v4.5，新增三个测试参考动作不控制 003，抓人质参考待重录。当前修改见 `ARM_FIXED_ACTION_V4_5.md`；以下为原整合及 v4.4 记录。

以下为整合基线的历史记录。本测试副本已更新为 v4.4：000=915～1800、001=947～2500、002=500～1874；按用户确认恢复旧 P1800 抬起姿态通过限位检查。当前以 `ARM_REAR_BOX_V4_4.md` 为准，原 team-sync 目录未改动；微调已加入后方箱体估计包络检查，旧固定动作入口未接入该保护，实际间隙待验证。

本工作区从队友 `origin/main@db9c1d6` 创建，随后对照队友最新 `origin/main@3021961` 重新适配手机协议和非机械臂修复，分支为 `feature/arm-team-sync`。整合工作只发生在 `D:\工科大\e-control-team-sync`；旧工作区 `D:\工科大\e-control`、`feature/arm-integration` 及其未提交修改未被改动。本分支当前未提交、未推送、未烧录。

## 保留的队友功能

- 视觉、路线、任务状态机、射击中心区、射击请求重试和手机方向键35 RPM保持队友最新版行为。
- 29字节 `SERVO_MODE`、31字节九姿态bool、35字节bool+`GAP`、PE4物理按钮全部保留。
- 八个原始固定姿态统一复用 `servo_pose.c`，ID 003依次为 `2192、2192、500、500、1800、1800、500、500`。
- ID 000上限扩展到1800，使P1717和P1800任务姿态可用；其余队友姿态值不改。

## 协议兼容

固件同时接收5、17、21、23、25、27、29、31、33、35、41字节A5协议。无bool/GAP页面可使用33字节15-short布局；41字节同时兼容本地`(7).pro`扩展布局和队友最新`(20).pro`布局。队友布局的16个short顺序为：`JOY_Y / forward / backward / stop / strafe_left / strafe_right / right_90 / right_180 / Cam_T / Shot / LEFT_90 / servo_mode / joy_x / armcmd / armx / army`，之后是`GAP`。

| short索引 | 字段 | 范围/含义 |
| ---: | --- | --- |
| 12 | `ARM_CMD` | 0释放；10/11腕；12/13夹爪；14下一固定姿态；20～25会话按钮 |
| 13 | `ARM_X` | -1000～1000，机械臂平面前后 |
| 14 | `ARM_Y` | -1000～1000，机械臂升降 |

27/29/31字节的旧机械臂叠加格式仅在机械臂会话期间优先解释；会话外仍属于普通27字节、29字节`SERVO_MODE`和31字节bool协议。35字节先用`servo_mode=-8`识别bool+`GAP`，否则按13-short+3-short双摇杆格式处理。41字节先识别带`servo_mode=-8`标记的本地布局，否则按队友`(20).pro`字段顺序解释。两种41字节布局都能同时刷新底盘和机械臂输入。

队友41字节首个`GAP`值只建立基准，不会在刚连接时误动夹爪。机械臂姿态、GAP或独立按钮在车辆运动时会被拒绝，但不再锁住或停止底盘。

`@ARM LINK`可查看最近有效车控帧长度、最近机械臂帧长度、两个序号、错误帧计数和连接状态，用于确认手机实际发送的是哪种布局。

## 固定姿态和串口仲裁

`ARM_CMD=14`按八姿态循环；也可发送`@ARM PRESET 0..7`或`@ARM PRESET NEXT`。每次固定动作1500 ms，同一按键持续发送只触发一次，必须先回0再触发。固定动作只有在以下条件同时成立时才接受：

- 双摇杆会话已建立并授权；
- 蓝牙有效，底盘状态为`CAR_READY`且四轮控制空闲；
- 底盘摇杆、方向按钮、视觉/发射按钮均归中；
- 机械臂状态机空闲。

条件不满足时直接拒绝，不停止底盘、不排队。PE4仍可离线请求模式1，但机械臂忙、故障、停止中或存在`@ARM`/双摇杆会话时同样拒绝，不再覆盖正在执行的动作。

USART3保持单一所有权：`servo_remote`、固定姿态和双摇杆只向`arm_control`提交动作，实际ZL-IS2发送均由`arm_control.c`完成。

## 构建与烧录

只编译、不接硬件：

```powershell
Set-Location 'D:\工科大\e-control-team-sync'
powershell -ExecutionPolicy Bypass -File .\scripts\build_firmware.ps1
```

输出位于`firmware_direct`。本轮禁止自动烧录；确认机械臂支撑、轮子悬空、SWD接线和供电后，用户可自行运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\flash_firmware.ps1 -ConfirmHardwareReady
```

默认OpenOCD路径为`D:\Tool\xpack-openocd-0.12.0-7\bin\openocd.exe`，可通过`-OpenOcdExecutable`覆盖。

## 验证边界

主机测试验证协议拆包/连包、校验错误、连接恢复、机械臂状态机、固定动作、底盘、视觉、路线、任务、ZL-IS2和逆运动学。固件构建生成ELF/HEX/BIN。所有“到位”仍是按命令时间估算，没有舵机位置或夹持力反馈；舵机控制板自身的上电动作组不由STM32固件改写。
