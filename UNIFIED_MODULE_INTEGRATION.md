# v4.7 统一机械臂服务接入说明

工程：`D:\工科大\e-control-merged-publish-20261003`，发布分支：`feature/merged-team-arm-20261003`，构建标识：`v4.7-unified-20261003`。基于已推送合并分支提交 `765a35c252ce1521ae8fee0b93067056b2d8c2ad` 修改；队友底盘基线仍为 `e7404c592df92c7c5e7a9d09562e302ebf52e2b3`。本次v4.7发布整理于2026-10-04；验证与版本差异见[V4_7_RELEASE_REPORT.md](V4_7_RELEASE_REPORT.md)、[V4_7_CHANGES.md](V4_7_CHANGES.md)，接手见[V4_7_HANDOFF.md](V4_7_HANDOFF.md)。本轮未操作实车。

手机字段、控件和顺序见 [UNIFIED_CONTROL_GUIDE.md](UNIFIED_CONTROL_GUIDE.md)。本文面向后续接代码的人。

## 解决的问题

上一合并版有两个入口：微调服务用 Servo 独占令牌发送，队友蓝牙姿态直接发送不带令牌的命令。建立微调参考后服务保留令牌，旧入口因而收到 BUSY。本版将两种页面的固定姿态、夹爪和微调请求全部送到 `ArmTrimService`，底盘继续接收自己的遥控快照。

用户确认的规则已落实：

- 所有固定姿态只发送000～002，包括RST、AIM、旧抓取/放置姿态。003仅由夹紧、松开或GAP单独请求。
- 长按微调时请求固定姿态，先减速停稳，再自动执行。只保存一个待切换姿态。
- 固定姿态、有限位移、夹爪或停止忙碌时拒绝其他动作；拒绝的按下沿已消耗，空闲后不会补执行，需松开重按。
- 切换固定姿态后，即使仍按着wt，也不自行恢复微调，需松开重按。
- 停止、输入链路错误或停车条件失效取消待切换。wt普通松手减速停止并保留参考；强制取消使参考失效。
- 夹爪独立指固定姿态不带动003；本版仍是一个串行服务，夹爪与臂杆不同时执行。

## 分层与主循环

| 层 | 文件 | 职责 |
| --- | --- | --- |
| 可移植核心 | arm_kinematics、arm_collision、arm_trim | 正逆解、固定高度/方向的路径、分支连续性、模型碰撞检查、速度轨迹、减速停止；不依赖HAL、蓝牙、底盘 |
| 安装适配 | arm_trim_project、arm_trim_project_config.h、arm_collision_config.h | 几何、两点P/角度标定、关节限位、后箱、包络、轨迹配置 |
| 统一服务 | arm_trim_service.h/.c | 固定姿态、参考同步、微调、夹爪、单个待切换姿态、Servo令牌及异步状态 |
| 输入适配 | arm_trim_input.h/.c | 页面/文本、按下沿、长按租约、停车互锁、冲突、回复 |
| 蓝牙 | bluetooth_driver.h/.c | 原41字节布局中的新增字段，底盘快照、机械臂薄分发 |
| 传输 | servo、uart_tx_queue、uart_driver | 协议、真实STARTED/TC时间、队列、超时、令牌保护 |

`main.c` 保持两次 `Car_Control_Process()` 分别在 `Motor_Process()` 前后。晚段先 `Bluetooth_DispatchServoActions()` 再 `ArmTrimInput_Process()`，后者先检查互锁，再推进服务；服务负责 `Servo_Process()`。禁用 `ARM_TRIM_ENABLE` 的编译路径保留旧分发。

## 服务公共接口

接口见 `Core/Inc/arm_trim_service.h`。前台调用；配置与时钟回调需有效。服务不读取蓝牙、视觉、GPIO或底盘，调用者负责停车等条件。

| 接口 | 用途/约束 |
| --- | --- |
| Init(config, now, user) | 复制配置并接入时钟；拥有Servo时拒绝重新初始化 |
| RunPreset(code) | 固定姿态仅000～002；先校验目标P限位；支持从长按微调平滑切换 |
| ReadyProfile(profile, synchronize) | 发BALL/HOSTAGE/BUCKET配置姿态；true完成后同步参考，false只执行姿态 |
| Begin(p, parked_stable) | 调用者确认实际000～002稳定且停车后同步P；不发送运动 |
| MoveRelativeX(dx_mm) | 相对当前预计位置的毫米位移；累计边界相对初始参考；超范围整次拒绝 |
| StartJog(direction) | direction只能±1；规划到该方向启用的连续模型边界 |
| ReleaseJog() | 正常减速停止，保留完成后的参考 |
| Grip(pwm) | 仅003；无需先建立参考；正常完成保留已有参考；夹500、松1800 |
| Cancel() | 移除待切换与未发送运动、请求对应通道停止、使参考失效 |
| End() | 退出释放令牌；忙碌时先取消，继续Process直至收尾 |
| ClearFault() | 已停止并释放令牌后清故障；仍需重新建立参考 |
| Process() | 主循环持续推进，不能用阻塞等待替代 |
| GetStatus() | 服务/核心/传输/参考、pose_pending及最近请求结果 |
| IsBusy() / OwnsMotion() | 忙碌/拥有令牌；静止有效参考也可拥有令牌，两者不同 |

上表函数都加前缀 `ArmTrimService_`。`RunPreset()`保留ServoCode 0～12。TB_M、TH_C、TH_PRE、BD_U映射到安装配置三组参考；其余读取 `servo.c` 动作表前三项。表内历史003保留供禁用微调的旧路径使用，统一服务忽略。

`ArmTrimInput_SubmitCombined(buttons, direction, preset, gap_pwm, requests, arrival_tick)`供蓝牙提交；requests是本帧姿态/GAP按下沿总数，用于拒绝多个动作。旧 `Submit()`兼容7字节页。

## 状态、结果与完成判断

| 值 | 状态 | 意义 |
| --- | --- | --- |
| 0 | IDLE | 无活动动作，参考可能无效 |
| 1 | PROFILE | 配置参考姿态执行/等待 |
| 2 | REFERENCE | 有效参考，可微调 |
| 3 | MOTION | 微调或核心取消流程 |
| 4 | GRIP | 仅003执行/等待 |
| 5 | STOPPING | 逐通道停止及传输收尾 |
| 6 | FAULT | 故障锁定，处理后CLEAR并重建参考 |
| 7 | FIXED | 其他固定姿态执行/等待；旧状态编号未改 |

结果：0接受、1参数无效、2忙碌、3需参考、4超范围、5路径无效、6传输错误、7超时、8故障锁定、9链路超时。接受不等于完成。last_request是最近一次请求结果，后来的BUSY会覆盖它，不是独立完成编号；`!IsBusy()`也不能单独当成功。

固定姿态仅预检查目标关节P限位，**没有对当前到目标的完整固定轨迹做碰撞预检**。微调检查固定高度/方向路径、关节插补及碰撞模型。通用固定姿态完成后尝试同步，模型不接受时清参考并回IDLE，可继续执行其他姿态；三组专用参考同步失败保留故障锁定处理。

等待从真实UART TC计时：固定T2000加300ms，夹爪T1500加300ms；微调最终稳定300ms。运行时不读实际P，无真实到位/抓住/放入反馈，所有位置和完成状态为估计。任务层应跟踪已接受操作及预期状态转换，同时检查参考、故障、传输结果。

## 安装配置和待办

| 项 | 本版沿用值 |
| --- | --- |
| P限位000 / 001 / 002 | 915～1800 / 821～2500 / 500～1874 |
| 003范围、夹、松 | 500～2500、500、1800 |
| BALL | 1356、1850、698 |
| HOSTAGE | 1684、2136、785；改装后仍待重录 |
| BUCKET | 1566、1896、673 |
| L1 / L2 | 104.85 / 84.75 mm，当前源码值，本次未重新标定 |
| 末端偏置X / Z | 121.1538 / 52.4 mm |
| 速度 / 加速度 | 10mm/s / 20mm/s² |
| 最大段 / 更新 / 搜索窗 | 2mm / 50ms / 参考±75mm |
| 后箱X / Y / Z | [−70,−30] / [−50,50] / [−31.2,38.8] mm |
| 臂杆/末端包络半径 | 15、20、60mm，估计值；额外间隙5mm |

原点为000轴心，X正远离车，Z正向上。±75mm是搜索窗，实际范围是关节、奇异性、轨迹、碰撞模型等约束的交集。模型区间不能代替实测安全范围。

**用户已要求将001下限由947改为821，使AIM固定姿态可执行**。当前AIM为1058、821、554，仅发送000～002，保持003；是否能微调需检查动作完成并同步后的REF，放宽P范围不保证微调模型同步成功。其他关节限位不变，本轮没有实车验证。TH_U关节目标合法，但当前保守后箱模型可能不允许建立微调参考。相机、线材及开合夹爪包络仍需测量。

## 后续视觉与自主任务

未来任务层调用同一服务：停车稳定 → RunPreset/ReadyProfile → 等预计完成且reference_valid → 视觉提供车体前后毫米偏差 → MoveRelativeX → 等完成 → Grip。像素到毫米需要标定，本版未加入视觉到机械臂闭环。

服务本身不要求遥控在线；当前输入层仍执行人工遥控停车/链路互锁。自主模式接入前需明确人工/自主控制权和输入失效规则，保留停车条件。当前没有模式仲裁，正式主循环仍未启动完整路线。`mission_fsm.c`的`Servo_Start()`目前只记录任务事件与等待标志，是尚未接硬件的占位接口；后续需将该事件接到统一服务，并在预计完成或失败时正确通知任务层。不存在已经运行的任务Servo调用与本服务争抢的情况；接入时仍应统一控制权，不能绕过服务令牌。

核心保留 `ArmTrimIO_t.lateral_report` / `ArmTrim_ReportLateral()`；当前服务回调NULL，未调用底盘左右纠偏。安装录入工具可导出多步组数据，固件尚无任意多步组播放器。

## 验证与烧录

```powershell
Set-Location "D:\工科大\e-control-merged-publish-20261003"
$env:PATH = "C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin;D:\c++\MinGW\bin;" + $env:PATH
powershell -ExecutionPolicy Bypass -File scripts/build_firmware.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_unified/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_trim_input/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_trim_service/run.ps1
powershell -ExecutionPolicy Bypass -File tests/servo/run.ps1
powershell -ExecutionPolicy Bypass -File tests/uart/run.ps1
powershell -ExecutionPolicy Bypass -File tests/car/run.ps1
```

受影响回归和完整ARM GCC构建已通过，另通过 `-DisableArmTrim -OutputDirectory build-local/no-trim` 旧分发构建。统一联动用真实蓝牙、输入、服务、Servo/UART队列，替代HAL/车状态，不是实车测试。覆盖页面切换清WT/PID/REFERENCE、微调自动切姿态、忙碌无重播、停止清待切换、冲突、003独立和底盘字段发布。

产物在firmware_direct。清单沿用文件名 `arm-trim-v4.6-manifest.json`，内容version为v4.7，记录实际输入与镜像SHA256。以内容和核对脚本为准。

本轮001下限改821后验证通过：统一链路33994项、服务3909项、真实Servo/UART管线7245项、核心48708项及异步327项、蓝牙解析324464项及输入210项、配置/回读工具40项。AIM精确发送 `{#000P1058T2000!#001P0821T2000!#002P0554T2000!}`，不含003；严格自定义限位仍整次拒绝越界请求。完整ARM GCC构建、259份输入哈希与镜像校验以及`-SymbolsOnly`诊断通过，未连接硬件。

当前HEX SHA256：`5D22BA220F20339F00E68C08FF0DD559B37726CD42DAAE43A29DD6B478F3845C`。后续重编译以更新后的清单为准，不复用旧镜像哈希或RAM地址。

确认STM32F407目标、接线、臂支撑和车轮架空后由使用者执行：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-merged-publish-20261003\scripts\flash_firmware.ps1" -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-merged-publish-20261003\scripts\verify_firmware.ps1"
```

烧录program/verify/reset；核对只比镜像，短暂halt再恢复，应看到CURRENT_FIRMWARE_VERIFIED。旧 `D:\工科大\flash_robot_version.ps1` 三版本指向未改，可继续回退对比。

本版实车待验收：统一蓝牙、固定姿态不影响夹爪、双向长按、切换/停止、底盘互锁、夹取/放桶及实际间隙。主机测试不能代替这些记录。
