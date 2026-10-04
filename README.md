# JNDS 电控：v4.7 整车与机械臂统一控制

本分支 `feature/merged-team-arm-20261003` 将队友整车控制与机械臂固定姿态、独立夹爪、逆运动学微调接入同一个机械臂服务。使用一张蓝牙页面，可以控制底盘、转向、车辆视觉对准、射靶、航向PID以及机械臂。

这次解决的是上一合并版的入口互锁问题：微调服务持有舵机执行权后，队友的固定姿态入口会被拒绝。本版保留执行权保护，把所有人工机械臂请求统一提交给 `ArmTrimService`。固定姿态与微调可以按明确规则切换，夹爪状态不会被姿态按钮覆盖。

## 从哪里开始

|你要做的事|先看这里|
|---|---|
|第一次用手机控制、配置字段和控件|[完整蓝牙操作指引](UNIFIED_CONTROL_GUIDE.md)|
|了解比上一合并版改了什么|[v4.7详细版本差异](V4_7_CHANGES.md)|
|接手源码、修改机构参数、接视觉或任务层|[v4.7交接文档](V4_7_HANDOFF.md)|
|查询服务接口、状态与底层接入|[模块接入说明](UNIFIED_MODULE_INTEGRATION.md)|
|在电脑端查询P、录限位及参数|[原CLI录入工具说明](ARM_SETUP_GUIDE.md)|
|了解主板引脚、底盘及历史协议|[整车硬件与历史参考](PROJECT_REFERENCE.md)|
|确认软件测试和实机验收边界|[本次发布验证记录](V4_7_RELEASE_REPORT.md)|

## 这版能实现什么

- 固定姿态只控制000、001、002。003夹爪由夹紧、松开或GAP单独控制；切姿态不会自行放掉已夹物体。
- 点抓球前TB_M、抓人质前TH_C、放球前BD_U，正常执行后尝试建立微调参考。
- 输入正负方向，按住wt保持参考高度和夹口方向连续前后平移，松开后减速停止；X正为远离车。
- 微调过程中点固定姿态，先减速、完成收尾，再执行一个待切换姿态。
- 固定姿态、夹爪、有限DX位移或停止流程忙碌时，拒绝新运动，空闲后不补执行；松开重按。
- 通过ASCII文本请求固定姿态、精确毫米偏移、状态、取消和退出；二进制遥控格式保持兼容。
- 保留队友底盘平移、转向、航向保持、车辆视觉对准/射靶、PID调参及其原主循环安全处理顺序。

夹爪独立表示通道与命令独立；当前机械臂服务仍串行执行，不能在臂杆运动时并行夹爪。运动完成与位置均为软件估计，固件不回读实际关节到位，也不检测物体是否抓住。

## 模块怎样接到原工程

|模块|职责|本版接入变化|
|---|---|---|
|`bluetooth_driver`|接收、校验手机帧，发布底盘快照|把固定姿态、夹爪、微调薄分发到机械臂输入层|
|`arm_trim_input`|按下沿、长按保活、冲突和停车互锁、文本入口|两个机械臂入口归入同一服务，提供统一回复|
|`arm_trim_service`|固定姿态、夹爪、微调、参考和执行权|持有同一个Servo令牌，负责切换、忙碌拒绝和取消收尾|
|`arm_trim_project`及配置头|当前机构几何、标定、限位和障碍物|把可复用核心与本车参数隔开|
|`arm_kinematics`、`arm_collision`、`arm_trim`|正逆解、连续分支、局部路径与速度轨迹|维持固定高度/方向的前后微调；不依赖蓝牙或底盘|
|`servo`及UART队列|ASCII舵机协议与真实传输状态|沿用原发送器，通过执行权防止其他入口插入命令|
|`car_control`、电机/转向/车辆视觉|车辆控制|保留职责与主循环顺序，机械臂输入层检查停车条件|

主循环晚段先调用 `Bluetooth_DispatchServoActions()`，再调用 `ArmTrimInput_Process()`；后者先检查互锁再推进服务，服务推进 `Servo_Process()`。不要绕过服务直接发舵机帧，否则会破坏预计位置、参考和执行权。详细逐文件解释见交接文档。

## 手机的统一页面

|类型|个数|
|---|---:|
|bool|16（按位打包为2字节）|
|byte|0|
|short|16（有符号、小端）|
|int|1（GAP）|
|float|0|
|总长度|41字节，包含包头/校验/包尾|

包头A5、包尾5A，校验为payload字节和的低8位，周期50ms。名称只作显示，类型与顺序决定含义；不要按名字猜字段。统一页没有BYTE0，也不用填84。完整字段表、控件类型及HEX示例见[蓝牙指引](UNIFIED_CONTROL_GUIDE.md)。

典型顺序：停车 → 点固定姿态并等待状态 → 确认REF=1 → ydnum输入+1或−1 → 按住wt微调 → 松开等待 → 独立夹紧/松开。

`ydnum`只取正负方向，填30不代表移动30mm。精确相对移动请使用以下文本，每条以换行结束：

```text
@BENCH POSE TB_M
@ARM TRIM STATUS
@ARM TRIM DX 2
@BENCH CLOSE
@ARM TRIM STOP
```

## 当前机器参数与已知问题

|项目|当前值|
|---|---|
|000 / 001 / 002 P范围|915～1800 / 821～2500 / 500～1874|
|003范围、夹紧、松开|500～2500、500、1800|
|BALL / TB_M|1356、1850、698|
|HOSTAGE / TH_C / TH_PRE|1684、2136、785，改装后待重录|
|BUCKET / BD_U|1566、1896、673|
|AIM|1058、821、554，按用户要求放宽001下限后可请求固定姿态|
|L1 / L2|104.85 / 84.75mm，源码初值；L2与历史口述87.5mm有差异，待实测核对|

参考±75mm是模型搜索窗，不是保证双向可走75mm。微调受关节限位、逆解连续性、奇异性与碰撞模型共同约束；看STATUS里的MODEL_MM、ENABLED_MM及ERR。固定姿态仅检查关节目标，完整固定运动路径尚无碰撞预检查。

车辆视觉对准功能已经保留，但视觉到机械臂的像素/毫米标定和自主抓取任务尚未接入。`Servo_Start()`仍是任务事件占位；任务层还需服务执行、完成/失败通知及人工/自主模式仲裁。本版没有任意多步动作组播放器。独立窗口录入工具和v4.8动作播放器属于后续测试副本，未纳入本次v4.7发布。

## 获取、构建和烧录

首次克隆指定分支：

```powershell
git clone --branch feature/merged-team-arm-20261003 --single-branch https://github.com/Morechips/e-control-arm-trim.git e-control-unified
Set-Location e-control-unified
```

ARM GCC工具链需在PATH。运行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/build_firmware.ps1
```

输出位于 `firmware_direct/`。构建清单的文件名沿用 `arm-trim-v4.6-manifest.json`，内容版本为 `v4.7-unified-20261003`；应读内容判断版本。Git保存源码和[发布构建清单](releases/v4.7-unified-20261003/BUILD_MANIFEST.json)，不提交生成的HEX/ELF/BIN；克隆后先构建。

确认目标板、SWD接线和供电，停车、架空车轮并支撑机械臂后执行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/flash_firmware.ps1 -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File scripts/verify_firmware.ps1
```

可附 `-OpenOcdExecutable "你的openocd.exe绝对路径"`。烧录会写入、校验并复位；核对只比对镜像，短暂停核验后恢复，成功应出现 `CURRENT_FIRMWARE_VERIFIED`。默认整车固件上电会自动使能四轮。

## 版本来源与验证

- 仓库main保持原发布历史；本次更新仅提交到 `feature/merged-team-arm-20261003`。
- 队友底盘基线：`gpnu-in-jnds/e-control` 的 `e7404c592df92c7c5e7a9d09562e302ebf52e2b3`。
- 上一合并提交：`765a35c252ce1521ae8fee0b93067056b2d8c2ad`；main的v4.6提交为 `9b61a6a`。
- 本版构建标识：`v4.7-unified-20261003`，发布整理日期2026-10-04。
- 发布目录复跑受影响回归及完整构建，结果见[验证记录](V4_7_RELEASE_REPORT.md)。实车任务全部待验收。

历史 `releases/v4.6/`、`releases/merged-team-20261003/` 和原报告保留追溯；不把历史参数、旧003行为或旧镜像哈希作为当前版本结论。
