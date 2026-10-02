# e-control 机械臂交接文档

更新日期：2026-10-02，Asia/Shanghai。面向接续开发、调试和验收的其他会话。

本文根据本会话的用户确认、本地源码、固件哈希及 Git 状态整理。软件模型结果、用户实测反馈、未完成验收分别标注；未访问或驱动硬件，也未提交、推送或发布 GitHub。硬件后续改变或用户追加测量后，以新确认值和重新构建的镜像为准。

## 1. 新会话首先需要知道的事

1. 当前实际开发、编译、台架测试目录是 `D:\工科大\e-control-trim-test-20261001`，当前交付软件为 **v4.5**。不要直接到原团队目录覆盖文件。
2. `D:\工科大\e-control-arm-trim-github` 是用户自己仓库的源码工作树，已完成首次提交和上传，main 跟踪 origin/main；首个源码提交为 **b4664dd**。两目录不能当作自动同步的同一工作树。
3. 独立微调只控制平面三关节 **000、001、002**，保持模型中的抓点高度及方向，仅改变前后 X。003 是夹爪，不属于逆运动学微调。
4. **v4.5 已修复三个固定参考动作附带 003 的问题**：抓球前、抓人质前、放球前都只发送 000～002，切换姿态保留当前夹爪。夹紧 P500、松开 P1800 由单独按钮控制。
5. **抓人质前动作还没重录**。用户确认机械结构已经改变；当前仍保留旧值 1684/2136/785，不得宣称它是新安装下已验证的姿态。需要用户录好新的三个 P 值。
6. 手机当前操作是：`ydnum` 输入正负方向，**按住 wt 连续平移、松开减速**。数字 2、30 都只代表正方向，已经不表示移动 2 mm、30 mm。文本 DX 和旧 11 字节协议仍以毫米为单位。
7. 当前轴心距 **L2=84.75 mm**，代码采用 `(82+87.5)/2`；000 限位 **915～1800**，001 **947～2500**，002 **500～1874**。不要恢复早期几何、1700 上限或累计 ±5 mm 的项目限制。
8. 后方长方体的坐标和尺寸已确认，但机械臂外廓半径仍是估计；**箱体检查只接入微调，不保护旧固定就位/抬起路径**。
9. 用户已报告修复前基线的功能基本可用、长按符合预期、抖动不明显；v4.5 的去夹爪修改有软件验证，**尚未收到本次实物验证或烧录校验回报**。
10. 用户要求：后续每次交付可烧录版本都附完整 PowerShell 烧录与校验命令；讲清楚命令怎么用。不要把 MCU 时间估计完成当成物理到位或夹取成功。

## 2. 用户目标、任务范围与工程取舍

### 2.1 原始目标

用户希望形成可迁移的机械臂模块：先执行已经可用的固定动作，再接受前后/上下偏移或目标坐标，稳定、平滑地完成修正，之后由其他模块夹取或放球。队友同时在重构底盘行进和转弯校准，最新可用底盘代码尚未完全上传；机械臂工作不必等待整个重构完成。

当前实际任务主要有三种参考：抓球前 BALL、抓人质前 HOSTAGE、放球到防爆桶前 BUCKET。抓取对象为两类，任务高度固定、前后变化不大；左右实测较准、夹口较宽，因此本阶段只解决会明显导致抓飞的前后误差。

### 2.2 已确定的阶段范围

- 车先尽可能对齐物体中线，并由外部保持停车。
- 固定动作到达参考姿态并等待稳定，再做机械臂局部 X 修正。
- 保持参考高度 Z 和夹口方向 phi，联动三个关节。
- 完成修正后，外部只改变夹爪，避免旧动作覆盖修正后的三关节。
- 本阶段用蓝牙输入代替视觉输入，先验证运动能力；视觉识别、毫米偏差转换、PID、固定动作组速度调节未实施。
- 留有左右偏差上报接口，当前不调用底盘纠偏。
- 用户最初估计物体前后活动范围约 15 cm，但已接受“各参考姿态的局部可达范围优先”，**不承诺双向 ±75 mm**。

微调价值主要是把固定动作附近的位置误差修正做成接口，并为之后视觉接入提供执行层。它不要求先实现完整自由空间 3D 抓取。高度、方向和横向被任务约束后，可以先让视觉只提供沿本模块 X 方向的毫米修正；未来要扩大到其他维度，再扩展观测和控制。

### 2.3 阶段验收标准

**蓝牙输入 → 微调预计完成并观察实物稳定 → 外部成功夹取；BUCKET 成功把球放入防爆桶。**

每场景在已实测可用范围内选近、中、远三个位置，各重复 10 次，记录成功次数与失败原因。高度、方向、平滑性作为调试记录；不把串口发送成功、状态 COMPLETE_ESTIMATED 或模型误差阈值当成真实精度承诺。完整次数统计目前尚未提交。

## 3. 机械结构、相机与反馈边界

- 机械臂建模为同一竖直平面内的三个旋转关节 3R；000 为基部轴、001 为中间轴、002 为腕部轴，003 控制夹爪开合。
- 相机刚性固定在腕部/夹爪支架，与夹爪的安装距离和角度一定，不会因夹爪开合而改变该安装关系。照片中能看到相机、支架、夹爪和线束；没有完整整机 CAD 或准确实体外廓数据。
- 开合会使夹爪进入或离开视区；夹持物体和夹爪可能遮挡画面。因此未来视觉不能假设每个运动时刻都看得清目标。
- 当前固件没有以关节位置或夹持力反馈建立微调闭环。同步、预计完成和继续移动都依赖模型、已发送 P 目标及时间估计。
- 用户通过上位机录 P 值用于参考姿态。不要仅凭“读到了 P”就认定已获得真实关节编码器角度或机械到位反馈。
- 上电自动复位动作由舵机控制板存储和执行，**不是当前 STM32 微调模块自动发出的动作**。修改 STM32 中的参考值不会自动改控制板上电动作。

## 4. 目录、仓库、基线与交付物

| 位置 | 用途及接续注意事项 |
|---|---|
| `D:\工科大\e-control-trim-test-20261001` | 当前 v4.5 测试工程，源码、测试、脚本、当前固件、旧镜像备份齐全；该目录没有 Git 仓库 |
| `D:\工科大\e-control-arm-trim-github` | GitHub 源码工作树，去掉 build、firmware_direct、回退包等生成内容；已完成首次上传，main 跟踪 origin/main |
| `D:\工科大\e-control-team-sync` | 原团队适配工作树；HEAD=db9c1d6，分支 feature/arm-team-sync，已有大量未提交修改；本轮不覆盖它 |
| `D:\工科大\e-control-arm-trim-v4.5-firmware.zip` | 当前软件验证镜像包，785086 字节；包括匹配 ELF/HEX/BIN/MAP、哈希、烧录/校验/诊断脚本及 OpenOCD 配置 |
| `D:\工科大\e-control-arm-trim-v4.4-firmware.zip` | 上一版保留的固件包，783056 字节；不可把它当成已去除固定动作 003 的版本 |

原团队 remote 为 `https://github.com/gpnu-in-jnds/e-control.git`。测试副本来自本地 team-sync，后者的历史整合记录对照过 3021961 的协议和非机械臂变更；“副本来源”不代表当前测试目录等于远端最新 main，更不代表包含队友尚未交付的底盘重构。

用户自己的仓库：`https://github.com/Morechips/e-control-arm-trim.git`。2026-10-02 已完成首次源码推送，远端 main 与首个本地提交 `b4664dd5fe00023c73d51cdde6d5bbe28ff375cf` 核对一致。未创建 GitHub Release、未上传预编译 ZIP 附件，抓人质重录和实车验收仍待完成。

### 4.1 当前固件身份

当前 `firmware_direct/stm32f407_bt_oled.*` 与 `FIRMWARE_SHA256.txt` 已再次核对一致：

| 格式 | SHA256 |
|---|---|
| BIN | `AAF2EF35015F80B956DF5F6ADB26B102A793AC7727B54D1B50210E9ABFD1B283` |
| ELF | `CC6F5B45E3F4B195121E505382B22C107E1E42DA864CE0332B7403C81CF72D06` |
| HEX | `1A483F1C17E29B9DCDEA57541E9A3ECCF6590727CB6803ADAF7F4D6CDDD6E852` |

调试启动标识：

```text
ARM TRIM FW=V4.5 L2_MM=84.75 WT=HOLD_JOG BYTE0=0_OR_84
```

新构建、换路径或更新参考后镜像哈希可能变化；随交付更新记录，不要把上述哈希永久写死为任意后续版本。板上是否是 v4.5 仍需用户执行当前镜像校验。

### 4.2 回退目录

测试工程保留 `rollback_team_sync_20260929`、`rollback_v3_20261001`、`rollback_v3_1_20261001`、`rollback_v4_20261002`、`rollback_v4_1_20261002`、`rollback_v4_2_20261002`、`rollback_v4_3_20261002`、`rollback_v4_4_20261002`。

最新回退 `rollback_v4_4_20261002` 保存去除 003 前的镜像和相关配置/源码。早期 v3～v4 的几何、限位、通信和按钮语义可能不同；恢复镜像时必须说明恢复了哪一套行为，不能只换文件名。

## 5. 当前坐标、几何和舵机标定

### 5.1 坐标约定

- 原点：000 轴心。
- X 正：朝抓取物体、远离车；X 负：靠近车、机械臂后方。
- Z 正：向上。
- 机械臂运动平面：Y=0；当前没有机械臂左右平移自由度。
- 正微调向外，负微调向车。累计偏移相对最初参考计算。
- `ArmPose2D_t` 使用 `x_mm`、`z_mm`、`phi_rad`；角度单位是弧度。

“箱体 X=−70～−30”表示：最近面距 000 后方 30 mm，箱体再向后延伸 40 mm。用户已看过示意图并确认理解一致，不必重新推翻坐标定义。

### 5.2 几何参数

位置：`Core/Inc/arm_config.h`；实际项目模型由 `ArmKinematics_ProjectGeometry` 读取。

| 参数 | 当前数值 | 含义 |
|---|---:|---|
| ARM_LINK_1_MM | 104.85 mm | 000 轴心到 001 轴心 |
| ARM_LINK_2_MM | **84.75 mm** | 001 轴心到 002 轴心，代码采用 (82+87.5)/2 |
| ARM_TOOL_X_MM | 121.1538 mm | 腕部坐标系内 002 到选定虚拟抓点的 X 分量 |
| ARM_TOOL_Z_MM | 52.4 mm | 同一工具向量的 Z 分量 |
| tool_axis_offset_rad | 0 | 当前工具方向相对腕部 X 方向的偏置 |
| ARM_TOOL_REACH_MM | 132 mm | 保留的工具长度常量；实际正逆解使用工具 X/Z 分量 |
| ARM_BASE_AXIS_HEIGHT_MM | 120 mm | 旧地面参考；正解 Z 本身相对 000，不自动加此值 |

**L2 曾出现多次口头澄清，包括“87.5”“先相加再除 2”。当前交付已经统一为 84.75 mm。** 新会话不要仅从早期一句“87.5”恢复另一个值；若又有新测量，应确认其是不是最终轴心距，再一起更新代码、模型与测试。

箱体测量中的 31.2 mm 是 000 轴心高于底板的距离，不等于地面高度，也没有用它覆盖 120 mm。局部微调和箱体碰撞都在 000 局部坐标中计算。

### 5.3 两点 P—角度标定

当前是线性两点标定，P 是控制器命令值，不是毫米或角度。角度是模型的关节局部约定：

| 关节 | P 点 A / 角度 A | P 点 B / 角度 B |
|---|---|---|
| 000 | 989 / 0° | 1309 / +45° |
| 001 | 1687 / 0° | 2344 / −90° |
| 002 | 1232 / 0° | 571 / −90° |

线性关系为 `q = qa + (P−Pa)/(Pb−Pa) × (qb−qa)`；P 增大不保证三个关节朝同一方向转。模型将 q0、q1、q2 组合为腕部方向，工具向量随腕部姿态旋转，再求虚拟抓点位置。

这组标定来自安装调整后的旧测量。机械改动若改变轴心距、舵机零位或工具支架，除了重录动作，还要检查这些参数是否仍适用。

### 5.4 当前关节活动范围

| 关节 | P 最小 | P 最大 |
|---|---:|---:|
| 000 | 915 | **1800** |
| 001 | 947 | 2500 |
| 002 | 500 | 1874 |

用户先测得 000 为 915～1724，后明确要求把固件上限也改为 1800、恢复旧抓人质抬起动作。因此当前上限 1800 是已授权选择，不是误写。

这些是关节活动度，**不是每一种多关节组合都不会碰到车体的范围**。微调配置引用同一组关节范围，不能以“某关节还没到 P 边界”推断抓口一定还能等高前进。

## 6. 三个固定参考与夹爪的最终行为

配置位置：`Core/Inc/arm_trim_project_config.h`。`arm_trim_bench.c` 的就位表和 `arm_trim_bluetooth.c` 的同步表共用以下宏。

| 名称 | 用途 | 000 | 001 | 002 | 003 |
|---|---|---:|---:|---:|---|
| BALL | 抓球前 | 1356 | 1850 | 698 | **不发送** |
| HOSTAGE | 抓人质前，旧值待重录 | 1684 | 2136 | 785 | **不发送** |
| BUCKET | 放球前 | 1566 | 1896 | 673 | **不发送** |

历史三组还带有 003=1480/1483/509，造成“单独夹紧后，切换动作又改变夹爪”。**v4.5 从参考配置表中删除第四个 P，并把 READY 和 PREP 的 joint_mask 从 0x0F 改为 0x07。** 不是把 003 填零、填一个保持值或重新发送上次夹爪目标，而是完全不发送 003。

- `@BENCH READY 场景`：实际发送三关节固定动作，运动 2000 ms、等待 300 ms，再自动建立微调参考、清零累计偏移。
- `@BENCH PREP 场景`：只执行就位，不自动建立参考；供底层调试和旧 11 字节页面使用。
- `@BENCH CLOSE`：只发 003=P500，1500 ms 加 300 ms 稳定等待。
- `@BENCH OPEN`：只发 003=P1800，同上。
- 高级旧接口 `@BENCH GRIP P` 允许 500～2500，仍只发送 003；保留旧兼容行为。
- 正常夹爪动作完成后保留三关节微调参考及累计偏移；夹爪运动期间拒绝微调/固定动作。

通用 `arm_config.h` 里的夹爪 900～2192、旧双摇杆参考 003=1200，属于另一个通用控制通路，不要误用它们替换独立测试页的已确认 500/1800。

**修改限于上述三个独立测试参考。** `servo_pose.c`、原团队 bool、数值模式、双摇杆 PRESET 等历史入口仍可能含有自己的 003 动作；没有全部移除。也不要拿原表的 BALL_REACH/HOSTAGE_REACH 名称代替本表的实际 P 值。

## 7. 模块架构与可迁移接口

| 文件 | 职责 |
|---|---|
| `Core/Inc/arm_kinematics.h`、`Core/Src/arm_kinematics.c` | 纯平面正逆运动学、固定肘分支、两点 P 标定与项目模型 |
| `Core/Inc/arm_trim.h`、`Core/Src/arm_trim.c` | 调用者持有上下文的纯 C 微调、轨迹预检查、分段执行、连续 jog、停止与状态 |
| `Core/Inc/arm_collision.h`、`Core/Src/arm_collision.c` | 三段胶囊与后方长方体的静态/插补边检查，无发送和 HAL |
| `Core/Inc/arm_collision_config.h` | 当前后方箱体、估计外廓和间隙 |
| `Core/Inc/arm_trim_project_config.h` | 三参考、微调启用搜索范围、独立夹爪值 |
| `Core/Src/arm_trim_bluetooth.c` | 微调会话、已有文本分发和回复队列、运动权仲裁、蓝牙按住保活 |
| `Core/Src/arm_trim_bench.c` | 测试版固定就位及独立夹爪，自动参考和 busy/fault gates |
| `Core/Src/arm_control.c` | 原 Arm 状态机、旧 16 步动作队列、关节范围与外部运动占用 |
| `Core/Src/arm_tuner.c` | 现有文本/旧调参/双摇杆分发，调用微调与 bench 处理 |
| `Core/Src/bluetooth_driver.c` | 二进制/文本收包、字段解析、上升沿、诊断计数；未用第二个接收回调覆盖原接收 |
| `Core/Src/zlis2_driver.c` | 舵机控制板串口命令发送及停止 |
| `Core/Src/servo_pose.c`、`servo_remote.c` | 原团队姿态/遥控入口，必须保持与微调的执行权互锁 |

核心主要 API：

```c
void ArmTrim_DefaultConfig(ArmTrimConfig_t *config);
ArmTrimResult_t ArmTrim_Init(ArmTrim_t *trim, const ArmTrimConfig_t *config,
                            const ArmTrimIO_t *io);
ArmTrimResult_t ArmTrim_Synchronize(ArmTrim_t *trim, const uint16_t p[3]);
ArmTrimResult_t ArmTrim_MoveRelativeX(ArmTrim_t *trim, float dx_mm);
ArmTrimResult_t ArmTrim_StartJog(ArmTrim_t *trim, int direction);
ArmTrimResult_t ArmTrim_ReleaseJog(ArmTrim_t *trim);
void ArmTrim_Process(ArmTrim_t *trim);
ArmTrimResult_t ArmTrim_Cancel(ArmTrim_t *trim);
ArmTrimResult_t ArmTrim_Exit(ArmTrim_t *trim);
ArmTrimResult_t ArmTrim_ClearFault(ArmTrim_t *trim);
ArmTrimStatus_t ArmTrim_GetStatus(const ArmTrim_t *trim);
void ArmTrim_ReportLateral(ArmTrim_t *trim, float error_mm);
```

`ArmTrimIO_t` 提供 `send(user,p[3],move_ms)`、`stop(user,joint_index)`、`now(user)`、可选 `lateral_report` 和 user。send/stop 成功表示通信调用成功，不表示机械完成；now 必须包含同步串口发送耗时。上下文、回调 user 存储及生命周期由调用者维护，运行期间不应重新初始化或直接改结构体。

迁移顺序：几何/标定/限位/碰撞配置 → 提供 IO → Init → 外部实物到参考且停车 → Synchronize → 相对移动或 StartJog → 主循环 Process → 正常完成读取状态 → Exit → 外部后续动作。

项目通过 `Arm_AcquireExternalMotion` / `Arm_ReleaseExternalMotion` / `Arm_ExternalMotionOwned` 与旧 Arm 入口互锁；已接受轨迹运行时拒绝新移动。停止入口会转发取消。新队友任务层不能绕过仲裁直接调用原始 ZLIS2 运动或控制板动作组。

核心不依赖视觉、底盘、蓝牙或 HAL；几何和标定由调用者输入。项目适配层负责停车条件及通信。默认核心碰撞关闭、启用范围仍是保守 ±5 mm；**当前工程显式启用箱体并开放完整局部模型范围**，迁移时不可忘记这一差别。

## 8. 轨迹、连续平移及限位含义

### 8.1 当前参数

| 项目 | 当前默认/项目值 |
|---|---|
| 最大空间分段 | 2 mm |
| 最高速度 | 10 mm/s |
| 加减速度 | 20 mm/s²；加速度单位不是 mm/s |
| 连续模式更新时间 | 50 ms |
| 最多分段 | 512，独立于旧 Arm 的 16 步队列 |
| 最终稳定等待 | 300 ms |
| 周期服务超时 | 250 ms |
| 分发迟到容忍 | 20 ms |
| 单段最大 P 变化 | 80 |
| 肘关节正弦阈值 | 0.05，拒绝接近奇异的路径 |
| 关节插补检查 | 每边采样含端点 9 点；模型高度偏差阈值 1 mm、方向阈值 1° |
| 连续蓝牙有效按住包超时 | 500 ms |
| 初始参考附近范围搜索 | ±75 mm，范围扫描分辨率 1 mm |

固定高度和方向求解整条 X 轨迹，保持起始肘部逆解分支；发送前检查逆解、关节范围、取整、解连续性、关节线性插补偏差和启用的碰撞模型。不可达或超范围 DX 整次拒绝，不截断、不换分支、不放宽限位。DX=0 不运动。

连续模式先预规划到所选方向的软件边界，按住持续执行；松开规划减速尾段并在预计完成后保留参考。达到边界正常结束，继续保持按住不会重复启动。不是无限逐段盲走到物理硬限位。

最高速度 10 mm/s、减速度 20 mm/s² 时，理想减速约 0.5 秒和 2.5 mm；还需考虑收到松开包的延迟及当前段执行。需立即取消时用 STOP，但停止发送成功仍不是物理停止反馈。

### 8.2 为什么看起来没到关节限位也会停

等高、等方向平移要求三个关节协同，一关节的活动空间不能直接等同于抓口的直线可达空间。当前还有几何伸展、肘部奇异、固定逆解分支、P 取整/插补、碰撞和 ±75 mm 搜索范围。早期“只能动两三下”还涉及当时项目累计 ±5 mm 台架限制；该旧项目限制已取消。

当前旧参考模型范围如下，正负方向相对初始参考：

| 场景 | 仅运动学模型 mm | 箱体与估计包络启用后 mm |
|---|---|---|
| BALL | −75～+4 | −75～+4 |
| HOSTAGE，旧参考 | −35～+56 | −11～+56 |
| BUCKET | −71～+14 | −42～+14 |

BALL 的 −75 是当前搜索边界，不证明真实物理限位就是 −75。表格不是已实车验证的安全范围；新参考、几何和外廓变化后需重新计算，实际每条请求仍再做完整预检查。

## 9. 后方障碍物与碰撞保护

用户暂把后方障碍视为覆盖机械臂宽度的长方体，已有示意图确认。参数：

| 参数 | 数值与来源 |
|---|---|
| 000 到箱体最近面后方距离 d | 30 mm，用户测量 |
| 箱体前后深度 L | 40 mm，用户要求按该值计算 |
| 箱体总宽 | 100 mm，居中覆盖运动平面 |
| 底板到箱体顶端 H | 70 mm |
| 000 高于底板 | 31.2 mm |
| 箱体 X 范围 | −70～−30 mm |
| 箱体 Y 范围 | −50～+50 mm |
| 箱体 Z 范围 | −31.2～+38.8 mm |
| 000→001 胶囊半径 | **15 mm，估计** |
| 001→002 胶囊半径 | **20 mm，估计** |
| 002→虚拟抓点胶囊半径 | **60 mm，估计** |
| 额外间隙 | 5 mm |
| 扫掠采样间的附加位移上界分辨率 | 0.5 mm |

用户粗估臂厚约 24 mm，但尚未明确是左右厚度、侧面外廓还是最大支架突出量。上述三个半径不是实测 CAD 外廓，特别是 60 mm 是否完整覆盖相机、支架、张开夹爪仍未证明；`ENVELOPE_ESTIMATED=1` 明确表示这一点。

检查接口：`ArmCollision_CheckPose`、`ArmCollision_CheckEdge`、`ArmCollision_ProjectModel`。三段胶囊绕轴心连线/工具向量表示实体，计算到箱体的距离；线性关节插补的采样之间还加入位移上界，不仅检查起点/终点。保护可靠性仍取决于真实实体完全落在估计包络内、实际插补足够接近模型。

已接入微调的参考同步、模型范围扫描、完整移动轨迹与松开减速尾段。**尚未接入固定就位、抬起、撤回和控制板自存动作组**。当前也没有自碰撞、桶口、目标物、其他车体部件模型，没有真实碰撞传感器。

旧抬起 `HOSTAGE_LIFT/TH_U` 的 1800/1855/528/500 符合当前关节活动范围，但模型判定与后方箱体有干涉；零厚度轴心线的最近间隙约 11.6 mm，小于第一段 15 mm 半径加 5 mm 间隙。用户授权恢复 P1800 并不意味着这个旧抬起姿态已证明无碰撞。新会话不要把“恢复限位可请求”写成“恢复安全抓取”。

示意图文件：`diagrams/arm-rear-box.drawio`、`diagrams/arm-rear-box.png`、`diagrams/arm-rear-box.json`。说明见 `ARM_REAR_BOX_V4_4.md`，其中历史固件哈希不代表当前 v4.5。

## 10. 手机蓝牙调试器：当前完整配置

用户使用的 App 名称为“蓝牙调试器”。数据包结构界面可选择 bool、byte、short 等类型，字段圆圈中的 0/1/2 是下标，不是实际发送值。不要再要求用户把占位 byte 固定设置为 84；当前默认 0 可正常接收。

### 10.1 类型与字节结构

| 类型 | 数量 | 传输占用 |
|---|---:|---:|
| bool | 8 | 1 字节位图，bool[0] 最低位 |
| byte | 1 | 1 字节，默认 0，无控件；兼容旧 84/0x54 |
| signed short | 1 | 2 字节，小端，有符号，ydnum |
| int、float | 0 | 0 |
| 包头、校验、包尾 | 各 1 | 3 字节 |

总共 **7 字节**，顺序：

```text
A5 | 按钮位图 | BYTE0 | short低字节 | short高字节 | SUM | 5A
SUM = (按钮位图 + BYTE0 + short低字节 + short高字节) & 0xFF
```

SUM 不含头尾。二进制包不追加换行。BT 侧 USART6 为 9600、8N1。不要删 byte，也不要把新页字段追加到原团队 41 字节包上。

### 10.2 字段、控件与顺序

| 字段 | 位值 | 名称 | 控件行为 |
|---|---|---|---|
| bool[0] | 0x01 | 抓球前 | 瞬时按钮；三关节就位后自动建 BALL 参考 |
| bool[1] | 0x02 | 抓人质前 | 同上；当前旧三关节值待重录 |
| bool[2] | 0x04 | 放球前 | 同上，建立 BUCKET 参考 |
| bool[3] | 0x08 | wt | **按住 True、松开 False**，连续微调 |
| bool[4] | 0x10 | jiaj/夹紧 | 瞬时按钮，仅 003=P500 |
| bool[5] | 0x20 | song/松开 | 瞬时按钮，仅 003=P1800 |
| bool[6] | 0x40 | start/查询 | 瞬时查询按钮，不是运行授权 |
| bool[7] | 0x80 | stop/停止 | 取消并使参考失效，停止优先 |
| byte[0] | — | BYTE0 | 无需绑定控件，保留默认 0 |
| short[0] | — | ydnum | 带符号整数输入，建议固定用 +1、−1 |

bool 初值全 False，不使用自锁开关；**完整包每 100 ms 发送**。除了 wt，其他动作只在新上升沿触发一次。一次一个动作按钮；组合冲突会拒绝，STOP 优先。只发一次按下包会在 500 ms 后停止并撤销参考。

`ydnum` 格式接受 −150～150；当前只读取符号：正=向外、负=向车、0=拒绝启动。2、30 均与 +1 同义，绝对值不是速度或目标毫米。

### 10.3 核对用 HEX

| 操作 | 完整帧 |
|---|---|
| 抓球前，方向 0 | `A5 01 00 00 00 01 5A` |
| 抓人质前，方向 0 | `A5 02 00 00 00 02 5A` |
| 放球前，方向 0 | `A5 04 00 00 00 04 5A` |
| 按住向外 +1，周期重复 | `A5 08 00 01 00 09 5A` |
| 松开且方向保持 +1 | `A5 00 00 01 00 01 5A` |
| 按住向车 −1，周期重复 | `A5 08 00 FF FF 06 5A` |
| 松开且方向保持 −1 | `A5 00 00 FF FF FE 5A` |
| 夹紧，方向 0 | `A5 10 00 00 00 10 5A` |
| 松开，方向 0 | `A5 20 00 00 00 20 5A` |
| 查询，方向 0 | `A5 40 00 00 00 40 5A` |
| 停止，方向 0 | `A5 80 00 00 00 80 5A` |

软件自动组包时不需要手输入 HEX。BYTE0 若仍用 84，校验必须按实际 byte 计算。

### 10.4 当前日常顺序

1. 外部确保车停车，按所需固定参考并松开。
2. 等至少约 3 秒并观察实物稳定；收到 `BENCH READY_ESTIMATED; TRIM_AUTO_INITIALIZED` 后已自动就绪，手机无需 BEGIN/授权控件。
3. ydnum 选 +1 或 −1，按住 wt 平移，松开减速，等 COMPLETE_ESTIMATED。
4. 需要改变方向时，先松开、等稳定，再改符号重按；按住时直接改符号会减速结束，要求重新松开再按。
5. 夹紧或松开只改 003。夹紧后可切换放球前并保持夹持，再微调并单独松开。
6. 正常松开完成后参考保留，可以再移动；取消、通信错误或故障后重新建立实际参考。

忙碌时按键请求不排队，被拒绝的按住不会在动作完成后自动补启动。到边界后持续按住也不会再次运动。

### 10.5 兼容协议与语义不能混用

旧测试页是 **11 字节、4 个 signed short**，字段顺序 cmd、dx_mm、close_p、open_p，均小端；cmd 决定动作，dx_mm 仍是相对毫米。当前 v4.5 下，其固定就位同样不再发送 003。

```text
A5 | cmd(2) | dx_mm(2) | close_p(2) | open_p(2) | SUM | 5A
```

旧 cmd：0 无新动作；1/2/3=PREP BALL/HOSTAGE/BUCKET，4/5/6=BEGIN 对应参考，7=DX，8=END，9/10=GRIP 分别取 close_p/open_p，11=STATUS，12=STOP，13=CLEAR。相同非零 cmd 重复包不再次触发，需先发 0 或改变 cmd。详细配置见 `BLUETOOTH_TEST_PAGE_SETUP_V2.md`。

原团队车控/机械臂组合包及 41 字节 bool/GAP 等入口仍保留。41 字节共同结构为 `A5 | 两字节bool | 16个short | GAP四字节 | SUM | 5A`。其中 ARM_CMD 在字节29～30，ARM_X在31～32，ARM_Y在33～34，GAP在35～38，SUM在39，尾在40（从0开始编号）。但本地 `(7).pro` 与队友 `(20).pro` 的 JOY_X/JOY_Y/旧 SERVO_MODE 排列不同，不能只看总长度就复用同一字段表；核对 `DecodeBoolArmGap41`、`DecodePhone20` 等实际解析分支。

这些不是当前独立 7 字节微调页；旧协议某些姿态按钮会同时改变夹爪，不能用来代替当前独立动作。旧独立 11 字节页和新 7 字节页也不刷新底盘/摇杆 keepalive。

## 11. 文本命令、状态与恢复

文本走已有蓝牙文本入口，每条以 LF 或 CRLF 结束，回复是 ASCII 文本，不是另一套 ValuePack 字段。手机没有日志控件时可通过 SWD 读状态。

| 命令 | 行为 |
|---|---|
| `@BENCH READY BALL` / `HOSTAGE` / `BUCKET` | 执行三关节固定动作并自动同步 |
| `@BENCH PREP BALL` / `HOSTAGE` / `BUCKET` | 执行三关节固定动作，不自动同步 |
| `@BENCH CLOSE`、`@BENCH OPEN` | 独立夹紧/松开 |
| `@ARM TRIM BEGIN BALL` / `HOSTAGE` / `BUCKET` | 外部确认已在该姿态，只建参考，**不发送就位运动** |
| `@ARM TRIM DX 2`、`DX -2` | 相对上次预计完成位置移动 ±2 mm；连续发送累计 |
| `@ARM TRIM JOG 1` / `JOG -1` | 启动连续方向运动 |
| `@ARM TRIM KEEP 1` / `KEEP -1` | 对应方向每 100 ms 保活 |
| `@ARM TRIM RELEASE` | 正常减速结束，保留参考 |
| `@ARM TRIM STATUS` | 查询范围、估计/目标 P、状态、段进度等 |
| `@ARM TRIM STOP`、`@ARM STOP` | 取消，发送停止，参考失效 |
| `@ARM TRIM END` | 空闲时退出，把运动权交回外部 |
| `@ARM TRIM CLEAR` | 停止尝试结束并人工检查后清故障，**不恢复参考** |
| `@BENCH CLEAR` | 清独立夹爪故障，不恢复参考 |

正常简化流程由 READY 自动建参考，无需手动 BEGIN/END；底层接口则必须按“实物就位 → 同步 → 微调 → END → 外部”理解。

`STATUS` 包含 `MODEL_MM`/`ENABLED_MM`、`OFFSET_X10` 或毫米偏移、关节限位、`L2_X100=8475`、`EST_P`、`TARGET_P`、段计数及 `REAR_BOX_GUARD=1 ENVELOPE_ESTIMATED=1 CLEARANCE_MM=5` 等。估计 P 不是真实位置。

| 状态值 | 名称 |
|---:|---|
| 0 | IDLE |
| 1 | READY |
| 2 | MOVING |
| 3 | SETTLING |
| 4 | COMPLETE_ESTIMATED |
| 5 | STOPPING |
| 6 | CANCELLED |
| 7 | FAULT |

| RESULT/ERR | 含义 |
|---:|---|
| 0 | OK |
| 1 | 参数/配置无效 |
| 2 | BUSY，拒绝排队或覆盖 |
| 3 | 缺少有效参考 |
| 4 | 超启用范围或关节范围 |
| 5 | 轨迹无效，含逆解/奇异/连续性/插补/箱体失败 |
| 6 | 串口发送或停止传输失败 |
| 7 | 主循环服务超时 |
| 8 | 故障锁定 |
| 9 | 按住连接保活超时 |

`ARM_REQUEST_ERROR` 表示最近请求结果，拒绝新请求不一定等于正在运行的状态已经故障。诊断时必须同时看 reference_valid、state、error 和最近请求结果。

取消、蓝牙接收错误恢复、外部改动关节、控制板复位或位置估计不再可信后，要先停止，再重新实物就位及同步。CLEAR 不会自动重试、自动动机械臂或恢复位置。旧调参会话如占用，按 `@ARM STOP`、等停止、`@ARM EXIT` 后才能建立新会话。

## 12. 串口拓扑及此前排障结论

当前机械臂链路：手机蓝牙 → STM32 USART6 收包/分发 → 三关节规划/bench → USART3 → 舵机控制板 → 舵机。串口和速率以 `board_app.c`、MSP、相关 config 为准，不要把蓝牙 9600 和舵机板 115200 混用。

| 通路 | STM32 TX / RX | 当前用途/设置 |
|---|---|---|
| USART6 | PC6 / PC7 | 蓝牙控制，9600、8N1；文本和二进制共享接收分发 |
| USART3 | PB10 / PB11 | ZLIS2 控制板命令，115200、8N1 |
| USART1 | PA9 / PA10 | 调试日志，当前115200、8N1；桥接占用按工程配置检查 |
| UART4 | PC10 / PC11 | MaxiCam，当前115200、8N1；并非已实施视觉微调 |
| USB 转 TTL/控制板 USB | 用户实际接线 | 用户上位机直连记录/查询，和 STM32 微调通信是不同链路 |

接收使用单字节中断和环形缓存，总 HAL Rx/Tx/Error 回调在 `uart_bridge.c` 分发到各模块。迁移时不得写一个新的 HAL 回调覆盖原蓝牙、电机、相机回调。主循环依序运行 Bluetooth、车控、电机处理等，再调用 ArmTuner、ServoRemote、Arm；独立 trim/bench 的周期处理在 ArmTuner 内转发，不能漏掉。

### 12.1 蓝牙从无响应到恢复

早期要求 BYTE0=84，但用户 App 未找到设置方式；当前接受默认 0 和旧 84。截图中“byte0 圆圈 0”是下标，不足以判断发送值。以后用实际原始帧与计数核对，不要只看 UI 猜协议。

独立测试页不发送底盘保活，早期夹爪要求 CAR_READY 会挡住操作；当前在 READY/OFF/WAIT_CENTER/BRAKE_LOCK/LINK_LOST 且电机空闲、无故障等条件下允许独立夹爪。物理停车仍由用户/任务层确保，状态许可不是实物停车传感器。

用户后来确认固定动作等按钮能用，微调也能动；旧阶段抓球只能动两三次、抓人质几乎不动、放球某方向受限，不应继续把这些旧反馈定性为“当前蓝牙完全失效”。后续改为连续模式、更新限位和箱体检查，用户报告基本符合预期。

用户某次原始 RAM 报告 `BT_TRIM_VALID=0x6d4`、`BT_PRESSES=0x1d`、`BT_BASE_VALID=0`、`BT_INVALID=0`：测试包和按钮已经识别，旧基础帧计数为 0 可以是独立页面的正常结果。环形缓存中的周期包 `A5 00 00 1E 00 1E 5A` 表示 ydnum=30、当前按钮位图=0；它不证明按下瞬间的 wt 也为 0。单次 raw snapshot 可能抓到松开帧。

### 12.2 Flash 校验/诊断脚本曾出现的问题

- `Programming Finished / Verified OK / Resetting Target` 表示该次烧写及校验成功，但不说明当时选择的是哪个版本。
- 用户曾对原 v3 获得 `CURRENT_FIRMWARE_VERIFIED`；后续目录镜像已换版本，这个旧结果不能替代 v4.5 校验。
- 早期 PowerShell/OpenOCD 内联 Tcl 报 `Unexpected command line argument: CURRENT_FIRMWARE_MISMATCH`，属于引号/参数传递问题，当次没有证明板上版本。
- 早期诊断只显示字段标签，没有内存数值，是 `read_memory` 返回值没有显式输出；维护脚本已修正，并按匹配 ELF 符号和类型读状态。
- 用户手写地址如 0x20001934 等是某一旧 ELF 的地址，新构建可能变动，**不可照搬**。
- 单独 `verify_image` 和连续读 tick 的旧命令没有完整 halt/resume 管理；TICK_A=TICK_B 不足以直接断定固件死机。现在用维护脚本保证最终恢复运行。

### 12.3 控制板读 P 值曾经无返回

用户使用 ASCII `#002PRAD!`、115200、8N1；USB 转 TTL 直连控制板，STM32 串口线已断开。换 COM13，短接 TX/RX 自测有回显；直连控制板时 TX 增加但 RX 不增加，旧 RX=915 是历史数据。控制板独立供电、绿工作灯约每秒闪、上电存储复位动作仍执行；控制板 USB 查询当时也没返回。

上位机曾输出控制板信息：`Board_version:ZL_KPZ2`、`Info_version:20240423`、`servo mode:Analog`、各 UART 配置 115200、`usb state:ON`。这些是当时上位机报告，不是持续读取的当前板状态。

**用户后来明确说这个问题已解决，但没有在会话中给出最终根因。** 不要把接线、掉电、波特率或某软件命令猜测写成已确定原因，也无需重新围绕此旧故障阻塞机械臂开发。

RX/TX 计数分别是串口助手已收到/已发送的字节累计值，TX 灯闪只能证明 USB 转 TTL 端发出活动，不能证明控制板收到合法命令或已返回数据。

## 13. 已做的软件验证与实测边界

| 版本/阶段 | 已有证据 | 尚未证明的内容 |
|---|---|---|
| v4.3 | arm 246 项、kinematics 61 项；新关节范围构建通过 | 新结构的全部抓取任务 |
| v4.4 | collision 40 项、trim 48676 项、Bluetooth 28780 项，完整构建；用户定性反馈长按基本符合预期、抖动不明显 | 完整实体外廓、所有间隙、近中远各 10 次任务验收 |
| **v4.5** | **Bluetooth 29186 项，完整 ARM GCC 构建通过**；新增夹紧/松开后切换全部 READY/PREP 不发 003 的回归 | 本次烧入确认、保持夹持实车检查、HOSTAGE 新参考及任务验收 |

v4.5 修改 bench 参考通路和启动版本号，没有改正逆解/轨迹/碰撞算法；未为了文档重新跑全套测试。上一轮实际运行过受影响 Bluetooth 套件及完整固件构建，历史其余结果不能冒充本轮重跑结果。

当前完整构建内存：text **82204 B**、data **108 B**、bss **24724 B**。主机测试使用 HAL 替身，检查发送帧、时序和状态，不是实物角度、碰撞或抓取传感器测量。

用户最终定性反馈出现在 v4.5 去除 003 之前的可用基线上，没有附上当时板上镜像的完整哈希；不能锁定为某个已核验构建，也不能记成 v4.5 验收成功。当前已编译、已打包不等于用户已烧录。

## 14. 构建、测试、模型检查命令

本机确认可用的工具：

```text
主机 GCC：D:\c++\MinGW\bin\gcc.exe
ARM GCC：C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin
Python：D:\Anaconda\python.exe
OpenOCD：D:\Tool\xpack-openocd-0.12.0-7\bin\openocd.exe
```

### 14.1 主机测试

```powershell
Set-Location "D:\工科大\e-control-trim-test-20261001"
$env:PATH = "D:\c++\MinGW\bin;" + $env:PATH
powershell -ExecutionPolicy Bypass -File .\tests\arm_bt\run.ps1
```

修改几何、参考、限位或碰撞后，按影响面还需运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\tests\arm_kinematics\run.ps1
powershell -ExecutionPolicy Bypass -File .\tests\arm\run.ps1
powershell -ExecutionPolicy Bypass -File .\tests\arm_trim\run.ps1
powershell -ExecutionPolicy Bypass -File .\tests\arm_collision\run.ps1
```

改旧蓝牙/车控/底层舵机驱动时，再运行受影响 `tests/car/run.ps1`、`tests/zlis2/run.ps1` 等；不要只写一个镜像实现的测试就宣布通过。

### 14.2 只在 PC 计算箱体及范围

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\inspect_rear_box.ps1
```

此脚本编译纯计算程序，不连接 OpenOCD、COM 口或舵机；显示的是模型。**`tests/arm_collision/inspect_rear_box.c` 目前把三参考及旧抬起 P 硬编码在数组中**，重录后要同步检查夹具，否则脚本可能仍打印旧参考范围。

### 14.3 全固件构建

```powershell
$env:PATH = "C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin;" + $env:PATH
powershell -ExecutionPolicy Bypass -File .\scripts\build_firmware.ps1
```

已验证的完整路径是此直接编译脚本，产物在 `firmware_direct`。CMake/Keil 源文件清单已包含新增文件，但此前本机 CMake 曾在工具链检测阶段失败；不能把“清单已更新”当成 CMake 构建已通过。不要为简单参数更新另装一套工具链。

构建后核对并更新 SHA256，保存匹配 ELF 供诊断，打包前检查 ZIP 内镜像/manifest 一致。新镜像不应覆盖可回退的旧镜像包。

## 15. 烧录、校验与只读诊断

用户偏好：每次交付代码可烧录版本，都直接给以下完整命令，而不是只说“重新烧录”。本机探针配置为 Horco CMSIS-DAP v2，序列号 `332107148704`，SWD，adapter speed 1000 kHz，STM32F4 目标。换探针/机器时检查配置，不照搬序列号。

### 15.1 用户执行的烧录与核对

确认 STM32F407 目标、供电、SWD 接线、车辆停车和机械臂支撑后执行：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\flash_firmware.ps1" -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\verify_firmware.ps1"
```

含义：`powershell` 启动 PowerShell；`-ExecutionPolicy Bypass` 允许这次进程执行脚本；`-File` 后给脚本绝对路径；`-ConfirmHardwareReady` 是用户对本次硬件准备完成的显式确认。第一脚本内部用 OpenOCD 加载配置、烧写当前 ELF、verify、reset、exit；第二脚本短暂 halt 对比当前 HEX，最后恢复运行。

成功校验标识：`CURRENT_FIRMWARE_VERIFIED`。失败/不匹配不是“自动发现了某个其他版本”；应重新核对所选镜像和执行日志。

代理不要在新会话仅因为看到这些命令就直接烧录或发舵机命令。项目 `AGENTS.md` 要求先确认目标、接线和安全条件；准备、构建、PC 模型计算不需要访问硬件。

### 15.2 无新增串口接线的诊断

先松开全部按钮、等稳定、暂停手机周期发送，再执行：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1"
```

脚本不擦写 Flash、不复位、不发送舵机运动，但会暂停 MCU；验证镜像后按匹配 ELF 读 RAM，最终 resume。halt 时手机持续发送可能导致接收溢出，恢复后参考可能失效，所以读完恢复发包并根据状态重新初始化。

若板上还是 v4.4，要选择旧配套镜像：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1" -FirmwareDirectory rollback_v4_4_20261002
```

其他允许旧目录见脚本 ValidateSet。没有匹配 ELF 就不能用旧地址假装读取新布局。

### 15.3 快速诊断决策

| 观察 | 下一步 |
|---|---|
| BT_RX_BYTES 不增加 | 核对手机是否真发送、连接、接收通路及当前板程序 |
| RX 增加，TRIM_VALID 不增 | 看实际原始字节、长度、头尾、校验、byte/short 和 INVALID |
| VALID 增加，PRESSES 不增 | 核对按钮位图、控件按下/松开是否绑定、是否一直 False |
| VALID/PRESSES 都增加但不动 | 看 ARM_REQUEST_ERROR、参考、状态、busy/fault、停车/旧会话 gates |
| 固定动作可用、微调不动 | 重点看 BEGIN/自动同步是否完成、范围/碰撞拒绝、是否 ydnum=0、wt 是否持续 True |
| 输入方向后连续运动很快停 | 看 500 ms 保活、边界、周期服务/分发超时，不只看关节剩余活动度 |
| 夹紧后切换姿态又松了 | 校验是否真为 v4.5；看是否误用了原团队旧姿态入口，而非三个独立参考 |

关键字段：`ARM_REFERENCE_VALID`、`ARM_STATE`、`ARM_ERROR`、`ARM_REQUEST_ERROR`、`ARM_JOGGING`、`ARM_EST_P`、`ARM_TARGET_P`、关节范围、MODEL/ENABLED、OFFSET、SEGMENT_INDEX/COUNT、BT_RAW_LAST_LENGTH/FRAME。原始帧只读 LAST_LENGTH 指示的前 N 字节，后面可能是历史残留。

## 16. 当前未完成事项与建议接续顺序

### 优先一：重录抓人质参考

用户目前还没录，因为机械臂改变、旧动作不再适配。新会话首先承接这件事，**不再索要 003**，也不要自行猜一个新三关节值。

1. 用已能正常工作的上位机/录制方式把实物调到抓人质前目标姿态，记录新的 000/001/002 P；固定动作重新返回几次确认重复性。
2. 核对本次机械变化是不是还改变几何、舵机安装零位、工具向量。仅换动作参数不能替代变更后的模型标定。
3. 更新 `ARM_TRIM_HOSTAGE_P0/P1/P2`，bench 和微调同步都使用同一组宏，不可各自写不同表。
4. 搜索测试和模型夹具中的旧字面量 1684、2136、785，区分当前项目参考期待值与有意保留的历史/通用模型夹具；同步必要的参考、范围和测试。
5. 重算 HOSTAGE 连续范围和箱体间隙；原 −11～+56 不能沿用为新姿态结果。
6. 跑受影响测试、全固件构建，保存旧镜像，交付新哈希及烧录/校验命令。
7. 实车先检查固定姿态、±小位移，再逐步验证抓取；任务统计按模板记录。

### 优先二：验证 v4.5 夹爪隔离

夹紧后切换 BALL/BUCKET，检查不改变夹爪；松开后切换，检查仍松开；HOSTAGE 重录后补同样检查。源码和主机测试已完成这项修复，实物结果待用户反馈。

### 优先三：继续在用户仓库记录变更

首次源码提交和上传已完成，Git 所有权检查也已处理。继续时不必重建/删除目录；先核对实际 status/log/remotes。当前仓库保存软件基线，HOSTAGE 未重录、v4.5 未实物验收；不要发布“全部验收通过”标签。

当前准备目录已同步 v4.5 源码、配置和测试，后续在测试目录改完必须同步；否则会出现板上固件和 Git 源码不匹配。两个目录不自动同步。`build/prepare_github.py` 是之前建立 v4.4 快照的一次性脚本，`build/package_v4_5.py` 也带“目录无Git/资产不存在”等前提断言；现在上传目录已有 Git，不能不经核对就重跑、删除目录或覆盖现有 Git 状态。

### 后续可选工作

- 用户愿意后补整机 CAD、准确相机/夹爪外廓和其他板载元件位置；此前未承诺做整机建模。
- 给固定就位/抬起/撤回增加模型路径检查，需要明确可靠起点和状态估计，不能仅拿终点安全代替路径安全。
- 连续平滑性可小步优化。当前用户反馈不算特别抖，优先保留可用行为，在新分支修改并比较实物。
- 视觉模块输出新鲜的前后毫米误差，做相机与工具坐标关系标定、延迟和遮挡处理；每张图的修正只消费一次，运动后等稳定再取新图。
- 如将来加入闭环控制，先确定真实可用的测量反馈，再选择控制器；不能在只有目标 P 和时间估计时直接声称完成 PID 位置闭环。

## 17. GitHub 进展、所有权异常与首次推送

最初制作交接文档时，上传目录 `main` 无提交、无 remote。后续 2026-10-02 已解决所有权检查，完成首次源码提交 `b4664dd`，origin 为 `https://github.com/Morechips/e-control-arm-trim.git`，推送成功且已用 ls-remote 核对远端 main 一致。本仓库现在不再是未提交的空 Git 仓库。

用户遇到：目录属于 `DESKTOP-DTPOII2/CodexSandboxOffline`，当前 PowerShell 用户为 `DESKTOP-DTPOII2/ljn`，Git 报 `detected dubious ownership`。解释是 Git 的所有权保护，不是代码损坏。

现已在用户 `C:\Users\ljn\.gitconfig` 只添加此目录的信任条目并读回确认。换用户/环境又出现同样错误时，在对应用户自己的 PowerShell 中检查或仅信任此目录：

```powershell
git config --global --add safe.directory "D:/工科大/e-control-arm-trim-github"
```

此前用户的标准配置没有这个信任条目；本轮直接确认并写入了该用户的配置路径。代理在沙箱用户下仅运行 --global 不能不核对配置来源就假定已修改 ljn。只读核对时可使用每次命令的 `git -c safe.directory=...`；不要设 `safe.directory=*`，也不要为了消除提示删除 `.git` 或粗暴改整盘权限。

用户之前写 `$repoUrl = Read-Host "https://..."` 后“卡住”：Read-Host 后面的 URL 是提示文字，仍在等待输入。可 Ctrl+C，然后直接赋值；不是 Git 网络卡死。

继续时先检查：

```powershell
Set-Location "D:\工科大\e-control-arm-trim-github"
git status --short --branch
git remote -v
```

以下是首次提交流程的历史说明，当前仓库已完成，不要整块重跑；只有另建空目录且确认暂无提交/remote 时才使用：

```powershell
git add .
git status --short
git diff --cached --stat
git commit -m "arm: save v4.5 planar-only reference actions"
$repoUrl = "https://github.com/Morechips/e-control-arm-trim.git"
git remote add origin $repoUrl
git remote -v
git push -u origin main
```

若已有 origin，先核对地址，不重复 add；若已有提交，不重复初始化。浏览器账号登录和双重验证由用户本人完成；不要求用户把账号凭据发给代理。不强推覆盖未知远端历史。

源码保存 Core、配置、tests、scripts、docs，忽略构建产物/回退包。匹配预编译镜像放 Release 附件，不直接 add 所有 ELF/HEX/BIN。`FIRMWARE_SHA256.txt` 记录镜像身份，不能让它继续指向旧构建。

`GITHUB_FIRST_PUSH.md`、`README.md` 和 `RELEASE_NOTES_V4_5.md` 已有流程及当前未验收边界。现在源码已上传，但没有 GitHub Release，固件 ZIP 仍是本地资产；不要把源码推送说成已发布镜像或已完成硬件验收。

## 18. 快速阅读清单和操作约定

建议接续顺序：

1. 本文及 `AGENTS.md`。
2. `Core/Inc/arm_config.h`、`arm_trim_project_config.h`、`arm_collision_config.h`。
3. `ARM_FIXED_ACTION_V4_5.md`、`BLUETOOTH_TEST_PAGE_SETUP.md`、`ARM_JOG_V4_TEST.md`。
4. `arm_trim.h/.c`、`arm_trim_bluetooth.c`、`arm_trim_bench.c`。
5. `ARM_REAR_BOX_V4_4.md`、`BLUETOOTH_DIAGNOSTICS.md`、`ARM_TRIM_GUIDE.md`、`ARM_TRIM_ACCEPTANCE.md`。
6. 对应 tests 和 build/flash/verify/diagnostics 脚本。

不要重复已解决的问题：不要求固定 byte=84，不新增参考/授权手机控件，不再让当前三个参考发送 003，不把 ydnum 当当前毫米目标，不回退 L2/关节限位，不把软件模型区间写成实物安全保证，不把既有串口读 P 故障重新当成未解决。

继续修改前读工程规则，保留用户现有文件与团队工作树。参数/行为改动后运行受影响测试和全固件构建；Markdown、记录和模板修改不需要重烧板子。模型计算、文档和准备镜像可先自主完成，硬件动作须有本次实际条件依据。

## 19. 可复制给新会话的开场说明

```text
请接续我的 e-control 机械臂开发，先完整阅读
D:\工科大\e-control-trim-test-20261001\ARM_HANDOFF_20261002.md
以及该目录 AGENTS.md，并核对当前源码和固件，别直接覆盖原团队目录。

当前 v4.5 已修复：抓球前、抓人质前、放球前三参考只控制000～002，
切换姿态保留003夹爪。夹紧500、松开1800单独控制。
长按wt连续等高等方向前后微调已实现，ydnum只取正负号。
目前最重要的待办是抓人质动作重录，我还没提供新的000/001/002 P值；
旧值1684/2136/785不能当成改装后的有效姿态。

L2当前代码84.75 mm，关节范围000=915～1800、001=947～2500、002=500～1874；
有后方箱体估计包络检查，但只保护微调。不要恢复早期±5mm项目限制。
继续处理新参考、验证夹爪隔离、实测验收及我的GitHub保存工作。
每次给我可烧录版本都附完整PowerShell烧录和校验指令；
不要把模型预计完成写成物理到位、无碰撞或夹取成功。
```

如新会话看不到这个本地目录，直接提供本文 Markdown 文件；仅粘贴开场说明无法替代完整参数、协议和状态记录。
