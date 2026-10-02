# v4.5 按住连续平移：测试与限位核对

2026-10-02。本目录为 v4.5：三个测试参考动作仅控制 000～002，切换姿态保留夹爪，抓人质旧三关节值待重录。详见 [固定动作与重录说明](ARM_FIXED_ACTION_V4_5.md)。L2 保持 **(82 + 87.5) ÷ 2 = 84.75 mm**，正解、逆解和微调共享几何及用户测量的关节范围。蓝牙控件和速度不变；本轮未烧录，板上版本需用镜像校验确认。以下范围仍按旧参考计算，结构改变和重录后须重新核对；箱体模型见 [碰撞测试说明](ARM_REAR_BOX_V4_4.md)。

## 手机操作

保留现有 8 bool、1 byte、1 signed short，总长 7 字节。byte 保持默认 0。bool 顺序不变；三个参考按钮现在不发送 003。

1. 车辆停车，按抓球前/抓人质前/放球前，松开并等至少 3 秒，确认实物稳定。
2. `ydnum` 填 **+1 向外、−1 向车靠近**，`wt` 绑定 bool[3]，**按住 True、松开 False**。现在只使用数字的正负号，2 和 30 均代表向外，绝对值不表示毫米或速度；格式仍接受 −150～150。
3. 完整数据包每 **100 ms** 发送，按住期间持续发 True，松开后发 False。只在按下时发一次的控件不能用于连续模式。用户截图 Tx≈118 B/s 表明有持续发包，仍须确认 bool 在按住期间保持 True。
4. 松开后减速，等预计完成后夹紧/松开。改变方向先松开、等稳定，再输入另一符号并按住。按住期间改符号会减速停下，要求重新松开再按。

输入 0 不启动移动。忙碌期间的按下被拒绝，不会在动作完成后自动补执行；需松开后重新按。到边界后仍按住不会重启。查询 `start` 是 bool[6]，停止是 bool[7]。

## 当前边界

| 关节 | v4.4 关节 P 下限 | v4.4 关节 P 上限 |
|---|---:|---:|
| 000 | 915 | 1800 |
| 001 | 947 | 2500 |
| 002 | 500 | 1874 |

通用 `arm_config.h` 与微调使用相同的活动范围；这些是关节活动度，不是碰撞安全边界。003 不参与微调，外部夹紧 P500、松开 P1800。按用户确认，000 上限现为 1800；旧 HOSTAGE_LIFT/TH_U（1800/1855/528/500）恢复通过关节限位检查。恢复可请求不代表已验证无碰撞，当前包络模型判定该旧抬起姿态有干涉，需核对间隙或重录；此处不改写其P值。三个测试参考动作不变。

旧 v3.1 的累计 ±5 mm 台架限制已取消。v4.4 在参考点 ±75 mm 的搜索范围内，使用 L2=84.75 mm 和当前关节限位计算出的连续区间：

| 场景 | 历史 v3.1 开放累计范围 mm | v4.4 模型开放范围 mm | 主要边界原因 |
|---|---|---|---|
| 抓球 | −5～+4 | −75～+4 | 向车侧到本次 −75 mm 搜索边界；向外由几何伸展限制 |
| 抓人质 | −4～+5 | −11～+56 | 向车侧由后方箱体估计包络限制；向外由几何伸展限制 |
| 放球 | −5～+5 | −42～+14 | 向车侧由后方箱体估计包络限制；向外由几何伸展限制 |

模型区间按 1 mm 扫描，含逆解、同一分支、关节限位、近奇异限制和插补误差检查；整条连续路径仍单独预检查。000 上限已按用户明确确认更新为 1800。后方长方体位置和尺寸已记录，见 `ARM_REAR_BOX_V4_4.md` 与 `diagrams/arm-rear-box.png`；微调已启用箱体碰撞预检查；15/20/60 mm包络半径与5 mm间隙仍为估计，需要核对实物外廓。旧固定动作入口不使用本箱体保护。

抓球向外 +5 mm 时，当前模型无法维持同一高度和方向，不是所有关节 P 都到上限。需要更大的向外余量，应在重录参考姿态时留出弯曲余量，并核对模型标定；提高 P 上限不能解决几何伸展边界。

## 连续性与停止

- 按固定 50 ms 时间采样三角形/梯形速度曲线，三轴同步发送。默认最高 10 mm/s、加减速度 20 mm/s²，路径中不插入 300 ms 稳定等待；只在最终结束后等待。
- 发送节拍从 UART 发送开始计时，发送耗时包含在周期内；修正了旧实现每段到时后再发送所增加的间隙。最多 512 个独立轨迹点，不使用旧 16 步队列。
- 松开是正常减速结束并保留位置估计。最高速度下减速本身约 0.5 s、2.5 mm，另有蓝牙和已发送段的延迟。停止按钮立即请求取消，参考失效，之后重新按固定动作。
- 500 ms 没收到有效按住包时取消并使参考失效；接收/传输错误或明显调度超时同样停止。不会补发积压点来追赶时间。
- 默认插补检查为模型位置误差 ≤1 mm、角度误差 ≤1°，单段关节变化 ≤80 P，肘关节 |sin|≥0.05。以上不是实物精度保证。

模块没有实际关节位置、力或机械限位开关反馈，不能检测“碰到机械挡块”。停止依据为配置 P 边界、模型可达边界及任务搜索边界。新开放区间没有完成碰撞和抓取验收，第一次逐段验证车体、相机支架和线材间隙。

## 烧录、回退和诊断

核对关节安全范围后，由用户保持停车、确认供电与探针连接，运行：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\flash_firmware.ps1" -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\verify_firmware.ps1"
```

固件在 `firmware_direct`，完整 SHA256 在 `FIRMWARE_SHA256.txt`。`rollback_v4_4_20261002` 保留此次夹爪分离前的 v4.4；`rollback_v4_3_20261002` 保留加入箱体保护前的v4.3；`rollback_v4_2_20261002` 保留000上限1724的v4.2；`rollback_v4_1_20261002` 保留 v4.1 镜像。`rollback_v4_20261002` 保留 v4 镜像及旧 L2=88.65 mm 配置；旧 ZIP 同样保留。

先松开所有按键并等稳定，暂停手机周期发送，再读诊断，读取后恢复手机发送。否则 halt 期间仍收到字节可能触发 UART 接收溢出，恢复运行后参考会失效，需要重新固定动作。脚本先短暂 halt、验证参考镜像、读取 RAM，最后 resume；不烧写、不复位、不发舵机指令。地址和字段宽度来自参考 ELF，不能继续用旧的手写 RAM 地址。

```powershell
# 烧入 v4.5 后读取当前固件：
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1"
# 若板上为更新前 v4.4，选择对应镜像读取：
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1" -FirmwareDirectory rollback_v4_4_20261002
# 若板上为更新前 v4.3，选择对应镜像读取：
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1" -FirmwareDirectory rollback_v4_3_20261002
# 若板上为更新前 v4.2，选择对应镜像读取：
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1" -FirmwareDirectory rollback_v4_2_20261002
# 若板上为更新前 v4.1，选择对应镜像读取：
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1" -FirmwareDirectory rollback_v4_1_20261002
# 若板上为更新前 v4，选择对应的旧镜像读取：
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1" -FirmwareDirectory rollback_v4_20261002
# 若板上仍是 v3.1，选择保留的旧镜像读取：
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\read_bluetooth_diagnostics.ps1" -FirmwareDirectory rollback_v3_1_20261001
```

已修正旧脚本只输出标签、不输出内存数值的问题。新脚本另外输出参考有效性、状态、关节估计、目标、P 限位、模型/开放范围及累计偏移；v4 增加最近请求结果和连续状态。状态 2=MOVING、3=SETTLING、4=COMPLETE_ESTIMATED、7=FAULT。请求结果 2=BUSY、3=REFERENCE_REQUIRED、4=OUT_OF_RANGE、5=PATH_INVALID、7=SERVICE_TIMEOUT、9=LINK_TIMEOUT。

抓人质与放球在参考点的 ±2 mm 均通过本机路径检查。例如抓人质 +2 的模型目标为 1677,2130,786，−2 为 1691,2142,783，变化比抓球小。实物不动时应读取参考/请求结果和目标 P，再区分未执行与机械响应，不能仅凭外观认定是限位。

## 验证状态

软件测试覆盖三姿态双向连续路径、速度/高度/方向模型检查、UART 节拍、松开减速、停止/断连/传输故障、按住反向不重启、初始化与夹爪互锁、既有协议兼容。主机 HAL 模拟和完整固件构建通过；本轮未烧录或驱动实物。实际平滑性、可靠范围和抓取结果填写 `ARM_TRIM_ACCEPTANCE.md`。

v4.5 软件验证（2026-10-02）：蓝牙 29186 项检查及完整固件构建通过；text 82204 B、data 108 B、bss 24724 B。覆盖夹紧/松开后切换三个 READY/PREP 动作不发送 003，既有帧协议、互锁和连续微调回归通过。抓人质参考待重录，本轮未进行硬件测试。

历史 v4.4 软件验证（2026-10-02，后方箱体模型，L2=84.75 mm；机械臂/正逆解沿用v4.3已通过结果）：
- tests/arm_trim/run.ps1：48676；tests/arm_bt/run.ps1：28780；tests/arm_collision/run.ps1：40；tests/arm/run.ps1：246；tests/arm_kinematics/run.ps1：61 项检查通过。
- 覆盖三关节各上下边界外一步无发送、边界内端点接受、P1800 抬起动作恢复接受、夹爪兼容和微调全路径遵守新限位。
- scripts/build_firmware.ps1 完整构建通过：text 82204 B、data 108 B、bss 24724 B。本轮未烧录，真实部件包络和实物测试仍待确认。

历史 v4.1 软件验证（2026-10-02，L2=84.75 mm，旧限位）：
- tests/arm_kinematics/run.ps1：61 项检查通过，参考正解另以独立三角函数计算核对。
- tests/arm_trim/run.ps1：33459 项检查通过；tests/arm_bt/run.ps1：28159 项检查通过；tests/arm/run.ps1：201 项检查通过。
- scripts/build_firmware.ps1 完整 ARM GCC 编译链接通过：text 80028 B、data 108 B、bss 24676 B。
- STATUS 增加 `L2_X100=8475`；调试 UART 启动标识为 `ARM TRIM FW=V4.1 L2_MM=84.75 WT=HOLD_JOG BYTE0=0_OR_84`。
- 本轮没有烧录或驱动硬件；新几何下的实际可靠范围、高度/方向保持和平滑性待实测。

历史 v4 软件验证（2026-10-01，L2=88.65 mm）：
- tests/arm_trim/run.ps1：33339 项检查通过；三参考双向路径、速度/姿态模型、UART 周期、松开减速、近边界数值误差、超时/传输错误、预检查无发送。
- tests/arm_bt/run.ps1：28159 项检查通过；按住保活、松开/改向、断连/停止、边界不重启、固定动作/夹爪互锁、只控制 000～002、既有协议兼容。
- tests/arm/run.ps1：201；tests/arm_kinematics/run.ps1：61；tests/zlis2/run.ps1：123；tests/car/run.ps1 完整通过。
- scripts/build_firmware.ps1 完整 ARM GCC 编译链接通过：text 79956 B、data 108 B、bss 24676 B。
- 诊断脚本使用真实 OpenOCD Tcl 的离线夹具验证：read_memory 仅返回、不打印时仍能输出数值；新旧 ELF 字段解析、浮点/十进制转换、镜像不匹配后恢复路径通过。夹具未配置探针或目标，未访问实板。
- 本轮没有烧录/驱动硬件；实物连续性、实际可靠范围和阶段抓取验收仍待用户测试。
